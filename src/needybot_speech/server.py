#!/usr/bin/env python

import actionlib
import hashlib
import json
import os
import pyvona
import rospkg
import rospy
import string
import subprocess
import sys
import threading
import yaml

from needybot_msgs.msg import *
from pydash import _
from std_msgs.msg import Empty


class SoundProcess(threading.Thread):
    """
    Helper class inheriting from ``threading.Thread``. Creates a sox subprocess
    to play a sound file.

    :param: filename -- The full path of the sound file to be played
    :param: effects  -- The effects array from the SpeechActionServer instance
    """

    def __init__(self, filename, effects):
        self.complete = False
        self.effects = effects
        self.filename = filename
        self.kill_sub = rospy.Subscriber(
            '/needybot/speech/kill',
            Empty,
            self.handle_kill_msg)
        self.process = None
        super(SoundProcess, self).__init__()

    def handle_process_complete(self):
        """
        Completion handler for when a sound file has completed playback
        """
        self.complete = True

    def handle_kill_msg(self, msg):
        """
        ROS subscription handler the topic ``/needybot/speech/kill`` which
        takes a ``std_msgs.Empty`` message type. Kills the internal sox
        subprocess (stops playback).

        :param: msg -- std_msgs.Empty message type
        """
        if self.process:
            self.process.kill()

    def run(self):
        """
        Overrides ``run`` method of inherited ``threading.Thread`` class. This
        method creates a new sox subprocess.
        """
        self.process = subprocess.Popen(
            ['play', self.filename] + self.effects,
            shell=False)
        self.process.wait()
        self.handle_process_complete()


class SpeechActionServer(object):
    """
    This class is an implementation of a ROS actionlib server. It handles
    incoming action messages of type ``SpeechAction`` found in the
    ``needybot_msgs`` package.

    :param: start_server -- [True] a boolean signaling to start the server on
                            initialization
    """

    def __init__(self, start_server=True):
        # Private properties
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path('needybot_speech')

        self._cache_dir = rospy.get_param(
            '/needybot/speech/cache_dir',
            os.path.join(os.path.realpath(self._package_path), 'cache')
        )
        self._cache_manifest = os.path.join(self._cache_dir, 'manifest.json')

        self._feedback = SpeechFeedback()
        self._kill_pub = rospy.Publisher(
            '/needybot/speech/kill', Empty, queue_size=10)
        self._result = SpeechResult()
        self._server = actionlib.SimpleActionServer(
            'needybot_speech',
            SpeechAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        # Public properties
        self.effects = [
            'ladspa', 'tap_deesser', 'tap_deesser', '-30', '6200',
            'pitch', '200',
            'contrast', '75',
            'norm'
        ]
        self.voice = pyvona.create_voice(
            os.environ.get('IVONA_ACCESS_KEY'),
            os.environ.get('IVONA_SECRET_KEY'))
        self.voice.voice_name = rospy.get_param(
            '/needybot/speech/voice/name', 'Justin')
        self.voice.speech_rate = rospy.get_param(
            '/needybot/speech/voice/speech_rate', 'medium')
        self.voice.codec = rospy.get_param('/needybot/speech/voice/codec', 'ogg')
        self.sound_process = None

        self.cleaned_pub = rospy.Publisher(
            '/needybot/speech/cache/cleaned',
            Empty,
            queue_size=10,
            latch=True
        )

        self.warmed_pub = rospy.Publisher(
            '/needybot/speech/cache/warmed',
            Empty,
            queue_size=10,
            latch=True
        )

        if not os.path.exists(self._cache_manifest):
            self.create_cache_manifest()

        if start_server:
            self._server.start()

    def add_to_cache(self, payload):
        """
        Adds a validated payload to the cache manifest.

        :param: payload -- dictionary, see ``generate_cache_payload`` for
                           details.
        """
        if not self.validate_payload(payload):
            raise TypeError('Invalid payload send to speech server')

        with open(self._cache_manifest, 'r+b') as m:
            j = json.load(m)
            if not j.get(payload['key'], None):
                j[payload['key']] = {}
            if not j[payload['key']].get(payload['hash'], None):
                j[payload['key']][payload['hash']] = {}
            j[payload['key']][payload['hash']]['file'] = payload.get('file')
            j[payload['key']][payload['hash']]['template'] = payload.get('template')
            j[payload['key']][payload['hash']]['effects'] = payload.get('effects')
            j[payload['key']][payload['hash']]['params'] = payload.get('params')
            j[payload['key']][payload['hash']]['voice'] = payload.get('voice')
            m.seek(0)
            m.write(json.dumps(j))
            m.truncate()
        return True

    def clean_cache(self):
        """
        This method completely cleans the cache directory of this
        ``SpeechActionServer`` instance. Use with care.
        """
        rospy.loginfo('Purging speech cache...')
        for f in os.listdir(self._cache_dir):
            if f == '.gitkeep':
                continue
            try:
                p = os.path.join(self._cache_dir, f)
                if os.path.isfile(p):
                    os.unlink(p)
            except Exception as e:
                rospy.logerr(e)
        rospy.loginfo('Speech cache has been purged.')
        self.cleaned_pub.publish()

    def clean_string(self, s):
        """
        Helper method to remove excess whitespace within a string.

        :param: s -- str, the string to be cleaned
        :returns: str
        """
        return ' '.join(s.split())

    def create_cache_manifest(self):
        """
        Creates a blank cache manifest with the expected structure.
        {
            'hello_world': {
                '${hash}': {
                    'file': '${abs/path/to/hash.ogg}'
                    'effects': '${joined effects array',
                    'voice': {
                        'name': 'name',
                        'speech_rate': 'speech_rate',
                        'codec': 'codec'
                    },
                    'template': 'template_text',
                    'params': {
                        'name': 'name'
                    }
                }
            }
        }
        """
        with open(self._cache_manifest, 'w') as manifest:
            manifest.seek(0)
            manifest.write(json.dumps({}))
            manifest.truncate()

    def speech_key_to_param(self, key):
        """
        Helper method to construct a proper rosparam dialog string.

        :param: key -- str, the dialog identifier
        :returns: str
        """
        return '/needybot/speech/dialog/{}'.format(self.clean_string(key))

    def execute_cb(self, action):
        """
        Actionlib server callback method. This method is called whenever a new
        action is sent to the server.

        :param: action -- SpeechAction
        """
        success = False

        gen = self.generate_cache_payload(action)
        payload = self.fetch_from_cache(action.key, gen.get('hash'))

        if payload is None:
            payload = gen
            if self.validate_payload(payload):
                self.add_to_cache(payload)
                self.generate_raw_speech(payload)
            else:
                raise TypeError('Invalid payload send to speech server')

        self.play_sound_file(payload['file'])

        while not rospy.is_shutdown():
            if self._server.is_preempt_requested():
                success = False
                self._kill_pub.publish()
                self._server.set_preempted()
                break

            elif self.sound_process.complete:
                success = True
                break

            self._feedback.playing = True
            self._server.publish_feedback(self._feedback)

        if success:
            self._result.success = True
            self._server.set_succeeded(self._result)

    def fetch_from_cache(self, goal_key, hsh):
        """
        Retrieves a dialog entry from the cache manifest.

        :param: goal_key -- str, the dialog identifier
        :param: hsh      -- str, the unique digest for the entry
        :returns: dict
        """
        with open(self._cache_manifest, 'r') as m:
            manifest = json.load(m)
            if not manifest.get(goal_key, None):
                return None
            if not manifest[goal_key].get(hsh, None):
                return None
            return manifest[goal_key][hsh]

    def generate_digest(self, goal_key, parsed_text):
        """
        Creates a unique hash given this server's settings.

        :param: goal_key    -- str, the dialog identifier
        :param: parsed_text -- str, the parsed dialog template
        :returns: str
        """
        digest = hashlib.sha256()
        digest.update(self.clean_string(goal_key))
        digest.update(self.clean_string(parsed_text))
        digest.update(self.clean_string(self.voice.voice_name))
        digest.update(self.clean_string(self.voice.speech_rate))
        digest.update(self.clean_string(self.voice.codec))
        digest.update(self.clean_string(' '.join(self.effects)))
        return digest.hexdigest()

    def generate_cache_payload(self, action):
        """
        Builds a dictionary object to be stored in the cache manifest.

        :param: action -- SpeechAction
        :returns: dict
        """
        entry = rospy.get_param(self.speech_key_to_param(action.key), None)
        params = _.assign(
            entry.get('params', {}),
            self.generate_dict_from_params(action.params))
        digest = self.generate_digest(
            action.key,
            self.parse_template(entry.get('template'), params))
        return {
            'key': action.key,
            'hash': digest,
            'file': os.path.join(
                self._cache_dir, '{}.{}'.format(digest, self.voice.codec)),
            'effects': ' '.join(self.effects),
            'voice': {
                'name': self.voice.voice_name,
                'speech_rate': self.voice.speech_rate,
                'codec': self.voice.codec
            },
            'template': entry.get('template'),
            'params': params
        }

    def generate_dict_from_params(self, params):
        """
        Helper method to turn SpeechParam messages into a standard dictionary.

        :param: params -- [SpeechParam], array of speech params
        :returns: dict
        """
        d = {}
        for p in params:
            d[p.key] = p.value
        return d

    def generate_raw_speech(self, payload):
        """
        Makes an API call to Ivona to generate the raw voice sound file.

        :param: payload -- dict, the dialog entry payload
        """
        self.voice.fetch_voice(
            self.parse_template(payload['template'], payload['params']),
            payload['file'])

    def parse_template(self, template, params):
        """
        Helper method to parse a dialog template given a set of params.

        :param: template -- str, the raw dialog template
        :param: params   -- dict, the dialog params in dictionary format
        :returns: str
        """
        if not params:
            return template

        s = string.Template(template)
        return s.safe_substitute(params)

    def play_sound_file(self, filename):
        """
        Creates a new SoundProcess and plays the given audio file.

        :param: filename -- str, the full path to the desired audio file.
        """
        try:
            self.sound_process = SoundProcess(filename, self.effects)
            self.sound_process.start()
        except OSError as e:
            rospy.logerr('OSError: play failed! %s', e)
        except TypeError as e:
            rospy.logerr('TypeError: %s', e)

    def validate_payload(self, payload):
        """
        Validates the structure of a cache manifest payload.

        ex:
        payload = {
            key: 'goal_key',
            hash: 'hash',
            file: 'filename.codec',
            effects: 'joined effects array',
            voice: { name, speech_rate, codec }
            template: 'template text',
            params: { merged_template_params },
        }

        :param: payload -- dict
        """
        ok = True
        # checking for params key is unnecessary as it is optional
        for k in ['key', 'hash', 'file', 'effects', 'voice', 'template']:
            if not payload.get(k, None):
                ok = False
                break
        if ok:
            # check types
            for k, v in payload.iteritems():
                if k in ['key', 'hash', 'file', 'effects', 'template']:
                    if not type(v) is str:
                        ok = False
                        break

                elif k == 'voice':
                    if not isinstance(v, dict):
                        ok = False
                        break
                    for j in ['name', 'speech_rate', 'codec']:
                        if not v.get(j, None):
                            ok = False
                            break
                    else:
                        continue
                    break

                elif k == 'params':
                    if v is None:
                        continue
                    if not isinstance(v, dict):
                        ok = False
                        break
                    for pk, pv in v.iteritems():
                        if not type(pv) is str:
                            ok = False
                            break
                    else:
                        continue
                    break

        return ok

    def warm_cache(self):
        """
        Collects all dialog in your manifest and stores them in the cache
        directory as well as creating entries in the cache manifest.
        """
        rospy.loginfo('Warming speech cache...')
        m = rospy.get_param('/needybot/speech/dialog')
        for k, v in m.iteritems():
            goal = SpeechGoal(key=k)
            gen = self.generate_cache_payload(goal)
            payload = self.fetch_from_cache(k, gen.get('hash'))
            if payload is None:
                payload = gen
                if self.validate_payload(payload):
                    self.add_to_cache(payload)
                    self.generate_raw_speech(payload)
                else:
                    raise TypeError('Invalid payload send to speech server')
        rospy.loginfo('Speech cache is ready.')
        self.warmed_pub.publish()


if __name__ == '__main__':
    rospy.init_node('needybot_speech')
    action_server = SpeechActionServer()

    clear_cache = rospy.get_param('/needybot/speech/clear_cache', False)
    if clear_cache:
        action_server.clear_cache()

    warm_cache = rospy.get_param('/needybot/speech/warm_cache', True)
    if warm_cache:
        action_server.warm_cache()

    rospy.spin()
