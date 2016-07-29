#!/usr/bin/env python

import json
import os
import re
import rospy
import rostest
import sys
import unittest

from needybot_speech.server import *
from needybot_msgs.msg import *

from pydash import _


class TestNeedybotSpeech(unittest.TestCase):

    def setUp(self):
        self.server = SpeechActionServer()
        self.server.warm_cache()
        self.action = SpeechGoal(
            key='hello_world',
            params=[
                SpeechParam(
                    key='name',
                    value='a needy robot'
                )
            ]
        )

    def tearDown(self):
        self.server.clean_cache()
        self.server = None

    def test_method_add_to_cache(self):
        self.server.warm_cache()
        payload = self.server.generate_cache_payload(self.action)
        self.assertIsNone(
            self.server.fetch_from_cache(payload['key'], payload['hash']),
            'payload has already been cached')
        self.server.add_to_cache(payload)
        self.assertIsNotNone(
            self.server.fetch_from_cache(payload['key'], payload['hash']),
            'payload has not been cached')
        with open(self.server._cache_manifest, 'r') as f:
            m = json.load(f)
            self.assertTrue(payload['key'] in m, 'cache is missing goal key')
            self.assertTrue(
                payload['hash'] in m[payload['key']],
                'cache is missing our digest key')

    def test_method_clean_string(self):
        self.assertEqual(
            'a nice string',
            self.server.clean_string('   a     nice  string   '),
            'string was not cleaned as expected')

    def test_method_create_cache_manifest(self):
        if os.path.isfile(self.server._cache_manifest):
            os.unlink(self.server._cache_manifest)

        self.assertTrue(
            not os.path.isfile(self.server._cache_manifest),
            'cache manifest still exists before method call')

        self.server.create_cache_manifest()

        self.assertTrue(
            os.path.isfile(self.server._cache_manifest),
            'cache manifest was not created.')
        with open(self.server._cache_manifest) as f:
            self.assertEqual(
                json.load(f), {},
                'cache manifest is not an empty dictionary')

    def test_method_speech_key_to_param(self):
        self.assertEqual(
            '/needybot/speech/dialog/my_special_key',
            self.server.speech_key_to_param('my_special_key'),
            'failed to create identical rosparam key')
        self.assertEqual(
            '/needybot/speech/dialog/my_special_key',
            self.server.speech_key_to_param('my_special_key '),
            'failed to create identical rosparam key')

    def test_method_fetch_from_cache(self):
        gen = self.server.generate_cache_payload(self.action)
        self.server.add_to_cache(gen)
        payload = self.server.fetch_from_cache(gen['key'], gen['hash'])
        self.assertIsNotNone(payload, 'unexpected empty payload')
        payload = self.server.fetch_from_cache(
            'nonexistent key', 'nonexistent hash')
        self.assertIsNone(payload, 'got valid payload from invalid keys')

    def test_method_generate_digest(self):
        key = 'hello_world'
        entry = rospy.get_param(self.server.speech_key_to_param(key))
        parsed = self.server.parse_template(entry['template'], entry['params'])
        digest_a = self.server.generate_digest(key, parsed)

        self.assertIsNotNone(digest_a)
        self.assertIsInstance(digest_a, str)

        parsed = self.server.parse_template(
            entry['template'],
            _.assign(entry['params'], {'name': 'Needy'}))
        digest_b = self.server.generate_digest(key, parsed)

        self.assertIsNotNone(digest_b, 'is none')
        self.assertIsInstance(digest_b, str, 'is not a string')
        self.assertNotEqual(digest_a, digest_b,
                            'equal digests with diff params')

    def test_method_generate_cache_payload(self):
        digest = self.server.generate_digest(
            'hello_world',
            'Hello, world! My name is a needy robot.')
        expected = {
            'key': 'hello_world',
            'hash': digest,
            'file': os.path.join(
                self.server._cache_dir,
                '{}.{}'.format(digest, self.server.voice.codec)),
            'effects': ' '.join(self.server.effects),
            'voice': {
                'name': self.server.voice.voice_name,
                'speech_rate': self.server.voice.speech_rate,
                'codec': self.server.voice.codec
            },
            'template': rospy.get_param(
                self.server.speech_key_to_param('hello_world'))['template'],
            'params': {
                'name': 'a needy robot'
            }
        }
        payload = self.server.generate_cache_payload(self.action)
        self.assertIsNotNone(payload, 'no payload created')
        self.assertEqual(payload, expected, 'generated payload is incorrect')

    def test_method_generate_dict_from_params(self):
        params = [
            SpeechParam(key='a', value='one'),
            SpeechParam(key='b', value='two')
        ]
        expected = {
            'a': 'one',
            'b': 'two'
        }
        self.assertEqual(
            self.server.generate_dict_from_params(params), expected,
            'dictionaries are not equal')

    def test_method_generate_raw_speech(self):
        payload = self.server.generate_cache_payload(self.action)
        self.server.clean_cache()
        self.assertTrue(len(os.listdir(self.server._cache_dir)) == 1)
        self.server.generate_raw_speech(payload)
        self.assertTrue(len(os.listdir(self.server._cache_dir)) == 2)
        self.assertTrue(
            os.path.isfile(payload['file']),
            'sound file was not created')

    def test_method_parse_template(self):
        template = 'foo ${bar}'
        params = {'bar': 'baz'}
        self.assertEqual(
            self.server.parse_template(template, params),
            'foo baz',
            'template was parsed incorrectly')
        self.assertEqual(
            self.server.parse_template(template, None),
            template,
            'template wanted params was given none')

    def test_method_validate_payload_with_params(self):
        payload = {
            'key': 'goal_key',
            'hash': 'areallylonghash',
            'file': 'soundfile.ogg',
            'effects': 'joined effects array',
            'voice': {
                'name': 'Justin',
                'speech_rate': 'medium',
                'codec': 'ogg'
            },
            'template': 'a ${template}',
            'params': {
                'template': 'value'
            }
        }
        self.assertTrue(self.server.validate_payload(payload))

    def test_method_validate_payload_without_params(self):
        payload = {
            'key': 'goal_key',
            'hash': 'areallylonghash',
            'file': 'soundfile.ogg',
            'effects': 'joined effects array',
            'voice': {
                'name': 'Justin',
                'speech_rate': 'medium',
                'codec': 'ogg'
            },
            'template': 'a ${template}'
        }
        self.assertTrue(self.server.validate_payload(payload))

    def test_method_clean_cache(self):
        # manually remove all files from cache directory
        for f in os.listdir(self.server._cache_dir):
            if f == '.gitkeep':
                continue
            try:
                p = os.path.join(self.server._cache_dir, f)
                os.unlink(p)
            except Exception as e:
                self.fail(e)

        remaining = os.listdir(self.server._cache_dir)
        self.assertEqual(
            len(remaining), 1, 'cache directory was not prepped properly')
        self.assertTrue(
            '.gitkeep' in remaining,
            '.gitkeep file was removed (or missing) in cache directory')

        self.server.create_cache_manifest()
        self.server.warm_cache()
        self.assertTrue(
            os.path.isfile(self.server._cache_manifest),
            'cache manifest does not exist')
        cached = os.listdir(self.server._cache_dir)
        ok = False
        for f in cached:
            if re.search(r'.ogg$', f):
                ok = True
                break
        if not ok:
            self.fail('no .ogg files present in cache directory')

        self.server.clean_cache()
        remaining = os.listdir(self.server._cache_dir)
        self.assertEqual(
            len(remaining), 1, 'cache was not cleaned properly')
        self.assertTrue(
            '.gitkeep' in remaining,
            '.gitkeep file was removed (or missing) in cache directory')

    def test_method_warm_cache(self):
        # clean cache
        self.server.clean_cache()
        remaining = os.listdir(self.server._cache_dir)
        self.assertEqual(
            len(remaining), 1, 'cache was not cleaned properly')
        self.assertTrue(
            '.gitkeep' in remaining,
            '.gitkeep file was removed (or missing) in cache directory')

        # seed
        self.server.create_cache_manifest()
        self.server.warm_cache()
        self.assertTrue(
            os.path.isfile(self.server._cache_manifest),
            'cache manifest does not exist')
        cached = os.listdir(self.server._cache_dir)
        ok = False
        for f in cached:
            if re.search(r'.ogg$', f):
                ok = True
                break
        if not ok:
            self.fail('no .ogg files present in cache directory')


if __name__ == '__main__':
    rospy.init_node('test_needybot_speech')
    rostest.rosrun(
        'needybot_speech',
        'test_needybot_speech',
        TestNeedybotSpeech,
        sys.argv)
