#!/usr/bin/env python

import rospy
import sys

from server import SpeechActionServer

if __name__ == '__main__':
    speech_manifest = sys.argv[1]
    cache_dir = sys.argv[2]

    rospy.init_node('needybot_speech')
    action_server = SpeechActionServer()
    action_server.warm_cache()
    rospy.signal_shutdown('done warming.')
