

import rospy

from dynamic_reconfigure.server import Server
from interactive_ai4hri.cfg import audiotranscriber_ai4hriConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {language_str_param} """.format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("interactive_ai4hri", anonymous = False)

    srv = Server(audiotranscriber_ai4hriConfig, callback)
    rospy.spin()
