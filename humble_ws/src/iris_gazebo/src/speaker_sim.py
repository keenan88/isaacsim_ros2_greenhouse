#!/usr/bin/env python3
import rospy
import rospkg
from std_srvs.srv import SetBool
from pygame import mixer


class Speaker(object):

    _turned_on = False

    def __init__(self, name):
        rospy.init_node(name, anonymous=True)

        # simulation speaker service
        self._speaker_server = rospy.Service(
            '/iris/turn_on_alarm_speaker', SetBool, self._handle_turn_on_alarm_speaker)

        filename = rospkg.RosPack().get_path("iris_gazebo") + "/sounds/long_beep.mp3"

        # init sound playing
        mixer.init()
        mixer.music.set_volume(0.3)
        mixer.music.load(filename)

        rospy.loginfo(f'{name} node started')

    def _handle_turn_on_alarm_speaker(self, req):
        if req.data:
            mixer.music.play(loops=-1)
        else:
            mixer.music.stop()
        return True, ''


def main():
    try:
        speaker = Speaker('speaker')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
