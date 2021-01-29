#!/usr/bin/env python
# -*- coding: utf-8 -*-


# https://answers.ros.org/question/276176/print-message-from-launch-file/

import rospy

def main():
    rospy.init_node('print_to_screen', anonymous=True)
    rospy.myargv(argv=sys.argv)
    msg = rospy.get_param("/msg")
    print(msg)
    rospy.spin()

if __name__ == '__main__':
    main()
