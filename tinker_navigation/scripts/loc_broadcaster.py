# !/usr/bin/env python
import rospy

from tf_conversions import transformations
from math import pi
import tf


class Robot:

    def __init__(self):
        self.tf_listener = tf.TransformListener()  # 创建了监听器，它通过线路接收tf转换，并将其缓冲10s。若用C++写，需设置等待10s缓冲。
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(),rospy.Duration(1.0)) # 猜测：等待Duration=1s，判断map是否转换为base_link
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
        # try ... except ... 进行异常处理，若 try... 出现异常，则将错误直接输出打印，而不是以报错的形式显示。

    def get_pos(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        # rospy_Time(0)指最近时刻存储的数据
        # 得到从 '/map' 到 '/base_link' 的变换，在实际使用时，转换得出的坐标是在 '/base_link' 坐标系下的。
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
        euler = transformations.euler_from_quaternion(rot)  # 将四元数转换为欧拉角
        # print euler[2] / pi * 180

        x = trans[0]
        y = trans[1]
        th = euler[2] / pi * 180
        return (x, y, th)


if __name__ == "__main__":
    rospy.init_node('get_pos_demo', anonymous=True)
    # 启动节点get_pos_demo, 同时为节点命名， 若anonymous为真则节点会自动补充名字，实际名字以get_pos_demo_12345等表示
    # 若为假，则系统不会补充名字，采用用户命名。如果有重名，则最新启动的节点会注销掉之前的同名节点。
    robot = Robot()
    r = rospy.Rate(100)  # 设置速率，每秒发100次
    r.sleep()
    
    while not rospy.is_shutdown():  # 如果节点已经关闭则is_shutdown()函数返回一个True，反之返回False
        print(robot.get_pos())  # 输出坐标。
        r.sleep()