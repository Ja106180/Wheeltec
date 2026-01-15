#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import subprocess
import os
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus


class WheeltecControlNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('wheeltec_control_node', anonymous=True)

        # 加载配置参数
        self.load_parameters()

        # 初始化串口
        self.serial_conn = None
        self.init_serial()

        # 控制状态
        self.navigation_active = False
        self.current_goal = None

        # 运动状态跟踪
        self.motion_state = "STOP"  # STOP, FORWARD, BACKWARD
        self.control_rate = 10  # 控制频率 Hz
        self.control_timer = None

        # 发布器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # 订阅器
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.navigation_result_callback)

        # 启动控制循环
        self.start_control_loop()

        # 启动串口接收线程
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        rospy.loginfo("=== Wheeltec控制节点已启动 ===")
        rospy.loginfo("串口端口: %s, 波特率: %d", self.serial_port, self.serial_baudrate)
        rospy.loginfo("等待指令...")

    def load_parameters(self):
        """加载配置参数"""
        # 串口配置
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.serial_baudrate = rospy.get_param('~serial_baudrate', 115200)

        # 指令字符串
        self.cmd_forward = rospy.get_param('~cmd_forward', 'FA111FB')
        self.cmd_stop = rospy.get_param('~cmd_stop', 'FA222FB')
        self.cmd_backward = rospy.get_param('~cmd_backward', 'FA333FB')
        self.cmd_nav_start = rospy.get_param('~cmd_nav_start', 'FA444FB')
        self.cmd_nav_a = rospy.get_param('~cmd_nav_a', 'FA555FB')
        self.cmd_nav_b = rospy.get_param('~cmd_nav_b', 'FA666FB')
        self.feedback_a = rospy.get_param('~feedback_a', 'FA000FB')
        self.feedback_b = rospy.get_param('~feedback_b', 'FA999FB')

        # 导航点坐标 - 修改为完整的四元数参数
        # A点
        self.point_a_x = rospy.get_param('~point_a_x', 0.217826616715)
        self.point_a_y = rospy.get_param('~point_a_y', -6.60869242058)
        self.point_a_z = rospy.get_param('~point_a_z', 0.0)
        self.point_a_ox = rospy.get_param('~point_a_ox', 0.0)
        self.point_a_oy = rospy.get_param('~point_a_oy', 0.0)
        self.point_a_oz = rospy.get_param('~point_a_oz', -0.732203432831)
        self.point_a_ow = rospy.get_param('~point_a_ow', 0.681085995268)

        # B点
        self.point_b_x = rospy.get_param('~point_b_x', -7.15665620556)
        self.point_b_y = rospy.get_param('~point_b_y', -1.64871493298)
        self.point_b_z = rospy.get_param('~point_b_z', 0.0)
        self.point_b_ox = rospy.get_param('~point_b_ox', 0.0)
        self.point_b_oy = rospy.get_param('~point_b_oy', 0.0)
        self.point_b_oz = rospy.get_param('~point_b_oz', -0.00956196270291)
        self.point_b_ow = rospy.get_param('~point_b_ow', 0.99995428339)

        # A点路径的第一个中间点
        self.point_a1_x = rospy.get_param('~point_a1_x', -0.825736924438)
        self.point_a1_y = rospy.get_param('~point_a1_y', -0.812425076261)
        self.point_a1_z = rospy.get_param('~point_a1_z', 0.0)
        self.point_a1_ox = rospy.get_param('~point_a1_ox', 0.0)
        self.point_a1_oy = rospy.get_param('~point_a1_oy', 0.0)
        self.point_a1_oz = rospy.get_param('~point_a1_oz', -0.165205868668)
        self.point_a1_ow = rospy.get_param('~point_a1_ow', 0.986259104373)

        # A点路径的第二个中间点
        self.point_a2_x = rospy.get_param('~point_a2_x', 0.498849057537)
        self.point_a2_y = rospy.get_param('~point_a2_y', -3.65134776464)
        self.point_a2_z = rospy.get_param('~point_a2_z', 0.0)
        self.point_a2_ox = rospy.get_param('~point_a2_ox', 0.0)
        self.point_a2_oy = rospy.get_param('~point_a2_oy', 0.0)
        self.point_a2_oz = rospy.get_param('~point_a2_oz', -0.652618938876)
        self.point_a2_ow = rospy.get_param('~point_a2_ow', 0.757686294333)

        # B点路径的第一个中间点
        self.point_b1_x = rospy.get_param('~point_b1_x', 0.115560920075)
        self.point_b1_y = rospy.get_param('~point_b1_y', -2.38158436022)
        self.point_b1_z = rospy.get_param('~point_b1_z', 0.0)
        self.point_b1_ox = rospy.get_param('~point_b1_ox', 0.0)
        self.point_b1_oy = rospy.get_param('~point_b1_oy', 0.0)
        self.point_b1_oz = rospy.get_param('~point_b1_oz', -0.451618734473)
        self.point_b1_ow = rospy.get_param('~point_b1_ow', 0.892211028105)

        # B点路径的第二个中间点
        self.point_b2_x = rospy.get_param('~point_b2_x', -1.00069282335)
        self.point_b2_y = rospy.get_param('~point_b2_y', -0.893619670263)
        self.point_b2_z = rospy.get_param('~point_b2_z', 0.0)
        self.point_b2_ox = rospy.get_param('~point_b2_ox', 0.0)
        self.point_b2_oy = rospy.get_param('~point_b2_oy', 0.0)
        self.point_b2_oz = rospy.get_param('~point_b2_oz', -0.230449654844)
        self.point_b2_ow = rospy.get_param('~point_b2_ow', 0.973084249478)

        rospy.loginfo("参数加载完成")

    def init_serial(self):
        """初始化串口连接"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                bytesize=serial.EIGHTBITS,  # 8位数据位
                parity=serial.PARITY_NONE,  # 无校验位
                stopbits=serial.STOPBITS_ONE,  # 1位停止位
                timeout=1
            )
            rospy.loginfo("串口连接成功: %s", self.serial_port)
        except Exception as e:
            rospy.logerr("串口连接失败: %s", str(e))
            self.serial_conn = None

    def start_control_loop(self):
        """启动控制循环，持续发布速度指令"""
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)

    def control_loop(self, event):
        """控制循环，根据当前状态持续发布速度"""
        # 如果正在导航，不发布任何速度指令
        if self.navigation_active and self.current_goal is not None:
            return

        if self.motion_state == "FORWARD":
            self.publish_continuous_velocity(0.2, 0.0)  # 前进速度
        elif self.motion_state == "BACKWARD":
            self.publish_continuous_velocity(-0.2, 0.0)  # 后退速度
        elif self.motion_state == "STOP":
            self.publish_continuous_velocity(0.0, 0.0)  # 停止

    def publish_continuous_velocity(self, linear_x, angular_z):
        """发布持续速度指令"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def receive_data(self):
        """持续接收串口数据"""
        buffer = b""  # 使用字节缓冲区
        while not rospy.is_shutdown():
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    # 读取原始字节数据
                    raw_data = self.serial_conn.read(self.serial_conn.in_waiting or 1)
                    if raw_data:
                        buffer += raw_data
                        rospy.loginfo("收到原始数据: %s", raw_data)  # 打印原始字节

                        # 在字节级别查找指令
                        while b'FA' in buffer and b'FB' in buffer:
                            start_idx = buffer.find(b'FA')
                            end_idx = buffer.find(b'FB', start_idx)

                            if end_idx != -1:
                                # 提取完整指令字节
                                command_bytes = buffer[start_idx:end_idx + 2]

                                # 直接比较字节，不进行解码
                                if command_bytes == b'FA111FB':
                                    rospy.loginfo("收到指令: 前进")
                                    self.move_forward()
                                elif command_bytes == b'FA222FB':
                                    rospy.loginfo("收到指令: 停止")
                                    self.stop()
                                elif command_bytes == b'FA333FB':
                                    rospy.loginfo("收到指令: 后退")
                                    self.move_backward()
                                elif command_bytes == b'FA444FB':
                                    rospy.loginfo("收到指令: 启动导航模式")
                                    self.start_navigation_mode()
                                elif command_bytes == b'FA555FB':
                                    rospy.loginfo("收到指令: 导航到A点")
                                    self.navigate_to_point_a()
                                elif command_bytes == b'FA666FB':
                                    rospy.loginfo("收到指令: 导航到B点")
                                    self.navigate_to_point_b()
                                # 新增：功能控制指令
                                elif command_bytes == b'FA888FB':
                                    rospy.loginfo("收到指令: 开启KCF跟踪")
                                    self.start_kcf_tracking()
                                elif command_bytes == b'FA112FB':
                                    rospy.loginfo("收到指令: 开启ROSAPP控制")
                                    self.start_rosapp_control()
                                elif command_bytes == b'FA113FB':
                                    rospy.loginfo("收到指令: 开启人体骨架识别与跟踪")
                                    self.start_body_follow()
                                elif command_bytes == b'FA114FB':
                                    rospy.loginfo("收到指令: 开启视觉跟踪")
                                    self.start_visual_follower()
                                elif command_bytes == b'FA115FB':
                                    rospy.loginfo("收到指令: 开启视觉巡线")
                                    self.start_line_follower()
                                else:
                                    rospy.logwarn("未知指令: %s", command_bytes)

                                # 移除已处理的指令
                                buffer = buffer[end_idx + 2:]
                            else:
                                break  # 没有找到完整的指令，等待更多数据
                except Exception as e:
                    rospy.logwarn("串口读取错误: %s", str(e))
                    self.init_serial()  # 尝试重新连接
            else:
                # 如果串口未连接，尝试重新连接
                self.init_serial()
                rospy.sleep(1)

    def process_command(self, command):
        """处理接收到的指令"""
        command = command.strip()

        if command == self.cmd_forward:
            self.move_forward()
        elif command == self.cmd_stop:
            self.stop()
        elif command == self.cmd_backward:
            self.move_backward()
        elif command == self.cmd_nav_start:
            self.start_navigation_mode()
        elif command == self.cmd_nav_a:
            self.navigate_to_point_a()
        elif command == self.cmd_nav_b:
            self.navigate_to_point_b()
        else:
            rospy.logwarn("未知指令: %s", command)

    def move_forward(self):
        """设置前进状态"""
        self.motion_state = "FORWARD"
        rospy.loginfo("设置为持续前进")

    def stop(self):
        """设置停止状态"""
        self.motion_state = "STOP"
        rospy.loginfo("停止运动")

    def move_backward(self):
        """设置后退状态"""
        self.motion_state = "BACKWARD"
        rospy.loginfo("设置为持续后退")

    def start_navigation_mode(self):
        """启动3D导航模式并打开RViz"""
        rospy.loginfo("启动3D导航模式")

        # 停止当前运动
        self.stop()

        try:
            # 使用gnome-terminal在新终端中启动3D导航
            subprocess.Popen([
                'gnome-terminal', '--',
                'roslaunch', 'turn_on_wheeltec_robot', '3d_navigation.launch'
            ])
            rospy.loginfo("3D导航已启动")

            # 等待导航启动
            rospy.sleep(25)

            # 使用gnome-terminal在新终端中启动RViz
            subprocess.Popen([
                'gnome-terminal', '--', 'rviz'
            ])
            rospy.loginfo("RViz已启动")

            self.navigation_active = True
            rospy.loginfo("3D导航模式就绪")

        except Exception as e:
            rospy.logerr("启动导航失败: %s", str(e))

    def start_kcf_tracking(self):
        """开启KCF跟踪"""
        rospy.loginfo("开启KCF跟踪")
        try:
            # 使用gnome-terminal在新终端中启动KCF跟踪
            subprocess.Popen([
                'gnome-terminal', '--',
                'roslaunch', 'kcf_track', 'kcf_tracker.launch'
            ])
            rospy.loginfo("KCF跟踪已启动")
        except Exception as e:
            rospy.logerr("启动KCF跟踪失败: %s", str(e))

    def start_rosapp_control(self):
        """开启ROSAPP控制"""
        rospy.loginfo("开启ROSAPP控制")
        try:
            # 使用gnome-terminal在新终端中启动USB摄像头
            subprocess.Popen([
                'gnome-terminal', '--',
                'roslaunch', 'usb_cam', 'usb_cam-test.launch'
            ])
            rospy.loginfo("USB摄像头已启动")

            # 使用gnome-terminal在新终端中启动导航
            subprocess.Popen([
                'gnome-terminal', '--',
                'roslaunch', 'turn_on_wheeltec_robot', 'navigation.launch'
            ])
            rospy.loginfo("导航已启动")

            rospy.loginfo("ROSAPP控制已启动")
        except Exception as e:
            rospy.logerr("启动ROSAPP控制失败: %s", str(e))

    def start_body_follow(self):
        """开启人体骨架识别与跟踪"""
        rospy.loginfo("开启人体骨架识别与跟踪")
        try:
            # 使用gnome-terminal在新终端中启动人体跟随
            subprocess.Popen([
                'gnome-terminal', '--',
                'roslaunch', 'bodyreader', 'bodyfollow.launch'
            ])
            rospy.loginfo("人体骨架识别已启动")

            # 使用gnome-terminal在新终端中启动图像查看器
            subprocess.Popen([
                'gnome-terminal', '--', 'rqt_image_view'
            ])
            rospy.loginfo("rqt_image_view已启动")

            rospy.loginfo("人体骨架识别与跟踪已启动")
        except Exception as e:
            rospy.logerr("启动人体骨架识别与跟踪失败: %s", str(e))

    def start_visual_follower(self):
        """开启视觉跟踪"""
        rospy.loginfo("开启视觉跟踪")
        try:
            # 使用gnome-terminal在新终端中启动视觉跟随
            subprocess.Popen([
                'gnome-terminal', '--',
                'roslaunch', 'simple_follower', 'visual_follower.launch'
            ])
            rospy.loginfo("视觉跟踪已启动")

            # 使用gnome-terminal在新终端中启动rqt
            subprocess.Popen([
                'gnome-terminal', '--', 'rqt'
            ])
            rospy.loginfo("rqt已启动")

            rospy.loginfo("视觉跟踪已启动")
        except Exception as e:
            rospy.logerr("启动视觉跟踪失败: %s", str(e))

    def start_line_follower(self):
        """开启视觉巡线"""
        rospy.loginfo("开启视觉巡线")
        try:
            # 使用gnome-terminal在新终端中启动视觉巡线
            subprocess.Popen([
                'gnome-terminal', '--',
                'roslaunch', 'simple_follower', 'line_follower.launch'
            ])
            rospy.loginfo("视觉巡线已启动")
        except Exception as e:
            rospy.logerr("启动视觉巡线失败: %s", str(e))

    def navigate_to_point_a(self):
        """导航到A点（三阶段：中间点1 → 中间点2 → 最终点）"""
        if not self.navigation_active:
            rospy.logwarn("导航模式未启动，请先发送 %s 启动导航", self.cmd_nav_start)
            return

        # 第一阶段：发送第一个中间点
        self.send_goal(self.point_a1_x, self.point_a1_y, self.point_a1_z,
                       self.point_a1_ox, self.point_a1_oy, self.point_a1_oz, self.point_a1_ow,
                       "A点-中间点1")
        self.current_goal = "A"
        self.nav_phase = 1  # 第一阶段
        rospy.loginfo("开始第一阶段导航：前往A点中间点1")

    def navigate_to_point_b(self):
        """导航到B点（三阶段：中间点1 → 中间点2 → 最终点）"""
        if not self.navigation_active:
            rospy.logwarn("导航模式未启动，请先发送 %s 启动导航", self.cmd_nav_start)
            return

        # 第一阶段：发送第一个中间点
        self.send_goal(self.point_b1_x, self.point_b1_y, self.point_b1_z,
                       self.point_b1_ox, self.point_b1_oy, self.point_b1_oz, self.point_b1_ow,
                       "B点-中间点1")
        self.current_goal = "B"
        self.nav_phase = 1  # 第一阶段
        rospy.loginfo("开始第一阶段导航：前往B点中间点1")

    def send_goal(self, x, y, z, ox, oy, oz, ow, point_name):
        """发送目标点 - 修改为使用完整的四元数"""
        goal = PoseStamped()

        # 设置header
        goal.header.seq = 0
        goal.header.stamp = rospy.Time.now()  # 使用当前时间
        goal.header.frame_id = "map"

        # 设置位置
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z

        # 设置朝向（完整四元数）
        goal.pose.orientation.x = ox
        goal.pose.orientation.y = oy
        goal.pose.orientation.z = oz
        goal.pose.orientation.w = ow

        self.goal_pub.publish(goal)
        rospy.loginfo("已发送目标点: %s", point_name)

    def navigation_result_callback(self, msg):
        """处理导航结果"""
        if msg.status.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("成功到达目标点")

            if self.current_goal == "A":
                if self.nav_phase == 1:
                    # 第一阶段完成：到达中间点1，开始第二阶段
                    rospy.loginfo("已到达A点中间点1，开始前往中间点2")
                    rospy.sleep(1)
                    self.send_goal(self.point_a2_x, self.point_a2_y, self.point_a2_z,
                                   self.point_a2_ox, self.point_a2_oy, self.point_a2_oz, self.point_a2_ow,
                                   "A点-中间点2")
                    self.nav_phase = 2  # 进入第二阶段

                elif self.nav_phase == 2:
                    # 第二阶段完成：到达中间点2，开始第三阶段
                    rospy.loginfo("已到达A点中间点2，开始前往最终点")
                    rospy.sleep(1)
                    self.send_goal(self.point_a_x, self.point_a_y, self.point_a_z,
                                   self.point_a_ox, self.point_a_oy, self.point_a_oz, self.point_a_ow,
                                   "A点-最终点")
                    self.nav_phase = 3  # 进入第三阶段

                elif self.nav_phase == 3:
                    # 第三阶段完成：到达最终点
                    rospy.loginfo("已到达A点最终点，导航完成！")
                    self.send_serial_data(self.feedback_a)
                    rospy.loginfo("已到达A点，发送反馈: %s", self.feedback_a)
                    self.current_goal = None
                    self.nav_phase = 0

            elif self.current_goal == "B":
                if self.nav_phase == 1:
                    # 第一阶段完成：到达中间点1，开始第二阶段
                    rospy.loginfo("已到达B点中间点1，开始前往中间点2")
                    rospy.sleep(1)
                    self.send_goal(self.point_b2_x, self.point_b2_y, self.point_b2_z,
                                   self.point_b2_ox, self.point_b2_oy, self.point_b2_oz, self.point_b2_ow,
                                   "B点-中间点2")
                    self.nav_phase = 2  # 进入第二阶段

                elif self.nav_phase == 2:
                    # 第二阶段完成：到达中间点2，开始第三阶段
                    rospy.loginfo("已到达B点中间点2，开始前往最终点")
                    rospy.sleep(1)
                    self.send_goal(self.point_b_x, self.point_b_y, self.point_b_z,
                                   self.point_b_ox, self.point_b_oy, self.point_b_oz, self.point_b_ow,
                                   "B点-最终点")
                    self.nav_phase = 3  # 进入第三阶段

                elif self.nav_phase == 3:
                    # 第三阶段完成：到达最终点
                    rospy.loginfo("已到达B点最终点，导航完成！")
                    self.send_serial_data(self.feedback_b)
                    rospy.loginfo("已到达B点，发送反馈: %s", self.feedback_b)
                    self.current_goal = None
                    self.nav_phase = 0

        elif msg.status.status == GoalStatus.ABORTED:
            rospy.logwarn("导航失败")
        elif msg.status.status == GoalStatus.PREEMPTED:
            rospy.loginfo("导航被取消")

    def send_serial_data(self, data):
        """通过串口发送数据"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(data.encode('utf-8'))
                rospy.loginfo("发送串口数据: %s", data)
            except Exception as e:
                rospy.logerr("串口发送失败: %s", str(e))

    def run(self):
        """运行节点"""
        rospy.spin()

    def __del__(self):
        """析构函数，关闭串口"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()


if __name__ == '__main__':
    try:
        node = WheeltecControlNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Wheeltec控制节点已关闭")
