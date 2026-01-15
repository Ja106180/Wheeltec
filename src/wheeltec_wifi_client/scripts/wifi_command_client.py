#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
WiFi命令客户端ROS节点
功能：向其他小车发送命令，支持串口接收指令并通过WiFi转发
"""

import rospy
import socket
import json
import threading
import signal
import sys
import serial
from std_msgs.msg import String

class WiFiCommandClientNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("wifi_command_client", anonymous=True)
        
        # 获取WiFi参数
        self.target_host = rospy.get_param("~target_host", "192.168.31.27")  # 目标小车IP地址
        self.target_port = rospy.get_param("~target_port", 8888)  # 目标小车端口
        self.auto_reconnect = rospy.get_param("~auto_reconnect", True)  # 自动重连
        self.reconnect_interval = rospy.get_param("~reconnect_interval", 5.0)  # 重连间隔（秒）
        
        # 获取串口参数
        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB0")  # 串口设备
        self.serial_baudrate = rospy.get_param("~serial_baudrate", 115200)  # 串口波特率
        self.enable_serial = rospy.get_param("~enable_serial", True)  # 是否启用串口
        
        # Socket连接
        self.socket = None
        self.connected = False
        self.connection_lock = threading.Lock()
        self.shutdown_flag = False
        
        # 串口连接
        self.serial_conn = None
        
        # 串口指令到WiFi命令的映射（Python 2: 使用unicode字符串）
        self.serial_to_wifi_map = {
            b'FA116FB': u"前进",
            b'FA117FB': u"后退",
            b'FA118FB': u"停止",
            b'FA119FB': u"前往目标点一",
            b'FA120FB': u"前往目标点二",
            b'FA121FB': u"开启导航模式",
            b'FA122FB': u"获取方位",
            b'FA123FB': u"小车视觉跟随",
            b'FA124FB': u"小车雷达跟随",
            b'FA125FB': u"开启巡逻模式",
            b'FA126FB': u"开启表演模式"
        }
        
        # ROS发布者 - 发布服务器响应
        self.response_pub = rospy.Publisher("wifi_client_response", String, queue_size=10)
        self.status_pub = rospy.Publisher("wifi_client_status", String, queue_size=10)
        self.serial_command_pub = rospy.Publisher("serial_command_received", String, queue_size=10)
        
        # ROS订阅者 - 订阅命令
        rospy.Subscriber("wifi_client_command", String, self.command_callback)
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 启动WiFi连接（非阻塞）
        self.connect()
        
        # 初始化串口
        if self.enable_serial:
            self.init_serial()
            # 启动串口接收线程
            if self.serial_conn:
                serial_thread = threading.Thread(target=self.receive_serial_data)
                serial_thread.daemon = True
                serial_thread.start()
                rospy.loginfo("串口接收线程已启动")
        
        # 发布状态
        self.publish_status("初始化完成")
        
        rospy.loginfo("WiFi命令客户端节点已启动")
        rospy.loginfo("目标地址: %s:%d", self.target_host, self.target_port)
        if self.enable_serial:
            rospy.loginfo("串口: %s (波特率: %d)", self.serial_port, self.serial_baudrate)
        rospy.loginfo("等待命令...")
        
    def signal_handler(self, signum, frame):
        """处理Ctrl+C信号"""
        rospy.loginfo("收到退出信号，正在关闭...")
        self.shutdown_flag = True
        self.disconnect()
        self.disconnect_serial()
        sys.exit(0)
        
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
            self.publish_status("串口已连接")
        except Exception as e:
            rospy.logerr("串口连接失败: %s", str(e))
            self.serial_conn = None
            self.publish_status("串口连接失败: %s" % str(e))
    
    def disconnect_serial(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
                rospy.loginfo("串口已断开")
            except:
                pass
            self.serial_conn = None
    
    def receive_serial_data(self):
        """持续接收串口数据"""
        buffer = b""  # 使用字节缓冲区
        while not rospy.is_shutdown() and not self.shutdown_flag:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    # 读取原始字节数据
                    raw_data = self.serial_conn.read(self.serial_conn.in_waiting or 1)
                    if raw_data:
                        buffer += raw_data
                        rospy.loginfo("收到串口原始数据: %s", raw_data)  # 打印原始字节

                        # 在字节级别查找指令
                        while b'FA' in buffer and b'FB' in buffer:
                            start_idx = buffer.find(b'FA')
                            end_idx = buffer.find(b'FB', start_idx)

                            if end_idx != -1:
                                # 提取完整指令字节
                                command_bytes = buffer[start_idx:end_idx + 2]

                                # 查找映射的WiFi命令
                                if command_bytes in self.serial_to_wifi_map:
                                    wifi_command = self.serial_to_wifi_map[command_bytes]
                                    # Python 2兼容：确保日志输出时unicode字符串被正确编码
                                    if isinstance(wifi_command, unicode):
                                        wifi_command_str = wifi_command.encode('utf-8')
                                    else:
                                        wifi_command_str = wifi_command
                                    rospy.loginfo("收到串口指令: %s -> WiFi命令: %s", command_bytes, wifi_command_str)
                                    
                                    # 发布串口指令接收消息
                                    serial_msg = String()
                                    serial_msg.data = "串口指令: %s" % command_bytes
                                    self.serial_command_pub.publish(serial_msg)
                                    
                                    # 通过WiFi发送命令
                                    result = self.send_command(wifi_command)
                                    if result:
                                        rospy.loginfo("串口指令已通过WiFi发送成功")
                                    else:
                                        rospy.logwarn("串口指令WiFi发送失败")
                                else:
                                    rospy.logwarn("未知串口指令: %s", command_bytes)

                                # 移除已处理的指令
                                buffer = buffer[end_idx + 2:]
                            else:
                                break  # 没有找到完整的指令，等待更多数据
                except Exception as e:
                    rospy.logwarn("串口读取错误: %s", str(e))
                    self.init_serial()  # 尝试重新连接
                    rospy.sleep(1)
            else:
                # 如果串口未连接，尝试重新连接
                if self.enable_serial:
                    self.init_serial()
                rospy.sleep(1)
        
    def connect(self):
        """连接到目标小车"""
        with self.connection_lock:
            if self.connected and self.socket:
                return True
                
            try:
                rospy.loginfo("正在连接到 %s:%d...", self.target_host, self.target_port)
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(3.0)  # 设置连接超时为3秒
                self.socket.connect((self.target_host, self.target_port))
                self.connected = True
                rospy.loginfo("已连接到目标小车: %s:%d", self.target_host, self.target_port)
                self.publish_status("已连接")
                return True
            except socket.timeout:
                rospy.logwarn("连接超时: 无法连接到 %s:%d", self.target_host, self.target_port)
                self.connected = False
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None
                self.publish_status("连接超时")
                return False
            except socket.error as e:
                rospy.logwarn("连接失败: %s (目标: %s:%d)", str(e), self.target_host, self.target_port)
                self.connected = False
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None
                self.publish_status("连接失败: %s" % str(e))
                return False
            except Exception as e:
                rospy.logerr("连接异常: %s", str(e))
                self.connected = False
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None
                self.publish_status("连接异常: %s" % str(e))
                return False
    
    def disconnect(self):
        """断开连接"""
        with self.connection_lock:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                self.socket = None
            self.connected = False
            rospy.loginfo("已断开连接")
            self.publish_status("已断开")
    
    def send_command(self, command):
        """
        发送命令到目标小车
        :param command: 命令字符串
        :return: 服务器响应（字典格式）或None
        """
        # 检查连接
        if not self.connected or not self.socket:
            if self.auto_reconnect:
                rospy.loginfo("尝试重新连接...")
                if not self.connect():
                    return None
            else:
                rospy.logwarn("未连接到目标小车")
                return None
        
        try:
            # 发送命令（JSON格式）
            # Python 2兼容：确保command是unicode字符串
            if isinstance(command, str):
                # Python 2: str是字节串，需要解码为unicode
                command_unicode = command.decode('utf-8')
            elif isinstance(command, unicode):
                command_unicode = command
            else:
                command_unicode = unicode(command, 'utf-8')
            
            message_dict = {"command": command_unicode}
            message_json = json.dumps(message_dict, ensure_ascii=False)
            # Python 2: json.dumps返回unicode字符串，需要编码为字节串
            if isinstance(message_json, unicode):
                message_bytes = message_json.encode('utf-8')
            else:
                message_bytes = message_json
            self.socket.send(message_bytes)
            rospy.loginfo("发送WiFi命令: %s", command_unicode.encode('utf-8') if isinstance(command_unicode, unicode) else command_unicode)
            
            # 接收响应
            self.socket.settimeout(5.0)  # 设置接收超时
            response_bytes = self.socket.recv(1024)
            response_data = response_bytes.decode('utf-8')
            result = json.loads(response_data)
            
            # 发布响应到ROS话题
            response_msg = String()
            # Python 2兼容：确保响应数据是字符串（不是unicode）
            response_json = json.dumps(result, ensure_ascii=False)
            if isinstance(response_json, unicode):
                response_msg.data = response_json.encode('utf-8')
            else:
                response_msg.data = response_json
            self.response_pub.publish(response_msg)
            
            # 日志输出
            log_json = json.dumps(result, ensure_ascii=False)
            if isinstance(log_json, unicode):
                rospy.loginfo("收到响应: %s", log_json.encode('utf-8'))
            else:
                rospy.loginfo("收到响应: %s", log_json)
            return result
            
        except socket.timeout:
            rospy.logwarn("接收响应超时")
            self.connected = False
            return None
        except socket.error as e:
            rospy.logwarn("Socket错误: %s", str(e))
            self.connected = False
            return None
        except Exception as e:
            rospy.logerr("发送命令失败: %s", str(e))
            return None
    
    def command_callback(self, msg):
        """ROS话题回调函数，接收命令并发送"""
        command = msg.data.strip()
        if command:
            rospy.loginfo("收到ROS命令: %s", command)
            result = self.send_command(command)
            if result:
                rospy.loginfo("命令执行成功")
            else:
                rospy.logwarn("命令执行失败")
    
    def publish_status(self, status):
        """发布状态信息"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def run(self):
        """运行节点"""
        # 如果启用自动重连，启动重连线程
        if self.auto_reconnect:
            reconnect_thread = threading.Thread(target=self._reconnect_loop)
            reconnect_thread.daemon = True
            reconnect_thread.start()
        
        # 保持节点运行
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("收到键盘中断信号")
        finally:
            # 节点关闭时断开连接
            self.disconnect()
            self.disconnect_serial()
    
    def _reconnect_loop(self):
        """自动重连循环"""
        while not rospy.is_shutdown() and not self.shutdown_flag:
            rospy.sleep(self.reconnect_interval)
            if not self.connected and not self.shutdown_flag:
                rospy.loginfo("尝试自动重连...")
                self.connect()

def main():
    try:
        node = WiFiCommandClientNode()
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo("WiFi命令客户端节点已关闭")
        sys.exit(0)
    except rospy.ROSInterruptException:
        rospy.loginfo("WiFi命令客户端节点已关闭")
    except Exception as e:
        rospy.logerr("节点运行错误: %s", str(e))
        sys.exit(1)

if __name__ == "__main__":
    main()
