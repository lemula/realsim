import pygame
import socket
import time
import json
from datetime import datetime

class DualJoystickSender:
    def __init__(self, jetson_ip, jetson_port):
        # 初始化pygame和游戏杆系统
        pygame.init()
        pygame.joystick.init()
        
        # 存储两个操作杆对象
        self.joysticks = []
        self.joystick_names = []
        
        # 尝试初始化两个操作杆
        try:
            joystick_count = pygame.joystick.get_count()
            print(f"检测到 {joystick_count} 个操作杆设备")
            
            # 初始化最多两个操作杆
            for i in range(min(2, joystick_count)):
                js = pygame.joystick.Joystick(i)
                js.init()
                self.joysticks.append(js)
                name = js.get_name()
                self.joystick_names.append(name)
                print(f"已连接操作杆 {i}: {name}")
                
            if len(self.joysticks) < 1:
                print("未检测到任何操作杆，程序退出")
                exit()
                
        except Exception as e:
            print(f'初始化操作杆失败: {e}')
            exit()
        
        # 网络配置
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.jetson_address = (jetson_ip, jetson_port)
        self.connected = False
        
        # 操作杆轴和按钮映射（可根据实际设备调整）
        # 每个操作杆: [X轴, Y轴, Z轴]
        self.controller_mappings = [
            [0, 1, 4],  # 第一个操作杆映射
            [0, 1, 4]   # 第二个操作杆映射（可根据实际情况修改）
        ]

    def verify_connection(self):
        """验证与Jetson的连接"""
        print(f"正在验证与 {self.jetson_address} 的连接...")
        try:
            # 发送握手请求
            handshake = json.dumps({
                "type": "handshake", 
                "timestamp": datetime.now().timestamp(),
                "controllers": self.joystick_names
            })
            self.sock.sendto(handshake.encode('utf-8'), self.jetson_address)
            
            # 设置超时等待响应
            self.sock.settimeout(3.0)
            response, addr = self.sock.recvfrom(1024)
            
            if addr == self.jetson_address:
                resp_data = json.loads(response.decode('utf-8'))
                if resp_data.get("status") == "ready":
                    self.connected = True
                    print("连接成功，准备发送数据...")
                    return True
        except socket.timeout:
            print("连接超时，未收到响应")
        except json.JSONDecodeError:
            print("收到无效的响应格式")
        except Exception as e:
            print(f"连接验证失败: {e}")
        finally:
            self.sock.settimeout(None)  # 恢复默认阻塞模式
        return False

    def read_joystick_data(self, js_index):
        """读取单个操作杆数据并格式化（构造带单位的字符串）"""
        if js_index >= len(self.joysticks):
            return None
            
        js = self.joysticks[js_index]
        mapping = self.controller_mappings[js_index]
        
        # 读取轴数据并转换为实际控制范围
        X = js.get_axis(mapping[0])
        Y = js.get_axis(mapping[1])
        Z = js.get_axis(mapping[2])
        
        # 构造格式化字符串（带单位）
        formatted_data = (
            f"操作杆 {js_index+1} ({self.joystick_names[js_index]}):\n"
            f"  X{js_index+1}: {round(X, 2)}\n"
            f"  Y{js_index+1}: {round(Y, 2)}\n"
            f"  Z{js_index+1}: {round(Z, 2)}"
        )
        
        return formatted_data

    def read_all_data(self):
        """读取所有连接的操作杆数据（拼接为完整字符串）"""
        pygame.event.get()  # 更新所有控制器状态
        
        all_data = {
            "timestamp": datetime.now().timestamp(),
            "controllers": []
        }
        
        # 收集每个操作杆的格式化字符串
        formatted_str = ""
        for i in range(len(self.joysticks)):
            js_data = self.read_joystick_data(i)
            if js_data:
                formatted_str += js_data + "\n"  # 拼接多个操作杆数据
                
        # 封装为字典（方便JSON序列化）
        all_data["formatted_data"] = formatted_str.strip()  # 去除末尾空行
        return all_data

    def run(self):
        """主运行逻辑：验证连接后发送数据"""
        if not self.verify_connection():
            print("无法建立连接，程序退出")
            self.cleanup()
            return
        
        print("开始发送数据，按Ctrl+C停止...")
        last_print_time = time.time()  # 用于控制打印频率
        
        try:
            while True:
                start_time = time.time()
                
                # 读取并发送数据
                data = self.read_all_data()
                self.sock.sendto(json.dumps(data).encode('utf-8'), self.jetson_address)
                
                # 每隔1秒打印一次数据（本地调试用）
                current_time = time.time()
                if current_time - last_print_time >= 1.0:
                    print(f"\n[{datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S')}]")
                    print(data["formatted_data"])  # 打印格式化后的内容
                    last_print_time = current_time
                
                # 控制发送频率为10Hz（每100ms一次）
                elapsed = time.time() - start_time
                sleep_time = max(0.1 - elapsed, 0)  # 确保至少间隔100ms
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n用户中断，停止发送")
        finally:
            self.cleanup()

    def cleanup(self):
        """资源清理"""
        # 关闭所有操作杆
        for js in self.joysticks:
            js.quit()
        pygame.joystick.quit()
        pygame.quit()
        self.sock.close()
        print("资源已释放，程序退出")

if __name__ == "__main__":
    # 配置Jetson的IP和端口（根据实际情况修改）
    #JETSON_IP = "192.168.0.85"  # 替换为实际IP
    JETSON_IP = "192.168.2.88"# 替换为实际IP
    JETSON_PORT = 8888           # 替换为实际端口
    
    sender = DualJoystickSender(JETSON_IP, JETSON_PORT)
    sender.run()
