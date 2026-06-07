import pygame
import socket
import time
import json
from datetime import datetime

class G29Sender:
    def __init__(self, jetson_ip, jetson_port):
        # 初始化pygame和游戏杆
        pygame.init()
        try:
            self.js = pygame.joystick.Joystick(0)
            self.js.init()
            print(f"已连接游戏杆: {self.js.get_name()}")
        except Exception as e:
            print(f'无法找到游戏杆: {e}')
            exit()
        
        # 网络配置
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.jetson_address = (jetson_ip, jetson_port)
        self.connected = False
        
        # G29轴和按钮映射（根据实际设备调整）
        self.steering_axis = 0    # 方向盘
        self.throttle_axis = 1    # 油门
        self.brake_axis = 2       # 刹车
        self.up_button = 3        # 上升按钮
        self.down_button = 0      # 下降按钮
        self.lock_button = 1      # 差速锁按钮

    def verify_connection(self):
        """验证与Jetson Nano的连接"""
        print(f"正在验证与 {self.jetson_address} 的连接...")
        try:
            # 发送握手请求
            handshake = json.dumps({"type": "handshake", "timestamp": datetime.now().timestamp()})
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

    def read_g29_data(self):
        """读取G29控制器数据并格式化"""
        pygame.event.get()  # 更新控制器状态
        
        # 读取轴数据并转换为实际控制范围
        steering = self.js.get_axis(self.steering_axis)
        steering_angle = steering * 45 +90 # 转换为45-135度范围
        
        throttle = self.js.get_axis(self.throttle_axis)
        if throttle == 0:
            throttle_val= 0
        else:
            throttle_val = ((-throttle + 1) / 2) * 100  # 转换为0-100%
        
        
        brake = self.js.get_axis(self.brake_axis)
        if brake == 0:
            brake_val = 0
        else:
            brake_val = ((-brake + 1) / 2) * 100  # 转换为0-100%
        
        # 读取按钮状态
        up = 1 if self.js.get_button(self.up_button) else 0
        down = 1 if self.js.get_button(self.down_button) else 0
        lock = 1 if self.js.get_button(self.lock_button) else 0
        
        return {
            "timestamp": datetime.now().timestamp(),
            "steering": round(steering_angle, 2),
            "throttle": round(throttle_val, 2),
            "brake": round(brake_val, 2),
            "up": up,
            "down": down,
            "lock": lock
        }

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
                data = self.read_g29_data()
                self.sock.sendto(json.dumps(data).encode('utf-8'), self.jetson_address)
                
                # 每隔1秒打印一次数据
                current_time = time.time()
                if current_time - last_print_time >= 1.0:
                    print(f"\n[{datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S')}]")
                    print(f"方向盘角度: {data['steering']}°")
                    print(f"油门开度: {data['throttle']}%")
                    print(f"刹车开度: {data['brake']}%")
                    print(f"上升: {data['up']}, 下降: {data['down']}, 差速锁: {data['lock']}")
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
        if hasattr(self, 'js'):
            self.js.quit()
        pygame.quit()
        self.sock.close()
        print("资源已释放，程序退出")

if __name__ == "__main__":
    # 配置Jetson Nano的IP和端口（根据实际情况修改）
    JETSON_IP = "192.168.2.60"  #换为实际IP
    #JETSON_IP = "192.168.0.85"  # 替换为实际IP
    JETSON_PORT = 8888           # 替换为实际端口
    
    sender = G29Sender(JETSON_IP, JETSON_PORT)
    sender.run()
