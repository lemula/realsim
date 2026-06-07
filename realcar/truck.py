import json
import logging
import signal
import socket
import time
from datetime import datetime

from car import (
    CHANNEL_DIFF_LOCK,
    CHANNEL_LIFT_DIR,
    CHANNEL_LIFT_SPEED,
    CHANNEL_STEERING,
    CHANNEL_THROTTLE,
    ServoController,
)

logger = logging.getLogger(__name__)


def request_shutdown(_signum, _frame):
    raise KeyboardInterrupt


class JetsonReceiver:
    def __init__(self, host="192.168.0.85", port=8888, socket_timeout=0.5):
        self.host = host
        self.port = port
        self.socket_timeout = socket_timeout
        self.last_print_time = 0  # 记录上次打印时间
        self.outputs_active = False
        self.controller = ServoController()  # 初始化控制器
        self.setup_socket()

    def setup_socket(self):
        """创建并配置UDP socket"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.settimeout(self.socket_timeout)
        try:
            self.server_socket.bind((self.host, self.port))
            logger.info(f"服务已启动，监听 {self.host}:{self.port}")
        except Exception as e:
            logger.error(f"Socket绑定失败: {e}")
            raise

    def handle_handshake(self, address):
        """处理客户端握手请求"""
        response = json.dumps({"status": "ready", "message": "connected"})
        self.server_socket.sendto(response.encode("utf-8"), address)
        logger.info(f"已与客户端 {address} 建立连接")

    def stop_outputs(self, emergency=False):
        if emergency:
            self.controller.emergency_stop(hold_seconds=0.25)
        else:
            self.controller.stop()
            self.controller.reset_auxiliary_outputs()
        self.outputs_active = False

    def process_data(self, data, address):
        """处理接收到的数据并控制设备"""
        # 提取控制参数
        try:
            steering = data["steering"]
            throttle = data["throttle"]
            brake = data["brake"]
            up = data["up"]
            down = data["down"]
            lock = data["lock"]
        except KeyError as e:
            logger.error(f"数据缺少必要字段: {e}")
            self.stop_outputs()
            return

        # 执行控制逻辑
        # 1. 转向控制
        self.controller.set_servo_angle(CHANNEL_STEERING, steering)

        # 2. 油门和刹车控制
        if brake > 0:
            self.controller.set_motor_speed(CHANNEL_THROTTLE, -brake)
        else:
            self.controller.set_motor_speed(CHANNEL_THROTTLE, throttle)

        # 3. 升降控制
        if up == 1:
            self.controller.set_servo_angle(CHANNEL_LIFT_DIR, 60)  # 上升方向
            self.controller.set_motor_speed(CHANNEL_LIFT_SPEED, 70)  # 上升速度
        elif down == 1:
            self.controller.set_servo_angle(CHANNEL_LIFT_DIR, 150)  # 下降方向
            self.controller.set_motor_speed(CHANNEL_LIFT_SPEED, 70)  # 下降速度
        else:
            self.controller.set_servo_angle(CHANNEL_LIFT_DIR, 90)  # 停止方向
            self.controller.set_motor_speed(CHANNEL_LIFT_SPEED, 0)  # 停止升降

        # 4. 差速锁控制
        if lock == 1:
            self.controller.set_servo_angle(CHANNEL_DIFF_LOCK, 30)  # 锁定状态
        else:
            self.controller.set_servo_angle(CHANNEL_DIFF_LOCK, 90)  # 解锁状态
        self.outputs_active = True

        # 每隔1秒打印一次数据
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0:
            try:
                print("\n" + "=" * 40)
                print(f"[{datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S')}]")
                print(f"来自客户端: {address}")
                print(f"方向盘角度: {steering}°")
                print(f"油门开度: {throttle}%")
                print(f"刹车开度: {brake}%")
                print(f"上升按钮: {'按下' if up == 1 else '未按'}")
                print(f"下降按钮: {'按下' if down == 1 else '未按'}")
                print(f"差速锁: {'锁定' if lock == 1 else '解锁'}")
                print("=" * 40)

                self.last_print_time = current_time
            except Exception as e:
                logger.error(f"打印数据时出错: {e}")

    def run(self):
        """主循环：接收并处理数据"""
        try:
            while True:
                # 接收数据
                try:
                    data_bytes, address = self.server_socket.recvfrom(1024)
                except socket.timeout:
                    if self.outputs_active:
                        self.stop_outputs(emergency=True)
                    continue

                try:
                    # 解析JSON数据
                    data = json.loads(data_bytes.decode("utf-8"))

                    # 处理握手请求
                    if data.get("type") == "handshake":
                        self.handle_handshake(address)
                    else:
                        # 处理控制数据
                        self.process_data(data, address)

                except json.JSONDecodeError:
                    logger.error(f"无效的JSON数据: {data_bytes}")
                    self.stop_outputs(emergency=True)
                except Exception as e:
                    logger.error(f"数据处理错误: {e}")
                    self.stop_outputs(emergency=True)

        except KeyboardInterrupt:
            logger.info("\n程序已停止")
        finally:
            self.stop_outputs(emergency=True)
            self.server_socket.close()
            logger.info("Socket已关闭，所有设备已复位")


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, request_shutdown)
    receiver = JetsonReceiver(host="192.168.2.60", port=8888)
    receiver.run()
