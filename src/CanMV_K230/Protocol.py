import struct
import time

from Base import logger

class FC_Protocol:
    """飞控协议层，简化实现"""
    
    # 模式定义
    MODE_MANUAL = 0    # 姿态模式
    MODE_HEIGHT = 1    # 定高模式
    MODE_POS_VEL = 3   # 位置速度模式
    MODE_POS = 4       # 位置模式
    
    # 命令定义
    CMD_NULL = 0           # 无命令
    CMD_TAKE_OFF = 1       # 起飞
    CMD_LANDING = 2        # 降落  
    CMD_HOVERING = 3       # 悬停
    CMD_MOVING = 4         # 移动
    CMD_USER_DEF = 0xF0    # 用户自定义

    def __init__(self, base_com):
        """初始化协议层
        
        Args:
            base_com: 基础通信层实例
        """
        self.base_com = base_com
        self.target_yaw = 0.0
    
    def poll(self):
        """轮询，需要在主循环中调用"""
        self.base_com.poll()
    
    def quit(self):
        """退出"""
        self.base_com.quit()
    
    def send_realtime_control_data(
        self, 
        throttle=0, 
        roll=0, 
        pitch=0, 
        yaw=0, 
        mode=None, 
        cmd=0, 
        cmd_1=0,
        need_ack=False
    ):
        """发送实时控制数据
        
        Args:
            throttle: 油门 (-1000~1000)
            roll: 横滚角 (-1000~1000)
            pitch: 俯仰角 (-1000~1000)
            yaw: 偏航角 (-1000~1000)
            mode: 模式
            cmd: 命令
            cmd_1: 命令参数
            need_ack: 是否需要应答
        """
        data = bytearray(10)
        struct.pack_into("<hhhh", data, 0, 
                        throttle, roll, pitch, yaw)
        
        data[8] = mode if mode is not None else self.base_com.state.mode.value
        data[9] = cmd
        return self.base_com.send_data_to_fc(data, 0x01, need_ack)
    
    def set_flight_mode(self, mode, need_ack=True):
        """设置飞行模式
        
        Args:
            mode: 模式
            need_ack: 是否需要应答
        """
        if mode != self.base_com.state.mode.value:
            logger.info(f"[FC] Setting flight mode: {mode}")
            return self.send_realtime_control_data(
                mode=mode, 
                need_ack=need_ack
            )
        return True
    
    def send_command(self, cmd, cmd_1=0, need_ack=True):
        """发送命令
        
        Args:
            cmd: 命令
            cmd_1: 命令参数
            need_ack: 是否需要应答
        """
        logger.info(f"[FC] Sending command: {cmd}, param: {cmd_1}")
        return self.send_realtime_control_data(
            cmd=cmd, 
            cmd_1=cmd_1, 
            need_ack=need_ack
        )
    
    def unlock(self, need_ack=True):
        """解锁"""
        if self.base_com.state.unlock.value:
            logger.info("[FC] Already unlocked")
            return True
        logger.info("[FC] Unlocking...")
        
        # 在MicroPython中，我们使用简单的二进制数据构造
        data = bytearray([0xA5, 0x5A])
        ret = self.base_com.send_data_to_fc(data, 0x02, need_ack)
        time.sleep(0.5)  # 等待解锁完成
        return ret is not None
    
    def lock(self, need_ack=True):
        """上锁"""
        if not self.base_com.state.unlock.value:
            logger.info("[FC] Already locked")
            return True
        logger.info("[FC] Locking...")
        
        data = bytearray([0x5A, 0xA5])
        ret = self.base_com.send_data_to_fc(data, 0x02, need_ack)
        time.sleep(0.5)  # 等待上锁完成
        return ret is not None
    
    def calibrate_imu(self, need_ack=True):
        """校准IMU"""
        logger.info("[FC] Calibrating IMU...")
        data = bytearray([0x67, 0x89])
        return self.base_com.send_data_to_fc(data, 0x03, need_ack) is not None
    
    def calibrate_mag(self, need_ack=True):
        """校准磁力计"""
        logger.info("[FC] Calibrating MAG...")
        data = bytearray([0x78, 0x56])
        return self.base_com.send_data_to_fc(data, 0x03, need_ack) is not None
    
    def calibrate_acc(self, need_ack=True):
        """校准加速度计"""
        logger.info("[FC] Calibrating ACC...")
        data = bytearray([0x78, 0x57])
        return self.base_com.send_data_to_fc(data, 0x03, need_ack) is not None
    
    def take_off(self, height=80, need_ack=True):
        """起飞
        
        Args:
            height: 起飞高度(cm)
            need_ack: 是否需要应答
        """
        if not self.base_com.state.unlock.value:
            logger.warning("[FC] Cannot take off when locked")
            return False
            
        logger.info(f"[FC] Taking off to {height}cm...")
        return self.send_command(self.CMD_TAKE_OFF, height, need_ack) is not None
    
    def land(self, need_ack=True):
        """降落"""
        logger.info("[FC] Landing...")
        return self.send_command(self.CMD_LANDING, 0, need_ack) is not None
    
    def hover(self, need_ack=True):
        """悬停"""
        logger.info("[FC] Hovering...")
        return self.send_command(self.CMD_HOVERING, 0, need_ack) is not None
    
    def move_to(self, x=0, y=0, z=None, yaw=None, speed=40, need_ack=True):
        """移动到指定位置
        
        Args:
            x: x坐标(cm)
            y: y坐标(cm)
            z: z坐标(cm)，若为None则保持当前高度
            yaw: 偏航角(deg)，若为None则保持当前偏航角
            speed: 移动速度(cm/s)
            need_ack: 是否需要应答
        """
        if z is None:
            z = self.base_com.state.alt.value
            
        if yaw is None:
            yaw = int(self.base_com.state.yaw.value)
        else:
            # 保存目标偏航角
            self.target_yaw = yaw
            
        # 确保位置模式
        if self.base_com.state.mode.value != self.MODE_POS:
            if self.base_com.settings.auto_change_mode:
                self.set_flight_mode(self.MODE_POS)
        
        # 构造移动命令参数，打包为二进制格式
        data = bytearray(11)
        struct.pack_into("<hhhHB", data, 0, x, y, z, int(yaw * 100), speed)
        
        logger.info(f"[FC] Moving to x={x}, y={y}, z={z}, yaw={yaw}, speed={speed}")
        return self.base_com.send_data_to_fc(data, 0x10, need_ack) is not None
    
    def move_by(self, dx=0, dy=0, dz=0, dyaw=0, speed=40, need_ack=True):
        """相对移动
        
        Args:
            dx: x方向移动距离(cm)
            dy: y方向移动距离(cm)
            dz: z方向移动距离(cm)
            dyaw: 偏航角变化(deg)
            speed: 移动速度(cm/s)
            need_ack: 是否需要应答
        """
        x = self.base_com.state.pos_x.value + dx
        y = self.base_com.state.pos_y.value + dy
        z = self.base_com.state.alt.value + dz
        
        if hasattr(self, 'target_yaw'):
            yaw = self.target_yaw + dyaw
        else:
            yaw = self.base_com.state.yaw.value + dyaw
            
        # 更新目标偏航角
        self.target_yaw = yaw
        
        return self.move_to(x, y, z, yaw, speed, need_ack)