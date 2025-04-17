import struct
import time

from Base import logger,FC_Base_Uart_Communication

class FC_Protocol(FC_Base_Uart_Communication):
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

    def __init__(self, base_com: FC_Base_Uart_Communication = None):
        """初始化协议层
        
        Args:
            base_com: 基础通信层实例
        """
        self.base_com = base_com
        self.target_yaw = 0.0
    
    def poll(self):
        """轮询，需要在主循环中调用"""
        if self.base_com:
            self.base_com.poll()
    
    def quit(self):
        """退出"""
        if self.base_com:
            self.base_com.quit()

    def _send_32_command(
        self, suboption: int, data: bytes = b"", need_ack=False
    ):
        suboption = suboption & 0xFF
        cmd_data = bytes([suboption]) + data
        sended = self.base_com.send_data_to_fc(
            cmd_data, 0x01, need_ack=need_ack
        )
        # logger.debug(f"[FC] Send: {bytes_to_str(sended)}")
        return sended

    def _send_imu_command_frame(self, CID: int, CMD0: int, CMD1: int, CMD_data=b""):
        """发送IMU命令帧
        
        Args:
            CID: 命令ID
            CMD0: 命令参数0
            CMD1: 命令参数1
            CMD_data: 命令数据
        
        Returns:
            发送的数据或None(失败时)
        """
        # 转换参数为单字节
        cid_byte = bytes([CID & 0xFF])
        cmd0_byte = bytes([CMD0 & 0xFF])
        cmd1_byte = bytes([CMD1 & 0xFF])
        
        # 处理命令数据
        bytes_data = bytes(CMD_data)
        if len(bytes_data) < 8:
            bytes_data += b"\x00" * (8 - len(bytes_data))
        if len(bytes_data) > 8:
            raise Exception("CMD_data length is too long")
        
        # 组合要发送的数据
        data_to_send = cid_byte + cmd0_byte + cmd1_byte + bytes_data
        
        # 发送数据
        sended = self.base_com.send_data_to_fc(data_to_send, 0x02, need_ack=True)
        
        # 记录最后发送的命令
        self.last_sended_command = (CID, CMD0, CMD1)
        
        # logger.debug(f"[FC] Send: {bytes_to_str(sended)}")
        return sended

    def send_realtime_control_data(
        self, vel_x: int = 0, vel_y: int = 0, vel_z: int = 0, yaw: int = 0, need_ack: bool = False
    ):
        """
        发送实时控制帧, 仅在定点模式下有效(MODE=2), 切换模式前需要确保遥控器摇杆全部归中
        在飞控内有实时控制帧的安全检查, 每个帧有效时间只有1s, 因此发送频率需要大于1Hz
        注意记得切换模式!!!
        Args:
            vel_x: x轴速度
            vel_y: y轴速度
            vel_z: z轴速度
            yaw: 偏航角
            need_ack: 是否需要应答

            
        Returns:
            bool: 发送是否成功
        """         
        try:
            
            data = struct.pack("<hhhh", int(vel_x), int(vel_y), int(vel_z), int(-yaw))
            # print(data)
            # 实时控制帧
            # 发送数据并获取结果
            result = self._send_32_command(0x03, 
                                           data+b'\x33', 
                                           )
            return result is not None  # 确保返回布尔值
            
        except Exception as e:
            logger.error(f"发送实时控制数据失败: {e}")
            return False
    
    def set_flight_mode(self, mode, need_ack=True):
        """   
        设置飞行模式: (随时有效)
        0: 姿态自稳 (危险,禁用)
        1: 定高
        2: 定点
        3: 程控
        Args:
            mode: 模式
            need_ack: 是否需要应答
            
        Returns:
            bool: 设置是否成功
        """
        try:
                    
            if mode not in [1, 2, 3]:
                raise ValueError("mode must be 1,2,3")
            
            temp = bytes([mode])
            self._send_imu_command_frame(0x01, 0x01, 0x01, temp)
            
        except Exception as e:
            logger.error(f"设置飞行模式失败: {e}")
            return False
        

    def _check_mode(self, target_mode) -> bool:
        """
        检查当前模式是否与需要的模式一致
        """
        mode_dict = {1: "HOLD ALT", 2: "HOLD POS", 3: "PROGRAM"}
        
        # 检查必要的属性是否存在
        if not hasattr(self.base_com.state, 'mode_sta'):
            logger.error("mode_sta 属性不存在，无法检查模式")
            return False
        
        current_mode = getattr(self.base_com.state, 'mode_sta', None)
        if current_mode is None:
            logger.error("当前模式未知")
            return False
        
        if current_mode != target_mode:
            # 检查是否有自动切换模式的属性
            auto_change = getattr(self.base_com.state, 'auto_change_mode', False)
            
            if auto_change:
                logger.info(f"自动切换到模式: {mode_dict.get(target_mode, target_mode)}")
                self.set_flight_mode(target_mode)
                time.sleep(0.1)  # 等待模式改变完成
                return True
            else:
                current_mode_name = mode_dict.get(current_mode, f"未知({current_mode})")
                target_mode_name = mode_dict.get(target_mode, f"未知({target_mode})")
                logger.error(
                    f"[FC] 模式错误: 需要的模式是 {target_mode_name}，但当前模式是 {current_mode_name}")
                return False
    
        return True
        # mode_dict = {1: "HOLD ALT", 2: "HOLD POS", 3: "PROGRAM"}
        # if self.base_com.state.mode_sta != target_mode:
        #     if self.base_com.state.auto_change_mode:
        #         logger.info("auto mode set", mode_dict[target_mode])
        #         self.set_flight_mode(target_mode)
        #         time.sleep(0.1)  # 等待模式改变完成
        #         return True
        #     else:
        #         logger.error(
        #             f"[FC] Mode error: action required mode is {mode_dict[target_mode]}, but current mode is {mode_dict[self.base_com.state.mode_sta]}")
                
        #         return False
        # return True
    
    
    def unlock(self):
        """解锁
        
        Returns:
            bool: 解锁是否成功
        """
        self._send_imu_command_frame(0x10, 0x00, 0x01)
        logger.info("unlock")

    
    def lock(self) -> None:
        """
        锁定电机 / 紧急锁浆 (随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x02)
        logger.info("lock")

    def stablize(self) -> None:
        """
        恢复定点悬停, 将终止正在进行的所有控制 (随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x04)
        logger.info("stablize")
    
    
    def take_off(self, height:int = 0):
        """起飞
        
        Args:
            height: 起飞高度(cm)
            need_ack: 是否需要应答
            
        Returns:
            bool: 起飞命令是否成功发送
        """
             
        try:
            temp = bytes([height & 0xFF])
            self._send_imu_command_frame(0x10, 0x00, 0x05, temp)
            logger.info(f"[FC] 起飞到高度: {height} cm")
            
        except Exception as e:
            logger.error(f"起飞失败: {e}")
            return False
    
    def land(self, need_ack=True):
        """降落
        
        Returns:
            bool: 降落命令是否成功发送
        """
         
        try:
            
            self._send_imu_command_frame(0x10, 0x00, 0x06)
            logger.info("[FC] 正在降落...")
            
        except Exception as e:
            logger.error(f"降落失败: {e}")
            return False

    def horizontal_move(self, distance: int, speed: int, direction: int) -> None:
        """
        水平移动: (程控模式下有效)
        移动距离:0-10000 cm
        移动速度:10-300 cm/s
        移动方向:0-359 度 (当前机头为0参考,顺时针)
        """
    
        try:
            # 检查当前模式是否为程控模式
            if not self._check_mode(3):
                logger.error("水平移动失败：当前不是程控模式")
                return False
            
            
            data = struct.pack("<HHH", 
                            distance & 0xFFFF, 
                            speed & 0xFFFF, 
                            direction & 0xFFFF)
            
            result = self._send_imu_command_frame(0x10, 0x02, 0x03, data)
            
            logger.info(f"[FC] 水平移动: {distance}cm, {speed}cm/s, {direction}度")
            return result is not None
            
        except Exception as e:
            logger.error(f"水平移动命令发送失败: {e}")
            return False


        # self._check_mode(3)
        
        # # 使用struct打包数据，取代_byte_temp变量
        # data = struct.pack("<HHH", 
        #                 distance & 0xFFFF, 
        #                 speed & 0xFFFF, 
        #                 direction & 0xFFFF)
        
        # self._send_imu_command_frame(0x10, 0x02, 0x03, data)
        
        # # 使用logger替代_action_log
        # logger.info(f"[FC] 水平移动: {distance}cm, {speed}cm/s, {direction}度")

    def go_up(self, distance: int, speed: int) -> None:
        """
        上升: (程控模式下有效)
        上升距离:0-10000 cm
        上升速度:10-300 cm/s
        """
        self._check_mode(3)
        
        # 使用struct打包数据
        data = struct.pack("<HH", 
                        distance & 0xFFFF, 
                        speed & 0xFFFF)
        
        self._send_imu_command_frame(0x10, 0x02, 0x01, data)
        
        # 使用logger替代_action_log
        logger.info(f"[FC] 上升: {distance}cm, {speed}cm/s")

    def go_down(self, distance: int, speed: int) -> None:
        """
        下降: (程控模式下有效)
        下降距离:0-10000 cm
        下降速度:10-300 cm/s
        """
        self._check_mode(3)
        
        # 使用struct打包数据
        data = struct.pack("<HH", 
                        distance & 0xFFFF, 
                        speed & 0xFFFF)
        
        self._send_imu_command_frame(0x10, 0x02, 0x02, data)
        
        # 使用logger替代_action_log
        logger.info(f"[FC] 下降: {distance}cm, {speed}cm/s")

    def turn_left(self, deg: int, speed: int) -> None:
        """
        左转: (程控模式下有效)
        左转角度:0-359 度
        左转速度:5-90 deg/s
        """
        self._check_mode(3)
        
        # 使用struct打包数据
        data = struct.pack("<HH", 
                        deg & 0xFFFF, 
                        speed & 0xFFFF)
        
        self._send_imu_command_frame(0x10, 0x02, 0x07, data)
        
        # 使用logger替代_action_log
        logger.info(f"[FC] 左转: {deg}度, {speed}度/秒")

    def turn_right(self, deg: int, speed: int) -> None:
        """
        右转: (程控模式下有效)
        右转角度:0-359 度
        右转速度:5-90 deg/s
        """
        self._check_mode(3)
        
        # 使用struct打包数据
        data = struct.pack("<HH", 
                        deg & 0xFFFF, 
                        speed & 0xFFFF)
        
        self._send_imu_command_frame(0x10, 0x02, 0x08, data)
        
        # 使用logger替代_action_log
        logger.info(f"[FC] 右转: {deg}度, {speed}度/秒")

    def set_height(self, source: int, height: int, speed: int) -> None:
        """
        设置高度: (程控模式下有效)
        高度源: 0:融合高度 1:激光高度
        高度:0-10000 cm
        速度:10-300 cm/s
        """
        logger.info(f"[FC] 设置高度: {'融合' if source == 0 else '激光'}, {height}cm, {speed}cm/s")
        
        # 这里需要访问state的属性，可能需要从base_com获取
        if source == 0:
            alt = self.base_com.state.alt_fused
        elif source == 1:
            alt = self.base_com.state.alt_add
        
        # 确保高度值为数值，而不是对象
        current_height = float(alt)
        
        if height < current_height:
            self.go_down(int(current_height - height), speed)
        elif height > current_height:
            self.go_up(int(height - current_height), speed)

    def set_yaw(self, yaw: int, speed: int) -> None:
        """
        设置偏航角: (程控模式下有效)
        偏航角:-180-180 度
        偏航速度:5-90 deg/s
        """
        logger.info(f"[FC] 设置偏航角: {yaw}度, {speed}度/秒")
        current_yaw = self.base_com.state.yaw
        
        # 确保yaw值为数值
        if isinstance(current_yaw, (int, float)):
            current_yaw_value = current_yaw
        else:
            # 如果是对象，尝试获取value属性
            current_yaw_value = getattr(current_yaw, 'value', 0)
        
        # 计算最短旋转路径
        if yaw < current_yaw_value:
            left_turn_deg = abs(current_yaw_value - yaw)
            right_turn_deg = abs(360 - left_turn_deg)
        else:
            right_turn_deg = abs(current_yaw_value - yaw)
            left_turn_deg = abs(360 - right_turn_deg)
        
        # 选择最短路径旋转
        if left_turn_deg < right_turn_deg:
            self.turn_left(left_turn_deg, speed)
        else:
            self.turn_right(right_turn_deg, speed)

    def set_target_position(self, x: int, y: int) -> None:
        """
        设置目标位置: (程控模式下有效)
        x:+-100000 cm
        y:+-100000 cm
        """
        self._check_mode(3)
        
        # 使用struct打包数据，s32表示有符号32位整数
        data = struct.pack("<ii", x, y)
        
        self._send_imu_command_frame(0x10, 0x01, 0x01, data)
        
        logger.info(f"[FC] 设置目标位置: x={x}, y={y}")

    def set_target_height(self, height: int) -> None:
        """
        设置目标高度: (程控模式下有效)
        目标对地高度:+100000 cm
        """
        if height < 0:
            height = 0
        self._check_mode(3)
        
        # 使用struct打包数据，i表示有符号32位整数
        data = struct.pack("<i", height)
        
        self._send_imu_command_frame(0x10, 0x01, 0x02, data)
        
        logger.info(f"[FC] 设置目标高度: {height}cm")