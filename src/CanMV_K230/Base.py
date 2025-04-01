# 简化日志功能
class SimpleLogger:
    def info(self, msg):
        print("[INFO] " + msg)
    
    def error(self, msg):
        print("[ERROR] " + msg)
    
    def warning(self, msg):
        print("[WARN] " + msg)
    
    def debug(self, msg):
        print("[DEBUG] " + msg)

logger = SimpleLogger()

# 导入MP_Serial类
from Serial import MP_Serial
import time

# FC状态结构体
class FC_State_Struct:
    """飞控状态结构体，简化版"""
    def __init__(self):
        """初始化状态结构体"""
        
        self.connected = False     # 连接状态
        
        # 姿态角（deg）
        self.rol = 0.0
        self.pit = 0.0
        self.yaw = 0.0

        self.alt_fused = 0.0
        self.alt_add = 0.0
        
        # 位置（cm）
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        
        # 速度（cm/s）
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        
        # 其他状态
        self.voltage = 0.0          # 电池电压（V）
        self.mode_sta = 0      # 模式
        self.auto_change_mode = 0  # 自动切换模式
        self.unlock_sta = False      # 解锁状态


# 飞控通信基础实现
class FC_Base_Uart_Communication:
    def __init__(self):
        """初始化通信"""
        self.state = FC_State_Struct()
        self.connected = False
        self.serial = None
        self.callback = None
        self._running = False
        self._print_data = False
        self.last_heartbeat_time = 0
        self.heartbeat_interval = 0.99
        # 添加ACK相关字段
        self._waiting_ack = False
        self._received_ack = None
        self.ack_timeout = 1.0  # ACK超时时间，秒
        self.max_retry_count = 3  # 最大重试次数

    def send_data_to_fc(self, data, option, need_ack=False, retry_count=None):
        """发送数据到飞控
        
        Args:
            data: 要发送的数据
            option: 选项/命令类型
            need_ack: 是否需要ACK确认
            retry_count: 重试计数
            
        Returns:
            发送的数据或None(失败时)
        """
        # 初始化重试计数
        if retry_count is None:
            retry_count = self.max_retry_count
        
        # 重试次数检查
        if retry_count < 0:
            logger.error("发送数据达到最大重试次数")
            return None
        
        # 计算ACK校验值
        if need_ack:
            self._waiting_ack = True
            self._received_ack = None
            send_time = time.time()
            check_sum = option
            for b in data:
                check_sum = (check_sum + b) & 0xFF
        
        # 设置发送配置并发送数据
        try:
            self.serial.send_config(startBit=[0xAA, 0x22], optionBit=[option])
            result = self.serial.write(data)
            if self._print_data:
                logger.debug(f"发送数据: {' '.join([hex(b) for b in data])}")
        except Exception as e:
            logger.error(f"发送数据失败: {e}")
            return None
        
        # 如果需要ACK，等待接收
        if need_ack:
            # 等待ACK响应
            while self._waiting_ack:
                self.poll()  # 轮询处理，接收数据
                
                # 检查是否超时
                if time.time() - send_time > self.ack_timeout:
                    logger.warning("等待ACK超时，重试")
                    self._waiting_ack = False
                    return self.send_data_to_fc(data, option, need_ack, retry_count - 1)
                
                # 避免CPU占用过高
                time.sleep(0.01)
            
            # 检查ACK是否正确
            if self._received_ack is None or self._received_ack != check_sum:
                logger.warning("收到的ACK不正确，重试")
                return self.send_data_to_fc(data, option, need_ack, retry_count - 1)
        
        return result

    # 添加处理ACK的方法
    def process_ack(self, ack_data):
        """处理接收到的ACK数据"""
        if self._waiting_ack:
            self._received_ack = ack_data
            self._waiting_ack = False
            if self._print_data:
                logger.debug(f"收到ACK: {hex(ack_data)}")


    def start_listen_serial(self, uart_id=2, bit_rate=500000, print_state=False, callback=None):
        """启动串口监听
        
        Args:
            uart_id: UART ID
            bit_rate: 波特率
            print_state: 是否打印状态信息
            callback: 回调函数
        """
        from machine import UART
        
        # 使用MP_Serial代替直接操作UART
        self.serial = MP_Serial(uart_id, bit_rate, tx_pin=11, rx_pin=12)
        self.serial.send_config(startBit=[0xAA, 0x22], optionBit=[0x00])
        self.serial.read_config(startBit=[0xAA, 0x55])
        
        self._print_data = print_state
        self.callback = callback
        self._running = True
        self.last_heartbeat_time = time.time()  # 初始化心跳时间
        logger.info("串口监听启动成功")

    def _listen_serial_task(self,tick):
        """处理串口接收到的数据
        
        Args:
            tick: 当前时间戳（未使用，保留参数兼容性）
        """

        try:
            if self.serial and self.serial.read():
                data = self.serial.rx_data

                if self._print_data:
                    logger.debug(f"接收到数据: {' '.join([hex(b) for b in data])}")
                # 检查数据包长度
                if len(data) < 1:
                    return  # 数据包过短，无法解析
                    
                # 解析数据包头
                cmd = data[0]
                payload = data[1:]

                if self._print_data:
                    logger.debug(f"解析数据包: cmd={hex(cmd)}, payload_length={len(payload)}")
                
                # 根据命令类型处理数据
                if cmd == 0x02:  # ACK返回
                    if len(payload) > 0:
                        self.process_ack(payload[0])
                
                elif cmd == 0x01:  # 飞控状态数据
                    # 直接调用已有的解析函数
                    self.parse_data_packet(payload)
                    
                    # 更新连接状态
                    if not self.state.connected:
                        self.state.connected = True
                        logger.info("已连接到飞控")
                    
                else:  # 未知命令
                    if self._print_data:
                        logger.debug(f"收到未知命令: {hex(cmd)}, 数据: {' '.join([hex(b) for b in payload])}")
                
                # 如果设置了回调函数，调用回调
                if self.callback:
                    self.callback(cmd, payload)
                    
        except Exception as e:
            logger.error(f"串口数据解析异常: {e}")
        
        # 发送心跳包
        self.send_heartbeat()
        # current_time = time.time()
        # if current_time - self.last_heartbeat_time > self.heartbeat_interval:
        #     self.send_heartbeat()
        #     self.last_heartbeat_time = current_time
            


    def send_cmd_32(self, data):
        self.serial.send_config(startBit=[0xAA, 0x22], optionBit=[0x01])
        self.serial.write(data)



    def send_bytes(self, data):
        """发送数据
        
        Args:
            data: 要发送的数据(字节或列表)
            
        Returns:
            bool: 发送是否成功
        """
        if not self.serial:
            logger.error("串口未初始化")
            return False
            
        try:
            self.serial.write(data)
            return True
        except Exception as e:
            logger.error("发送数据失败: " + str(e))
            return False

    def send_heartbeat(self):
        """发送心跳包"""
        if not self.serial:
            return False
            
        try:
            # 发送心跳包数据 0x01
            self.serial.send_config(startBit=[0xAA, 0x22], optionBit=[0x00])
            result = self.serial.write(b'\x01')
            # if self._print_data:
            #     logger.debug("发送心跳包: " + " ".join([hex(b) for b in result]))
            return True
        except Exception as e:
            logger.error("发送心跳包失败: " + str(e))
            return False

    def poll(self):
        """轮询处理，需要在主循环中调用"""
        if not self._running or not self.serial:
            return
        
        # # 检查是否需要发送心跳包
        # current_time = time.time()
        # if current_time - self.last_heartbeat_time >= self.heartbeat_interval:
        #     self.send_heartbeat()
        #     self.last_heartbeat_time = current_time
            
        # # 尝试读取数据
        # if self.serial.read():
        #     # 处理读取到的数据
        #     self.state.connected = True
        #     data = self.serial.rx_data
        #     self.parse_data_packet(data)

    def set_heartbeat_interval(self, interval):
        """设置心跳包发送间隔
        
        Args:
            interval: 间隔时间(秒)
        """
        self.heartbeat_interval = interval
        logger.info("心跳包发送间隔已设置为" + str(interval) + "秒")

    def quit(self):
        """退出通信"""
        self._running = False
        if self.serial:
            self.serial.close()
            self.serial = None
        logger.info("通信已关闭")

    
    def parse_data_packet(self, data):
        """解析飞控发送的状态数据包
        AA 55 长度 cmd [数据...] checksum
        """
        if len(data) != 35 :
            print(f"ERR: 数据长度不符 ({len(data)} != 35)")
            return None

        # 解析16位有符号整数(小端序)
        def parse_s16(low, high):
            value = (high << 8) | low
            if value & 0x8000:  # 检查最高位是否为1
                value -= 0x10000
            return value / 100.0  # 角度值需除以100
        
        # 解析32位有符号整数(小端序)
        def parse_s32(b0, b1, b2, b3):
            value = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
            if value & 0x80000000:
                value -= 0x100000000
            return value
        
        # 解析16位无符号整数(小端序)
        def parse_u16(low, high):
            return ((high << 8) | low) / 100.0  # 电压值需除以100
        
        # 原始数据从索引3开始(跳过AA 55 len cmd)
        # 注意C代码中结构是按字节对齐的，这里按照确切的字段结构解析
        
        # 角度数据 (前3个s16)
        rol = parse_s16(data[0], data[1])
        pit = parse_s16(data[2], data[3])
        yaw = parse_s16(data[4], data[5])
        
        # 高度数据 (2个s32)
        alt_fused = parse_s32(data[6], data[7], data[8], data[9]) / 100.0  # 转换为米
        alt_add = parse_s32(data[10], data[11], data[12], data[13]) / 100.0  # 转换为米
        
        # 速度数据 (3个s16)
        vel_x = parse_s16(data[14], data[15]) / 100.0  # 转换为m/s
        vel_y = parse_s16(data[16], data[17]) / 100.0
        vel_z = parse_s16(data[18], data[19]) / 100.0
        
        # 位置数据 (2个s32)
        pos_x = parse_s32(data[20], data[21], data[22], data[23]) / 100.0  # 转换为米
        pos_y = parse_s32(data[24], data[25], data[26], data[27]) / 100.0
        
        # 电池电压 (1个u16)
        voltage = parse_u16(data[28], data[29])
        
        # 飞控状态 (3个u8)
        fc_mode_sta = data[30]
        unlock_sta = data[31]
        cid = data[32]
        cmd_0 = data[33]
        cmd_1 = data[34]
        
        # 更新状态对象
        self.state.rol = rol
        self.state.pit = pit
        self.state.yaw = yaw
        self.state.alt_fused = alt_fused
        self.state.alt_add = alt_add
        self.state.vel_x = vel_x
        self.state.vel_y = vel_y
        self.state.vel_z = vel_z
        self.state.pos_x = pos_x
        self.state.pos_y = pos_y
        self.state.voltage = voltage
        self.state.mode_sta = fc_mode_sta
        self.state.unlock_sta = unlock_sta
        # self.state.cid = cid
        # self.state.cmd_0 = cmd_0
        # self.state.cmd_1 = cmd_1
        
        if self._print_data:
            print(f"姿态: rol={rol:.2f}° pit={pit:.2f}° yaw={yaw:.2f}°")
            print(f"高度: {alt_fused:.2f}m 附加高度: {alt_add:.2f}m")
            print(f"位置: x={pos_x:.2f}m y={pos_y:.2f}m")
            print(f"速度: vx={vel_x:.2f}m/s vy={vel_y:.2f}m/s vz={vel_z:.2f}m/s")
            print(f"电压: {voltage:.2f}V 模式:{fc_mode_sta} 解锁:{unlock_sta}")
        
        return True


def main():
    # 初始化飞控通信实例
    fc_com = FC_Base_Uart_Communication()
    
    # 启动串口监听（使用UART2，500000波特率，启用状态打印）
    fc_com.start_listen_serial(uart_id=2, bit_rate=500000, print_state=True)
    

    # 初始化时间记录
    start_time = time.time()
    last_status_print = time.time()
    status_interval = 5  # 状态打印间隔
    count = 0
    count_lock = 0
    logger.info(f"程序启动时间: {start_time}")

    try:
        while True:
            # 轮询处理通信
            fc_com.poll()
            
            # 定期打印系统状态
            current_time = time.time()
            if current_time - last_status_print >= status_interval:
                if fc_com.state.mode_sta == 1:
                    if fc_com.state.unlock_sta == 0:
                        fc_com.send_cmd_32(b'\x07')

                if fc_com.state.mode_sta == 1:
                    if fc_com.state.unlock_sta == 1:
                        fc_com.send_cmd_32(b'\x08')
                
                if fc_com.state.mode_sta == 3:
                    if fc_com.state.unlock_sta == 1:
                        if count < 2:
                            fc_com.send_cmd_32(b'\x09')
                            count+=1

                if count >= 2:
                    fc_com.send_cmd_32(b'\x0A')
                    count_lock+=1
                
                if count_lock >= 3:
                    fc_com.send_cmd_32(b'\0x0B')



                
                # 构建状态信息
                status_msg = [
                    f"运行时长: {current_time - start_time:.1f}s",
                    f"飞控连接: {'已连接' if fc_com.state.connected else '未连接'}",
                    f"当前模式: {fc_com.state.mode_sta if fc_com.state.connected else 'N/A'}",
                    f"电池电压: {fc_com.state.voltage if fc_com.state.connected else 'N/A':.2f}V"
                ]
                
                
                logger.info("系统状态 - " + " | ".join(status_msg))
                last_status_print = current_time
                
#            # 降低CPU占用
#            time.sleep(0.001)

    except KeyboardInterrupt:
        fc_com.quit()
        logger.info("程序已正常退出")

if __name__ == "__main__":
    main()