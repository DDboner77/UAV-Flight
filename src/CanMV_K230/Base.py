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
        from Byte_Var import Byte_Var
        
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
        self.mode_sta = 0.0         # 模式
        self.unlock_sta = False      # 解锁状态


# 飞控通信基础实现
class FC_Base_Uart_Communication:
    """飞控串口通信基类"""
    
    def __init__(self):
        """初始化通信"""
        self.state = FC_State_Struct()
        self.connected = False
        self.serial = None
        self.callback = None
        self._running = False
        self._print_data = False
        self.last_heartbeat_time = 0  # 记录上次发送心跳包的时间
        self.heartbeat_interval = 0.25  # 心跳包发送间隔，单位秒

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
            if self._print_data:
                logger.debug("发送心跳包: " + " ".join([hex(b) for b in result]))
            return True
        except Exception as e:
            logger.error("发送心跳包失败: " + str(e))
            return False

    def poll(self):
        """轮询处理，需要在主循环中调用"""
        if not self._running or not self.serial:
            return
        
        # 检查是否需要发送心跳包
        current_time = time.time()
        if current_time - self.last_heartbeat_time >= self.heartbeat_interval:
            self.send_heartbeat()
            self.last_heartbeat_time = current_time
            
        # 尝试读取数据
        if self.serial.read():
            # 处理读取到的数据
            self.state.connected = True
            data = self.serial.rx_data
            self.parse_data_packet(data)

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

    
    def parse_data_packet(self,data):
        """解析40字节的状态数据包,36字节数据, AA 55 cmd 长度  checknum"""
        if len(data) != 36:
            print("ERR: 数据长度不符")
            return None
        

        # 解析16位有符号整数
        def parse_s16(msb, lsb):
            value = (msb << 8) | lsb
            if value & 0x8000:  # 检查最高位是否为1
                value -= 0x10000
            return value
        
        # 解析32位有符号整数
        def parse_s32(byte3, byte2, byte1, byte0):  # 小端字节序
            value = byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24)
            if value & 0x80000000:  # 检查最高位是否为1
                value -= 0x100000000
            return value
        
        # 解析16位无符号整数
        def parse_u16(msb, lsb):
            return (msb << 8) | lsb
        
        # 解析32位无符号整数
        def parse_u32(byte3, byte2, byte1, byte0):  # 小端字节序
            return byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24)
        
        # 解析数据
        rol = parse_s16(data[5 - 3], data[4 - 3])          # rol_x100 (s16)
        pit = parse_s16(data[7 - 3], data[6 - 3])          # pit_x100 (s16)
        yaw = parse_s16(data[9 - 3], data[8 - 3])          # yaw_x100 (s16)
        alt_fused = parse_s32(data[13 - 3], data[12 - 3], data[11 - 3], data[10 - 3])  # alt_fused (s32) 小端
        alt_add = parse_s32(data[17 - 3], data[16 - 3], data[15 - 3], data[14 - 3])    # alt_add (s32) 小端
        vel_x = parse_s16(data[19 - 3], data[18 - 3])      # vel_x (s16)
        vel_y = parse_s16(data[21 - 3], data[20 - 3])      # vel_y (s16)
        vel_z = parse_s16(data[23 - 3], data[22 - 3])      # vel_z (s16)
        pos_x = parse_s32(data[27 - 3], data[26 - 3], data[25 - 3], data[24 - 3])  # pos_x (s32) 小端
        pos_y = parse_s32(data[31 - 3], data[30 - 3], data[29 - 3], data[28 - 3])  # pos_y (s32) 小端
        voltage_100 = parse_u16(data[31 - 3], data[30 - 3])  # voltage_100 (u16) 小端
        fc_mode_sta = data[34 -3]                     # fc_mode_sta (u8)
        unlock_sta = data[35 - 3]                      # unlock_sta (u8)
        
        # 转换为浮点数
        self.state.rol = rol                    # 单位：度
        self.state.pit = pit 
        self.state.yaw = yaw 
        self.state.alt_fused = alt_fused        # 单位：米
        self.state.alt_add = alt_add 
        self.state.vel_x = vel_x                 # 单位：m/s
        self.state.vel_y = vel_y 
        self.state.vel_z = vel_z 
        self.state.pos_x = pos_x                 # 单位：米
        self.state.pos_y = pos_y 
        self.state.voltage = voltage_100         # 单位：伏特
        self.state.mode_sta = fc_mode_sta
        self.state.unlock_sta = unlock_sta
        #print("[原始数据] 十六进制:", data.hex(' '))  # 打印完整十六进制数据
        # if self._print_data:
            # #  logger.debug(f"姿态: rol={self.state.rol:.2f} pit={self.state.pit:.2f} yaw={self.state.yaw:.2f}")
            #  logger.debug(f"高度: alt_fused={self.state.alt_fused:.2f} alt_add={self.state.alt_add:.2f}")
            #  logger.debug(f"位置: pos_x={self.state.pos_x:.2f} pos_y={self.state.pos_y:.2f} pos_z={self.state.pos_z:.2f}")
            # #  logger.debug(f"速度: vel_x={self.state.vel_x:.2f} vel_y={self.state.vel_y:.2f} vel_z={self.state.vel_z:.2f}")
#        logger.debug(f"电压: {self.state.voltage:.2f}V 模式:{self.state.mode_sta} 解锁:{self.state.unlock_sta}")



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