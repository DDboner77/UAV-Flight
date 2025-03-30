
# 适用于MicroPython的串口通信类
from machine import UART,FPIOA
import time
import gc

class MP_Serial:
    def __init__(self, uart_id, baudrate, tx_pin=None, rx_pin=None, timeout=0.5):
        """
        初始化串口
        :param uart_id: UART ID (例如：1、2，具体取决于MCU)
        :param baudrate: 波特率
        :param tx_pin: 发送引脚 (可选，取决于板子)
        :param rx_pin: 接收引脚 (可选，取决于板子)
        :param timeout: 超时时间(秒)
        """
        # 初始化串口
        # 复用串口2 11、12引脚
        fpioa = FPIOA()
        fpioa.set_function(11, FPIOA.UART2_TXD)
        fpioa.set_function(12, FPIOA.UART2_RXD)
        self.ser = UART(uart_id, baudrate=baudrate,bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

        self.read_buffer = b''
        self.read_save_buffer = b''
        self.reading_flag = False
        self.pack_count = 0
        self.pack_length = 0
        self.byte_order = 'little'  # MicroPython通常使用little-endian
        self.waiting_buffer = b''
        self.send_start_bit = []  # 发送数据包头
        self.send_option_bit = []  # 发送选项位
        self.read_start_bit = []   # 接收数据包头
        
    def send_config(self, startBit=[], optionBit=[]):
        """设置发送配置"""
        self.send_start_bit = startBit
        self.send_option_bit = optionBit

    def read_config(self, startBit=[]):
        """设置接收配置"""
        self.read_start_bit = startBit

    def check_rx_data_sum(self):
        """校验接收数据的校验和"""
        length = len(self.read_buffer)
        checksum = 0
        for i in self.read_start_bit:
            checksum += i
            checksum &= 0xFF
        
        checksum += self.pack_length_bit
        checksum &= 0xFF
        
        for i in range(0, length):
            checksum += self.read_buffer[i]
            checksum &= 0xFF
            
        # 读取校验和
        received_byte = self.ser.read(1)
        if not received_byte:
            return 0
            
        received_checksum = received_byte[0]  # MicroPython字节访问方式
        
        return 1 if received_checksum == checksum else 0

    def read(self):
        """读取一个完整的数据包"""
        tmp = self.ser.read(1)  # 尝试读取1字节
        if tmp is None:  # 无数据可读
            return False
            
        _len = len(self.read_start_bit)
        
        if not self.reading_flag:
            self.waiting_buffer += tmp

            if len(self.waiting_buffer) > 100:  # 假设最大包头等待长度100
                self.waiting_buffer = b''
                return False

            if len(self.waiting_buffer) >= _len:
                # 检查是否匹配开始位
                if self.waiting_buffer[-_len:] == bytes(self.read_start_bit):
                    self.reading_flag = True
                    self.read_buffer = b''
                    self.pack_count = 0
                    self.pack_length = -1
                    self.waiting_buffer = b''
            return False  # 还在等待包头
            
        if self.pack_length == -1:
            # 读取包长度
            self.pack_length_bit = tmp[0]
            self.pack_length = self.pack_length_bit & 0xFF
            return False  # 还在读取包长度
            
        # 读取数据
        self.pack_count += 1
        self.read_buffer += tmp
        
        if self.pack_count >= self.pack_length:
            self.reading_flag = False
            if self.check_rx_data_sum():
                # 拷贝数据
                self.read_save_buffer = self.read_buffer
                self.read_buffer = b''
                return True
            else:
                self.read_buffer = b''
                return False
        return False

    @property
    def rx_data(self):
        """获取接收到的数据"""
        return self.read_save_buffer

    def close(self):
        """关闭串口"""
        # MicroPython的UART没有close方法，可以通过deinit()释放
        self.ser.deinit()
        self.ser = None

    def write(self, data):
        """向串口写入数据"""
        if isinstance(data, list):
            data = bytes(data)
        if not isinstance(data, bytes):
            raise TypeError("数据必须是字节类型")
            
        # 构建发送数据
        len_as_byte = bytes([len(data)])
        send_data = bytes(self.send_start_bit) + bytes(self.send_option_bit) + len_as_byte + data
        
        # 计算校验和
        checksum = 0
        for i in range(len(send_data)):
            checksum += send_data[i]
            checksum &= 0xFF
            
        # 添加校验和
        send_data += bytes([checksum])
        
        # 发送数据
        self.ser.write(send_data)
        
        return send_data
        
    def any(self):
        """返回可读取的字节数"""
        return self.ser.read()
        

# def main():
#     serial = MP_Serial(UART.UART2, 500000 , tx_pin=11, rx_pin=12)

#     # 配置发送和接收参数
#     serial.send_config(startBit=[0xAA, 0x22], optionBit=[0x00])
#     serial.read_config(startBit=[0xAA, 0x55])

#     # 发送数据
#     serial.write(b'\x01')  # 发送心跳包

#     # 添加启动时间戳
#     print(f"程序启动时间: {time.time()}")

#     # 记录上次打印时间，定期打印保持活动消息
#     last_heartbeat = time.time()
#     heartbeat_interval = 5  # 每5秒打印一次心跳信息
#     last_probe_time = time.time()
#     probe_interval = 1

#     while True:
#         current_time = time.time()
        
#         if current_time - last_probe_time >= probe_interval:
#             try:
#                 probe_data = b'\x01'  # 简单的探测数据
#                 send_result = serial.write(probe_data)
#                 print(f"[{current_time}] 发送探测包: {send_result}")
#                 last_probe_time = current_time  # 更新上次发送时间
#             except Exception as e:
#                 print(f"[{current_time}] 发送探测包失败: {e}")

#         if serial.read():
#             data = serial.rx_data
#             print(f"[{current_time}] 收到数据:", [hex(b) for b in data])
#             last_heartbeat = current_time
        
#         # 即使没有数据收到，也定期打印心跳信息
#         if current_time - last_heartbeat >= heartbeat_interval:
#             print(f"[{current_time}] 系统正在运行，等待数据...")
#             last_heartbeat = current_time
        
#         time.sleep(0.001)

# if __name__ == "__main__":
#     main()