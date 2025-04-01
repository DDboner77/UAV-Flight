# 适用于MicroPython的串口通信类 - 优化版 (v2 - 状态机实现)
from machine import UART, FPIOA
import time
import gc

# --- 定义状态常量 ---
STATE_IDLE = 0             # 空闲，等待包头第一个字节
STATE_WAIT_HEADER_2 = 1    # 收到包头第一个字节，等待第二个字节
STATE_WAIT_LEN = 2         # 收到完整包头，等待长度字节
STATE_READING_DATA = 3     # 正在读取数据字节
STATE_WAIT_CHECKSUM = 4    # 数据读取完毕，等待校验和字节
# --------------------

class MP_Serial:
    def __init__(self, uart_id, baudrate, tx_pin=None, rx_pin=None, timeout=0.01): # 降低默认超时，但主要依赖非阻塞读取
        """
        初始化串口
        :param uart_id: UART ID (例如：1、2，具体取决于MCU)
        :param baudrate: 波特率
        :param tx_pin: 发送引脚 (可选，取决于板子)
        :param rx_pin: 接收引脚 (可选，取决于板子)
        :param timeout: UART读取超时(秒)，设置较小值，主要用于ser.read()
        """
        # 初始化串口
        self.uart_id = uart_id
        self.baudrate = baudrate
        self.read_buffer_size = 256
        # 注意：UART的timeout参数影响read()的行为，但我们主要用非阻塞方式
        # 如果read()不带参数，timeout可能无效或行为不同，具体看MicroPython实现
        # 我们主要依赖ser.any()和ser.read()来获取数据

        # 只有当提供了引脚信息时才进行引脚映射
        if tx_pin is not None and rx_pin is not None:
            try:
                # FPIOA 可能不存在于所有 MicroPython 移植版上
                fpioa = FPIOA()
                # 使用传入的引脚参数而不是硬编码
                if uart_id == UART.UART2:
                    fpioa.set_function(tx_pin, FPIOA.UART2_TXD)
                    fpioa.set_function(rx_pin, FPIOA.UART2_RXD)
                elif uart_id == UART.UART1:
                    fpioa.set_function(tx_pin, FPIOA.UART1_TXD)
                    fpioa.set_function(rx_pin, FPIOA.UART1_RXD)
                # 可以添加其他UART支持
                else:
                    print("警告: FPIOA 未在此 MicroPython 移植版上找到，跳过引脚映射。请确保引脚已正确配置。")
            except Exception as e:
                raise RuntimeError(f"引脚映射失败: {str(e)}")

        try:
            # 设置 read_buf_len 以便 ser.any() 更准确，并可能提高 read() 效率
            # 这个大小需要根据预期的数据包大小和频率进行调整
            # 如果不确定，可以不设置或设置一个合理的值（如 256 或 512）

            self.ser = UART(
                uart_id,
                baudrate=baudrate,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE,
            )
        except Exception as e:
            raise RuntimeError(f"串口初始化失败: {str(e)}")

        # 状态和缓冲区
        self._state = STATE_IDLE
        self._read_buffer = bytearray() # 用于构建当前正在接收的数据包
        self._save_buffer = bytes()     # 保存最后一个完整接收的数据包
        self._pack_length = 0           # 当前包的预期数据长度
        self._pack_length_byte = 0      # 原始长度字节，用于校验

        # 协议相关配置 (保持不变)
        self.send_start_bit = bytes([0xAA, 0x22])  # 使用 bytes 类型
        self.send_option_bit = bytes([0x00])       # 使用 bytes 类型
        self.read_start_bit = bytes([0xAA, 0x55])  # 使用 bytes 类型

        # 内存管理计数器
        self._gc_counter = 0
        self._gc_collect_interval = 100 # 每处理 N 次 read() 调用 gc.collect()

    def _reset_state(self):
        """重置读取状态和缓冲区"""
        self._state = STATE_IDLE
        self._read_buffer = bytearray() # 清空或重新分配
        self._pack_length = 0
        self._pack_length_byte = 0
        # print("State reset to IDLE") # Debugging

    def _calculate_checksum(self, data_bytes, length_byte):
        """计算校验和 (包含包头、选项位(无)、长度、数据)"""
        checksum = 0
        # 计算起始位校验和
        for byte_val in self.read_start_bit:
            checksum = (checksum + byte_val) & 0xFF
        # 添加包长度字节校验
        checksum = (checksum + length_byte) & 0xFF
        # 添加数据校验
        for byte_val in data_bytes:
            checksum = (checksum + byte_val) & 0xFF
        return checksum

    def send_config(self, startBit=None, optionBit=None):
        """设置发送配置"""
        if startBit is not None:
            self.send_start_bit = bytes(startBit)
        if optionBit is not None:
            self.send_option_bit = bytes(optionBit)

    def read_config(self, startBit=None):
        """设置接收配置"""
        if startBit is not None:
            self.read_start_bit = bytes(startBit)

    def read(self):
        """
        非阻塞读取和解析串口数据（状态机版本）。
        每次调用处理串口缓冲区中的所有可用数据。
        :return: bool - 如果本次调用成功接收并校验了一个完整的数据包，则返回 True，否则返回 False。
                 成功接收的数据可通过 self.rx_data 获取。
        """
        
        packet_received = False
        data = self.ser.read(self.read_buffer_size) # 读取所有可用数据，非阻塞

        if not data:
            # 没有新数据，直接返回
            return False

        # # --- 调试用：打印接收到的原始数据 ---
        # print(f"Raw recv: {bytes(data)}")
        # # ------------------------------------

        for current_byte in data:
            # --- 状态机逻辑 ---
            if self._state == STATE_IDLE:
                if current_byte == self.read_start_bit[0]:
                    self._state = STATE_WAIT_HEADER_2
                    # print("State -> WAIT_HEADER_2") # Debugging
                # else: 忽略不在包头的字节

            elif self._state == STATE_WAIT_HEADER_2:
                if current_byte == self.read_start_bit[1]:
                    self._state = STATE_WAIT_LEN
                    # print("State -> WAIT_LEN") # Debugging
                elif current_byte == self.read_start_bit[0]:
                    # 接收到连续的第一个包头字节，保持状态
                    pass
                else:
                    # 第二个字节错误，重置状态
                    self._reset_state()

            elif self._state == STATE_WAIT_LEN:
                self._pack_length = current_byte
                self._pack_length_byte = current_byte # 保存原始长度字节

                # 包长度有效性检查 (假设数据长度至少为0，如果协议要求至少为1，则改为 >= 1)
                # 注意：这里允许长度为0的数据包，如果不需要，修改为 > 0
                if 0 <= self._pack_length <= 255: # 根据实际协议调整范围
                     # 如果长度为0，直接进入等待校验和状态
                    if self._pack_length == 0:
                        self._read_buffer = bytearray() # 确保是空的
                        self._state = STATE_WAIT_CHECKSUM
                        # print("State -> WAIT_CHECKSUM (len 0)") # Debugging
                    else:
                        self._read_buffer = bytearray() # 准备接收数据
                        self._state = STATE_READING_DATA
                        # print(f"State -> READING_DATA (len={self._pack_length})") # Debugging
                else:
                    # 无效长度，重置状态
                    # print(f"Invalid length: {self._pack_length}") # Debugging
                    self._reset_state()

            elif self._state == STATE_READING_DATA:
                self._read_buffer.append(current_byte)
                # 检查是否已接收足够的数据字节
                if len(self._read_buffer) == self._pack_length:
                    self._state = STATE_WAIT_CHECKSUM
                    # print("State -> WAIT_CHECKSUM") # Debugging

            elif self._state == STATE_WAIT_CHECKSUM:
                received_checksum = current_byte
                # 计算期望的校验和
                expected_checksum = self._calculate_checksum(self._read_buffer, self._pack_length_byte)

                # print(f"Checksum: Got {received_checksum}, Expected {expected_checksum}") # Debugging

                if received_checksum == expected_checksum:
                    # 校验成功，保存数据
                    self._save_buffer = bytes(self._read_buffer) # 创建不可变副本
                    packet_received = True
                    # print(f"Packet OK: {self._save_buffer}") # Debugging
                else:
                    # 校验失败
                    # print("Checksum error!") # Debugging
                    pass # _reset_state 会处理

                # 无论校验成功与否，都重置状态以寻找下一个包
                self._reset_state()

                # 如果校验成功，我们已经完成了一个包，可以提前退出循环吗？
                # 不行，因为可能在一个 read() 调用中接收到多个包或一个包的末尾和下一个包的开头。
                # 需要继续处理缓冲区中剩余的字节。

            # --- 状态机逻辑结束 ---

        # --- 内存管理 ---
        self._gc_counter += 1
        if self._gc_counter >= self._gc_collect_interval:
            gc.collect()
            self._gc_counter = 0
        # ---------------

        return packet_received # 返回本次调用是否成功接收了一个包

    @property
    def rx_data(self):
        """获取最后一次成功接收到的数据包内容 (不包括头、长度、校验和)"""
        return self._save_buffer

    def close(self):
        """关闭串口"""
        try:
            if self.ser:
                self.ser.deinit()
                self.ser = None
                # print("Serial port closed.") # Debugging
        except Exception as e:
            print(f"关闭串口错误: {str(e)}")

    def write(self, data):
        """向串口写入数据 (保持不变，但确保类型正确)

        Args:
            data: 要发送的数据(bytes、bytearray 或 list of ints)

        Returns:
            bytes: 发送的完整数据包（包括头部、选项位、长度、数据和校验和）或 None（如果出错）
        """
        try:
            # 标准化输入数据为 bytes
            if isinstance(data, list):
                payload = bytes(data)
            elif isinstance(data, bytearray):
                payload = bytes(data)
            elif isinstance(data, bytes):
                payload = data
            else:
                raise TypeError("数据必须是 bytes, bytearray 或 list of ints 类型")

            # 检查数据长度是否超出协议限制（一个字节表示长度）
            if len(payload) > 255:
                raise ValueError("数据长度不能超过 255 字节")

            # 构建发送数据
            len_as_byte = bytes([len(payload)])
            # 注意：发送协议可能包含 option_bit，这里包含了
            send_frame_no_checksum = self.send_start_bit + self.send_option_bit + len_as_byte + payload

            # 计算校验和 (校验范围：包头 + 选项位 + 长度 + 数据)
            checksum = 0
            for byte_val in send_frame_no_checksum:
                checksum = (checksum + byte_val) & 0xFF

            # 添加校验和并发送
            send_frame = send_frame_no_checksum + bytes([checksum])
            bytes_written = self.ser.write(send_frame)

            # 可选：检查是否所有字节都已写入 (write 可能不是阻塞的)
            if bytes_written != len(send_frame):
                 print(f"警告: 尝试写入 {len(send_frame)} 字节, 但只写入了 {bytes_written} 字节.")
                 # 可以考虑重试或抛出错误

            # print(f"Sent: {send_frame}") # Debugging
            return send_frame

        except Exception as e:
            print(f"写入数据错误: {str(e)}")
            return None

    def any(self):
        """
        返回串口接收缓冲区中可读取的字节数。
        注意：在某些 MicroPython 移植版上，ser.any() 可能不准确或行为不同。
        ser.read() 不带参数通常是更好的非阻塞读取方式。
        此方法保留主要是为了兼容性或特定场景。
        """
        try:
            # 优先使用 UART.any() 方法（如果存在且可靠）
            if hasattr(self.ser, 'any'):
                return self.ser.any()
            else:
                # 备选方案：尝试非阻塞读取并返回长度，但这会消耗数据！
                # 不推荐这种方式替代 any()
                # data = self.ser.read(0) # 尝试读取0字节，看是否返回缓冲区信息（不标准）
                # return len(data) if data is not None else 0
                # 更安全的备选是返回 0 或引发错误，表示不支持
                print("警告: UART.any() 不可用，无法准确检查可读字节数。")
                return 0
        except Exception as e:
            print(f"检查可读字节数错误: {str(e)}")
            return 0

# --- 使用示例 ---
if __name__ == '__main__':
    # ！！！注意：以下引脚和 UART ID 需要根据你的具体硬件修改 ！！！
    # 例如 K210, ESP32, Raspberry Pi Pico 等开发板的引脚定义不同
    # K210 Maix Dock/Bit 示例:
    # TX_PIN = 10 # 假设连接到 UART2 TX
    # RX_PIN = 11 # 假设连接到 UART2 RX
    # UART_ID = UART.UART2

    # ESP32 示例:
    # TX_PIN = 17
    # RX_PIN = 16
    # UART_ID = 2 # ESP32 UART ID 通常是数字 1 或 2

    # Raspberry Pi Pico 示例:
    # TX_PIN = 0 # GP0
    # RX_PIN = 1 # GP1
    # UART_ID = 0 # Pico UART ID 是 0 或 1

    # --- 请修改以下参数以匹配你的硬件 ---
    TARGET_TX_PIN = 10  # 修改为你的发送引脚
    TARGET_RX_PIN = 11  # 修改为你的接收引脚
    TARGET_UART_ID = UART.UART2 # 修改为你的 UART ID
    TARGET_BAUDRATE = 115200
    # --------------------------------------

    print(f"初始化 UART {TARGET_UART_ID} (TX:{TARGET_TX_PIN}, RX:{TARGET_RX_PIN}) @ {TARGET_BAUDRATE}...")

    try:
        # 如果你的板子不需要 FPIOA 引脚映射 (如 ESP32, Pico)，可以将 tx_pin 和 rx_pin 设为 None
        # serial_comm = MP_Serial(TARGET_UART_ID, TARGET_BAUDRATE) # 无需引脚映射的示例
        serial_comm = MP_Serial(TARGET_UART_ID, TARGET_BAUDRATE, TARGET_TX_PIN, TARGET_RX_PIN)

        print("串口初始化成功.")

        # 配置自定义的接收头 (如果需要)
        # serial_comm.read_config(startBit=[0xBB, 0x66])

        # 配置自定义的发送头和选项位 (如果需要)
        # serial_comm.send_config(startBit=[0xCC, 0x33], optionBit=[0x01])

        print("开始循环读取和发送...")
        last_send_time = time.time()

        while True:
            # === 读取数据 ===
            # read() 现在是非阻塞的，可以在循环中频繁调用
            packet_found = serial_comm.read()

            if packet_found:
                received_data = serial_comm.rx_data
                print(f"[{time.time():.2f}] 收到数据包: {received_data} (长度: {len(received_data)})")
                # 在这里处理接收到的数据包...

            # === 发送数据 (示例：每秒发送一次) ===
            current_time = time.time()
            if current_time - last_send_time >= 1.0:
                test_data_to_send = list(range(5)) # 发送 [0, 1, 2, 3, 4]
                print(f"[{time.time():.2f}] 发送数据: {test_data_to_send}")
                sent_frame = serial_comm.write(test_data_to_send)
                if sent_frame:
                    print(f"    -> 完整帧: {sent_frame}")
                else:
                    print("    -> 发送失败!")
                last_send_time = current_time

            # === 短暂延时 ===
            # 减少 CPU 占用，根据需要调整延时
            # 10ms 对应 0.01s
            time.sleep(0.01) # 控制循环频率接近 10ms

    except RuntimeError as e:
        print(f"运行时错误: {e}")
    except KeyboardInterrupt:
        print("用户中断")
    finally:
        if 'serial_comm' in locals() and serial_comm.ser is not None:
            serial_comm.close()
            print("串口已关闭。")
        gc.collect()