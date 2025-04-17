# 适用于 Pyserial (Jetson Nano) 的串口通信类 - 优化版 (v2 - 状态机实现)
import serial
import time
import gc

# --- 定义状态常量 ---
STATE_IDLE = 0             # 空闲，等待包头第一个字节
STATE_WAIT_HEADER_2 = 1    # 收到包头第一个字节，等待第二个字节
STATE_WAIT_LEN = 2         # 收到完整包头，等待长度字节
STATE_READING_DATA = 3     # 正在读取数据字节 (Cmd + Payload)
STATE_WAIT_CHECKSUM = 4    # 数据读取完毕，等待校验和字节
# --------------------

class MP_Serial: # Renaming to just Serial might be clearer, but keeping MP_Serial for consistency with other files for now
    def __init__(self, port, baudrate, timeout=0.01): # Removed unused uart_id, tx_pin, rx_pin. Added 'port'.
        """
        初始化串口 (适用于 PySerial)
        :param port: 串口设备路径 (例如: "/dev/ttyTHS1", "/dev/ttyUSB0")
        :param baudrate: 波特率
        :param timeout: 内部读取超时(秒)，影响 ser.read()，但主要用 read_all()
        """
        self.port = port
        self.baudrate = baudrate
        self.read_buffer_size = 256 # Max bytes to read in one go if using read() with size

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout # Set timeout for blocking read operations if used
            )
            # Wait a moment for the serial port to initialize
            time.sleep(0.5) # Reduced sleep time
            if not self.ser.is_open:
                 raise RuntimeError(f"无法打开串口: {self.port}")

        except serial.SerialException as e:
             raise RuntimeError(f"串口 {self.port} 初始化失败: {str(e)}")
        except Exception as e:
            raise RuntimeError(f"串口初始化时发生未知错误: {str(e)}")

        # 状态和缓冲区
        self._state = STATE_IDLE
        self._read_buffer = bytearray() # 用于构建当前正在接收的数据包 (Cmd + Payload)
        self._save_buffer = bytes()     # 保存最后一个完整接收的数据包 (Cmd + Payload)
        self._pack_length = 0           # 当前包的预期数据长度 (Cmd + Payload 的长度)
        self._pack_length_byte = 0      # 原始长度字节，用于校验

        # 协议相关配置
        self.send_start_bit = bytes([0xAA, 0x22])
        self.send_option_bit = bytes([0x00]) # Default send option
        self.read_start_bit = bytes([0xAA, 0x55])

        # 内存管理计数器
        self._gc_counter = 0
        self._gc_collect_interval = 100 # 每处理 N 次 read() 调用 gc.collect()

    def _reset_state(self):
        """重置读取状态和缓冲区"""
        self._state = STATE_IDLE
        self._read_buffer = bytearray()
        self._pack_length = 0
        self._pack_length_byte = 0
        # print("State reset to IDLE") # Debugging

    def _calculate_checksum(self, data_bytes, length_byte):
        """计算接收校验和 (包头 + 长度字节 + 数据[Cmd+Payload])"""
        checksum = 0
        # 计算起始位校验和
        for byte_val in self.read_start_bit:
            checksum = (checksum + byte_val) & 0xFF
        # 添加包长度字节校验
        checksum = (checksum + length_byte) & 0xFF
        # 添加数据(Cmd+Payload)校验
        for byte_val in data_bytes:
            checksum = (checksum + byte_val) & 0xFF
        return checksum

    def send_config(self, startBit=None, optionBit=None):
        """设置发送配置 (包头和选项字节)"""
        if startBit is not None:
            self.send_start_bit = bytes(startBit)
        if optionBit is not None:
            self.send_option_bit = bytes(optionBit) # Store the option byte

    def read_config(self, startBit=None):
        """设置接收配置 (仅包头)"""
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
        try:
            # Read all available data from serial buffer non-blockingly
            if self.ser and self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
            else:
                data = None # No data available
        except serial.SerialException as e:
            print(f"读取串口时出错: {e}") # Or use logger
            # Consider closing the port or attempting re-opening
            self.close() # Example: close on error
            return False
        except Exception as e:
             print(f"读取串口时发生未知错误: {e}")
             return False


        if not data:
            return False # No new data

        # --- 调试用 ---
        # print(f"Raw recv ({len(data)} bytes): {data.hex(' ')}")
        # -------------

        for current_byte in data:
            # --- 状态机逻辑 ---
            if self._state == STATE_IDLE:
                if current_byte == self.read_start_bit[0]:
                    self._state = STATE_WAIT_HEADER_2
            elif self._state == STATE_WAIT_HEADER_2:
                if current_byte == self.read_start_bit[1]:
                    self._state = STATE_WAIT_LEN
                elif current_byte == self.read_start_bit[0]:
                    pass # Still waiting for second byte
                else:
                    self._reset_state() # Incorrect second byte
            elif self._state == STATE_WAIT_LEN:
                self._pack_length = current_byte # This is the length of (Cmd + Payload)
                self._pack_length_byte = current_byte
                # 包长度有效性检查 (Cmd + Payload can be 0 bytes?) Adjust if needed.
                if 0 <= self._pack_length <= (self.read_buffer_size - 10): # Ensure fits buffer reasonably
                    if self._pack_length == 0:
                        self._read_buffer = bytearray()
                        self._state = STATE_WAIT_CHECKSUM
                    else:
                        self._read_buffer = bytearray() # Prepare for data
                        self._state = STATE_READING_DATA
                else:
                    # print(f"Invalid length: {self._pack_length}") # Debugging
                    self._reset_state() # Invalid length
            elif self._state == STATE_READING_DATA:
                self._read_buffer.append(current_byte)
                if len(self._read_buffer) == self._pack_length:
                    self._state = STATE_WAIT_CHECKSUM
            elif self._state == STATE_WAIT_CHECKSUM:
                received_checksum = current_byte
                expected_checksum = self._calculate_checksum(self._read_buffer, self._pack_length_byte)

                # print(f"Checksum: Got {received_checksum}, Expected {expected_checksum}") # Debugging

                if received_checksum == expected_checksum:
                    self._save_buffer = bytes(self._read_buffer) # Save Cmd + Payload
                    packet_received = True
                    # print(f"Packet OK: {self._save_buffer.hex(' ')}") # Debugging
                else:
                    # print("Checksum error!") # Debugging
                    pass # Reset below handles error case

                # Reset state to look for the next packet regardless of checksum result
                self._reset_state()
            # --- 状态机逻辑结束 ---

        # --- 内存管理 ---
        self._gc_counter += 1
        if self._gc_counter >= self._gc_collect_interval:
            gc.collect()
            self._gc_counter = 0
        # ---------------

        return packet_received # Return True if a packet was successfully processed in *this call*

    @property
    def rx_data(self):
        """获取最后一次成功接收到的数据包内容 (Cmd + Payload)"""
        return self._save_buffer

    def close(self):
        """关闭串口"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                # print("Serial port closed.") # Debugging
            self.ser = None
        except Exception as e:
            print(f"关闭串口错误: {str(e)}") # Or use logger

    def write(self, data_payload):
        """向串口写入数据 (构建完整帧并发送)

        Args:
            data_payload: 要发送的数据负载 (bytes, bytearray, list of ints)

        Returns:
            bytes: 发送的完整数据包或 None (如果出错)
        """
        if not self.ser or not self.ser.is_open:
            print("写入错误：串口未打开或未初始化") # Or use logger
            return None

        try:
            # 标准化输入数据为 bytes
            if isinstance(data_payload, list):
                payload = bytes(data_payload)
            elif isinstance(data_payload, bytearray):
                payload = bytes(data_payload)
            elif isinstance(data_payload, bytes):
                payload = data_payload
            else:
                raise TypeError("数据必须是 bytes, bytearray 或 list of ints 类型")

            # 检查数据长度是否超出协议限制（一个字节表示长度）
            if len(payload) > 255:
                raise ValueError("数据负载长度不能超过 255 字节")

            # 构建发送数据帧
            len_as_byte = bytes([len(payload)])
            # Frame: Start Bit (2) + Option Bit (1) + Length (1) + Payload (N)
            send_frame_no_checksum = self.send_start_bit + self.send_option_bit + len_as_byte + payload

            # 计算发送校验和 (校验范围：包头 + 选项位 + 长度 + 数据负载)
            checksum = 0
            for byte_val in send_frame_no_checksum:
                checksum = (checksum + byte_val) & 0xFF

            # 添加校验和并发送
            send_frame = send_frame_no_checksum + bytes([checksum])

            bytes_written = self.ser.write(send_frame)
            self.ser.flush() # Ensure data is sent out

            # 可选：检查是否所有字节都已写入
            if bytes_written != len(send_frame):
                 print(f"警告: 尝试写入 {len(send_frame)} 字节, 但只写入了 {bytes_written} 字节.") # Use logger

            # print(f"Sent: {send_frame.hex(' ')}") # Debugging
            return send_frame

        except serial.SerialException as e:
             print(f"写入串口时出错: {e}") # Use logger
             return None
        except Exception as e:
            print(f"写入数据错误: {str(e)}") # Use logger
            return None

    def any(self):
        """返回串口接收缓冲区中可读取的字节数 (使用 Pyserial 的 in_waiting)。"""
        if not self.ser or not self.ser.is_open:
            return 0
        try:
            return self.ser.in_waiting
        except Exception as e:
            print(f"检查可读字节数错误: {str(e)}") # Use logger
            return 0