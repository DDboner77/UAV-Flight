# 简化日志功能 (保持不变)
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
from Serial import MP_Serial # Class name kept as MP_Serial for now
import time
import struct # Import struct here as it's used in parse_data_packet
import threading

# FC状态结构体 (保持不变)
class FC_State_Struct:
    """飞控状态结构体，简化版"""
    def __init__(self):
        self.connected = False
        self.rol = 0.0
        self.pit = 0.0
        self.yaw = 0.0
        self.alt_fused = 0.0
        self.alt_add = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        # self.pos_z = 0.0 # Original C struct seems to miss pos_z? Included it based on comment. Check protocol.
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.voltage = 0.0
        self.mode_sta = 0
        self.auto_change_mode = False # Defaulting to False, seems safer
        self.unlock_sta = False

# 飞控通信基础实现 (Revised __init__, start, poll, removed main)
class FC_Base_Uart_Communication:
    def __init__(self, serial_instance: MP_Serial): # Correctly typed parameter
        """初始化通信"""
        self.state = FC_State_Struct()
        self.connected = False
        self.serial = serial_instance
        if not isinstance(self.serial, MP_Serial): # Use the class name used in Serial.py
            raise TypeError("serial_instance must be an instance of MP_Serial")
        self.callback = None
        self._running = False
        self._print_data = False # Controls detailed debug printing
        self.last_heartbeat_time = 0
        self.heartbeat_interval = 0.25 # Seconds
        # ACK related
        self._waiting_ack = False
        self._received_ack_val = None # Store received ACK value
        self._expected_ack_val = None # Store expected ACK value
        self._ack_send_time = 0
        self.ack_timeout = 1.0 # Seconds
        self.max_retry_count = 3
        # poll 轮询
        self._poll_thread = None
        self.poll_interval = 0.01  # 10ms轮询间隔

    def send_data_to_fc(self, data_payload, option, need_ack=False, retry_count=None):
        """发送数据到飞控 (Payload only, option byte is handled here)

        Args:
            data_payload: 要发送的数据负载
            option: 选项/命令类型字节 (e.g., 0x01 for command, 0x02 for IMU frame)
            need_ack: 是否需要ACK确认
            retry_count: 当前重试计数 (内部使用)

        Returns:
            发送的完整帧(bytes)或None(失败时)
        """
        if not self.serial:
            logger.error("发送失败：串口未初始化")
            return None

        current_retry = retry_count if retry_count is not None else self.max_retry_count

        if current_retry < 0:
            logger.error(f"发送数据(Option: {hex(option)})达到最大重试次数")
            self._waiting_ack = False # Ensure waiting flag is cleared
            return None

        # Prepare for ACK if needed
        if need_ack:
            # Calculate expected checksum for ACK (option + payload bytes)
            self._expected_ack_val = option
            for b in data_payload:
                self._expected_ack_val = (self._expected_ack_val + b) & 0xFF

            self._received_ack_val = None # Clear previous received value
            self._waiting_ack = True
            self._ack_send_time = time.time()
            # logger.debug(f"Expecting ACK: {hex(self._expected_ack_val)}")

        # Set the option byte for the serial writer
        self.serial.send_config(optionBit=[option])

        # Send the payload using the serial writer
        # The serial writer will add header, option, length, payload, checksum
        sent_frame = self.serial.write(data_payload)

        if sent_frame is None:
            logger.error(f"发送数据失败 (Option: {hex(option)}) - 串口写入失败")
            self._waiting_ack = False # Clear waiting flag on send failure
            # Optionally retry immediately? Or let the caller handle it.
            # For now, just return None.
            return None

        if self._print_data:
            logger.debug(f"发送: {sent_frame.hex(' ')}")

        # If ACK needed, wait for it (poll driven)
        if need_ack:
            while self._waiting_ack:
                # We need to process incoming data to receive the ACK
                self.poll() # Process reads

                # Check if ACK was received and processed by poll()
                if not self._waiting_ack:
                    break # ACK received (or failed internally)

                # Check for timeout
                if time.time() - self._ack_send_time > self.ack_timeout:
                    logger.warning(f"等待ACK超时 (Expected: {hex(self._expected_ack_val)}), 重试 (剩余 {current_retry-1})...")
                    self._waiting_ack = False # Stop waiting for this attempt
                    # Recursive call for retry
                    return self.send_data_to_fc(data_payload, option, need_ack, current_retry - 1)

                # Small delay to prevent busy-waiting
                time.sleep(0.005) # 5ms

            # After loop: Check if ACK was successful ( _waiting_ack is False )
            if self._received_ack_val is None or self._received_ack_val != self._expected_ack_val:
                # This case should ideally be caught by timeout or retry logic
                logger.warning(f"ACK 验证失败 (Got: {self._received_ack_val}, Expected: {self._expected_ack_val}). 可能已重试.")
                # Do not retry here again, the loop should have handled it. Return failure.
                return None
            else:
                # logger.debug(f"ACK 成功接收: {hex(self._received_ack_val)}")
                pass

        return sent_frame # Return the actually sent frame if successful

    def process_ack(self, ack_byte_val):
        """处理接收到的ACK字节 (由 poll 调用)"""
        if self._waiting_ack:
            self._received_ack_val = ack_byte_val
            if self._received_ack_val == self._expected_ack_val:
                 # logger.debug(f"ACK Correct: {hex(ack_byte_val)}")
                 self._waiting_ack = False # Stop waiting, ACK received successfully
            else:
                 logger.warning(f"收到错误的 ACK: {hex(ack_byte_val)}, 期望: {hex(self._expected_ack_val)}")
                 # Keep waiting? Or maybe trigger retry immediately?
                 # Current logic relies on timeout for retry.
                 # Let's clear the received value so timeout retries cleanly.
                 self._received_ack_val = None
                 # Do NOT set _waiting_ack = False here, let timeout handle retry


    def start(self, print_state=False, callback=None):
        """启动通信处理"""
        if not self.serial:
            logger.error("串口实例未在初始化时提供。")
            return False
        self._print_data = print_state
        self.callback = callback
        self._running = True
        self.last_heartbeat_time = time.time()

        # 创建并启动轮询线程
        self._poll_thread = threading.Thread(target=self._poll_task, daemon=True)
        self._poll_thread.start()

        self.state.connected = False # Reset connection status on start
        logger.info("飞控通信已启动。")
        return True

    def poll(self):
        """轮询处理，需要在主循环中频繁调用"""
        if not self._running or not self.serial:
            return

        # self.send_heartbeat() # Optional: Send heartbeat if needed
        # --- 处理接收 ---
        try:
            # Call MP_Serial's read method, it handles the state machine
            # read() returns True if a complete packet was processed in this call
            packet_received = self.serial.read()

            if packet_received:
                # Get the Cmd + Payload part
                raw_data = self.serial.rx_data

                if not raw_data: # Should not happen if packet_received is True, but safety check
                    return

                # First byte is command, rest is payload
                cmd = raw_data[0]
                payload = raw_data[1:]

                if self._print_data:
                    logger.debug(f"接收到帧: Cmd={hex(cmd)}, Payload Len={len(payload)}, Data={payload.hex(' ')}")

                # --- 根据命令类型处理数据 ---
                if cmd == 0x01: # 飞控状态数据 (Status)
                    if self.parse_data_packet(payload): # Pass only payload
                        # Successfully parsed status, update connection state
                        if not self.state.connected:
                             self.state.connected = True
                             logger.info("已连接到飞控 (收到有效状态包)")
                        # Reset heartbeat timer on receiving valid status? Optional.
                        # self.last_heartbeat_time = time.time()
                    else:
                         # Parsing failed (e.g., wrong length)
                        logger.warning(f"解析状态包失败 (Payload Len: {len(payload)})")

                elif cmd == 0x02: # ACK 返回
                    if len(payload) > 0:
                        self.process_ack(payload[0]) # Process the ACK byte value
                    else:
                        logger.warning("收到空的ACK包")

                # Add elif for other expected command bytes from FC if any

                else: # 未知命令
                    logger.warning(f"收到未知命令: {hex(cmd)}, Payload: {payload.hex(' ')}")

                # 调用回调 (如果设置) - pass raw Cmd + Payload
                if self.callback:
                    try:
                        self.callback(cmd, payload)
                    except Exception as e:
                        logger.error(f"执行回调函数时出错: {e}")

        except Exception as e:
            logger.error(f"串口轮询或数据处理异常: {e}")
            # Consider adding logic to handle persistent serial port errors,
            # e.g., attempting to reopen or setting self._running = False


        # --- 处理发送 (例如心跳) ---
        current_time = time.time()
        # Send heartbeat periodically only if considered connected
        if  (current_time - self.last_heartbeat_time >= self.heartbeat_interval):
             # logger.debug("Sending heartbeat...") # Debug
             self.send_heartbeat() # Heartbeat doesn't usually need ACK
             self.last_heartbeat_time = current_time

    def _poll_task(self):
        """轮询任务，作为线程运行"""
        while self._running:
            self.poll()  # 执行常规轮询
            time.sleep(self.poll_interval)  # 按照设定的间隔休眠
            
    def send_heartbeat(self):
        """发送心跳包 (Option=0x00, Payload=0x01)"""
        # Heartbeat typically doesn't require an ACK response
        # Option byte 0x00, Payload is single byte 0x01
        return self.send_data_to_fc(data_payload=b'\x01', option=0x00, need_ack=False)


    def set_heartbeat_interval(self, interval):
        """设置心跳包发送间隔 (秒)"""
        self.heartbeat_interval = interval
        logger.info(f"心跳包发送间隔已设置为 {interval} 秒")

    def quit(self):
        """退出通信"""
        self._running = False
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None
        logger.info("通信已关闭")


    def parse_data_packet(self, payload_data):
        """解析飞控发送的状态数据包的 Payload 部分
           Assumes payload_data is exactly 35 bytes long.
        """
        EXPECTED_PAYLOAD_LEN = 35
        if len(payload_data) != EXPECTED_PAYLOAD_LEN:
            # logger.warning(f"解析错误: 状态数据Payload长度不符 ({len(payload_data)} != {EXPECTED_PAYLOAD_LEN})")
            return False # Indicate parsing failure

        try:
            # --- Parsing functions ---
            # 解析16位有符号整数(小端序), scaling applied
            def parse_s16_scaled(byte_list, index, scale=100.0):
                value = struct.unpack('<h', byte_list[index:index+2])[0]
                return value / scale

            # 解析32位有符号整数(小端序), scaling applied
            def parse_s32_scaled(byte_list, index, scale=100.0):
                value = struct.unpack('<i', byte_list[index:index+4])[0]
                return value / scale

            # 解析16位无符号整数(小端序), scaling applied
            def parse_u16_scaled(byte_list, index, scale=100.0):
                value = struct.unpack('<H', byte_list[index:index+2])[0]
                return value / scale
            # --- End Parsing functions ---

            # Parse according to the C structure offsets within the 35-byte payload
            offset = 0
            rol = parse_s16_scaled(payload_data, offset) # 0, 1
            offset += 2
            pit = parse_s16_scaled(payload_data, offset) # 2, 3
            offset += 2
            yaw = parse_s16_scaled(payload_data, offset) # 4, 5
            offset += 2

            alt_fused = parse_s32_scaled(payload_data, offset) # 6, 7, 8, 9
            offset += 4
            alt_add = parse_s32_scaled(payload_data, offset) # 10, 11, 12, 13
            offset += 4

            # Velocities are often cm/s in FC, but protocol description says m/s here after /100. Verify unit.
            vel_x = parse_s16_scaled(payload_data, offset) # 14, 15
            offset += 2
            vel_y = parse_s16_scaled(payload_data, offset) # 16, 17
            offset += 2
            vel_z = parse_s16_scaled(payload_data, offset) # 18, 19
            offset += 2

            # Positions often cm in FC, but protocol description says m here after /100. Verify unit.
            pos_x = parse_s32_scaled(payload_data, offset) # 20, 21, 22, 23
            offset += 4
            pos_y = parse_s32_scaled(payload_data, offset) # 24, 25, 26, 27
            offset += 4

            voltage = parse_u16_scaled(payload_data, offset) # 28, 29
            offset += 2

            # FC status bytes (u8)
            fc_mode_sta = payload_data[offset] # 30
            offset += 1
            unlock_sta_byte = payload_data[offset] # 31
            offset += 1
            cid = payload_data[offset] # 32 (Command ID being executed?)
            offset += 1
            cmd_0 = payload_data[offset] # 33 (CMD0 being executed?)
            offset += 1
            cmd_1 = payload_data[offset] # 34 (CMD1 being executed?)
            # offset += 1 # End of 35 bytes

            # --- Update state object ---
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
            self.state.unlock_sta = bool(unlock_sta_byte) # Convert to boolean
            # Optional: store CID/CMD0/CMD1 if needed
            # self.state.cid = cid
            # self.state.cmd_0 = cmd_0
            # self.state.cmd_1 = cmd_1

            # --- Optional: Print parsed data ---
            if self._print_data:
                 print(f"  Parsed State: RPY={rol:.1f},{pit:.1f},{yaw:.1f} | Alt(F/A)={alt_fused:.2f}/{alt_add:.2f} | Pos(X/Y)={pos_x:.2f}/{pos_y:.2f} | Vel(X/Y/Z)={vel_x:.1f}/{vel_y:.1f}/{vel_z:.1f} | V={voltage:.2f} | Mode={fc_mode_sta} | Lock={self.state.unlock_sta}")

            return True # Indicate successful parsing

        except struct.error as e:
            logger.error(f"解析状态数据时发生struct错误: {e} (Payload Len: {len(payload_data)})")
            return False
        except IndexError as e:
            logger.error(f"解析状态数据时发生索引错误: {e} (Payload Len: {len(payload_data)})")
            return False
        except Exception as e:
            logger.error(f"解析状态数据时发生未知错误: {e}")
            return False

# Removed the __main__ block from Base.py