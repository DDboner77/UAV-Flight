from machine import UART, FPIOA
import time
from binascii import hexlify

# 配置UART2引脚
fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)

# 初始化UART2，波特率需与设备端一致
uart = UART(UART.UART2, baudrate=500000)

# 心跳包格式: AA 22 00 01 01 [CheckSum]
heartbeat = bytearray([0xAA, 0x22, 0x00, 0x01, 0x01])
heartbeat.append(sum(heartbeat) & 0xFF)  # 计算校验和

# 数据接收缓冲区
recv_buffer = bytearray()

def parse_data_packet(data):
    """解析40字节的状态数据包"""
    if len(data) != 40:
        print("ERR: 数据长度不符")
        return None
    
    # 校验和验证
    checksum = sum(data[:-1]) & 0xFF
    if checksum != data[-1]:
        print(f"ERR: 校验和错误 预期:{checksum:02X} 实际:{data[-1]:02X}")
        return None
    
    # 解析结构化数据（参考_to_user_st）
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
    rol = parse_s16(data[4], data[5])          # rol_x100 (s16)
    pit = parse_s16(data[6], data[7])          # pit_x100 (s16)
    yaw = parse_s16(data[8], data[9])          # yaw_x100 (s16)
    alt_fused = parse_s32(data[13], data[12], data[11], data[10])  # alt_fused (s32) 小端
    alt_add = parse_s32(data[17], data[16], data[15], data[14])    # alt_add (s32) 小端
    vel_x = parse_s16(data[18], data[19])      # vel_x (s16)
    vel_y = parse_s16(data[20], data[21])      # vel_y (s16)
    vel_z = parse_s16(data[22], data[23])      # vel_z (s16)
    pos_x = parse_s32(data[27], data[26], data[25], data[24])  # pos_x (s32) 小端
    pos_y = parse_s32(data[31], data[30], data[29], data[28])  # pos_y (s32) 小端
    voltage_100 = parse_u16(data[30], data[31])  # voltage_100 (u16) 小端
    fc_mode_sta = data[34]                     # fc_mode_sta (u8)
    unlock_sta = data[35]                      # unlock_sta (u8)
    
    # 转换为浮点数
    rol_float = rol / 100.0                    # 单位：度
    pit_float = pit / 100.0
    yaw_float = yaw / 100.0
    alt_fused_float = alt_fused / 1000.0       # 单位：米
    alt_add_float = alt_add / 1000.0
    vel_x_float = vel_x / 100.0                # 单位：m/s
    vel_y_float = vel_y / 100.0
    vel_z_float = vel_z / 100.0
    pos_x_float = pos_x / 100.0                # 单位：米
    pos_y_float = pos_y / 100.0
    voltage_float = voltage_100         # 单位：伏特
    
    return {
        'rol': rol_float,
        'pit': pit_float,
        'yaw': yaw_float,
        'alt_fused': alt_fused_float,
        'alt_add': alt_add_float,
        'vel_x': vel_x_float,
        'vel_y': vel_y_float,
        'vel_z': vel_z_float,
        'pos_x': pos_x_float,
        'pos_y': pos_y_float,
        'voltage': voltage_float,
        'fc_mode_sta': fc_mode_sta,
        'unlock_sta': unlock_sta
    }

# 主循环
last_receive_time = time.time()
timeout = 0.5  # 超时时间（秒）

while True:
    # 1. 发送心跳包（每秒1次）
    uart.write(heartbeat)
    print("Sent heartbeat:", bytes(heartbeat).hex())
    
    # 2. 接收数据（非阻塞）
    data = uart.read()
    if data:
        recv_buffer.extend(data)
        print("Raw received:", data.hex())
        last_receive_time = time.time()
        
        # 3. 尝试解析完整数据包（40字节）
        while len(recv_buffer) >= 40:
            # 查找包头0xAA 0x55
            start = -1
            for i in range(len(recv_buffer) - 1):
                if recv_buffer[i] == 0xAA and recv_buffer[i+1] == 0x55:
                    start = i
                    break
            if start == -1:
                print("未找到包头")
                recv_buffer.clear()
                break
            # 检查长度是否足够
            if len(recv_buffer) < start + 40:
                print("数据长度不足")
                break
            # 提取数据包
            packet = recv_buffer[start:start+40]
            parsed = parse_data_packet(packet)
            if parsed:
                print("解析成功:")
                print(f"  姿态 (rol): {parsed['rol']:.2f} 度")
                print(f"  姿态 (pit): {parsed['pit']:.2f} 度")
                print(f"  姿态 (yaw): {parsed['yaw']:.2f} 度")
                print(f"  高度 (alt_fused): {parsed['alt_fused']:.3f} 米")
                print(f"  高度 (alt_add): {parsed['alt_add']:.3f} 米")
                print(f"  速度 (vel_x): {parsed['vel_x']:.2f} cm/s")
                print(f"  速度 (vel_y): {parsed['vel_y']:.2f} cm/s")
                print(f"  速度 (vel_z): {parsed['vel_z']:.2f} cm/s")
                print(f"  位置 (pos_x): {parsed['pos_x']:.2f} c米")
                print(f"  位置 (pos_y): {parsed['pos_y']:.2f} c米")
                print(f"  电压: {parsed['voltage']:.2f} V")
            # 清除已处理数据
            recv_buffer = recv_buffer[start+40:]
    
    
    # 5. 间隔控制（建议0.5-1秒）
    time.sleep_ms(100)