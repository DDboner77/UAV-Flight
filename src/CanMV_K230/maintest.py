from machine import UART, FPIOA
import time
import sys

# 导入FC SDK模块
from Base import FC_Base_Uart_Communication, logger
from Protocol import FC_Protocol
from Serial import MP_Serial

def test_serial_module():
    """测试serial.py模块的MP_Serial类"""
    logger.info("===== 测试MP_Serial类 =====")
    
    # 设置引脚映射
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    
    # 创建并测试MP_Serial
    try:
        # 初始化串口
        serial = MP_Serial(UART.UART2, 500000)
        logger.info("MP_Serial初始化成功")
        
        # 配置发送和接收参数
        serial.send_config(startBit=[0xAA, 0x22], optionBit=[0x00])
        serial.read_config(startBit=[0xAA, 0x55])
        logger.info("MP_Serial配置完成")
        
        # 发送测试数据
        logger.info("发送测试数据...")
        result = serial.write(b'\x01\x02\x03')
        # 避免多行字符串拼接
        hex_result = ""
        for b in result:
            hex_result += hex(b) + " "
        logger.info("发送结果: " + hex_result)
        
        # 等待接收数据
        logger.info("等待5秒接收数据...")
        start_time = time.time()
        received = False
        while time.time() - start_time < 5:
            if serial.read():
                data = serial.rx_data
                # 避免多行字符串拼接
                hex_data = ""
                for b in data:
                    hex_data += hex(b) + " "
                logger.info("收到数据: " + hex_data)
                received = True
                break
            time.sleep(0.1)
        
        if not received:
            logger.info("未接收到数据")
        
        # 关闭串口
        serial.close()
        logger.info("MP_Serial测试完成")
        return True
    except Exception as e:
        logger.error("MP_Serial测试失败: " + str(e))
        if hasattr(sys, 'print_exception'):
            sys.print_exception(e)
        return False

def test_fc_communication(use_mp_serial=True):
    """测试与飞控通信"""
    logger.info("===== 测试飞控通信 =====")
    
    try:
        # 设置引脚映射
        fpioa = FPIOA()
        fpioa.set_function(11, FPIOA.UART2_TXD)
        fpioa.set_function(12, FPIOA.UART2_RXD)
        
        if use_mp_serial:
            # 创建通信对象 - 传入serial.ser作为uart参数
            base_com = FC_Base_Uart_Communication()
            base_com.start_listen_serial(
                uart_id=2,  # 直接指定UART ID
                bit_rate=500000,
                print_state=True
            )

        else:
            # 直接使用UART
            uart = UART(2, baudrate=500000)
            uart.init(500000, bits=8, parity=None, stop=1)
            # 创建通信对象
            base_com = FC_Base_Uart_Communication()
            base_com.start_listen_serial(
                uart=uart,
                bit_rate=500000,
                print_state=True
            )

        
        # 创建协议对象
        fc = FC_Protocol(base_com)
        logger.info("初始化FC通信成功")
        
        # 等待连接
        logger.info("等待连接飞控...")
        start_time = time.time()
        connected = False
        while time.time() - start_time < 10:
            fc.poll()  # 调用poll保持通信  
            if base_com.connected:
                connected = True
                logger.info("飞控连接成功！")
                break
            time.sleep(0.1)
        
        if not connected:
            logger.info("飞控连接超时！")
            base_com.quit()
            return False
        
        # 显示飞控状态
        logger.info("电池电压: " + str(round(base_com.state.bat.value, 2)) + "V")
        logger.info("当前模式: " + str(base_com.state.mode.value))
        unlock_status = "已解锁" if base_com.state.unlock.value else "未解锁"
        logger.info("解锁状态: " + unlock_status)
        
        # 发送几个简单命令测试通信
        logger.info("发送几个简单命令测试通信...")
        
        # 读取IMU数据
        logger.info("Roll: " + str(round(base_com.state.rol.value, 2)) + "°")
        logger.info("Pitch: " + str(round(base_com.state.pit.value, 2)) + "°")
        logger.info("Yaw: " + str(round(base_com.state.yaw.value, 2)) + "°")
        
        # 保持通信一段时间
        logger.info("保持通信10秒...")
        start_time = time.time()
        while time.time() - start_time < 10:
            fc.poll()
            # if time.time() - start_time > 5 and time.time() - start_time < 5.1:
                # 分行打印状态信息，避免多行拼接
            logger.info("持续接收中...")
            logger.info("Roll: " + str(round(base_com.state.rol.value, 2)) + "°")
            logger.info("Pitch: " + str(round(base_com.state.pit.value, 2)) + "°") 
            logger.info("Yaw: " + str(round(base_com.state.yaw.value, 2)) + "°")
            time.sleep(0.01)
        
        # 关闭通信
        base_com.quit()
        logger.info("通信测试完成")
        return True
    except Exception as e:
        logger.error("飞控通信测试失败: " + str(e))
        if hasattr(sys, 'print_exception'):
            sys.print_exception(e)
        if 'base_com' in locals():
            base_com.quit()
        return False

def flight_control_demo(use_mp_serial=True):
    """飞控功能演示"""
    logger.info("===== 飞控功能演示 =====")
    
    try:
        # 设置引脚映射
        fpioa = FPIOA()
        fpioa.set_function(11, FPIOA.UART2_TXD)
        fpioa.set_function(12, FPIOA.UART2_RXD)
        
        if use_mp_serial:
            # 使用MP_Serial创建UART接口
            serial = MP_Serial(UART.UART2, 500000)
            # 创建通信对象 - 传入serial.ser作为uart参数
            base_com = FC_Base_Uart_Communication()
            base_com.start_listen_serial(
                uart=serial.ser,
                bit_rate=500000,
                print_state=True
            )
        else:
            # 直接使用UART
            uart = UART(2, baudrate=500000)
            uart.init(500000, bits=8, parity=None, stop=1)
            # 创建通信对象
            base_com = FC_Base_Uart_Communication()
            base_com.start_listen_serial(
                uart=uart,
                bit_rate=500000,
                print_state=True
            )
        
        # 创建协议对象
        fc = FC_Protocol(base_com)
        logger.info("初始化FC通信成功")
        
        # 等待连接
        logger.info("等待连接飞控...")
        start_time = time.time()
        connected = False
        while time.time() - start_time < 5:
            fc.poll()
            if base_com.connected:
                connected = True
                logger.info("飞控连接成功！")
                break
            time.sleep(0.1)
        
        if not connected:
            logger.error("飞控连接超时！")
            base_com.quit()
            return False
        
        # 显示飞控状态
        logger.info("电池电压: " + str(round(base_com.state.bat.value, 2)) + "V")
        logger.info("当前模式: " + str(base_com.state.mode.value))
        unlock_status = "已解锁" if base_com.state.unlock.value else "未解锁"
        logger.info("解锁状态: " + unlock_status)
        
        # 菜单循环
        while True:
            logger.info("\n===== 飞控功能菜单 =====")
            logger.info("1. 解锁飞控")
            logger.info("2. 上锁飞控")
            logger.info("3. 校准IMU")
            logger.info("4. 校准磁力计")
            logger.info("5. 起飞至指定高度")
            logger.info("6. 降落")
            logger.info("7. 悬停")
            logger.info("8. 移动控制")
            logger.info("0. 退出")
            
            try:
                choice = input("请选择操作 (0-8): ")
                
                if choice == "0":
                    break
                elif choice == "1":
                    logger.info("解锁飞控...")
                    if fc.unlock():
                        logger.info("解锁成功")
                    else:
                        logger.error("解锁失败")
                elif choice == "2":
                    logger.info("上锁飞控...")
                    if fc.lock():
                        logger.info("上锁成功")
                    else:
                        logger.error("上锁失败")
                elif choice == "3":
                    logger.info("校准IMU...")
                    if fc.calibrate_imu():
                        logger.info("IMU校准成功")
                    else:
                        logger.error("IMU校准失败")
                elif choice == "4":
                    logger.info("校准磁力计...")
                    if fc.calibrate_mag():
                        logger.info("磁力计校准成功")
                    else:
                        logger.error("磁力计校准失败")
                elif choice == "5":
                    try:
                        height = int(input("请输入起飞高度(cm): "))
                        logger.info("起飞至" + str(height) + "厘米...")
                        if fc.take_off(height=height):
                            logger.info("起飞命令已发送")
                        else:
                            logger.error("起飞命令发送失败")
                    except ValueError:
                        logger.error("输入的高度无效")
                elif choice == "6":
                    logger.info("执行降落...")
                    if fc.land():
                        logger.info("降落命令已发送")
                    else:
                        logger.error("降落命令发送失败")
                elif choice == "7":
                    logger.info("执行悬停...")
                    if fc.hover():
                        logger.info("悬停命令已发送")
                    else:
                        logger.error("悬停命令发送失败")
                elif choice == "8":
                    try:
                        dx = int(input("请输入X方向移动距离(cm): "))
                        dy = int(input("请输入Y方向移动距离(cm): "))
                        dz = int(input("请输入Z方向移动距离(cm): "))
                        logger.info("移动: X=" + str(dx) + "cm, Y=" + str(dy) + "cm, Z=" + str(dz) + "cm")
                        if fc.move_by(dx=dx, dy=dy, dz=dz):
                            logger.info("移动命令已发送")
                        else:
                            logger.error("移动命令发送失败")
                    except ValueError:
                        logger.error("输入的参数无效")
                else:
                    logger.error("无效的选择")
                    
                # 每次操作后调用poll更新状态
                for _ in range(5):  # 多次调用以确保状态更新
                    fc.poll()
                    time.sleep(0.1)
                    
            except Exception as e:
                logger.error("操作执行出错: " + str(e))
                if hasattr(sys, 'print_exception'):
                    sys.print_exception(e)
                
        # 确保安全退出
        if base_com.state.unlock.value:
            logger.info("飞控仍处于解锁状态，尝试上锁...")
            fc.lock()
        
        # 关闭通信
        base_com.quit()
        logger.info("飞控演示完成")
        return True
        
    except Exception as e:
        logger.error("飞控演示失败: " + str(e))
        if hasattr(sys, 'print_exception'):
            sys.print_exception(e)
        if 'base_com' in locals():
            try:
                if base_com.state.unlock.value:
                    logger.info("紧急上锁...")
                    fc.lock()
                base_com.quit()
            except:
                pass
        return False

def main():
    """主函数"""
    logger.info("===== 飞控SDK测试程序 =====")
    

    logger.info("\n===== 主菜单 =====")
    logger.info("1. 测试MP_Serial类")
    logger.info("2. 使用MP_Serial测试飞控通信")
    logger.info("3. 使用标准UART测试飞控通信")
    logger.info("4. 使用MP_Serial进行飞控演示")
    logger.info("5. 使用标准UART进行飞控演示")
    logger.info("0. 退出")
    
    try:
        choice = "2"
        
        if choice == "0":
            logger.info("程序结束")
        elif choice == "1":
            test_serial_module()
        elif choice == "2":
            test_fc_communication(use_mp_serial=True)
        elif choice == "3":
            test_fc_communication(use_mp_serial=False)
        elif choice == "4":
            flight_control_demo(use_mp_serial=True)
        elif choice == "5":
            flight_control_demo(use_mp_serial=False)
        else:
            logger.error("无效选择，请重试")
    except Exception as e:
        logger.error("操作执行出错: " + str(e))
        if hasattr(sys, 'print_exception'):
            sys.print_exception(e)
    


if __name__ == "__main__":
    main()