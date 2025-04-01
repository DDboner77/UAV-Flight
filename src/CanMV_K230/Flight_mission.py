from Protocol import FC_Protocol
from Base import FC_Base_Uart_Communication, logger
import time
from machine import Timer

def display_fc_status(base_com):
    """显示飞控状态信息"""
    if base_com.state.connected:
        print("=========== 飞控状态 ===========")
        print(f"姿态: rol={base_com.state.rol:.2f}° pit={base_com.state.pit:.2f}° yaw={base_com.state.yaw:.2f}°")
        print(f"高度: {base_com.state.alt_fused:.2f}m 附加高度: {base_com.state.alt_add:.2f}m")
        print(f"位置: x={base_com.state.pos_x:.2f}m y={base_com.state.pos_y:.2f}m")
        print(f"速度: vx={base_com.state.vel_x:.2f}m/s vy={base_com.state.vel_y:.2f}m/s vz={base_com.state.vel_z:.2f}m/s")
        print(f"电压: {base_com.state.voltage:.2f}V 模式:{base_com.state.mode_sta} 解锁:{base_com.state.unlock_sta}")
        print("================================")
    else:
        print("飞控未连接，无法显示状态")

def main():
    """测试飞控起飞和降落功能"""
    
    # 初始化基础通信层
    base_com = FC_Base_Uart_Communication()
    logger.info("初始化基础通信层...")
    
    # 启动串口监听，使用UART2，波特率500000，启用状态打印
    base_com.start_listen_serial(uart_id=2, bit_rate=500000, print_state=True)  
    base_com._print_data = True  # 启用数据打印
    # 初始化协议层
    fc = FC_Protocol(base_com)
    logger.info("初始化协议层...")

    # 使用定时器监听线程处理串口数据
    tim = Timer(-1)
    tim.init(period=10, mode=Timer.PERIODIC, callback=fc.base_com._listen_serial_task)

    
    logger.info("等待飞控连接...")
    connection_timeout = 15
    start_time = time.time()

    while not base_com.state.connected:
        if time.time() - start_time > connection_timeout:
            logger.error("连接超时，请检查连接")
            tim.deinit()
            fc.quit()
            return
        time.sleep(0.1) 

    logger.info("飞控已连接!") # Add confirmation
    

    
    # 开启自动切换模式
    base_com.state.auto_change_mode = True

    try:
        # 切换至程控模式
        logger.info("切换至程控模式...")
        fc.set_flight_mode(3)  # 切换到程控模式
        display_fc_status(base_com)
        time.sleep(1)  # 等待1秒，避免过快切换模式
        

        # 检查模式是否正确切换到程控模式
        if base_com.state.mode_sta != 3:
            logger.error(f"模式切换失败，当前模式是 {base_com.state.mode_sta}，无法继续")
            raise ValueError("模式切换失败")

        # 解锁
        logger.info("正在解锁...")
        fc.unlock()
        time.sleep(2)
        display_fc_status(base_com)  # 显示解锁后的状态
        
        if not base_com.state.unlock_sta:
            logger.error("解锁失败，无法继续")
            raise ValueError("解锁失败")

        # 起飞
        logger.info("起飞到50cm高度...")
        fc.take_off(50)
        time.sleep(1)
        display_fc_status(base_com)  # 显示起飞过程中的状态

        # 尝试水平移动
        logger.info("向前飞行1米...")
        display_fc_status(base_com)  # 移动前状态
        fc.horizontal_move(100, 30, 90)  # 增加速度到30cm/s
        time.sleep(2)
        display_fc_status(base_com)  # 显示移动过程中的状态

        # 降落
        logger.info("降落...")
        fc.land()
        time.sleep(1)
        display_fc_status(base_com)  # 显示降落过程中的状态

        # 锁定
        logger.info("正在锁定...")
        fc.lock()
        display_fc_status(base_com)  # 显示锁定后的状态
        time.sleep(2)


    except Exception as e:
        logger.error(f"测试过程中出错: {e}")
    finally:
        # 确保安全关闭
        try:
            fc.lock()  # 确保锁定
        except:
            pass
            
        # 停止定时器
        tim.deinit()
        
        fc.quit()
        logger.info("通信已关闭")

if __name__ == "__main__":
    main()