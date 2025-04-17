import time
import sys
import serial

from Serial import MP_Serial
from Protocol import FC_Protocol
from Base import FC_Base_Uart_Communication, logger

# 配置参数
JETSON_SERIAL_PORT = "/dev/ttyTHS1"
BAUD_RATE = 500000
CONNECTION_TIMEOUT_S = 15
FLIGHT_ALTITUDE_CM = 100
HOVER_TIME_S = 5

def display_fc_status(base_com):
    """显示飞控状态"""
    if hasattr(base_com, 'state') and hasattr(base_com.state, 'connected') and base_com.state.connected:
        state = base_com.state
        print("\n=========== 飞控状态 ===========")
        print(f" 连接状态:   {state.connected}")
        print(f" 解锁状态:   {state.unlock_sta}")
        print(f" 模式:       {state.mode_sta} (1:高度, 2:位置保持, 3:程序)")
        print(f" 电压:       {state.voltage:.2f}V")
        print(f" 高度:       融合={state.alt_fused:.2f}m, 附加={state.alt_add:.2f}m")
        print(f" 位置:       X={state.pos_x:.2f}m, Y={state.pos_y:.2f}m")
        print(f" 速度:       VX={state.vel_x:.2f}m/s, VY={state.vel_y:.2f}m/s, VZ={state.vel_z:.2f}m/s")
        print(f" 姿态:       横滚={state.rol:.1f}°, 俯仰={state.pit:.1f}°, 偏航={state.yaw:.1f}°")
        print("=================================\n")
    elif hasattr(base_com, 'state'):
         print("\n[状态] 飞控未连接或状态未初始化\n")
    else:
         print("\n[状态] 基础通信对象无效\n")

def main():
    serial_comm = None
    base_com = None
    fc = None

    try:
        # 初始化串口
        logger.info(f"初始化串口 {JETSON_SERIAL_PORT}, 波特率 {BAUD_RATE}...")
        try:
            serial_comm = MP_Serial(port=JETSON_SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.02)
            serial_comm.read_config(startBit=[0xAA, 0x55])
            serial_comm.send_config(startBit=[0xAA, 0x22], optionBit=[0x01])
        except RuntimeError as e:
            logger.error(f"串口初始化失败: {e}")
            return

        logger.info("串口初始化完成")

        # 初始化基础通信层
        logger.info("初始化基础通信层...")
        base_com = FC_Base_Uart_Communication(serial_instance=serial_comm)

        # 初始化协议层
        logger.info("初始化协议层...")
        fc = FC_Protocol(base_com)

        # 启动通信
        base_com.start(print_state=False)

        # 等待飞控连接
        logger.info("等待飞控连接...")
        start_time = time.time()
        while not base_com.state.connected:
            base_com.poll()
            if time.time() - start_time > CONNECTION_TIMEOUT_S:
                logger.error(f"连接超时 ({CONNECTION_TIMEOUT_S}秒)，请检查接线和飞控电源")
                raise TimeoutError("飞控连接超时")
            time.sleep(0.05)

        logger.info("飞控已连接!")
        display_fc_status(base_com)

        # 设置飞行模式
        target_mode = 2
        logger.info(f"设置飞行模式为 {target_mode} (位置保持)...")
        time.sleep(1)
        base_com.poll()
        display_fc_status(base_com)
        if base_com.state.mode_sta != target_mode:
             logger.error(f"模式切换失败! 当前模式: {base_com.state.mode_sta}. 期望: {target_mode}")

        # 解锁
        logger.info("尝试解锁...")
        fc.unlock()
        time.sleep(3)
        for _ in range(5):
            base_com.poll()
            time.sleep(0.1)
        display_fc_status(base_com)
        if not base_com.state.unlock_sta:
             logger.error("解锁失败，检查飞控状态")
             logger.warning("继续测试...")

        # 起飞
        logger.info(f"命令起飞至 {FLIGHT_ALTITUDE_CM} 厘米...")
        fc.take_off(FLIGHT_ALTITUDE_CM)
        logger.info(f"等待 {HOVER_TIME_S} 秒...")
        hover_start_time = time.time()
        while time.time() - hover_start_time < HOVER_TIME_S:
            base_com.poll()
            time.sleep(0.1)

        logger.info("悬停时间结束")
        display_fc_status(base_com)

        # 降落
        logger.info("命令降落...")
        fc.land()
        logger.info("等待降落序列完成...")
        landing_wait_start = time.time()
        while time.time() - landing_wait_start < 15:
             base_com.poll()
             time.sleep(0.2)

        logger.info("降落序列可能已完成")
        display_fc_status(base_com)

        # 锁定电机
        logger.info("锁定电机...")
        fc.lock()
        time.sleep(1)
        base_com.poll()
        display_fc_status(base_com)

    except serial.SerialException as e:
        logger.error(f"串口通信错误: {e}")
    except TimeoutError as e:
        logger.error(f"操作超时: {e}")
    except RuntimeError as e:
        logger.error(f"飞行序列运行时错误: {e}")
    except KeyboardInterrupt:
        logger.info("检测到键盘中断，尝试降落并锁定...")
        if fc:
            try:
                logger.info("发送降落命令...")
                fc.land()
                time.sleep(3)
                logger.info("发送锁定命令...")
                fc.lock()
                time.sleep(1)
            except Exception as emergency_e:
                 logger.error(f"紧急降落/锁定时出错: {emergency_e}")
        else:
             logger.warning("协议对象未初始化，无法发送紧急命令")

    except Exception as e:
        logger.error(f"发生意外错误: {e}", exc_info=True)
        if fc:
            try:
                 logger.error("尝试紧急锁定...")
                 fc.lock()
                 time.sleep(1)
            except Exception as final_e:
                 logger.error(f"发送紧急锁定失败: {final_e}")

    finally:
        # 清理
        logger.info("执行清理...")
        if fc:
            try:
                logger.info("确保电机锁定...")
                fc.lock()
                time.sleep(0.5)
            except Exception as final_lock_e:
                logger.error(f"最终锁定尝试出错: {final_lock_e}")
            fc.quit()
        elif base_com:
             base_com.quit()
        elif serial_comm:
             serial_comm.close()

        logger.info("清理完成，退出")
        sys.exit()

if __name__ == "__main__":
    main()