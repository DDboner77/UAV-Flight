from Protocol import FC_Protocol
from Base import FC_Base_Uart_Communication, logger
import time, os, sys

from media.sensor import *
from media.display import *
from media.media import *
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

    picture_width = 400
    picture_height = 240

    sensor_id = 2
    sensor = None

    # 显示模式选择：可以是 "VIRT"、"LCD" 或 "HDMI"
    DISPLAY_MODE = "LCD"

    # 根据模式设置显示宽高
    if DISPLAY_MODE == "VIRT":
        # 虚拟显示器模式
        DISPLAY_WIDTH = ALIGN_UP(1920, 16)
        DISPLAY_HEIGHT = 1080
    elif DISPLAY_MODE == "LCD":
        # 3.1寸屏幕模式
        DISPLAY_WIDTH = 800
        DISPLAY_HEIGHT = 480
    elif DISPLAY_MODE == "HDMI":
        # HDMI扩展板模式
        DISPLAY_WIDTH = 1920
        DISPLAY_HEIGHT = 1080
    else:
        raise ValueError("未知的 DISPLAY_MODE，请选择 'VIRT', 'LCD' 或 'HDMI'")

    try:
        # 构造一个具有默认配置的摄像头对象
        sensor = Sensor(id=sensor_id)
        # 重置摄像头sensor
        sensor.reset()

        # 无需进行镜像翻转
        # 设置水平镜像
        # sensor.set_hmirror(False)
        # 设置垂直翻转
        # sensor.set_vflip(False)

        # 设置通道0的输出尺寸为1920x1080
        sensor.set_framesize(width=picture_width, height=picture_height, chn=CAM_CHN_ID_0)
        # 设置通道0的输出像素格式为RGB565
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

        # 根据模式初始化显示器
        if DISPLAY_MODE == "VIRT":
            Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, fps=60)
        elif DISPLAY_MODE == "LCD":
            Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
        elif DISPLAY_MODE == "HDMI":
            Display.init(Display.LT9611, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)

        # 初始化媒体管理器
        MediaManager.init()
        # 启动传感器
        sensor.run()


        """测试飞控起飞和降落功能"""
        
        # 初始化基础通信层
        base_com = FC_Base_Uart_Communication()
        logger.info("初始化基础通信层...")
        
        # 启动串口监听，使用UART2，波特率500000，启用状态打印
        base_com.start_listen_serial(uart_id=2, bit_rate=500000, print_state=True)  
        base_com._print_data = False  # 启用数据打印
        # 初始化协议层
        fc = FC_Protocol(base_com)
        logger.info("初始化协议层...")

        # 使用定时器监听线程处理串口数据
        tim = Timer(-1)
        tim.init(period=10, mode=Timer.PERIODIC, callback=fc.base_com._listen_serial_task)

        
        logger.info("等待飞控连接...")
        connection_timeout = 15
        start_time = time.time()

        clock = time.clock()
        while not base_com.state.connected:
            if time.time() - start_time > connection_timeout:
                logger.error("连接超时，请检查连接")
                tim.deinit()
                fc.quit()
                return
            time.sleep(0.1) 

        logger.info("飞控已连接!") # Add confirmation

         # 切换至程控模式
        logger.info("切换至程控模式...")
        fc.set_flight_mode(2)  # 切换到程控模式
        display_fc_status(base_com)
        time.sleep(1)  # 等待1秒，避免过快切换模式
        

        # 检查模式是否正确切换到程控模式
        if base_com.state.mode_sta != 2:
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
        logger.info("起飞到130cm高度...")
        fc.take_off(130)
        time.sleep(5)
        display_fc_status(base_com)  # 显示起飞过程中的状态10

        # while base_com.state.alt_add < 1.2:
        #     time.sleep(1)

        move_speed = 10  # cm/s 基础速度值（需根据设备特性调整）
        
        while True:
            clock.tick()
            os.exitpoint()

            # 捕获通道0的图像
            img = sensor.snapshot(chn=CAM_CHN_ID_0)

            # 查找线段并绘制
            circles = img.find_circles(threshold=6000)
            count = 0  # 初始化线段计数器
        # ========== 新增运动控制逻辑 ==========
            if len(circles) > 0:
                # 选择面积最大的圆形（可根据需求改为最近中心策略）
                main_circle = max(circles, key=lambda c: c.r())
                
                # 获取圆心坐标（图像坐标系：左上角原点）
                target_x = main_circle.x()
                target_y = main_circle.y()
                radius = main_circle.r()
                
                # 计算图像中心坐标
                center_x = picture_width // 2
                center_y = picture_height // 2
                
                # 计算位置偏差（图像坐标系Y轴向下为正）
                dx = target_x - center_x  # 正值表示目标在右侧
                dy = target_y - center_y  # 正值表示目标在下侧
                
                # 绘制辅助标记（调试用）
                img.draw_cross(target_x, target_y, color=(255,0,0), size=10)  # 圆心
                img.draw_cross(center_x, center_y, color=(0,255,0), size=10)  # 图像中心
                img.draw_circle(center_x, center_y, radius, color=(0,0,255))  # 绘制期望位置
                
                # 设置移动阈值（像素容差，可调）
                deadzone = 15
                
                
                # X轴方向控制（左右移动）
                if abs(dx) > deadzone:
                    if dx > 0:
                        print(f"→ 向右移动 | 偏差:{dx}px | 速度:{move_speed}")
                        fc.send_realtime_control_data(vel_x= 0, vel_y= -move_speed, vel_z=0, yaw=0, need_ack=False)
                        # fc.horizontal_move(5, move_speed, 90)
                        #time.sleep(1)  # 等待1秒
                        # motor_right(move_speed)  # 实际调用右转函数
                    else:
                        print(f"← 向左移动 | 偏差:{dx}px | 速度:{move_speed}")
                        fc.send_realtime_control_data(vel_x= 0, vel_y= move_speed, vel_z=0, yaw=0, need_ack=False)
                        #time.sleep(1)  # 等待1秒
                        # motor_left(move_speed)
                # else:
                #     print(f"X轴已居中 | 偏差:{dx}px")
                #     fc.land()  # 停止横向移动
                    
                # Y轴方向控制（前后移动）
                if abs(dy) > deadzone:
                    if dy > 0:
                        print(f"↓ 向后移动 | 偏差:{dy}px | 速度:{move_speed}")
                        fc.send_realtime_control_data(vel_x= -move_speed, vel_y= 0, vel_z=0, yaw=0, need_ack=False)
                        #time.sleep(1)  # 等待1秒
                        # motor_backward(move_speed)
                    else:
                        print(f"↑ 向前移动 | 偏差:{dy}px | 速度:{move_speed}")
                        fc.send_realtime_control_data(vel_x= move_speed, vel_y= 0, vel_z=0, yaw=0, need_ack=False)
                        #time.sleep(1)  # 等待1秒
                        # motor_forward(move_speed)
                # else:
                #     print(f"Y轴已居中 | 偏差:{dy}px")
                #     fc.land()  # 停止横向移动
                #     # motor_stop_y()  # 停止纵向移动
                    
                # 综合停止条件
                if abs(dx) <= deadzone and abs(dy) <= deadzone:
                    print("★ 目标已居中 ★")
                    fc.send_realtime_control_data(vel_x= move_speed, vel_y= 0, vel_z=0, yaw=0, need_ack=False)
                    time.sleep(1)
                    fc.set_flight_mode(3)  # 切换到程控模式
                    time.sleep(1)
                    fc.land()
                    time.sleep(3)  # 等待3秒
                    break
                    
            else:
                print("未检测到圆形，停止移动")
                fc.send_realtime_control_data(vel_x= 0, vel_y= 0, vel_z=0, yaw=0, need_ack=False)
                #time.sleep(1)  # 等待1秒
            # ========== 运动控制结束 ==========
            print("------圆形统计开始------")
            for circle in circles:
                # 若想获取更详细的四个顶点，可使用 rect.corners()，该函数会返回一个有四个元祖的列表，每个元组代表圆形的四个顶点，从左上角开始，按照顺时针排序。
                img.draw_circle(circle.circle(), color=(1, 147, 230), thickness=3)  # 绘制线段
                print(f"Circle {count}: {circle}")  # 打印线段信息
                count += 1  # 更新计数器
            print("---------END---------")

            # 显示捕获的图像，中心对齐，居中显示
            Display.show_image(img, x=int((DISPLAY_WIDTH - picture_width) / 2), y=int((DISPLAY_HEIGHT - picture_height) / 2))
            
            time.sleep(0.001)

        fc.lock()
    except KeyboardInterrupt as e:
        print("用户停止: ", e)
    except BaseException as e:
        print(f"异常: {e}")
    finally:
        # 停止传感器运行
        if isinstance(sensor, Sensor):
            sensor.stop()
        # 反初始化显示模块
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        # 释放媒体缓冲区
        MediaManager.deinit()
    

if __name__ == "__main__":
    main()
    