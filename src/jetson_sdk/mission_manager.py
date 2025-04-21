import os
import sys
import argparse # 添加 argparse
from datetime import datetime # 添加 datetime
import threading

####### 清理日志 #######
# path = os.path.dirname(os.path.abspath(__file__))
# log_path = os.path.join(path, "fc_log.log")
# try:
#     os.remove(log_path)
# except:
#     pass
####################
from time import sleep, time

import cv2
import numpy as np
from configManager import ConfigManager
from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D
from FlightController.Camera import My_Camera
# from hmi import HMI

def self_reboot():
    logger.info("[MANAGER] Manager Restarting")
    os.execl(sys.executable, sys.executable, *sys.argv)


# 添加命令行参数解析
parser = argparse.ArgumentParser(description='Mission Manager with Video Recording')
parser.add_argument('--save-video', '-s', action='store_true',
                    help='是否保存摄像头视频流')
parser.add_argument('--output-dir', '-o', type=str, default='output_videos',
                    help='视频保存目录 (默认: output_videos)')
args = parser.parse_args()

args.save_video = True # 强制保存视频




# 初始化本地飞控控制层，进行串口连接
try:
    fc = FC_Controller()
    fc.start_listen_serial("/dev/ttyTHS1", print_state=False)
    fc.wait_for_connection(5)
except:
    logger.error("[MANAGER] Local Mode Failed, Restarting")
    sleep(1)
    self_reboot()

# 
fc.event.key_short.clear()
fc.event.key_double.clear()
fc.event.key_long.clear()
fc.set_rgb_led(0, 0, 0)
fc.set_action_log(False)


# 初始化摄像头
try:
    cam_manager = My_Camera(
        index=0,
        save_video=args.save_video,
        output_dir=args.output_dir,
        fps=60
    )
    # 设置飞控引用
    cam_manager.set_flight_controller(fc)
    cam_manager.start()
    logger.info("[MANAGER] Camera Manager Initialized")
except Exception as e:
    logger.error(f"[MANAGER] Camera Manager Initialization Failed: {e}")





# 激光雷达初始化
# try:
#     radar = LD_Radar()
#     radar.start(
#         "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
#         "LD06",
#     )
# except:
#     logger.warning("[MANAGER] Radar Connecting Failed")
#     while True:
#         fc.set_rgb_led(255, 0, 0)
#         sleep(0.5)
#         fc.set_rgb_led(0, 0, 0)
#         sleep(0.5)
#         if fc.event.key_short.is_set():
#             fc.quit()
#             self_reboot()



# 串口屏初始化
# try:
#     hmi = HMI("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0")
#     hmi.command("page init")
#     sleep(1)
# except:
#     logger.warning("[MANAGER] HMI Connecting Failed")
#     while True:
#         fc.set_rgb_led(255, 0, 255)
#         sleep(0.5)
#         fc.set_rgb_led(0, 0, 0)
#         sleep(0.5)
#         if fc.event.key_short.is_set():
#             fc.quit()
#             cam.release()
#             radar.stop()
#             self_reboot()


############################## 参数 ##############################
cfg = ConfigManager()
# camera_down_pwm = 32.5
# camera_down_45_pwm = 52.25
# camera_up_pwm = 72
# camera_up_45_pwm = 91.75
set_button_led = lambda x: fc.set_digital_output(1, x)
set_buzzer = lambda x: fc.set_digital_output(0, x)
############################## 初始化 ##############################
logger.info("[MANAGER] Self-Checking Passed")
fc.set_rgb_led(0, 255, 0)
sleep(1)
fc.set_rgb_led(0, 0, 0)

fc.set_rgb_led(0, 0, 0)
fc.set_flight_mode(fc.PROGRAM_MODE)
set_button_led(False)

target_mission = None
_testing = False

logger.info("[MANAGER] Selecting mission...")



# 选择任务
target_mission = 1



############################## 开始任务 ##############################
logger.info(f"[MANAGER] Target Mission: {target_mission}")
fc.set_action_log(True)
mission = None
try:
    # 根据目标任务选择相应的任务类
    # 目前只有一个任务 Mission_QR_Circle，其他的还在测试
    if target_mission == 1:
        from Test.mission_qr_circle import Mission

        mission = Mission(fc, cam_manager)
        # 将任务对象传递给摄像头管理器
        cam_manager.set_mission(mission)

    logger.info("[MANAGER] Calling Mission")

    mission.run()

    logger.info("[MANAGER] Mission Finished")
except Exception as e:
    import traceback

    logger.error(f"[MANAGER] Mission Failed: {traceback.format_exc()}")
finally:
    logger.info("[MANAGER] Entering final cleanup...")
    if mission is not None:
        logger.info("[MANAGER] Stopping mission object...")
        mission.stop()



    if fc and fc.connected and fc.state.unlock.value: # 检查 fc 是否有效
        logger.warning("[MANAGER] Auto Landing")
        try:
            fc.set_flight_mode(fc.HOLD_POS_MODE)#先停下飞机
            fc.stablize()
            fc.land()
            sleep(2) # 等待降落指令发出
            fc.lock()
            logger.info("[MANAGER] Auto Landing and Locking")
        except Exception as land_err:
            logger.error(f"[MANAGER] Error during auto landing/locking: {land_err}")


    cam_manager.stop()


############################## 结束任务 ##############################
print(f"Mission{target_mission} finished")
if fc and fc.connected: # 检查 fc 是否有效
    fc.set_action_log(False)
    set_buzzer(True)
    sleep(0.5)
    set_buzzer(False)
    fc.quit()



########################## 重启自身 #############################
# if not _testing:
#     self_reboot()



