import os
import sys
import argparse # 添加 argparse
from datetime import datetime # 添加 datetime

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


# USB摄像头初始化
try:
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        cam.open(0)
    assert cam.isOpened()

    # 如果需要保存视频，则初始化 VideoWriter
    if args.save_video:
        # 获取摄像头的实际帧率和尺寸
        fps = cam.get(cv2.CAP_PROP_FPS)
        if fps == 0: # 如果获取失败，使用默认值
             fps = 30
        width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 创建输出目录
        os.makedirs(args.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_path = os.path.join(args.output_dir, f"mission_{timestamp}.avi")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG') 
        video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

        if not video_writer.isOpened():
            logger.error(f"[MANAGER] Failed to create video writer at {video_path}")
            video_writer = None # 创建失败则不写入
        else:
            logger.info(f"[MANAGER] Recording video to {video_path}")
except:
    logger.warning("[MANAGER] Camera Opening Failed")
    while True:
        fc.set_rgb_led(255, 255, 0)
        sleep(0.5)
        fc.set_rgb_led(0, 0, 0)
        sleep(0.5)
        if fc.event.key_short.is_set():
            fc.quit()
            self_reboot()



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
        mission = Mission(fc, cam, video_writer)

    logger.info("[MANAGER] Calling Mission")

    mission.run()

    logger.info("[MANAGER] Mission Finished")
except Exception as e:
    import traceback

    logger.error(f"[MANAGER] Mission Failed: {traceback.format_exc()}")
finally:
    if mission is not None:
        mission.stop()
    if fc.state.unlock.value:
        logger.warning("[MANAGER] Auto Landing")
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stablize()
        fc.land()
        sleep(2)
        fc.lock()

############################## 结束任务 ##############################
print(f"Mission{target_mission} finished")
fc.set_action_log(False)
set_buzzer(True)
sleep(0.5)
set_buzzer(False)
fc.quit()
cam.release()
if video_writer :
    video_writer.release()
    logger.info(f"[MANAGER] Video saved to {video_path}")

########################## 重启自身 #############################
# if not _testing:
#     self_reboot()