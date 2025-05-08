import time
import cv2
import numpy as np
import pyapriltags as apriltag
from FlightController import logger
from time import sleep
import traceback # 用于打印详细错误

# --- 辅助函数 ---

def detect_apriltag(image, target_id, detector, min_margin=20):
    """
    检测图像中指定 ID 的 AprilTag。
    :param image: 输入图像 (NumPy array)
    :param target_id: 目标 AprilTag ID (整数)
    :param detector: pyapriltags.Detector 实例
    :param min_margin: 最低决策边界阈值
    :return: 如果检测到目标 ID 的标签，返回其信息字典 (包含 center, corners, id 等)；否则返回 None。
    """
    if image is None or detector is None:
        return None
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)

        for r in results:
            if r.tag_id == target_id and r.decision_margin >= min_margin:
                logger.info(f"[Mission AprilTag] Found Tag ID: {r.tag_id} at ({int(r.center[0])}, {int(r.center[1])}) "
                            f"Margin: {r.decision_margin:.1f}, Hamming: {r.hamming}")
                return {
                    'id': r.tag_id,
                    'center': r.center.astype(int),
                    'corners': r.corners.astype(int),
                    'decision_margin': r.decision_margin,
                    'hamming': r.hamming
                }
        # logger.debug(f"[Mission AprilTag] Target Tag ID {target_id} not found or margin too low.")
        return None
    except Exception as e:
        logger.error(f"[Mission AprilTag] Error during AprilTag detection: {e}")
        # logger.error(traceback.format_exc()) # 可选：打印完整堆栈跟踪
        return None

def detect_circle(image):
    """
    使用 OpenCV 的霍夫变换检测图像中的圆形降落区。
    返回: 如果检测到圆形，则返回其中心和半径 (x, y, radius)；否则返回 None。
    """
    if image is None:
        return None
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 增强对比度可能有助于检测
        gray = cv2.equalizeHist(gray)
        gray = cv2.medianBlur(gray, 5) # 中值滤波去噪

        # 霍夫圆检测参数需要根据实际场景和降落板样式仔细调整
        # minDist: 圆心之间的最小距离
        # param1: Canny 边缘检测的高阈值
        # param2: 圆心检测的累加器阈值 (值越小，检测到的假圆越多)
        # minRadius, maxRadius: 圆半径的范围 (像素)
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=rows / 4,
                                   param1=100, param2=40, # 降低 param2 尝试检测更多圆
                                   minRadius=30, maxRadius=200) # 调整半径范围

        if circles is not None:
            circles = np.uint16(np.around(circles))
            # 通常我们只需要最大的那个圆作为降落区
            # 这里简单返回第一个检测到的圆
            best_circle = circles[0, 0]
            center_x, center_y, radius = best_circle
            logger.info(f"[Mission AprilTag] Detected circle at ({center_x}, {center_y}) with radius {radius}")
            return (center_x, center_y, radius)
        # logger.debug("[Mission AprilTag] No circle detected in the current frame.")
        return None
    except Exception as e:
        logger.error(f"[Mission AprilTag] Error during circle detection: {e}")
        # logger.error(traceback.format_exc())
        return None

# --- Mission 类 ---

class Mission:
    def __init__(self, fc, Cam):
        """
        初始化 AprilTag 任务。
        :param fc: FlightController 实例
        :param Cam: My_Camera 实例 (包含摄像头对象)
        """
        self.fc = fc
        # 注意：Cam 是 My_Camera 实例，摄像头对象是 Cam.cam
        self.cam = Cam.cam if Cam else None
        self.running = False
        self.detector = None

        # --- 初始化 AprilTag 检测器 ---
        try:
            # 使用更鲁棒的参数初始化
            self.detector = apriltag.Detector(
                families='tag16h5', # 假设使用 tag16h5
                nthreads=2,         # 使用多线程
                quad_decimate=1.0,  # 不降采样
                quad_sigma=0.4,     # 适度高斯模糊
                refine_edges=True,  # 精细化边缘
                decode_sharpening=0.25 # 解码锐化
            )
            logger.info("[Mission AprilTag] AprilTag Detector initialized successfully.")
        except Exception as e:
            logger.error(f"[Mission AprilTag] Failed to initialize AprilTag Detector: {e}")
            logger.error(traceback.format_exc())
            # self.detector 将保持为 None
  
        # --- 任务参数 ---
        self.initial_takeoff_alt_cm = 150   # 初始起飞高度 (cm) - 较高
        self.search_alt_cm = 130            # 搜索 Tag 时的高度 (cm)
        self.approach_alt_cm = 80           # 看到 Tag 后降低到的高度 (cm)
        self.forward_dis = 20 # cm
        self.forward_speed = 15 # cm/s
        self.forward_angle = 0 # 
        self.search_speed_cm_s = 20         # 向前搜索速度 (cm/s)
        self.return_speed_cm_s = 25         # 返航速度 (cm/s)
        self.target_tag_id = 0              # 目标 AprilTag ID
        self.search_timeout_s = 60          # 搜索超时时间 (秒)
        self.yaw_rate_deg_s = 90            # 自转角速度 (度/秒)
        self.align_tolerance_px = 30        # 降落对准容忍度 (像素)
        self.align_speed_cm_s = 10          # 对准时的移动速度 (cm/s)
        # --- 任务参数结束 ---

        logger.info("[Mission AprilTag] Initialized")

    def _read_frame(self):
        """从摄像头读取一帧图像"""
        if not self.cam or not self.cam.isOpened():
             logger.error("[Mission AprilTag] Camera is not available.")
             return None
        ret, frame = self.cam.read()
        if not ret:
            logger.warning("[Mission AprilTag] Failed to read frame from camera")
            return None
        return frame

    def _take_off(self, altitude_cm):
        """起飞到指定高度"""
        logger.info(f"[Mission AprilTag] Taking off to {altitude_cm} cm...")
        if not self.fc.state.unlock.value:
            logger.info("[Mission AprilTag] Unlocking drone...")
            self.fc.unlock()
            sleep(1.5)
            if not self.fc.state.unlock.value:
                raise RuntimeError("Failed to unlock drone.")
        self.fc.take_off(100)
        sleep(3)
        self.fc.go_up(altitude_cm,20)

        # self.fc.set_height(1, altitude_cm,20) # 设置目标高度
        # #0:融合高度 1:激光高度

        logger.info("[Mission AprilTag] Waiting to reach altitude...")
        sleep(3) # 根据实际情况调整延时
        # 检查是否仍在运行且已解锁
        if not self.running or not self.fc.state.unlock.value:
            raise RuntimeError("Takeoff failed or mission stopped.")
        logger.info("[Mission AprilTag] Takeoff complete. Stabilizing...")
        self.fc.stablize()
        sleep(1) # 起飞后悬停稳定

    def _search_forward_for_tag(self):
        """以指定高度向前飞行搜索目标 AprilTag"""
        logger.info(f"[Mission AprilTag] Searching forward for Tag ID {self.target_tag_id} at {self.search_alt_cm} cm...")
        start_time = time.time()
        tag_info = None

        # 确保在正确的高度
        # self.fc.set_target_position(alt=self.search_alt_cm) # 可能需要等待到达
        # sleep(1)

        while self.running and (time.time() - start_time) < self.search_timeout_s:
            frame = self._read_frame()
            if frame is None:
                sleep(0.1)
                continue

            current_tag_info = detect_apriltag(frame, self.target_tag_id, self.detector)
            if current_tag_info:
                tag_info = current_tag_info
                logger.info(f"[Mission AprilTag] Target Tag ID {self.target_tag_id} found!")
                self.fc.stablize() # 停止前进
                break
            else:
                # 未找到，继续向前移动
                if self.fc.state.unlock.value:
                    # 使用 horizontal_move 前进一小段距离
                    # 注意：distance 参数是总距离，这里可能需要用 send_realtime_control_data 更合适
                    # 或者每次移动一小段固定距离
                    # self.fc.horizontal_move(distance=20, speed=self.search_speed_cm_s, deg=0)
                    # 使用实时速度控制前进
                    self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
                    self.fc.horizontzal_move(self.forward_dis, self.forward_speed, self.forward_angle)
                    sleep(0.1) # 持续发送速度指令
                else:
                    logger.warning("[Mission AprilTag] Cannot move forward, drone locked.")
                    sleep(1)

        # 循环结束后检查是否找到
        if tag_info is None:
            logger.warning(f"[Mission AprilTag] Target Tag ID {self.target_tag_id} not found within timeout.")
            self.fc.stablize() # 确保停止移动
        return tag_info

    def _descend_to(self, target_altitude_cm):
        """下降到指定高度"""
        logger.info(f"[Mission AprilTag] Descending to {target_altitude_cm} cm...")

        self.fc.go_down(target_altitude_cm, speed=20) # 下降速度可以调整
        # 等待到达目标高度
        timeout = 3 # 秒
        start_wait = time.time()
        while self.running and time.time() - start_wait < timeout:
            current_alt = self.fc.state.alt.value
            if abs(current_alt - target_altitude_cm) < 10: # 10cm 容差
                logger.info(f"[Mission AprilTag] Reached target altitude {target_altitude_cm} cm (current: {current_alt:.1f} cm).")
                self.fc.stablize()
                sleep(1) # 稳定一下
                return True
            sleep(0.2)
        logger.warning(f"[Mission AprilTag] Failed to reach target altitude {target_altitude_cm} cm within timeout.")
        self.fc.stablize()
        return True
        return False # 下降失败

    def _perform_360_yaw(self):
        """执行 360 度自转"""
        logger.info("[Mission AprilTag] Performing 360-degree yaw...")
        duration = 360.0 / self.yaw_rate_deg_s
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        time_started = time.time()

        while self.running and (time.time() - time_started) < duration:
            self.fc.send_realtime_control_data(0, 0, 0, yaw=self.yaw_rate_deg_s)
            sleep(0.2) # 保持指令发送

        self.fc.stablize()
        logger.info("[Mission AprilTag] 360-degree yaw complete.")
        sleep(1)

    def _turn_around(self):
        """掉头 (偏航 180 度)"""
        logger.info("[Mission AprilTag] Turning around (180-degree yaw)...")
        duration = 180.0 / self.yaw_rate_deg_s
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        time_started = time.time()

        self.fc.turn_right(180, speed=90) # 右转 180 度，速度可以调整
        sleep(2) # 等待转向完成
        self.fc.stablize()
        logger.info("[Mission AprilTag] Turn around complete.")
        sleep(1)

    def _fly_back_and_search_circle(self):
        """掉头后向前飞行并搜索圆形降落区"""
        logger.info("[Mission AprilTag] Flying back and searching for circular landing pad...")
        start_time = time.time()
        circle_info = None

        while self.running and (time.time() - start_time) < self.search_timeout_s:
            frame = self._read_frame()
            if frame is None:
                sleep(0.1)
                continue

            current_circle_info = detect_circle(frame)
            if current_circle_info:
                circle_info = current_circle_info
                logger.info("[Mission AprilTag] Circular landing pad found!")
                self.fc.stablize() # 停止前进
                # 可选：在这里调用对准函数
                if not self._align_over_circle(circle_info, frame.shape):
                     logger.warning("[Mission AprilTag] Failed to align over circle center.")
                     # 即使对准失败，也尝试降落
                break
            else:
                # 未找到，继续向前移动 (相对当前方向)
                if self.fc.state.unlock.value:
                    self.fc.send_realtime_control_data(vel_x=self.return_speed_cm_s, vel_y=0, vel_z=0, yaw=0)
                    sleep(0.5)
                else:
                    logger.warning("[Mission AprilTag] Cannot move forward, drone locked.")
                    sleep(0.5)

        if circle_info is None:
            logger.warning("[Mission AprilTag] Circular landing pad not found within timeout.")
            self.fc.stablize()
        return circle_info

    def _align_over_circle(self, circle_info, frame_shape):
        """尝试将无人机对准到圆形中心上方"""
        logger.info("[Mission AprilTag] Aligning over circle center...")
        cx, cy, radius = circle_info
        frame_height, frame_width = frame_shape[:2]
        target_x, target_y = frame_width // 2, frame_height // 2

        align_timeout = 15 # 对准超时时间
        start_align = time.time()

        while self.running and time.time() - start_align < align_timeout:
            # 重新检测圆心位置
            frame = self._read_frame()
            if frame is None:
                sleep(0.1); continue
            current_circle = detect_circle(frame)
            if current_circle is None:
                logger.warning("[Mission AprilTag] Lost circle during alignment.")
                self.fc.stablize()
                return False # 对准失败

            cx, cy, _ = current_circle
            offset_x = cx - target_x
            offset_y = cy - target_y # 注意：图像 Y 轴通常向下为正

            # 检查是否已对准
            if abs(offset_x) < self.align_tolerance_px and abs(offset_y) < self.align_tolerance_px:
                logger.info("[Mission AprilTag] Alignment successful.")
                self.fc.stablize()
                sleep(1)
                return True

            # 计算控制速度 (简单的比例控制)
            # 注意：vel_x 对应前进/后退，vel_y 对应左/右平移
            # 图像 x 偏移对应 vel_y，图像 y 偏移对应 vel_x (可能需要反号)
            # 需要根据实际测试调整 Kp 值和符号
            Kp = 0.1 # 比例增益 (需要调整)
            vel_y = np.clip(Kp * offset_x, -self.align_speed_cm_s, self.align_speed_cm_s)
            vel_x = np.clip(Kp * -offset_y, -self.align_speed_cm_s, self.align_speed_cm_s) # Y 轴反向

            logger.debug(f"[Mission AprilTag] Aligning: offset=({offset_x:.1f}, {offset_y:.1f})px -> vel=({vel_x:.1f}, {vel_y:.1f})cm/s")
            self.fc.send_realtime_control_data(vel_x, vel_y, 0, 0)
            sleep(0.2) # 控制频率

        logger.warning("[Mission AprilTag] Alignment timed out.")
        self.fc.stablize()
        return False # 对准超时

    def _land_at_center(self):
        """在当前位置执行降落"""
        logger.info("[Mission AprilTag] Initiating landing sequence...")
        if not (self.fc and self.fc.connected and self.fc.state.unlock.value):
            logger.warning("[Mission AprilTag] Cannot land, FC not ready or drone locked.")
            return False

        self.fc.land()
        logger.info("[Mission AprilTag] Waiting for landing and lock...")
        # 等待飞控自动上锁，增加超时
        ret = self.fc.wait_for_lock(5) # 增加降落等待时间
        if not ret:
            logger.warning("[Mission AprilTag] Drone did not lock automatically after landing command. Forcing lock.")
            try:
                if self.fc.state.unlock.value: # 再次检查是否需要上锁
                    self.fc.lock()
            except Exception as lock_err:
                logger.error(f"[Mission AprilTag] Error forcing lock: {lock_err}")
            # 即使强制上锁失败，也认为降落指令已发出
            return True # 或者根据需要返回 False

        logger.info("[Mission AprilTag] Landed and locked successfully.")
        return True

    def run(self):
        """执行完整的 AprilTag 任务流程"""
        self.running = True
        logger.info("========================================")
        logger.info("[Mission AprilTag] Starting mission run.")
        logger.info("========================================")

        # --- 检查 Detector 和 Camera 是否初始化成功 ---
        if self.detector is None:
            logger.error("[Mission AprilTag] Cannot start mission, AprilTag Detector failed to initialize.")
            self.running = False
            return
        if self.cam is None:
            logger.error("[Mission AprilTag] Cannot start mission, Camera is not available.")
            self.running = False
            return

        mission_success = False
        try:
            # --- 1. 起飞到较高高度 ---
            logger.info("[Mission AprilTag] Step 1: Take off to initial altitude")
            self._take_off(self.initial_takeoff_alt_cm)
            if not self.running: return

            # --- 2. 向前搜索 AprilTag ID 0 ---
            logger.info(f"[Mission AprilTag] Step 2: Search forward for Tag ID {self.target_tag_id}")
            tag_info = self._search_forward_for_tag()
            if tag_info is None or not self.running:
                raise RuntimeError(f"Failed to find Tag ID {self.target_tag_id} or mission stopped.")

            # --- 3. 降低高度 ---
            logger.info("[Mission AprilTag] Step 3: Descend to approach altitude")
            if not self._descend_to(self.approach_alt_cm) or not self.running:
                raise RuntimeError("Failed to descend to approach altitude or mission stopped.")

            # --- 4. 自转一圈 ---
            logger.info("[Mission AprilTag] Step 4: Perform 360-degree yaw")
            self._perform_360_yaw()
            if not self.running: return

            # --- 5. 掉头 ---
            logger.info("[Mission AprilTag] Step 5: Turn around")
            self._turn_around()
            if not self.running: return

            # --- 6. 返航并搜索圆形降落区 ---
            logger.info("[Mission AprilTag] Step 6: Fly back and search for landing pad")
            circle_info = self._fly_back_and_search_circle()
            if circle_info is None or not self.running:
                raise RuntimeError("Failed to find circular landing pad or mission stopped.")

            # --- 7. 降落 ---
            # 对准已在 _fly_back_and_search_circle 内部调用（如果找到圆）
            logger.info("[Mission AprilTag] Step 7: Land at target center")
            if not self._land_at_center():
                # 即使降落失败，也认为任务尝试完成，避免下面误报成功
                raise RuntimeError("Landing sequence failed or did not complete.")

            # --- 任务成功 ---
            logger.info("===========================================")
            logger.info("[Mission AprilTag] Mission completed successfully!")
            logger.info("===========================================")
            mission_success = True

        except Exception as e:
            logger.error(f"[Mission AprilTag] An error occurred during the mission: {e}")
            logger.error(traceback.format_exc())
        finally:
            # --- 任务结束处理 ---
            logger.info("[Mission AprilTag] Entering final cleanup phase.")
            self.running = False # 确保运行状态为 False

            if self.fc and self.fc.connected:
                # 只有在任务失败且无人机还在空中时才尝试紧急降落
                if not mission_success and self.fc.state.unlock.value:
                    logger.warning("[Mission AprilTag] Mission failed or was interrupted while flying. Attempting emergency land.")
                    try:
                        self.fc.stablize() # 先悬停
                        sleep(1)
                        self.fc.land()
                        self.fc.wait_for_lock(5) # 等待降落和上锁
                        if self.fc.state.unlock.value:
                             logger.warning("[Mission AprilTag] Forcing lock after emergency land attempt.")
                             self.fc.lock()
                    except Exception as land_err:
                        logger.error(f"[Mission AprilTag] Error during emergency land/lock: {land_err}")
                elif self.fc.state.unlock.value:
                     logger.info("[Mission AprilTag] Mission likely succeeded, drone might still be unlocked if landing failed.")
                     # 可以选择在这里也强制降落和上锁
                     # self.fc.land()
                     # self.fc.wait_for_lock(15)
                     # if self.fc.state.unlock.value: self.fc.lock()
                else:
                     logger.info("[Mission AprilTag] Drone is already locked.")
            else:
                logger.warning("[Mission AprilTag] FC not available during cleanup.")

            logger.info("[Mission AprilTag] Mission run finished.")

    def stop(self):
        """外部调用的停止任务方法"""
        logger.warning("[Mission AprilTag] Stop command received.")
        self.running = False
        try:
            # 如果无人机在空中，立即悬停
            if self.fc and self.fc.connected and self.fc.state.unlock.value:
                logger.info("[Mission AprilTag] Stabilizing drone due to stop command.")
                self.fc.stablize()
        except Exception as e:
             logger.error(f"[Mission AprilTag] Error stabilizing drone on stop: {e}")
