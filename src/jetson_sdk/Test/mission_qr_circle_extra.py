import time
import cv2
import numpy as np
from FlightController import logger
from time import sleep
# --- 添加 YOLOv8 Detector 导入 ---
from FlightController.Solutions.yolo_runtime.yolo_runtime import Detector
import traceback # 用于打印详细错误
from pyzbar import pyzbar


# --- 占位符函数 ---
def detect_circle(image):
    """
    使用 OpenCV 的霍夫变换检测图像中的圆形。
    返回: 如果检测到圆形，则返回其中心和半径 (x, y, radius)；否则返回 None。
    """
    if image is None:
        logger.warning("[Mission QR Circle Extra] detect_circle received None image.")
        return None
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        rows = gray.shape[0]
        # 霍夫圆检测参数可能需要根据实际情况调整
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, rows / 8,
                                   param1=100, param2=50,
                                   minRadius=60, maxRadius=150) # 调整半径范围

        if circles is not None:
            circles = np.uint16(np.around(circles))
            # 返回第一个检测到的圆的信息
            center_x, center_y, radius = circles[0, 0]
            logger.debug(f"[Mission QR Circle Extra] Detected circle at ({center_x}, {center_y}) with radius {radius}")
            return (center_x, center_y, radius)
    except cv2.error as e:
        logger.error(f"[Mission QR Circle Extra] OpenCV error in detect_circle: {e}")
    except Exception as e:
        logger.error(f"[Mission QR Circle Extra] Error in detect_circle: {e}")

    logger.debug("[Mission QR Circle Extra] No circle detected in the current frame.")
    return None

# --- Mission 类 ---
class Mission:
    """
    任务类，整合了二维码识别、目标检测、环绕飞行和降落功能。
    新增功能：
    1. 同时检测到目标和圆时，调整高度至100cm并进行微动。
    2. 检测到'orange'时，在画面上绘制为'apple'。
    """

    def __init__(self, fc, cam_manager):
        """
        初始化任务。
        :param fc: FlightController 实例
        :param cam_manager: My_Camera 实例 (用于获取摄像头和帧)
        """
        self.fc = fc
        self.cam_manager = cam_manager
        self.cam = cam_manager.cam # 直接从管理器获取摄像头对象
        self.running = False

        # 存储检测结果的属性
        self.last_bbox = None
        self.last_class = None

    ################################任务参数####################################
        self.take_off_alt = 60 # cm 起飞高度
        self.adjust_alt_cm = 100 # cm 检测到目标和圆后的调整高度
        self.micro_move_dist_cm = 10 # cm 微动距离
        self.micro_move_speed_cm_s = 5 # cm/s 微动速度

        self.forward_dis = 20 # cm 搜索二维码时的前进距离
        self.forward_speed = 20 # cm/s 搜索二维码时的前进速度
        self.forward_angle = 0 # 前进角度

        self.circle_yaw_rate = 90 # deg/s 环绕飞行时的偏航速率
        self.search_yaw_rate = 15 # deg/s 搜索目标时的偏航速率
    ################################任务参数####################################

        # --- 初始化 YOLO Detector ---
        self.detector = None
        try:
            # 确保路径相对于 mission_manager.py 或使用绝对路径
            engine_path = 'FlightController/Solutions/yolo_runtime/yolov8n.engine' # 或者根据实际情况调整
            self.detector = Detector(engine_path=engine_path, conf_thresh=0.5) # 使用 Detector 类
            logger.info("[Mission QR Circle Extra] YOLO Detector initialized successfully.")
            if not self.detector.names:
                 logger.warning("[Mission QR Circle Extra] Detector did not load class names correctly.")
        except FileNotFoundError:
             logger.error(f"[Mission QR Circle Extra] Failed to find YOLO engine file at: {engine_path}")
             self.detector = None
        except Exception as e:
            logger.error(f"[Mission QR Circle Extra] Failed to initialize YOLO Detector: {e}")
            logger.error(traceback.format_exc())
            self.detector = None

        logger.info("[Mission QR Circle Extra] Initialized")

    def _read_frame(self):
        """从摄像头管理器获取最新帧"""
        frame = self.cam_manager.read_original()
        if frame is None:
            # logger.warning("[Mission QR Circle Extra] Failed to read frame from camera manager")
            pass
        return frame


    def _find_qr_code(self, search_timeout=30):
        """
        向前飞行并搜索二维码。
        :param search_timeout: 搜索超时时间（秒）
        :return: 解码后的二维码数据字符串，如果未找到或超时则返回 None。
        """

        logger.info("[Mission QR Circle Extra] Searching for QR code...")
        start_time = time.time()
        qr_data = None

        while self.running and (time.time() - start_time) < search_timeout:
            frame = self._read_frame()
            if frame is None:
                sleep(0.1)
                continue

            # 使用原始帧进行二维码检测
            barcodes = pyzbar.decode(frame)
            if barcodes:
                barcode_data = barcodes[0].data.decode("utf-8")
                logger.info(f"[Mission QR Circle Extra] Found QR Code: {barcode_data}")
                qr_data = barcode_data
                break # 找到后退出循环
            else:
                # 未找到二维码，缓慢向前移动
                logger.debug("[Mission QR Circle Extra] QR not found, moving forward slowly")
                if self.fc and self.fc.connected and self.fc.state.unlock.value:
                    try:
                        # 使用程控模式进行精确移动
                        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
                
                        self.fc.horizontal_move(self.forward_dis, self.forward_speed, self.forward_angle)
                        sleep(0.1) # 移动一小段时间再检测
            
                    except Exception as move_err:
                         logger.error(f"[Mission QR Circle Extra] Error moving forward during QR search: {move_err}")
                         sleep(1) # 等待一下再尝试
                else:
                    logger.warning("[Mission QR Circle Extra] Cannot move forward, FC not ready or drone locked.")
                    sleep(1) # 等待飞控状态变化

        # 停止移动
        if self.fc and self.fc.connected:
            try:
                self.fc.stablize()
            except Exception as e:
                 logger.error(f"[Mission QR Circle Extra] Error stabilizing after QR search: {e}")

        if qr_data is None:
             logger.warning(f"[Mission QR Circle Extra] QR code not found within {search_timeout} seconds.")

        return qr_data

    def _fly_direction(self, direction, distance_m=1.2, speed=0.2):
        """
        向指定方向飞行一定距离。
        :param direction: "left","right","forward","backward"
        :param distance_m: 米
        :param speed: 米/秒
        """
        logger.info(f"[Mission QR Circle Extra] Flying {direction} for {distance_m}m at {speed}m/s")
        distance_cm = int(distance_m * 100)
        speed_cm_s = int(speed * 100)
        angle_map = {"forward": 0, "right": 90, "backward": 180, "left": 270}

        if direction not in angle_map:
            logger.warning(f"[Mission QR Circle Extra] Unknown direction: {direction}. Not moving.")
            return

        if not (self.fc and self.fc.connected and self.fc.state.unlock.value):
             logger.warning(f"[Mission QR Circle Extra] Cannot fly direction, FC not ready or drone locked.")
             return

        deg = angle_map[direction]
        try:
            self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
            sleep(0.2)
            self.fc.horizontal_move(distance_cm, speed_cm_s, deg)
            # 等待指令执行（注意：horizontal_move可能不是阻塞的，需要适当等待）
            # 这里的等待时间需要根据实际情况调整
            #wait_time = max(1.0, distance_cm / speed_cm_s ) # 估算移动时间并增加余量
            wait_time = 3.0 # 固定等待时间
            logger.info(f"[Mission QR Circle Extra] Waiting {wait_time:.1f}s for move to complete...")
            sleep(wait_time)

            self.fc.stablize()
            logger.info(f"[Mission QR Circle Extra] Finished flying {direction}.")
            sleep(1) # 悬停稳定
        except Exception as e:
            logger.error(f"[Mission QR Circle Extra] Error during directional flight: {e}")
            try: # 尝试悬停
                self.fc.stablize()
            except: pass

    def _find_target_and_circle(self, target_class, search_timeout=60):
        """
        搜索指定的对象类别和圆形靶标，并在找到两者时执行高度调整和微动。
        :param target_class: 需要检测的对象类别名称
        :param search_timeout: 搜索超时时间（秒）
        :return: 如果找到两者并完成调整，返回圆形信息 (x, y, radius)；否则返回 None。
        """
        logger.info(f"[Mission QR Circle Extra] Searching for target '{target_class}' and circle...")
        target_detected = False
        circle_info = None
        start_time = time.time()
        search_pattern_active = False # 标记是否在执行搜索模式

        while self.running and (time.time() - start_time) < search_timeout:
            frame = self._read_frame()
            if frame is None:
                sleep(0.1)
                continue


            # 1. YOLO 检测
            if self.detector:
                _, results = self.detector.detect(frame)
                self.last_results = results  # 保存供 Camera 绘制

                # 从检测结果中查找目标类别
                if results and results.boxes:
                    for box in results.boxes:
                        cls_id = int(box.cls)
                        if 0 <= cls_id < len(self.detector.names):
                            label = self.detector.names[cls_id]
                            logger.info(f"[DEBUG] Detected: {label} with conf {box.conf.item()}")
                            conf = box.conf.item()
                            if label == target_class and conf >= self.detector.conf:
                                if not target_detected:
                                    logger.info(f"[Mission QR Circle Extra] Target '{target_class}' detected.")
                                target_detected = True
                                bbox = box.xyxy[0].cpu().numpy().astype(int)
                                target_pos = ((bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2)
                                break # 找到第一个即可

            # 2. 圆形检测
            current_circle_info = detect_circle(frame)
            self.last_circle_info = current_circle_info  # 保存供 Camera 绘制
            if current_circle_info:
                if circle_info is None:
                    logger.info("[Mission QR Circle Extra] Circle detected.")
                circle_info = current_circle_info

            # 3. 检查是否都找到
            if target_detected and circle_info:
                logger.info("[Mission QR Circle Extra] Found both target and circle.")
                if self.fc and self.fc.connected:
                    try:
                        if search_pattern_active:
                            logger.info("[Mission QR Circle Extra] Stopping search pattern.")
                            self.fc.stablize() # 停止搜索动作
                            search_pattern_active = False
                            sleep(0.5)
                    except Exception as e:
                        logger.error(f"[Mission QR Circle Extra] Error stabilizing after finding target/circle: {e}")

                # <<< --- 高度调整与微动 --- >>>
                logger.info(f"[Mission QR Circle Extra] Adjusting altitude to {self.adjust_alt_cm}cm and performing micro-movements.")
                adjustment_success = self._adjust_altitude_and_move(self.adjust_alt_cm, self.micro_move_dist_cm, self.micro_move_speed_cm_s)
                if not adjustment_success:
                    logger.warning("[Mission QR Circle Extra] Altitude/Movement adjustment failed or was interrupted. Continuing mission...")
                
                return circle_info # 返回圆形信息

            # 4. 如果没都找到，执行搜索模式 (例如缓慢旋转)
            if not (target_detected and circle_info):
                if self.fc and self.fc.connected and self.fc.state.unlock.value:
                    if not search_pattern_active:
                        logger.info("[Mission QR Circle Extra] Target or circle not found, starting search pattern (slow yaw)...")
                        try:
                            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
                            sleep(0.2)
                            self.fc.send_realtime_control_data(0, 0, 0, yaw=self.search_yaw_rate)
                            search_pattern_active = True
                        except Exception as e:
                            logger.error(f"[Mission QR Circle Extra] Error starting search pattern: {e}")
                            sleep(0.5)
                    else:
                        # 持续发送实时控制命令，保持偏航
                        try:
                            self.fc.send_realtime_control_data(0, 0, 0, yaw=self.search_yaw_rate)
                        except Exception as e:
                             logger.error(f"[Mission QR Circle Extra] Error sending continuous yaw command: {e}")
                        sleep(0.2) # 避免过于频繁地发送命令
                else:
                    if not self.running: break # 如果任务已停止，退出循环
                    logger.warning("[Mission QR Circle Extra] Cannot start/continue search pattern, FC not ready or drone locked.")
                    sleep(1)

        # 超时或任务停止
        if self.fc and self.fc.connected:
            try:
                if search_pattern_active:
                    logger.info("[Mission QR Circle Extra] Stopping search pattern due to timeout or stop.")
                    self.fc.stablize() # 停止搜索动作
            except Exception as e:
                 logger.error(f"[Mission QR Circle Extra] Error stabilizing after search timeout: {e}")

        if not (target_detected and circle_info):
             logger.warning(f"[Mission QR Circle Extra] Failed to find both target and circle within {search_timeout} seconds.")
        return None # 未找到或超时

    def _adjust_altitude_and_move(self, target_alt_cm, move_dist_cm, move_speed_cm_s):
        """
        调整无人机高度至目标高度，并执行前后左右的微小移动。
        :param target_alt_cm: 目标高度 (cm)
        :param move_dist_cm: 微动距离 (cm)
        :param move_speed_cm_s: 微动速度 (cm/s)
        :return: True 如果成功完成, False 如果失败或中断
        """
        if not (self.fc and self.fc.connected and self.fc.state.unlock.value):
            logger.warning("[Mission QR Circle Extra] Cannot adjust altitude/move, FC not ready or drone locked.")
            return False

        try:
            # --- 1. 调整高度 ---
            logger.info(f"[Mission QR Circle Extra] Adjusting altitude to {target_alt_cm}cm...")
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE) # 定点模式方便控制速度
            sleep(0.5)

            alt_tolerance_cm = 20
            max_alt_adjust_time_s = 5 # 增加超时时间
            alt_adjust_start_time = time.time()
            kp_alt = 0.6 # P控制器增益 (需要调整!)

            alt_error = target_alt_cm - self.fc.state.alt_add.value # 获取当前高度
            current_alt_cm = self.fc.state.alt_add.value
            self.fc.go_up(alt_error,20)

            sleep(0.2)    
            logger.info(f"[Mission QR Circle Extra] Altitude {current_alt_cm:.1f}cm reached.")
            self.fc.send_realtime_control_data(0, 0, 0, 0) # 停止垂直移动
            # --- 2. 微动 ---
            logger.info("[Mission QR Circle Extra] Performing micro-movements...")
            self.fc.set_flight_mode(self.fc.PROGRAM_MODE) # 程控模式用于精确移动
            sleep(0.5)
            move_wait_time = max(1.0, move_dist_cm / move_speed_cm_s * 1.5) # 等待时间

            moves = [("forward", 0), ("backward", 180), ("left", 270), ("right", 90)]
            for move_name, angle in moves:
                if not self.running: return False # 检查任务状态
                logger.info(f"[Mission QR Circle Extra] Moving {move_name} slightly.")
                self.fc.horizontal_move(move_dist_cm, move_speed_cm_s, angle)
                sleep(move_wait_time)
                self.fc.stablize() # 每次移动后悬停一下
                sleep(0.5)

            self.fc.stablize() # 最终悬停
            logger.info("[Mission QR Circle Extra] Micro-movements complete.")
            return True

        except Exception as e:
            logger.error(f"[Mission QR Circle Extra] Error during altitude/movement adjustment: {e}")
            logger.error(traceback.format_exc())
            try: # 尝试悬停
                self.fc.stablize()
            except: pass
            return False

    def _circle_around_target(self, circle_info):
        """
        环绕检测到的圆形靶标飞行一圈。
        :param circle_info: (x, y, radius_px) - 当前未使用，但保留接口
        """
        if not (self.fc and self.fc.connected and self.fc.state.unlock.value):
            logger.warning("[Mission QR Circle Extra] Cannot perform circle maneuver, FC not ready or drone locked.")
            return False

        yaw_rate = self.circle_yaw_rate
        duration = 360.0 / abs(yaw_rate) # 完成360°需要的秒数
        logger.info(f"[Mission QR Circle Extra] Performing circle maneuver: yaw={yaw_rate}deg/s for {duration:.1f}s")

        try:
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE) # 定点模式下发实时偏航指令
            time_started = time.time()
            sleep(0.2)

            while self.running and (time.time() - time_started) < duration:
                self.fc.send_realtime_control_data(0, 0, 0, yaw=yaw_rate)
                sleep(0.2) # 保持 >1Hz 发送

            self.fc.stablize() # 停止偏航并悬停
            logger.info("[Mission QR Circle Extra] Circling maneuver complete.")
            sleep(1)
            return True
        except Exception as e:
            logger.error(f"[Mission QR Circle Extra] Error during circling maneuver: {e}")
            try:
                self.fc.stablize()
            except: pass
            return False

    def _land_at_center(self, circle_info):
        """
        在检测到的圆形靶标中心降落 (简化实现)。
        :param circle_info: 圆形信息 (x, y, radius_px) - 可用于未来的视觉伺服
        """
        logger.info("[Mission QR Circle Extra] Preparing to land at target center (simplified).")

        if not (self.fc and self.fc.connected and self.fc.state.unlock.value):
            logger.warning("[Mission QR Circle Extra] Cannot land, FC not ready or drone locked.")
            return False

        try:
            logger.info("[Mission QR Circle Extra] Initiating landing sequence.")
            self.fc.land()
            logger.info("[Mission QR Circle Extra] Waiting for landing and lock...")
            # 等待自动上锁，增加超时时间
            ret = self.fc.wait_for_lock(3)
            if not ret:
                logger.warning("[Mission QR Circle Extra] Drone did not lock automatically after landing command. Forcing lock.")
                try:
                    self.fc.lock() # 尝试强制锁定
                    sleep(1)
                    if not self.fc.state.unlock.value:
                         logger.info("[Mission QR Circle Extra] Force lock successful.")
                         return True
                    else:
                         logger.error("[Mission QR Circle Extra] Force lock failed.")
                         return False
                except Exception as lock_err:
                    logger.error(f"[Mission QR Circle Extra] Error forcing lock: {lock_err}")
                return False

            logger.info("[Mission QR Circle Extra] Landed and locked successfully.")
            return True
        except Exception as e:
            logger.error(f"[Mission QR Circle Extra] Error during landing sequence: {e}")
            return False

    def run(self):
        """执行完整的任务流程"""
        self.running = True
        logger.info("========================================")
        logger.info("[Mission QR Circle Extra] Starting mission run.")
        logger.info("========================================")

        # --- 检查 Detector 是否初始化成功 ---
        if self.detector is None:
            logger.error("[Mission QR Circle Extra] Cannot start mission, Detector failed to initialize.")
            self.running = False
            return

        mission_success = False
        try:
            # --- 0. 检查飞控和解锁 ---
            if not self.fc or not self.fc.connected:
                 raise RuntimeError("Flight controller not connected.")
            if not self.fc.state.unlock.value:
                logger.info("[Mission QR Circle Extra] Unlocking drone...")
                self.fc.unlock()
                sleep(2) # 等待解锁稳定
                if not self.fc.state.unlock.value:
                     raise RuntimeError("Failed to unlock drone.")
            logger.info("[Mission QR Circle Extra] Drone unlocked.")

            # --- 1. 起飞 ---
            logger.info(f"[Mission QR Circle Extra] Step 1: take_off to {self.take_off_alt}cm")
            self.fc.take_off(self.take_off_alt)
            logger.info("[Mission QR Circle Extra] Waiting for take_off altitude...")
            # 简单的延时等待起飞完成，更可靠的方式是检查高度
            sleep(3) # 等待起飞完成
            # 检查是否仍在运行且已解锁（起飞失败可能导致上锁）
            if not self.running or not self.fc.state.unlock.value:
                 logger.error("[Mission QR Circle Extra] take_off failed or mission stopped during take_off.")
                 raise RuntimeError("take_off failed")
            logger.info("[Mission QR Circle Extra] take_off possibly complete. Stabilizing...")
            self.fc.stablize()
            sleep(2) # 起飞后悬停稳定

            # --- 2. 搜索并解码二维码 ---
            logger.info("[Mission QR Circle Extra] Step 2: Find and Decode QR Code")
            qr_data_str = self._find_qr_code()
            if not qr_data_str or not self.running:
                raise RuntimeError("Failed to find QR code or mission stopped.")

            # --- 3. 解析二维码数据 ---
            logger.info("[Mission QR Circle Extra] Step 3: Parse QR Code Data")
            try:
                parts = qr_data_str.split(',')
                if len(parts) != 3:
                    raise ValueError(f"Invalid QR code format. Expected 3 parts, got {len(parts)}: '{qr_data_str}'")
                landing_trigger_class = parts[0].strip()

                ### 处理 'orange' 和 'apple' 的特殊情况
                if landing_trigger_class == 'apple':
                    landing_trigger_class = 'orange'

                ###
                # target_object_id = int(parts[1].strip()) # ID 暂时未使用
                direction = parts[2].strip().lower()
                if direction not in ["left", "right", "forward", "backward"]:
                     raise ValueError(f"Invalid direction in QR code: '{direction}'")
                logger.info(f"[Mission QR Circle Extra] QR Decoded: Landing Trigger='{landing_trigger_class}', Direction='{direction}'")
            except Exception as e:
                logger.error(f"[Mission QR Circle Extra] Failed to parse QR code data '{qr_data_str}': {e}")
                raise RuntimeError("QR code parsing failed")

            # --- 4. 向指定方向飞行 ---
            logger.info(f"[Mission QR Circle Extra] Step 4: Fly {direction}")
            self._fly_direction(direction) # 使用默认距离和速度
            if not self.running: return # 检查任务是否在飞行中停止

            # --- 5. 搜索目标和圆形靶标 (包含调整与微动) ---
            logger.info(f"[Mission QR Circle Extra] Step 5: Find Target ('{landing_trigger_class}') and Circle")
            circle_details = self._find_target_and_circle(landing_trigger_class, search_timeout=60) # 增加超时
            if not circle_details or not self.running:
                 raise RuntimeError("Failed to find target/circle or mission stopped.")

            # --- 6. 环绕靶标飞行 ---
            logger.info("[Mission QR Circle Extra] Step 6: Circle Around Target")
            if not self._circle_around_target(circle_details):
                 raise RuntimeError("Circling maneuver failed or was interrupted.")
            if not self.running: return # 检查任务是否在环绕中停止

            # --- 7. 降落 ---
            logger.info("[Mission QR Circle Extra] Step 7: Land at Target Center")
            if not self._land_at_center(circle_details):
                raise RuntimeError("Landing sequence failed or did not complete.")

            # --- 任务成功 ---
            logger.info("===========================================")
            logger.info("[Mission QR Circle Extra] Mission completed successfully!")
            logger.info("===========================================")
            mission_success = True

        except Exception as e:
            logger.error(f"[Mission QR Circle Extra] An error occurred during the mission: {e}")
            logger.error(traceback.format_exc())
        finally:
            # --- 任务结束处理 ---
            logger.info("[Mission QR Circle Extra] Entering final cleanup phase.")
            self.running = False # 确保运行状态为 False

            if self.fc and self.fc.connected:
                # 只有在任务失败或中断且无人机还在空中时才尝试降落
                if not mission_success and self.fc.state.unlock.value:
                    logger.warning("[Mission QR Circle Extra] Mission failed or was interrupted while flying. Attempting emergency land.")
                    try:
                        self.fc.land()
                        self.fc.wait_for_lock(5) # 等待降落完成
                        if self.fc.state.unlock.value:
                             logger.warning("[Mission QR Circle Extra] Forcing lock after emergency land attempt.")
                             self.fc.lock()
                    except Exception as land_err:
                        logger.error(f"[Mission QR Circle Extra] Error during emergency land/lock: {land_err}")
                elif not self.fc.state.unlock.value:
                     logger.info("[Mission QR Circle Extra] Drone is already locked.")
                else: # mission_success is True
                     logger.info("[Mission QR Circle Extra] Mission successful, drone should be locked.")
            else:
                logger.warning("[Mission QR Circle Extra] FC not available during cleanup.")

            logger.info("[Mission QR Circle Extra] Mission run finished.")

    def stop(self):
        """外部调用的停止任务方法"""
        logger.warning("[Mission QR Circle Extra] Stop command received.")
        self.running = False
        # 尝试悬停无人机（如果它正在飞行）
        try:
            if self.fc and self.fc.connected and self.fc.state.unlock.value:
                logger.info("[Mission QR Circle Extra] Stabilizing drone due to stop command.")
                self.fc.stablize()
        except Exception as e:
             logger.error(f"[Mission QR Circle Extra] Error stabilizing drone on stop: {e}")

