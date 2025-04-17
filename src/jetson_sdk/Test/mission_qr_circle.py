import time
import cv2
import numpy as np
from FlightController import logger
from time import sleep
# --- 添加 YOLOv5 Detector 导入 ---
from FlightController.Solutions.yolo_runtime.yolo_runtime import Detector
import traceback # 用于打印详细错误

from pyzbar import pyzbar
PYZBAR_AVAILABLE = True


# --- 移除旧的占位符函数 ---
# def detect_object(image, target_class):
#     """
#     占位符：检测图像中指定类别的对象。
#     在实际应用中，这里会调用您的对象检测模型。
#     返回: 如果检测到对象，则返回对象中心坐标 (x, y)；否则返回 None。
#     """
#     # 模拟检测逻辑
#     logger.debug(f"[Mission] Simulating detection of '{target_class}'...")
#     # 假设有时能检测到目标在图像中心附近
#     if np.random.rand() > 0.7:
#         h, w = image.shape[:2]
#         center_x, center_y = w // 2 + np.random.randint(-20, 20), h // 2 + np.random.randint(-20, 20)
#         logger.info(f"[Mission] Simulated detection of '{target_class}' at ({center_x}, {center_y})")
#         return (center_x, center_y)
#     return None

def detect_circle(image):
    """
    占位符：使用 OpenCV 的霍夫变换检测图像中的圆形。
    返回: 如果检测到圆形，则返回其中心和半径 (x, y, radius)；否则返回 None。
    """
    # ... existing detect_circle code ...
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
        logger.info(f"[Mission] Detected circle at ({center_x}, {center_y}) with radius {radius}")
        return (center_x, center_y, radius)
    logger.debug("[Mission] No circle detected in the current frame.")
    return None


# --- Mission 类 ---

class Mission:


    def __init__(self, fc, cam, video_writer):
        """
        初始化任务。
        :param fc: FlightController 实例
        :param cam: Camera (cv2.VideoCapture) 实例
        :param video_writer: VideoWriter 实例或 None
        """
        self.fc = fc
        self.cam = cam
        self.running = False
        if video_writer :
            self.video_writer = video_writer # 视频写入器实例

    ################################任务参数####################################
        self.take_off_alt = 100 # cm

        self.forward_dis = 20 # cm
        self.forward_speed = 20 # cm/s
        self.forward_angle = 0 # 



    ################################任务参数####################################




        # --- 初始化 YOLOv5 Detector ---
        self.detector = None
        try:
            # 假设 yolov5_runtime.py, best.onnx, category.names 都在 FlightController/Solutions/yolo_runtime/ 目录下
            self.detector = Detector(engine_path='FlightController/Solutions/yolo_runtime/yolov8n.engine')
            # 可选：根据需要调整检测器参数
            # self.detector.threshold = 0.6
            # self.detector.iou_thres = 0.5
            logger.info("[Mission QR Circle] YOLOv5 Detector initialized successfully.")
            logger.info(f"[Mission QR Circle] Detector using model: {self.detector.weights}")
            logger.info(f"[Mission QR Circle] Detector class names: {self.detector.names}")
        except Exception as e:
            logger.error(f"[Mission QR Circle] Failed to initialize YOLOv5 Detector: {e}")
            logger.error(traceback.format_exc())
            # 在这种情况下，self.detector 将保持为 None

        logger.info("[Mission QR Circle] Initialized")

    def _detect_object(self, image, target_class):
        """
        使用 YOLOv11 检测图像中指定类别的对象。
        :param image: 输入图像 (NumPy array)
        :param target_class: 目标类别的名称 (字符串)
        :return: 如果检测到目标类的第一个实例，返回其中心坐标 (x, y)；否则返回 None。
        """
        if self.detector is None:
            logger.warning("[Mission QR Circle] Detector is not available, cannot detect objects.")
            return None
        if image is None:
            logger.warning("[Mission QR Circle] Input image is None, skipping detection.")
            return None

        try:
            # 调用新接口，得到标注图和结果对象
            annotated_frame, result = self.detector.detect(image)
            # 提取 boxes, confidences, class_ids
            boxes       = result.boxes.xyxy.cpu().numpy()   # [[x1,y1,x2,y2], ...]
            confidences = result.boxes.conf.cpu().numpy()   # [c1, c2, ...]
            class_ids   = result.boxes.cls.cpu().numpy().astype(int)  # [id1, id2, ...]

            if boxes.shape[0] == 0:
                logger.debug(f"[Mission] No objects detected by YOLOv11.")
                return None

            for bbox, conf, cid in zip(boxes, confidences, class_ids):
                # 置信度过滤
                if conf < self.detector.conf:
                    continue
                # 类别ID 验证
                if cid < 0 or cid >= len(self.detector.names):
                    continue
                class_name = self.detector.names[cid]
                logger.debug(f"[Mission] YOLOv11 detected: {class_name} (conf: {conf:.2f})")
                # 匹配目标类别
                if class_name == target_class:
                    x1, y1, x2, y2 = bbox.astype(int)
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    logger.info(f"[Mission] Found target '{target_class}' at ({cx}, {cy}) with confidence {conf:.2f}")
                    return (cx, cy)

            logger.debug(f"[Mission] Target '{target_class}' not found among detected objects.")
            return None

        except Exception as e:
            logger.error(f"[Mission QR Circle] Error during YOLOv11 detection: {e}")
            logger.error(traceback.format_exc())
            return None


    def _read_frame(self):
        """从摄像头读取一帧图像,并写入视频文件"""
        # ... existing _read_frame code ...
        if not self.cam or not self.cam.isOpened():
             logger.error("[Mission QR Circle] Camera is not available.")
             return None
        ret, frame = self.cam.read()
        if not ret:
            logger.warning("[Mission QR Circle] Failed to read frame from camera")
            return None

        if self.video_writer is not None:
            # 将当前帧写入视频文件
            self.video_writer.write(frame)
        return frame


    def _find_qr_code(self, search_timeout=30):
        """
        向前飞行并搜索二维码。
        :param search_timeout: 搜索超时时间（秒）
        :return: 解码后的二维码数据字符串，如果未找到或超时则返回 None。
        """
        # ... existing _find_qr_code code ...
        if not PYZBAR_AVAILABLE:
            logger.error("[Mission QR Circle] Cannot search for QR code, pyzbar is not available.")
            return None

        logger.info("[Mission QR Circle] Searching for QR code...")
        start_time = time.time()
        qr_data = None

        while self.running and (time.time() - start_time) < search_timeout:
            frame = self._read_frame()
            if frame is None:
                sleep(0.1)
                continue

            barcodes = pyzbar.decode(frame)
            if barcodes:
                # 找到第一个有效的二维码
                barcode_data = barcodes[0].data.decode("utf-8")
                logger.info(f"[Mission QR Circle] Found QR Code: {barcode_data}")
                qr_data = barcode_data
                break # 找到后退出循环
            else:
                # 未找到二维码，缓慢向前移动
                logger.debug("[Mission QR Circle] QR not found, moving forward slowly")
                # 检查飞控是否连接且无人机是否解锁
                if self.fc and self.fc.connected and self.fc.state.unlock.value:
                    self.fc.move(self.forward_dis,self.forward_speed,self.forward_angle) # 调整速度
                    sleep(0.2) # 移动一小段时间再检测
                else:
                    logger.warning("[Mission QR Circle] Cannot move forward, FC not ready or drone locked.")
                    sleep(0.5) # 等待一下

        # 停止移动
        if self.fc and self.fc.connected:
            self.fc.stablize()

        if qr_data is None:
             logger.warning(f"[Mission QR Circle] QR code not found within {search_timeout} seconds.")

        return qr_data


    def _fly_direction(self, direction, distance_m=3.0, speed=0.5):
        """
        向指定方向飞行一定距离。
        :param direction: 方向 ("left", "right", "forward", "backward")
        :param distance_m: 飞行距离（米）
        :param speed: 飞行速度（米/秒）
        """
        # ... existing _fly_direction code ...
        logger.info(f"[Mission QR Circle] Flying {direction} for {distance_m}m at {speed}m/s")
        duration = distance_m / speed if speed > 0 else 0

        if duration <= 0:
             logger.warning(f"[Mission QR Circle] Invalid distance or speed for movement.")
             return

        move_params = {'duration': duration}
        if direction == "left":
            move_params['left'] = speed
        elif direction == "right":
            move_params['right'] = speed
        elif direction == "forward":
             move_params['forward'] = speed
        elif direction == "backward":
             move_params['backward'] = speed
        else:
            logger.warning(f"[Mission QR Circle] Unknown direction: {direction}. Not moving.")
            return

        if self.fc and self.fc.connected and self.fc.state.unlock.value:
            self.fc.move(**move_params)
            self.fc.stablize() # 确保移动结束后稳定悬停
            logger.info(f"[Mission QR Circle] Finished flying {direction}.")
            sleep(1) # 稳定悬停时间
        else:
            logger.warning(f"[Mission QR Circle] Cannot fly {direction}, FC not ready or drone locked.")


    def _find_target_and_circle(self, target_class, search_timeout=60):
        """
        搜索指定的对象类别和圆形靶标。
        可能需要进行搜索飞行模式（例如缓慢旋转或小范围移动）。
        :param target_class: 需要检测的对象类别名称 (e.g., "rabbit")
        :param search_timeout: 搜索超时时间（秒）
        :return: 如果同时找到目标和圆形，返回圆形信息 (x, y, radius)；否则返回 None。
        """
        logger.info(f"[Mission QR Circle] Searching for target '{target_class}' and circle...")
        target_detected = False
        circle_info = None
        start_time = time.time()

        search_pattern_active = False # 标记是否在执行搜索模式

        while self.running and (time.time() - start_time) < search_timeout:
            frame = self._read_frame()
            if frame is None:
                sleep(0.1)
                continue

            # --- 1. 检测目标类别 (e.g., "rabbit") ---
            # --- 修改调用点 ---
            target_pos = self._detect_object(frame, target_class)
            # --- 结束修改 ---
            if target_pos:
                if not target_detected:
                     logger.info(f"[Mission QR Circle] Target '{target_class}' detected.")
                target_detected = True
                # 可选：根据目标位置调整无人机姿态，使其居中

            # 2. 检测圆形
            current_circle_info = detect_circle(frame)
            if current_circle_info:
                if circle_info is None: # 首次检测到
                    logger.info(f"[Mission QR Circle] Circle detected.")
                circle_info = current_circle_info
                # 可选：根据圆形位置调整无人机姿态，使其居中

            # 3. 检查是否都找到
            if target_detected and circle_info:
                logger.info("[Mission QR Circle] Found both target and circle.")
                if self.fc and self.fc.connected:
                    self.fc.stablize() # 停止搜索动作
                return circle_info

            # 4. 如果没都找到，执行搜索模式 (例如缓慢旋转)
            if not search_pattern_active:
                 logger.info("[Mission QR Circle] Target or circle not found, starting search pattern (slow yaw)...")
                 if self.fc and self.fc.connected and self.fc.state.unlock.value:
                     self.fc.move(yaw_rate=15) # 开始缓慢向右转动 (15 deg/s)
                     search_pattern_active = True
                 else:
                     logger.warning("[Mission QR Circle] Cannot start search pattern, FC not ready or drone locked.")
                     sleep(0.5)
            else:
                 # 保持旋转或其他搜索动作
                 sleep(0.1) # 避免过于频繁地发送命令或检查

        # 超时或任务停止
        if self.fc and self.fc.connected:
            self.fc.stablize() # 停止搜索动作
        if not (target_detected and circle_info):
             logger.warning(f"[Mission QR Circle] Failed to find both target and circle within {search_timeout} seconds.")
        return None

    def _circle_around_target(self, circle_info, circle_radius_m=1.0):
        """
        环绕检测到的圆形靶标飞行一圈。
        这是一个简化的实现，仅执行360度偏航旋转。
        真实的环绕飞行需要更复杂的控制逻辑（视觉伺服或GPS航点）。
        :param circle_info: 圆形信息 (x, y, radius_px) - 当前未使用，未来可用于视觉伺服
        :param circle_radius_m: 环绕飞行的半径（米） - 当前未使用
        """
        # ... existing _circle_around_target code ...
        logger.info(f"[Mission QR Circle] Performing simplified circle maneuver (360 degree yaw turn).")

        yaw_rate = 30 # degrees per second
        circle_duration = 360.0 / yaw_rate
        logger.info(f"[Mission QR Circle] Starting yaw turn: rate={yaw_rate} deg/s, duration={circle_duration:.1f}s")

        if not (self.fc and self.fc.connected and self.fc.state.unlock.value):
            logger.warning("[Mission QR Circle] Cannot perform circle maneuver, FC not ready or drone locked.")
            return False

        self.fc.move(yaw_rate=yaw_rate) # 开始旋转
        start_time = time.time()
        while self.running and (time.time() - start_time) < circle_duration:
            sleep(0.1)
            # 检查是否意外停止
            if not self.fc.state.unlock.value:
                logger.warning("[Mission QR Circle] Drone became locked during circling maneuver.")
                self.fc.stablize() # 尝试停止旋转
                return False

        self.fc.stablize() # 停止旋转
        logger.info("[Mission QR Circle] Circling maneuver complete.")
        sleep(1) # 稳定时间
        return True


    def _land_at_center(self, circle_info):
        """
        在检测到的圆形靶标中心降落。
        这是一个简化的实现，直接执行降落命令。
        精确降落需要视觉伺服来对准中心。
        :param circle_info: 圆形信息 (x, y, radius_px) - 可用于未来的视觉伺服
        """
        # ... existing _land_at_center code ...
        logger.info("[Mission QR Circle] Preparing to land at target center (simplified).")

        if not (self.fc and self.fc.connected and self.fc.state.unlock.value):
            logger.warning("[Mission QR Circle] Cannot land, FC not ready or drone locked.")
            return False

        logger.info("[Mission QR Circle] Initiating landing sequence.")
        self.fc.land()
        logger.info("[Mission QR Circle] Waiting for landing and lock...")
        ret = self.fc.wait_for_lock(timeout=90)
        if not ret:
            logger.warning("[Mission QR Circle] Drone did not lock automatically after landing command. Forcing lock.")
            try:
                self.fc.lock() # 尝试强制锁定
            except Exception as lock_err:
                logger.error(f"[Mission QR Circle] Error forcing lock: {lock_err}")
            return False

        logger.info("[Mission QR Circle] Landed and locked successfully.")
        return True


    def run(self):
        """执行完整的任务流程"""
        # ... existing run code ...
        # (确保 run 方法中的逻辑能正确处理 self.detector 为 None 的情况，
        #  例如在调用 _find_target_and_circle 之前检查 self.detector 是否成功初始化)
        self.running = True
        logger.info("========================================")
        logger.info("[Mission QR Circle] Starting mission run.")
        logger.info("========================================")

        # --- 检查 Detector 是否初始化成功 ---
        if self.detector is None:
            logger.error("[Mission QR Circle] Cannot start mission, Detector failed to initialize.")
            self.running = False
            return # 提前退出 run 方法

        mission_success = False
        try:
            # --- 1. 起飞 ---
            logger.info(f"[Mission QR Circle] Step 1: take_off to {self.take_off_alt}m")
            # ... (起飞逻辑不变) ...
            if not self.fc.state.unlock.value:
                logger.info("[Mission QR Circle] Unlocking drone...")
                self.fc.unlock()
                sleep(1.5) # 等待解锁稳定
                if not self.fc.state.unlock.value:
                     raise RuntimeError("Failed to unlock drone.")


            # 假设 take_off 需要厘米
            self.fc.take_off(self.take_off_alt ) 
            logger.info("[Mission QR Circle] Waiting for take_off altitude...")
            if not self.running:
                 logger.error("[Mission QR Circle] take_off failed, timed out, or mission stopped.")
                 raise RuntimeError("take_off failed")
            logger.info("[Mission QR Circle] take_off complete. Stabilizing...")
            self.fc.stablize()
            sleep(2) # 起飞后悬停稳定


            # --- 2. 搜索并解码二维码 ---
            logger.info("[Mission QR Circle] Step 2: Find and Decode QR Code")
            qr_data_str = self._find_qr_code()
            if not qr_data_str or not self.running:
                raise RuntimeError("Failed to find QR code or mission stopped.")

            # --- 3. 解析二维码数据 ---
            logger.info("[Mission QR Circle] Step 3: Parse QR Code Data")
            # ... (解析逻辑不变) ...
            try:
                parts = qr_data_str.split(',')
                if len(parts) != 3:
                    raise ValueError(f"Invalid QR code format. Expected 3 parts, got {len(parts)}: '{qr_data_str}'")
                landing_trigger_class = parts[0].strip()
                direction = parts[2].strip().lower() # 转为小写以方便比较
                if direction not in ["left", "right", "forward", "backward"]:
                     raise ValueError(f"Invalid direction in QR code: '{direction}'")
                logger.info(f"[Mission QR Circle] QR Decoded: Landing Trigger='{landing_trigger_class}', Direction='{direction}'")
            except Exception as e:
                logger.error(f"[Mission QR Circle] Failed to parse QR code data '{qr_data_str}': {e}")
                raise RuntimeError("QR code parsing failed")


            # --- 4. 向指定方向飞行 ---
            logger.info(f"[Mission QR Circle] Step 4: Fly {direction}")
            self._fly_direction(direction) # 使用默认距离和速度
            if not self.running: return # 检查任务是否在飞行中停止

            # --- 5. 搜索目标和圆形靶标 ---
            logger.info(f"[Mission QR Circle] Step 5: Find Target ('{landing_trigger_class}') and Circle")
            circle_details = self._find_target_and_circle(landing_trigger_class)
            if not circle_details or not self.running:
                 raise RuntimeError("Failed to find target/circle or mission stopped.")

            # --- 6. 环绕靶标飞行 ---
            logger.info("[Mission QR Circle] Step 6: Circle Around Target")
            if not self._circle_around_target(circle_details):
                 raise RuntimeError("Circling maneuver failed or was interrupted.")
            if not self.running: return # 检查任务是否在环绕中停止

            # --- 7. 降落 ---
            logger.info("[Mission QR Circle] Step 7: Land at Target Center")
            if not self._land_at_center(circle_details):
                raise RuntimeError("Landing sequence failed or did not complete.")

            # --- 任务成功 ---
            logger.info("===========================================")
            logger.info("[Mission QR Circle] Mission completed successfully!")
            logger.info("===========================================")
            mission_success = True

        except Exception as e:
            logger.error(f"[Mission QR Circle] An error occurred during the mission: {e}")
            logger.error(traceback.format_exc())
        finally:
            # --- 任务结束处理 ---
            # ... (finally 块逻辑不变) ...
            logger.info("[Mission QR Circle] Entering final cleanup phase.")
            self.running = False # 确保运行状态为 False

            if self.fc and self.fc.connected:
                if self.fc.state.unlock.value and not mission_success:
                    logger.warning("[Mission QR Circle] Mission failed or was interrupted while flying. Attempting emergency land.")
                    try:
                        self.fc.land()
                        self.fc.wait_for_lock(30) # 给紧急降落一些时间
                        if self.fc.state.unlock.value:
                             logger.warning("[Mission QR Circle] Forcing lock after emergency land attempt.")
                             self.fc.lock()
                    except Exception as land_err:
                        logger.error(f"[Mission QR Circle] Error during emergency land: {land_err}")
                elif not self.fc.state.unlock.value:
                     logger.info("[Mission QR Circle] Drone is already locked.")
                else:
                     logger.info("[Mission QR Circle] Mission successful, drone should be locked.")
            else:
                logger.warning("[Mission QR Circle] FC not available during cleanup.")


            logger.info("[Mission QR Circle] Mission run finished.")


    def stop(self):
        """外部调用的停止任务方法"""
        # ... existing stop code ...
        logger.warning("[Mission QR Circle] Stop command received.")
        self.running = False
        try:
            if self.fc and self.fc.connected and self.fc.state.unlock.value:
                logger.info("[Mission QR Circle] Stabilizing drone due to stop command.")
                self.fc.stablize()
        except Exception as e:
             logger.error(f"[Mission QR Circle] Error stabilizing drone on stop: {e}")
