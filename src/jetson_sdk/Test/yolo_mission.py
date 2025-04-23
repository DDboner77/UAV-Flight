import cv2
import time
from time import sleep
from FlightController import logger
from FlightController.Solutions.yolo_runtime.yolo_runtime import Detector

class Mission:
    """
    任务类：仅使用 YOLO 检测器检测目标，检测结果存储于 self.last_results，
    Camera 管理器会读取该属性进行视频帧叠加和记录（视频保存由 Camera 管理器处理）。
    被 mission_manager 调用时执行 run() 方法。
    """

    def __init__(self, fc, cam_manager):
        """
        初始化任务。
        :param fc: FlightController 实例
        :param cam_manager: My_Camera 实例
        """
        self.fc = fc
        self.cam_manager = cam_manager
        self.running = False
        self.last_results = None

        # 初始化 YOLO Detector
        self.detector = None
        try:
            engine_path = 'FlightController/Solutions/yolo_runtime/yolov8n.engine'
            self.detector = Detector(engine_path=engine_path, conf_thresh=0.5)
            logger.info("[Mission YOLO Detection] YOLO Detector initialized successfully.")
            if not self.detector.names:
                logger.warning("[Mission YOLO Detection] Detector did not load class names correctly.")
        except Exception as e:
            logger.error(f"[Mission YOLO Detection] Failed to initialize YOLO Detector: {e}")
            self.detector = None

    def _read_frame(self):
        """
        从摄像头管理器获取最新帧。
        """
        frame = self.cam_manager.read_original()
        if frame is None:
            logger.warning("[Mission YOLO Detection] Failed to read frame from camera.")
        return frame

    def run(self):
        """
        执行 YOLO 目标检测任务，检测结果存储至 self.last_results，
        并由 Camera 管理器实时绘制和录制视频。
        """
        if self.detector is None:
            logger.error("[Mission YOLO Detection] Detector 未成功初始化，任务无法执行。")
            return

        logger.info("[Mission YOLO Detection] Starting mission run.")
        self.running = True
        start_time = time.time()
        run_duration = 60  # 任务运行总时长，可根据需求调整（单位：秒）

        try:
            while self.running and (time.time() - start_time < run_duration):
                frame = self._read_frame()
                if frame is None:
                    sleep(0.05)
                    continue

                # 使用 YOLO 检测目标
                _, results = self.detector.detect(frame)
                self.last_results = results  # 存储检测结果，Camera 管理器可用于叠加显示

                # 打印检测结果（供调试）
                if results and results.boxes:
                    for box in results.boxes:
                        cls_id = int(box.cls)
                        label = self.detector.names[cls_id] if 0 <= cls_id < len(self.detector.names) else "Unknown"
                        conf = box.conf.item()
                        logger.info(f"[Mission YOLO Detection] Detected: {label} with conf {conf:.2f}")
                sleep(0.1)
        except Exception as e:
            logger.error(f"[Mission YOLO Detection] 任务运行异常: {e}")
        finally:
            logger.info("[Mission YOLO Detection] Mission run finished.")

    def stop(self):
        """
        外部调用的停止任务方法。
        """
        logger.info("[Mission YOLO Detection] Stop command received.")
        self.running = False    