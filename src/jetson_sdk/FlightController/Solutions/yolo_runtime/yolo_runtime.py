import os
import sys
import time
import cv2
from ultralytics import YOLO

class  Detector:
    """
    封装 TensorRT 引擎推理，提供 detect(frame) → (annotated, result) 接口
    """
    def __init__(self, engine_path: str, conf_thresh: float = 0.5):
        """
        参数:
          engine_path: TensorRT 引擎文件
          conf_thresh: 检测置信度阈值
        """
        if not os.path.isfile(engine_path):
            raise FileNotFoundError(f"Engine not found: {engine_path}")
        self.engine_path = engine_path
        self.conf = conf_thresh

        # 加载模型
        print(f"[INFO] Loading engine: {engine_path}")
        self.model = YOLO(engine_path, task='detect')
        # print("[INFO] Warming up...")
        # # 矩阵或图片路径都可
        # _ = self.model(cv2.imread('https://ultralytics.com/images/bus.jpg') or 
        #                cv2.UMat(1,1,cv2.CV_8U), verbose=False)
        # print("[INFO] Warmup done.")

    def detect(self, frame):
        """
        对单帧图像做推理并返回标注图 & 原始结果
        返回:
          annotated_frame: 绘制边框和置信度后的 BGR 图
          result: ultralytics Results 对象（包含 boxes, masks…）
        """
        res = self.model(frame, verbose=False, conf=self.conf)[0]
        annotated = res.plot()
        return annotated, res

    def release(self):
        """释放资源（目前由 Python GC 处理）"""
        pass


# 修改代码使用无GUI方式
if __name__ == "__main__":
    # -------------------- 自测部分 --------------------
    ENGINE = sys.argv[1] if len(sys.argv) > 1 else 'yolov8n.engine'
    CAM_IDX = 0

    try:
        detector = Detector(ENGINE, conf_thresh=0.5)
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    cap = cv2.VideoCapture(CAM_IDX)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {CAM_IDX}")
        sys.exit(1)
    print(f"[INFO] Camera {CAM_IDX} opened. Press Ctrl+C to quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            _, results = detector.detect(frame)
            # 打印检测到的对象而不是显示
            for box in results.boxes:
                print(f"Detected: {results.names[int(box.cls)]} with confidence {box.conf.item():.2f}")
            
            time.sleep(0.1)  # 避免过快的循环
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cap.release()
        print("[INFO] Exit.")