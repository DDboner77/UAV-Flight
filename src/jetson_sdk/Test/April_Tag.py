import cv2
import pyapriltags as apriltag # 使用 pyapriltags
import time
import os
from datetime import datetime
import numpy as np # pyapriltags 需要 numpy 处理角点

# --- 配置参数 ---
CAMERA_INDEX = 0  # 摄像头索引，根据实际情况修改 (例如 0 或 1)
# 对于 Jetson CSI 摄像头，可能需要使用 GStreamer 管道字符串
# CAMERA_INDEX = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

FRAME_WIDTH = 640   # 输出视频帧宽度
FRAME_HEIGHT = 480  # 输出视频帧高度
FPS = 30            # 输出视频帧率
OUTPUT_DIR = "apriltag_videos_pyapriltags" # 视频保存目录 (使用不同名称以区分)
TAG_FAMILY = 'tag16h5' # AprilTag 家族，确保与 pyapriltags 支持的名称一致 (e.g., 'tag16h5', 'tag25h9', 'tag36h11', 'tagCircle21h7', 'tagStandard41h12')
# --- 配置结束 ---

def main():
    print("Initializing pyapriltags detector...")
    # 初始化 pyapriltags 检测器
    # 可以添加其他参数如 nthreads=4 等来利用多核
    detector = apriltag.Detector(families=TAG_FAMILY, nthreads=1) # 根据需要调整 nthreads
    print(f"Using tag family: {TAG_FAMILY}")

    print(f"Opening camera index: {CAMERA_INDEX}...")
    # 打开摄像头
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Could not open camera at index {CAMERA_INDEX}")
        # 如果使用 GStreamer 字符串失败，尝试使用简单索引
        if isinstance(CAMERA_INDEX, str):
            print("Trying default camera index 0...")
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                 print("Error: Could not open camera with default index 0 either.")
                 return
        else:
            return

    # 设置摄像头分辨率 (如果摄像头支持)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    # 获取实际的帧宽度和高度
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Actual camera resolution: {actual_width}x{actual_height}, FPS: {actual_fps:.2f}")

    # --- 设置视频保存 ---
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"Created output directory: {OUTPUT_DIR}")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filename = os.path.join(OUTPUT_DIR, f"apriltag_{timestamp}.mp4")
    # 使用 MP4V 编码器，适用于 MP4 格式
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # 使用实际获取到的分辨率创建 VideoWriter
    out = cv2.VideoWriter(output_filename, fourcc, FPS, (actual_width, actual_height))
    print(f"Saving annotated video to: {output_filename}")
    # --- 视频保存设置结束 ---

    print("Starting detection loop... Press 'q' to quit.")
    frame_count = 0
    start_time = time.time()

    try:
        while True:
            # 读取一帧
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to grab frame.")
                break

            # 转换到灰度图进行检测
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 检测 AprilTags
            # pyapriltags 的 detect 方法需要灰度图和相机参数（可选，用于姿态估计）
            # 这里我们只做检测，不估计姿态，所以不传相机参数
            # ... (之前的代码) ...

            # 检测 AprilTags
            results = detector.detect(gray)

            # --- 过滤和绘制结果 ---
            min_decision_margin = 25 # 设置决策边界阈值 (需要调整)
            max_hamming_distance = 1   # 设置最大汉明距离 (0 或 1 通常较好)

            for r in results:
                # --- 应用过滤条件 ---
                if r.decision_margin < min_decision_margin:
                    # print(f"Skipping tag {r.tag_id} due to low decision margin: {r.decision_margin}")
                    continue # 跳过低置信度的检测

                if r.hamming > max_hamming_distance:
                    # print(f"Skipping tag {r.tag_id} due to high hamming distance: {r.hamming}")
                    continue # 跳过汉明距离过大的检测

                # --- 如果通过过滤，则绘制 ---
                # 提取角点 (pyapriltags 返回 numpy 数组)
                corners = r.corners.astype(int) # 转换为整数坐标
                (ptA, ptB, ptC, ptD) = corners

                # 绘制边界框
                cv2.polylines(frame, [corners], True, (0, 255, 0), 2) # 使用 polylines 更方便

                # 绘制中心点
                center = r.center.astype(int)
                (cX, cY) = center
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

                # 绘制 Tag ID 和置信度信息 (可选)
                tag_id = r.tag_id
                info_text = f"ID: {tag_id} (M: {r.decision_margin:.1f}, H: {r.hamming})"
                cv2.putText(frame, info_text, (ptA[0], ptA[1] - 15), # 使用角 A 作为文本位置参考
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) # 调整字体大小和颜色

                # 输出 Tag ID 到控制台
                print(f"Detected AprilTag ID: {tag_id} at ({cX}, {cY}), Margin: {r.decision_margin:.2f}, Hamming: {r.hamming}")

            # 写入视频帧
            out.write(frame)

            # 显示标注后的图像 (可选)
            # cv2.imshow("AprilTag Detection (pyapriltags)", frame)



            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Quit key pressed. Exiting...")
                break

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")
    finally:
        # 清理资源
        end_time = time.time()
        elapsed_time = end_time - start_time
        avg_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        print(f"\nProcessed {frame_count} frames in {elapsed_time:.2f} seconds (Avg FPS: {avg_fps:.2f})")

        print("Releasing resources...")
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        print("Cleanup complete.")

if __name__ == "__main__":
    main()