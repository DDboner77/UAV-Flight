import cv2
import sys
import os
import numpy as np

# 将父目录 (jetson_sdk) 添加到 Python 路径
# 以允许从 FlightController 导入
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..')
sys.path.append(parent_dir)

try:
    # 导入二维码扫描器类
    # Make sure the necessary model files (qrcode-yolov3-tiny.cfg, qrcode-yolov3-tiny.weights)
    # are present in FlightController/Solutions/models/
    from FlightController.Solutions.Vision_Net import QRScanner_Yolo
except ImportError as e:
    print(f"导入 QRScanner_Yolo 时出错: {e}")
    print("请确保脚本从 'Test' 目录运行，")
    print("或者相应地调整 sys.path.append。")
    print("同时确保已安装必要的依赖项（如 OpenCV）。")
    exit()
except Exception as e:
    print(f"初始化期间发生错误（可能是模型文件丢失或损坏）: {e}")
    print("请检查 '../FlightController/Solutions/models/' 中的模型文件。")
    exit()


# --- 配置 ---
# 替换为包含二维码的图像的实际路径
# 如果要使用摄像头，需要修改图像加载部分
IMAGE_PATH = 'path/to/your/qrcode_image.png' # <<<--- 在此处更改图像路径

CONF_THRESHOLD = 0.6 # 检测的置信度阈值
NMS_THRESHOLD = 0.5  # 非极大值抑制阈值
DRAW_OUTPUT = True   # 设置为 True 以在输出图像上绘制边界框
# ---

def main():
    # 初始化二维码扫描器
    print("正在初始化 QR 扫描器...")
    try:
        qr_scanner = QRScanner_Yolo(
            confThreshold=CONF_THRESHOLD,
            nmsThreshold=NMS_THRESHOLD,
            drawOutput=DRAW_OUTPUT
        )
        print("QR 扫描器初始化完成。")
    except cv2.error as e:
         print(f"初始化 QRScanner_Yolo 时发生 OpenCV 错误: {e}")
         print("这通常意味着无法加载模型文件。")
         print("请确认以下文件存在于 '../FlightController/Solutions/models/' 目录下:")
         print(" - qrcode-yolov3-tiny.cfg")
         print(" - qrcode-yolov3-tiny.weights")
         exit()
    except Exception as e:
        print(f"初始化 QRScanner_Yolo 时出错: {e}")
        print("请确保模型文件存在于 '../FlightController/Solutions/models/'")
        print("并且已正确安装所需的库（例如 OpenCV DNN 模块）。")
        exit()

    # 加载图像
    print(f"正在加载图像: {IMAGE_PATH}")
    if not os.path.exists(IMAGE_PATH) or not os.path.isfile(IMAGE_PATH):
        print(f"错误: 图像文件未找到或不是文件: {IMAGE_PATH}")
        print("请更新脚本中的 IMAGE_PATH 变量。")
        # 创建一个虚拟黑色图像，以便脚本可以继续运行以进行演示
        print("将使用虚拟黑色 640x480 图像代替。")
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # 在虚拟图像上添加文本
        cv2.putText(frame, "Dummy Image - No QR Code", (50, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    else:
        frame = cv2.imread(IMAGE_PATH)
        if frame is None:
            print(f"错误: 无法从 {IMAGE_PATH} 加载图像")
            exit()
        print("图像加载成功。")

    # 检测二维码
    print("正在检测二维码...")
    try:
        # 如果 drawOutput 为 True，detect 方法会就地修改 'frame'
        results = qr_scanner.detect(frame)
        print("检测完成。")
    except Exception as e:
        print(f"检测过程中出错: {e}")
        exit()

    # 打印结果
    if results:
        print(f"检测到 {len(results)} 个二维码:")
        for i, (center, confidence) in enumerate(results):
            print(f"  二维码 {i+1}: 中心=({center[0]}, {center[1]}), 置信度={confidence:.4f}")
    else:
        print("在图像中未检测到二维码。")

    # 显示图像
    print("正在显示结果图像。按任意键退出。")
    cv2.imshow("QR Code Detection Result", frame)
    cv2.waitKey(0) # 无限期等待，直到按下某个键
    cv2.destroyAllWindows()
    print("正在退出。")

if __name__ == "__main__":
    main()