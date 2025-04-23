import os
import cv2
import threading
from datetime import datetime
from time import time, sleep


class My_Camera:
    """
    摄像头管理类：处理视频捕获、录制和画面叠加
    """
    def __init__(self, index=0, save_video=False, output_dir='output_videos', fps=30):
        """
        初始化摄像头管理器
        
        :param index: 摄像头索引
        :param save_video: 是否保存视频
        :param output_dir: 视频保存目录
        :param fps: 视频帧率
        """
        self.cam = cv2.VideoCapture(index)
        if not self.cam.isOpened():
            raise RuntimeError(f"无法打开索引为 {index} 的摄像头")
        
        # 视频录制配置
        self.save_video = save_video
        self.output_dir = output_dir
        self.fps = fps
        self.writer = None
        self.video_path = None
        
        # 线程控制
        self.recording = False
        self.frame_thread = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # 叠加信息
        self.fc = None
        self.mission = None
        
        # 初始化视频录制
        if save_video:
            self._init_video_writer()
    
    def _init_video_writer(self):
        """初始化视频写入器"""
        width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 创建视频文件名（使用时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_path = os.path.join(self.output_dir, f"mission_{timestamp}.avi")
        
        # 初始化视频写入器
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.writer = cv2.VideoWriter(self.video_path, fourcc, self.fps, (width, height))
        
        if not self.writer.isOpened():
            raise RuntimeError(f"无法创建视频写入器: {self.video_path}")
    
    def set_flight_controller(self, fc):
        """设置飞控引用"""
        self.fc = fc
    
    def set_mission(self, mission):
        """设置任务引用"""
        self.mission = mission
    
    def start(self):
        """启动摄像头和视频录制"""
        if self.recording:
            return
        
        self.recording = True
        self.frame_thread = threading.Thread(target=self._frame_loop, daemon=True)
        self.frame_thread.start()
    
    def _frame_loop(self):
        """帧处理循环"""
        interval = 1.0 / self.fps
        last_time = time()
        
        while self.recording:
            current_time = time()
            if current_time - last_time >= interval:
                ret, frame = self.cam.read()
                if not ret:
                    sleep(0.01)
                    continue
                
                # 添加叠加信息
                frame_with_overlay = self._add_overlay(frame)
                
                # 保存处理后的帧
                with self.frame_lock:
                    self.latest_frame = frame_with_overlay
                
                # 写入视频
                if self.save_video and self.writer:
                    self.writer.write(frame_with_overlay)
                
                last_time = current_time
            else:
                sleep(0.001)  # 短暂休眠以减少CPU占用
    
    def _add_overlay(self, frame):
        """
        向视频帧添加叠加信息
        
        :param frame: 原始帧
        :return: 添加了叠加信息的帧
        """
        if frame is None:
            return None
        
        # 复制一份帧以避免修改原始数据
        result = frame.copy()
        
        # 1. 添加飞控信息
        if self.fc and self.fc.connected:
            st = self.fc.state
            # 高度
            alt_info = f"alt: {st.alt.value} cm"
            cv2.putText(result, alt_info, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            # 速度
            speed_info = f"speed: x={st.vel_x.value:.1f}, y={st.vel_y.value:.1f} cm/s"
            cv2.putText(result, speed_info, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            # 模式
            mode_info = f"flight_mode: {st.mode.value}"
            cv2.putText(result, mode_info, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 2. 添加任务相关信息
        if self.mission:
            # 绘制检测框
            results = getattr(self.mission, 'last_results', None)
            detector = getattr(self.mission, 'detector', None)
            if results and results.boxes and detector and detector.names:
                for box in results.boxes:
                    cls_id = int(box.cls)
                    if cls_id < 0 or cls_id >= len(detector.names):
                        continue
                    label = detector.names[cls_id]
                    conf  = box.conf.item()
                    x1,y1,x2,y2 = box.xyxy[0].cpu().numpy().astype(int)
                    # 替换标签
                    display = 'apple' if label=='orange' else label
                    # 绘制
                    cv2.rectangle(result, (x1,y1),(x2,y2),(0,0,255),3)
                    cv2.putText(result, f'{display} {conf:.2f}', (x1,y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
        
            circle = getattr(self.mission, 'last_circle_info', None)
            if circle:
                cx, cy, radius = circle
                # 大圆
                cv2.circle(result, (cx, cy), radius, (0, 255, 0), 2)
                # 圆心小点
                cv2.circle(result, (cx, cy), 5, (0, 255, 0), -1)

        # 3. 添加时间戳
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(result, timestamp, (10, result.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return result
    
    def read_original(self):  
        frame = None

        ret,frame = self.cam.read()
        if not ret:
            return None
        
        return frame


    def read(self):
        """
        读取最新的一帧（带有叠加信息）
        
        :return: 带有叠加信息的视频帧，如果没有则返回None
        """
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
        
        # 如果没有缓存的帧，直接捕获一帧并添加叠加信息
        ret, frame = self.cam.read()
        if not ret:
            return None
        return self._add_overlay(frame)
    
    def stop(self):
        """停止摄像头和视频录制"""
        self.recording = False
        
        # 等待线程结束
        if self.frame_thread and self.frame_thread.is_alive():
            self.frame_thread.join(timeout=2.0)
        
        # 释放资源
        if self.writer:
            self.writer.release()
            self.writer = None
        
        if self.cam and self.cam.isOpened():
            self.cam.release()
            self.cam = None
        
        if self.save_video and self.video_path:
            print(f"视频已保存至: {self.video_path}")