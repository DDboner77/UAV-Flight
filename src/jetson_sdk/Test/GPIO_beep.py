import RPi.GPIO as GPIO
import time

# 配置区
PIN = 9          # 使用 BCM 编号，GPIO18（BOARD 编号 12）
INTERVAL = 0.5    # 切换间隔（秒）

def main():
    # 1. 初始化
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.OUT, initial=GPIO.LOW)

    state = False
    print(f"开始在 GPIO{PIN} 上输出高低电平，按 Ctrl+C 停止")
    try:
        while True:
            # 2. 切换电平
            state = not state
            GPIO.output(PIN, GPIO.HIGH if state else GPIO.LOW)
            print(f"[{time.strftime('%H:%M:%S')}] GPIO{PIN} -> {'HIGH' if state else 'LOW'}")
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        print("已中断，退出程序")
    finally:
        # 3. 清理
        GPIO.cleanup()
        print("GPIO 已清理")

if __name__ == '__main__':
    main()