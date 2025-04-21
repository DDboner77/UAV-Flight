# 烧录镜像
首先，在你的电脑安装一个vmware，配置一个22.04的ubuntu系统，只能高不能低，因为jetpack6.2不支持低版本的ubuntu，结束后去nvidia安装一个sdk-manager，在里面连接好短接配置引脚的板卡，然后选择你需要安装的库，里面会有两个选项需要你打钩，第二个写着install的直接勾上，不然后面还有好多要配置的，
- 烧录完了准备好
  - 键盘
  - 鼠标
  - 显示屏
- 就可以直接连接jetson了


# 安装必要依赖和DL库

==这很重要!!!!==

在这之后，首先参考[这个文章](https://blog.csdn.net/qq_45233572/article/details/146396937) ，
安装libnvdla_compiler.so静态库，否则后续的DL库都跑不起来，
这是Nvidia那边没做好，一定要记得做

==这很重要!!!!==

然后参考[这篇文章](https://blog.csdn.net/lida2003/article/details/145322174),安装好
- torch-2.5.1+l4t36.4 
- torchvision 0.20.0

其中torchvision你也可以选择直接用`sudo apt install torchvision=0.20.0`，这并没有什么区别

但是torch一定要安装上面那个版本，否则没办法使用GPU进行加速推理


# YOLOv11测试
然后参考[这篇文章](https://elinux.org/Jetson/L4T/TRT_Customized_Example#YoloV11_using_Pytorch),完成yolov11的测试，准备一个摄像头或者视频都可以，只要你run通了，就说明板卡基本配置好了。

# SSH+VNC

jetson user:xuzi
password: 207207
ssh :xuzi-desktop.local

连接我设置的WIFI：robot-111，jetson的ip是动态的，最好查询一下再用ip ssh，使用mobaxtern或者putty可以直接命令行控制它，
如果需要远程桌面控制，执行`/usr/lib/vino/vino-server`就可以在vnc中直接远程桌面，不过很卡