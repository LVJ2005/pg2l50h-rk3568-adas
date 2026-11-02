# PG2L50H + RK3568 智能驾驶辅助演示

本仓库是可公开整理的工程副本，包含 FPGA 视频采集与 PCIe 传输工程、RK3568 板端 ADAS 程序、Windows 上位机和运行模型。板端应用版本：`ADAS-quad-V2.0.10`。

## 功能

- FPGA 采集摄像头画面，经 DDR3 和 PCIe 向 RK3568 传输，板端显示四宫格画面。
- RK3568 对左上第一象限进行行人、斑马线和交通灯检测，分类灯态，并识别车道实线与虚线。
- 上位机通过 UDP 发送直行、左转、右转及左右变道意图，分别显示灯控、变道、行人制动判断与综合建议。
- 对应方向红灯停车、绿灯放行；未识别到右转专用灯而识别到路口其他红绿灯时，给出“可右转、须让行”建议。虚线允许变道，实线警告。斑马线行人风险优先发出**软件制动请求**；工程没有连接车辆制动执行器。

## 目录

```text
fpga/fpga/source/                    FPGA RTL
fpga/fpga/project/project.pds       Pango 工程入口
fpga/fpga/project/project.fdc       约束
fpga/fpga/project/ipcore/           工程引用的 IP 文件
arm/arm/run.sh                      板端构建与启动脚本
arm/arm/driver/                     PCIe DMA 内核模块
arm/arm/app_pcie/includes/          C++ 头文件
arm/arm/app_pcie/sources/           当前应用源码与测试
arm/arm/app_pcie/models/            RKNN 模型和类别表
arm/arm/app_pcie/tools/recognition_control.py  Windows 上位机
```

## 环境与运行

FPGA 工程使用 Pango Design Suite / Fabric Compiler 2022.2-SP6.4，目标器件 PG2L50H FBG484 -6。打开 `fpga/fpga/project/project.pds`，检查 IP、约束和源文件，再生成与板端 PCIe 驱动匹配的位流。仓库不包含历史综合产物和位流。

RK3568 板端需要匹配当前内核的头文件、g++、Make、pkg-config、OpenCV、GTK、kmod 和兼容模型的 RKNN Runtime。`arm/arm/driver/Makefile` 默认内核头文件路径为 `/usr/src/linux-headers-6.1-rockchip`；不同系统可在执行前设置 `KERNELDIR`。在板端工程根目录运行：

```sh
cd arm/arm
sudo sh run.sh
```

脚本构建驱动与应用、加载驱动并运行程序。默认监听 UDP `0.0.0.0:9527`，使用 `app_pcie/models/` 中的浮点模型。屏幕显示需要板端图形会话；可用 `sudo sh run.sh --headless` 做无窗口诊断。启动成功后日志应出现 `ADAS-quad-V2.0.10`。

Windows 上位机只需 Python 3 和 Tkinter（建议 Python 3.9 以上），在本仓库根目录运行：

```powershell
python .\arm\arm\app_pcie\tools\recognition_control.py --host 172.20.10.2 --port 9527
```

上位机与板端应处于可互通的网络。先在界面确认左上第一象限是要测试的场景，再按需点击行驶和变道按钮。行人制动请求可单独开关。当前灯态有效期默认 1000 ms，车道线 900 ms；漏检超过有效期会显示未知。右转专用灯漏检也可能被当成“无专用灯”，因此本项目仅用于比赛演示与验证，不作为真实车辆决策系统。

## 本地测试

板端可在 `arm/arm/app_pcie` 下运行 `make test`。Windows 有 MinGW g++ 时可独立编译规则测试：

```powershell
cd arm\arm\app_pcie
g++ -std=c++17 -O0 -pthread -Iincludes sources/tests/adas_core_test.cpp sources/adas_rules.cpp sources/adas_runtime.cpp sources/detector_decode.cpp sources/lane_detector.cpp -o adas_core_test.exe
.\adas_core_test.exe
```

完整板端编译需要 RK3568 Linux、RKNN Runtime 和 OpenCV/GTK 开发库，普通 Windows 环境无法验证驱动和整机显示。

## 模型与外部组件

检测模型类别顺序见 `models/det_classes.txt`，灯态分类类别顺序见 `models/cls_classes.txt`。模型、RKNN API 头文件及 FPGA IP 可能受其各自来源的授权条款约束；公开发布前请核对这些文件的再分发许可。本仓库未对第三方组件另行授予许可证。
