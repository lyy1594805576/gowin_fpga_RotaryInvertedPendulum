# FPGA 旋转倒立摆

基于 Gowin **GW2A-LV55PG484C8/I7** 的旋转倒立摆（Furuta pendulum）控制工程，包含 Verilog RTL、引脚/时钟约束和 Python 上位机。

本仓库整理自 `jiexiev6` 的 2026-11-02 源码快照。控制 RTL 与原目录逐字节一致，保留当前标定值；没有将其他工程的后续版本合入。`SOURCE_SNAPSHOT.json` 记录来源文件的 SHA256。

## 功能

- 自动共振起摆、手动捕获和串级角度/位置 PID；角度控制周期约 3ms，位置环约 51ms。
- ADC 密集采样、截尾平均和毛刺抑制；角度 D 支路两级低通滤波。
- 编码器位置反馈、位置目标/保持命令、梯形定位轨迹和运动/静止控制逻辑。
- 可调摩擦前馈、USB 串口运行时调参和约 100Hz 的二进制遥测。
- Python 上位机：实时波形、轨迹/定位操作、参数设置及 CSV/原始串口记录。

此快照的顶层默认摩擦前馈 **KS=0**（可在线设置），自动角度增益为 **1750/47/27000**、位置增益为 **1400/8/5509**。平衡点、方向和编码器每圈计数见 `src/pendulum_ctrl.v`，引脚见 `src/fo_rip_top.cst`。顶层遥测标识沿用 `0x26102402`，不能单凭该标识判断其他目录/分支的控制实现相同。

## 目录

```text
src/                       当前启用的 RTL、引脚和 50MHz 时钟约束
jiangxiedaolibai.gprj       Gowin 工程入口（沿用原工程名称）
build.tcl / build.ps1       构建入口
verify_build.py            本次构建、时序和位流一致性检查
tools/auto_swing_logger.py  串口记录与调参界面
tools/position_host/        位置控制与实时波形上位机
sim/                       独立模块仿真及可移植运行器
docs/                      构建、协议、验证范围
```


## 构建 FPGA

安装 Gowin EDA，用 IDE 打开 `jiangxiedaolibai.gprj` 并构建；原工程使用 Gowin **V1.9.12.03**，目标时钟 **50MHz**。使用自己的 Gowin 安装位置：

```powershell
./build.ps1 -GowinShell 'C:/Gowin/IDE/bin/gw_sh.exe' -PythonExe python
```

也可将 `gw_sh` 加入 PATH 后运行 `gw_sh build.tcl`，再运行 `python verify_build.py`。只有新构建的 Fmax≥50MHz、setup/hold TNS均为0且源文件未过期，校验才会成功。**本次开源整理未重新综合/布局布线，不把旧报告当作当前构建证明。**

引脚约束针对原硬件，包括不同 IO 电压和拨码开关；移植前按自己的板卡调整约束和标定。构建不会自动烧录或启动电机。

## 运行上位机

Python 3.10+，需要 Tkinter。Windows 官方 Python 通常包含 Tk；Linux 可安装 `python3-tk`。

```bash
python -m pip install -r requirements.txt
python tools/auto_swing_logger.py
python tools/position_host/host_app.py --demo
```

位置上位机连接实物：`python tools/position_host/host_app.py --port COM5`，将 COM5 换成实际串口。USB 为 **115200、8N1**。同时只让一个上位机占用串口；`--demo` 不连接设备。更多入口见 [上位机说明](docs/HOST.md)。

## 离线验证

安装 Icarus Verilog（`iverilog`、`vvp` 加入 PATH）：

```bash
python sim/run_tests.py
python sim/run_tests.py --host-tests
```

Windows 若没有加入 PATH：

```powershell
python sim/run_tests.py --iverilog-bin 'C:/iverilog/bin' --host-tests
```

第二条还运行虚拟串口和隐藏 Tk 窗口测试，**不连接电机或实际串口**。无桌面的 Linux 使用 `xvfb-run -a python sim/run_tests.py --host-tests`。验证范围与限制见 [TESTING.md](docs/TESTING.md)；本仓库保留八项独立模块仿真，未包含依赖旧工程目录的历史对照回归，也不声称覆盖全部控制状态或实机闭环稳定性。

## 许可与来源

项目自有代码采用 [MIT License](LICENSE)。第三方标记与历史算法来源说明见 [NOTICE.md](NOTICE.md)，保留其原有权利声明。
