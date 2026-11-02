# 上位机入口

## 记录与 PID 调参

`python tools/auto_swing_logger.py`

提供串口发现、自动波特率检测、原始串口/CSV记录，以及 PID、位置和摩擦前馈页。当前固件支持的操作以 `src/usb_pid_command.v` 为准；旧实验固件的 op6 不适用于当前顶层。

日志自动写入仓库本地 `logs/`，默认被 Git 忽略。录制元数据中的本地构建清单是本地文件信息，不是板上程序哈希读回。

## 位置控制与波形

```bash
python tools/position_host/host_app.py --demo
python tools/position_host/host_app.py --port COM5
```

前者使用演示数据，后者连接实际串口。包含轨迹、位置误差与参数界面；轨迹曲线由上位机调度位置命令。显示的 PWM 为控制请求值，不是实测电压、力矩或速度。op5 保持/冻结位置目标，并不等价于硬件断电急停。

协议实现分别见 `tools/runtime_pid.py` 和 `tools/position_host/protocol.py`；RTL 接收端是最终依据。遥测 v3 的帧头为 A6 6A 03 6F，帧长111字节，CRC8，通常100Hz。USB命令帧以5A A5开始，六个有符号32位载荷，CRC8，0D 0A结尾。

## 离线分析

`python tools/analyze_swing_log.py --help` 列出读取已有日志的选项。分析器不会打开串口。记录包含实际采样值和控制请求，不能单凭电机侧编码器推断减速箱输出轴的真实间隙或力矩。
