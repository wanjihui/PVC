# Mcar WiFi 上位机

版本日期：2026-10-13。配套RT1064麦轮小车，保留原版六页布局，提供参数/曲线、日志、三维姿态、摄像头、快捷命令和实时调参。

## 直接使用

运行 [release/WiFiSPI_Monitor.exe](release/WiFiSPI_Monitor.exe)，无需安装Python。SHA256见 [release/SHA256.json](release/SHA256.json)。这是当前单文件版；历史EXE、用户配置和采集数据不打包进仓库。

1. 电脑与小车接入同一网络，在车端config设置电脑目标IP。
2. 单端口固件选择“单端口MUDP”，本机监听8081；兼容固件选择“旧版三端口”，使用8081/8082/8083。
3. 默认“视觉循迹”解析37个字段。点击开始监听，观察初始化日志、图像和数据；K1发车、K2停车由车端执行。
4. 调参命令发车辆模块6666；先读取参数，再逐项应用，收到车辆回执后才确认。

配套固件工程位于仓库 `project/mdk/rt1064_single_udp.uvprojx` 和 `rt1064_curve_legacy.uvprojx`。电脑接收模式须与下载的固件匹配。

## 当前功能

- 原始灰度与左右边界、中线、补线、拐点叠加；两条绿线显示本帧实际取值区域，默认前瞻60对应50/70行。上下各20行不显示处理线条，原灰度仍完整。
- 等比例适应画布、手动倍率、暂停显示、保存图像；暂停显示仍继续接收。
- 参数实时值/曲线，CSV和JSONL采集；日志支持时间戳、自动滚动、暂停和保存。
- 离线也能显示三维飞机模型，接收roll/pitch/yaw后更新姿态；模型来源记录见model_assets/PROVENANCE.md。
- 实时调参23项：巡航/起步/视觉/陀螺/横移10项，弯道13项。车端GET协议仍36项，四轮kp/ki/kd及ff不提供编辑，轮速/占空比遥测保留。
- 旧23/36项参数方案可导入，忽略速度环字段，不自动发送；新方案MCAR_TUNING_3只保存当前23项。
- 单端口MUDP将参数/日志/MCAM图像分到各页面，保留旧协议。识别车辆数据后发送ALIVE确认，配合车端恢复；不能把SPI成功视为电脑收到。

数据目录优先 `%LOCALAPPDATA%/McarWiFiMonitor_SingleUDP`，不可写时回退程序旁 `McarWiFiMonitorData_SingleUDP`。旧版McarWiFiMonitor目录不覆盖。

## 从源码运行与构建

需要64位Windows、Python3.12。进入本目录：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.\.venv\Scripts\python.exe app.py
```

构建：

```powershell
.\tools\build.ps1
```

构建先运行tests，再生成单文件EXE和便携文件夹版；打包必须包含model_assets。便携版移动时须保留整个_internal目录。发布目录写入tools/release_path.txt。

## 验证与维护

当前130项软件测试通过；原生界面确认23行参数、速度环控件已移除，36项回执、旧方案兼容及独立UDP回环通过。单文件EXE回环验证80条模拟参数、8幅完整图像和CSV/日志导出，测试仅使用本机端口。

主要维护文件：app.py负责界面/采集；live_tuning.py负责参数定义/回执/方案；packet_parser.py负责字段解析；transport.py与multiplex_protocol.py负责接收和分流；camera_protocol.py负责分片重组/CRC/叠线；attitude_3d.py、aircraft_model.py及model_assets负责姿态显示。

软件回环不代表实车参数最优，也不能证明所有无线故障或出弯控制问题已解决。实际情况需结合图像、车辆回执和完整CSV判断。
