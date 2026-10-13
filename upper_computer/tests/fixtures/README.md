# 协议回归样本

这三份二进制是本机运行真实C协议写入函数、配合模拟外设生成的软件测试样本，不是实车数据。随测试提供，避免依赖开发电脑上另一个工程的绝对位置。

- single_udp_wire.bin：由车端review_single_port.py生成，涵盖MUDP参数/日志/MCAM、完整图像及前瞻元数据。
- speed_packet.bin与vision_camera_packets.bin：由review_vision_wifi.py相关源码测试生成，验证小端字段偏移、图像分片与叠线解析。

测试文件中的示例前瞻35/55等值是固定协议样本，不表示当前车端默认前瞻。当前默认60对应50/70，软件样本也不能作为传感器实测证据。
