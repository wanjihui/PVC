# 原版飞机模型

`aircraft_high.obj` 和 `aircraft_low.obj` 从用户提供的
`C:/Users/Lenovo/Desktop/WiFiSPI_UDP_IMU_Model(1).exe` 内提取，保持原始文件内容。
这两个文件就是原版 IMU 姿态页使用的实体飞机模型。

原版 `aircraft_model` 模块的文档字符串说明“读取第三方 CC0 OBJ 飞机网格”；
EXE 中未发现单独的模型许可文件或来源网址，因此这里只记录该内嵌说明，
不额外推断作者或下载来源。OBJ 文件自带 Art of Illusion 导出信息。

新的 `aircraft_model.py` 根据原版字节码恢复坐标变换、归一化和法线计算；
`attitude_3d.py` 根据原版反汇编恢复相机投影、网格、深度排序和颜色分区，
并补充精确俯视、可变绘图区尺寸和未收到数据时的零姿态显示。
原始代码注释与完整原始源码未能逐字恢复。
