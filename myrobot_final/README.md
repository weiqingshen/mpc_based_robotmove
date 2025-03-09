# SRC

基本程序

arm_control.cpp/.h 机械臂运动规划主要封装的程序 包括主要的类 抓取 物体 执行

communication.cpp/.h 通讯程序 控制接受信息 阻断 接受状态机和相机信息

trajectory.cpp/h 轨迹保存和执行保存的轨迹 读取config里面的yaml 

trajectory_buffer.cpp 缓存轨迹点 分批次发送给fines_serial  发送下位机执行



执行程序

arm_execute_circle 转盘

arm_execute_processing 放下在拿起

arm_execute_storage 放下

arm_execute_total 整个流程

后面加了trajectory的是 中间有轨迹被保存 会直接执行轨迹而不是规划

# launch

只要用到demo就行了 直接运行demo启动程序 后面在运行src

# config

param.yaml记录物体的大小 位置的提前已知

circle11.yaml 轨迹

processing11.yaml 轨迹
