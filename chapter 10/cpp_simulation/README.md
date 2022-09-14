# cpp_simulation
使用c++进行仿真，并绘制仿真以后的数据。

滤波器仿真包括卡尔曼滤波器仿真，均值滤波仿真。

控制模型仿真目前采用了PID仿真，以后会考虑加入ADRC以及LQR。

## 如何使用

环境：ubuntu 18.04

依次输入命令：

```bash
mkdir build
cmake ..
make
```
./filter_average是均值滤波，./filter_average是卡尔曼融合速度的滤波，./filter_kalman_pv表示的是卡尔曼融合速度与位置的滤波。

可能会存在python不兼容的问题，可以在cmake文件里面修改python的版本。注意python需要下载matplotlib库文件。
