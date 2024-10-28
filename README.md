# 命名规则
## 变量
1.单单词:dlg  
2.多单词:StateOnRobot
## 类、函数
1.单单词:Initial  
2.多单词:CheckStateOnRobot
## 文件夹
1.单单词:manager  
2.多单词:work_station  
## 文件
1.单单词:manager  
2.多单词:console_gui  

# 备忘录
1.最后修改完代码清理CMakeLists依赖
2.要使用tf2的dotransform函数需要tf2_geometry_msgs与tf2_sensor_msgs包,具体因为模板函数需要对传入的结构进行底层操作

## 坐标转换
1.robot.coor是相对于创建点的坐标(odom)  
2.robot.tf是创建点相对于世界坐标系的坐标  
3.计算路径path是世界坐标系,因此计算速度和距离时需要进行坐标转换