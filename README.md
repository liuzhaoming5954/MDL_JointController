# MDL_JointController
在V-REP中进行关节控制器级别的仿真
这个仿真项目是对上一篇论文的进一步发展。上一篇论文中，并没有把MDLg字符串的重构单元放到机器人的关节上。这个仿真项目将尝试使用V-REP的Joint callback function,把重构的功能放到关节上。   

项目计划：     

1、在小车上使用V-REP的回调函数，使用MDLg模型使小车走S型曲线轨迹，并进行对比；     

2、尝试使用关节控制器自编写功能（Joint callback function），把MDLg字符串的重构放在关节上，完成对比实验。
