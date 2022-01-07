# Annotation for Apollo

**本人邮箱：hanhy20@mails.jlu.edu.cn** **欢迎交流**。

**长期维护，持久工作**

基于[apollo-v6.0.0](https://github.com/ApolloAuto/apollo/releases/tag/v6.0.0)为其**感知模块** **规划** **Cyber(5.0)框架** 的源码作了细致解析和注释

* 在代码上传时，某些大文件(**主要是某些模型文件**)没有上传,下面给出了这些文件的详细路径，如果有需要的，可以去Apollo的github上下载，对应版本[apollo-v6.0.0](https://github.com/ApolloAuto/apollo/releases/tag/v6.0.0)

  

```c
*.caffemodel
*.model
*.caffemodel.metadata
*.onnx

modules/planning/data/model/*.pt

modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/params
modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/params
modules/perception/production/data/perception/camera/models/yolo_obstacle_detector/3d-r4-half/params
modules/perception/production/data/perception/camera/models/yolo_obstacle_detector/3d-yolo/params
modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/params
```

## 一些工作

与注释相关的文档，全部保存在**doc文件夹**中(详细内容未上传)，为了便于查找本readme文件会提供相应查找跳转链接。

### [Cyber](doc/cyber/cyber.md)

完成了**Cyber的cmake版移植**，详细代码见**cyber_cmake**

### [Perception](doc/perception/perception.md)

针对**感知模块(Perception)**,   [ApolloAnnotation](https://github.com/HaiYangLib/ApolloAnnotation)做了以下工作：

对Radar,Liadr,Camera子模块的代码进行了详细的注释。

* **Radar子模块**的注释工作主要涉及：Radar**数据预处理**，**IDMatch**和**匈牙利匹配**算法的讲解
* **Liadr子模块**的注释工作主要涉及：Lidar**数据预处理**，**Pointpillars算法**实现过程的分析与代码注释
* **Camera子模块**的注释工作暂时未做：

### [Prediction](doc/prediction/prediction.md)

暂时未做工作

### [Planning](doc/planning/Planning.md)

刚开始做注释



## 一些心得



