# Annotation for Apollo

基于[apollo-v6.0.0](https://github.com/ApolloAuto/apollo/releases/tag/v6.0.0)为其**感知模块** **控制模块** **Cyber框架** 的源码作了细致解析和注释

* 在代码上传时，某些大文件(**主要是某些模型文件**)没有上传,下面给出了这些文件的详细路径，如果有需要的，可以去Apollo的github上下载，对应版本[apollo-v6.0.0](https://github.com/ApolloAuto/apollo/releases/tag/v6.0.0)

```shell
find ./ -type f -size +50000k  
```

```
modules/perception/testdata/camera/lib/lane/postprocessor/darkSCNN/data/darkSCNN/deploy.caffemodel
modules/perception/testdata/camera/lib/lane/postprocessor/denseline/data/denseline/deploy.caffemodel
modules/perception/testdata/camera/lib/lane/detector/darkSCNN/data/darkSCNN/deploy.caffemodel
modules/perception/testdata/camera/lib/lane/detector/denseline/data/denseline/deploy.caffemodel设计
modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/dark_SCNN_vpt_8x_lighter_iter_160000.caffemodel
modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/params
modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/dark_SCNN_8x_lighter_iter_160000.caffemodel
modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/deploy.caffemodel
modules/perception/production/data/perception/camera/models/lane_detector/denseline/deploy.caffemodel
```

## 一些工作

**本人邮箱：hanhy20@mails.jlu.edu.cn** **欢迎交流**。

与注释相关的文档，全部保存在**doc文件夹**中，为了便于查找本readme文件会提供相应查找跳转链接。

### Cyber



### Canbus



### Drivers



### Perception

针对**感知模块(Perception)**,   [ApolloAnnotation](https://github.com/HaiYangLib/ApolloAnnotation)做了以下工作：

对Radar,Liadr,Camera子模块的代码进行了详细的注释。

* Radar子模块的注释工作主要涉及：Radar数据预处理，IDMatch和匈牙利匹配算法的讲解

* Liadr子模块的注释工作主要涉及：Lidar数据预处理，Pointpillars算法实现过程的分析与代码注释



  

### Control

针对**感知模块(Perception)**,   [ApolloAnnotation](https://github.com/HaiYangLib/ApolloAnnotation)做了以下工作：



### Drivers



## 一些心得



