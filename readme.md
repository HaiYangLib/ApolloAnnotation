* 在代码上传时，我某些大文件(**主要是某些模型文件**)没有上传,下面给出了这些文件的详细路径，如果有需要的，可以去Apollo的github上下载，对应版本[apollo-v6.0.0](https://github.com/ApolloAuto/apollo/releases/tag/v6.0.0)

```shell
find ./ -type f -size +50000k  
```

```
./modules/perception/testdata/camera/lib/lane/postprocessor/darkSCNN/data/darkSCNN/deploy.caffemodel
./modules/perception/testdata/camera/lib/lane/postprocessor/denseline/data/denseline/deploy.caffemodel
./modules/perception/testdata/camera/lib/lane/detector/darkSCNN/data/darkSCNN/deploy.caffemodel
./modules/perception/testdata/camera/lib/lane/detector/denseline/data/denseline/deploy.caffemodel
./modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/dark_SCNN_vpt_8x_lighter_iter_160000.caffemodel
./modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/params
./modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/dark_SCNN_8x_lighter_iter_160000.caffemodel
./modules/perception/production/data/perception/camera/models/lane_detector/darkSCNN/deploy.caffemodel
./modules/perception/production/data/perception/camera/models/lane_detector/denseline/deploy.caffemodel
```

​                                                                              
 
