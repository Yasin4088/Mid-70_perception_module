## 点云检测与目标跟踪

### 目录结构

```
pc_detector/
├── CMakeLists.txt      # 构建配置
├── package.xml         # ROS 包清单
├── README.md
├── include/
│   ├── Clustering.h       # DBSCAN 聚类实现
│   ├── KalmanFilter.h     # 卡尔曼滤波器类声明
│   ├── RosNode.h          # DetectorNode 节点类声明
│   ├── TargetMap.h        # 目标跟踪器类声明
│   ├── Utility.h          # 通用工具（BoundingBox、Z_Map 场地高度图）
│   ├── VisualizerHelper.h # 可视化辅助（点云选择窗口）
│   └── VoxelGrid.h        # 体素占据网格类声明
└── src/
    ├── KalmanFilter.cpp    # 卡尔曼滤波实现
    ├── RosNode_prepare.cpp # 节点初始化与参数声明
    ├── RosNode_solve.cpp   # 主解算流程
    ├── TargetMap.cpp       # 目标跟踪实现
    └── VoxelGrid.cpp       # 体素占据网格实现
```
