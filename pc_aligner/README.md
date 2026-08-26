## 雷达点云与场地模型的配准

### 目录结构

```
pc_aligner/
├── CMakeLists.txt      # 构建配置
├── package.xml         # ROS 包清单
├── README.md
├── include/
│   └── aligner_node.hpp # AlignerNode 节点类声明
└── src/
    ├── aligner_node.cpp  # 节点初始化与点云采样订阅
    ├── auto_aligner.cpp  # 自动 ICP 配准
    ├── main.cpp          # 程序入口
    └── manual_aligner.cpp # 交互式手动配准
```
