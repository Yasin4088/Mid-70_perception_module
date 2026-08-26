## 接受激光雷达数据

### 目录结构

```
livox_lidar/
├── CMakeLists.txt      # 构建配置
├── package.xml         # ROS 包清单
├── README.md
├── include/
│   ├── fast_crc.hpp    # 数据包 CRC 校验实现
│   ├── protocal.hpp    # Livox 协议常量与数据结构定义
│   └── publisher.hpp   # LidarPublisher 节点类声明
└── src/
    └── publisher.cpp   # 节点实现：UDP 收包与点云发布
```