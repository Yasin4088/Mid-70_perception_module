## 模块间数据接口定义

### 目录结构

```
radar_interface/
├── CMakeLists.txt      # 构建配置
├── package.xml         # ROS 包清单
├── README.md
├── include/
│   └── livox_struct.hpp # Livox 点结构定义与消息校验
├── msg/
│   ├── Target.msg      # 单个目标消息定义
│   └── TargetArray.msg # 目标数组消息定义
└── srv/
    └── AutoAlign.srv   # 自动配准服务定义
```
