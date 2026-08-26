## 跟踪目标的可视化

### 目录结构

```
target_visualizer/
├── package.xml         # ROS 包清单
├── setup.py            # Python 包安装配置
├── setup.cfg           # 打包配置
├── README.md
├── resource/
│   └── target_visualizer # ament_index 包注册标记（空文件）
├── target_visualizer/
│   ├── __init__.py       # 包初始化
│   └── target_visualizer.py # 可视化节点实现
└── test/
    ├── test_copyright.py # 版权检查
    ├── test_flake8.py    # 代码风格检查
    └── test_pep257.py    # 文档字符串检查
```
