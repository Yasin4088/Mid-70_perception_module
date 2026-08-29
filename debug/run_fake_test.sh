#!/bin/bash
# 26 场地背景滤波联调测试：detector + 假数据 4 簇，观察哪些目标被检出
source /opt/ros/humble/setup.bash
source /home/robot/Mid-70_perception_module/install/setup.bash
ros2 daemon stop >/dev/null 2>&1

LOG=/tmp/test_launch.log
rm -f $LOG

# 1. 启动 launch（不开真雷达/aligner/visualizer/rviz；静态 TF identity -> world==场地坐标）
timeout 90 ros2 launch /home/robot/Mid-70_perception_module/launch/mid70_perception_launch.py \
    lidar:=false align:=false visualizer:=false rviz:=false > $LOG 2>&1 &
LAUNCH_PID=$!

# 2. 等 detector 拿到 world TF 并订阅 pc_raw
READY=0
for i in $(seq 1 45); do
    if grep -q "Preparing lidar" $LOG 2>/dev/null; then READY=1; break; fi
    sleep 1
done
if [ "$READY" != "1" ]; then
    echo "FAIL: detector 未就绪"; tail -30 $LOG; kill $LAUNCH_PID 2>/dev/null; exit 1
fi
echo "detector 就绪，启动假数据源..."

# 3. 假数据源（与本脚本同目录）
python3 "$(dirname "$0")/fake_livox_pub.py" > /tmp/test_pub.log 2>&1 &
PUB_PID=$!
sleep 12

# 4. 监听 10 秒 targets
timeout 10 python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from radar_interface.msg import TargetArray

class Sniffer(Node):
    def __init__(self):
        super().__init__('targets_sniffer')
        self.n = 0
        self.sub = self.create_subscription(TargetArray, 'pc_detector/targets', self.cb, 10)
        self.timer = self.create_timer(10.0, lambda: rclpy.shutdown())
    def cb(self, msg):
        self.n += 1
        if self.n % 20 == 0:  # 每 2s 打印一帧
            for t in msg.targets:
                print(f"target id={t.id} pos=({t.position[0]:5.2f},{t.position[1]:5.2f}) observed_z={t.observed_pos[2]:.2f}")

rclpy.init()
sn = Sniffer()
rclpy.spin(sn)
EOF

# 5. 清理
kill $PUB_PID $LAUNCH_PID 2>/dev/null
sleep 1
echo "=== detector 日志尾部 ==="
grep -vE "TF_SELF_TRANSFORM|TF_NO_" $LOG | tail -15
