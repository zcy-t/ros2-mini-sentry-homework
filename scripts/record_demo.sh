#!/usr/bin/env bash
# 作业 3：录 1 分钟 bag（只录轻量话题，别把 /image 录进来）
#
# 两个版本差异（都在这处理了）：
#   Humble / Jazzy：ros2 bag record -o DIR /odom /tf ...
#   Lyrical 及以后：话题必须用 --topics 传，默认存储是 mcap 而不是 sqlite3
# 停止方式统一用 timeout 发 SIGINT（Lyrical 的 -d 是"单个文件最长时长"，不是总时长，
# 直接 Ctrl-C 或 SIGINT 才会正常写 metadata.yaml）
set -e

BAG_DIR="bags/hw3_$(date +%H%M%S)"
mkdir -p bags

if ros2 bag record -h 2>&1 | grep -q -- '--topics'; then
  echo "==> 检测到新版 rosbag2：用 --topics + sqlite3 存储"
  timeout -s INT 60 ros2 bag record -s sqlite3 -o "${BAG_DIR}" \
    --topics /odom /tf /tf_static /game_status || true
else
  echo "==> 检测到经典版 rosbag2：话题直接跟在 -o 后面"
  timeout -s INT 60 ros2 bag record -o "${BAG_DIR}" \
    /odom /tf /tf_static /game_status || true
fi

echo "==> 录制结束，摘要："
ros2 bag info "${BAG_DIR}"
echo
echo "回放：bash scripts/replay_demo.sh ${BAG_DIR}"
