#!/usr/bin/env bash
# 作业 3：带仿真时钟回放 bag（不 --clock 的话 RViz2 会报 extrapolation into the future）
set -e

BAG_DIR="${1:-$(ls -d bags/hw3_* 2>/dev/null | tail -n 1)}"
if [ -z "${BAG_DIR}" ] || [ ! -d "${BAG_DIR}" ]; then
  echo "找不到 bag：先跑 bash scripts/record_demo.sh" >&2
  exit 1
fi

# --clock 会多发一个 /clock 话题；订阅端节点要 use_sim_time:=true 才会用它
# --rate 是倍速，写成 --rate 2.0（别写成裸的 200）
echo "==> 回放 ${BAG_DIR}（--clock，2 倍速）"
echo "    RViz2：ros2 run rviz2 rviz2 -d config/sentry.rviz --ros-args -p use_sim_time:=true"
ros2 bag play "${BAG_DIR}" --clock --rate 2.0
