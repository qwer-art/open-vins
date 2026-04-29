#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 数据保存目录
DATA_DIR="${SCRIPT_DIR}/data"

# 创建data目录（如果不存在）
mkdir -p ${DATA_DIR}

# 文件名：data/open_vins_{录制开始时间}
OUTPUT_NAME="${DATA_DIR}/open_vins_$(date +%Y%m%d_%H%M%S)"

# 录制话题
ros2 bag record -o ${OUTPUT_NAME} \
  /cam0/image_raw \
  /cam1/image_raw \
  /imu0 \
  /ov_msckf/poseimu \
  /ov_msckf/odomimu \
  /ov_msckf/points_msckf \
  /tf \
  /tf_static
