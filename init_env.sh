#!/bin/bash

# OpenVINS 环境初始化脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 项目路径
OPENVINS_DIR="/home/jerett/OpenProject/LidarSlam/open-vins"
SRC_DIR="${OPENVINS_DIR}/src"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}     OpenVINS 环境初始化${NC}"
echo -e "${GREEN}========================================${NC}"

# 加载ROS2环境
echo -e "${YELLOW}[1/3] 加载ROS2环境...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}  ✓ ROS2 Humble 环境已加载${NC}"
else
    echo -e "${RED}  ✗ 未找到ROS2 Humble环境${NC}"
    return 1 2>/dev/null || exit 1
fi

# 编译检查
echo -e "${YELLOW}[2/3] 检查编译状态...${NC}"
if [ ! -f "${SRC_DIR}/install/setup.bash" ]; then
    echo -e "${YELLOW}  未检测到编译产物，开始编译...${NC}"

    # 检查Ceres库冲突
    if [ -f "/usr/local/lib/libceres.so" ] && [ ! -f "/tmp/libceres.so.bak" ]; then
        echo -e "${YELLOW}  检测到Ceres库冲突，需要手动处理：${NC}"
        echo -e "  sudo mv /usr/local/lib/libceres.so* /tmp/"
        echo -e "  sudo mv /usr/local/lib/cmake/Ceres /tmp/"
        echo -e "  sudo mv /usr/local/include/ceres /tmp/"
        echo -e "  sudo ldconfig"
        echo -e "${RED}  请先执行上述命令后重新运行此脚本${NC}"
        return 1 2>/dev/null || exit 1
    fi

    cd ${SRC_DIR}
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}  ✓ 编译成功${NC}"
    else
        echo -e "${RED}  ✗ 编译失败${NC}"
        return 1 2>/dev/null || exit 1
    fi
else
    echo -e "${GREEN}  ✓ 编译产物已存在${NC}"
fi

# 加载工作空间环境
echo -e "${YELLOW}[3/3] 加载OpenVINS工作空间...${NC}"
source ${SRC_DIR}/install/setup.bash
echo -e "${GREEN}  ✓ OpenVINS工作空间已加载${NC}"

export PATH="/snap/bin:$PATH"

# 设置常用别名
alias ov_run='ros2 launch ov_msckf subscribe.launch.py config:=tum_vi rviz_enable:=true'
alias ov_play='ros2 bag play ${SRC_DIR}/Data/dataset-room1_512_16_ros2 --clock'
alias ov_cd='cd ${OPENVINS_DIR}'
alias plotjuggler='/opt/ros/humble/lib/plotjuggler/plotjuggler'

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}     环境初始化完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "常用命令："
echo -e "  ov_run   - 启动OpenVINS节点"
echo -e "  ov_play  - 播放bag文件"
echo -e "  ov_cd    - 进入项目目录"
echo ""
echo -e "手动运行："
echo -e "  ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav rviz_enable:=true"
echo -e "  ros2 bag play ${SRC_DIR}/Data/dataset-room1_512_16_ros2 --clock"
echo ""
