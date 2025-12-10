#!/bin/bash
# 点云转换工具 - 混合版本安装脚本

set -e

echo "========================================="
echo "  点云转换工具 - 安装"
echo "========================================="
echo ""

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 安装系统依赖
echo "[1/3] 安装系统依赖..."
sudo apt-get update
sudo apt-get install -y \
    python3 python3-pip \
    cmake build-essential \
    pdal libpdal-dev \
    liblas-bin liblas-dev \
    liblaszip-dev \
    libpcl-dev \
    libyaml-cpp-dev

# 安装 Python 依赖（可选，如果要修改 GUI）
echo "[2/3] 安装 Python 依赖..."
pip3 install --user -r requirements.txt

# 编译 C++ 工具
echo "[3/3] 编译 C++ 工具..."

# 编译 las2pcd
cd las2pcd
mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd "$SCRIPT_DIR"

# 编译 pointcloud_divider
cd pointcloud_divider-master
mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd "$SCRIPT_DIR"

# 生成配置文件
if [ ! -f "config.yaml" ]; then
    cp config.yaml.example config.yaml
fi

# 设置权限
chmod +x bin/pointcloud-converter

echo ""
echo "✅ 安装完成！"
echo ""
echo "启动程序: ./run.sh"
