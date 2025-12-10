#!/bin/bash
# 启动点云转换工具

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ ! -f "config.yaml" ]; then
    cp config.yaml.example config.yaml
fi

./pointcloud-converter
