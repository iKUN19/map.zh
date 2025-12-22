# 点云转换工具

用于处理和转换 LAS 格式点云数据的工具集，包含格式转换、点云分割等功能。

## 系统要求

- Ubuntu 18.04 或更高版本（推荐 20.04/22.04）
- 至少 4GB 内存
- CMake 3.5+
- GCC/G++ 支持 C++17

## 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/iKUN19/map.zh.git
cd map.zh
```

### 2. 运行安装脚本

```bash
./setup.sh
```

安装脚本会自动完成以下步骤：
- 安装系统依赖（PCL, libLAS, PDAL 等）
- 安装 Python 依赖（PyQt5, PyYAML）
- 编译 libLAS 库并安装到系统
- 编译 las2pcd 工具（LAS 转 PCD 格式）
- 编译 pointcloud_divider 工具（点云分割）
- 生成配置文件

### 3. 运行程序

```bash
./run.sh
```

## 工具说明

### las2pcd
将 LAS 格式转换为 PCD 格式的工具：
- `las2pcd` - 基础转换
- `las2pcd_intensity` - 包含强度信息的转换
- `pcd_enhancer` - PCD 文件增强

### pointcloud_divider
点云分割工具，支持：
- 网格化分割
- 下采样
- 批量处理

### pointcloud-converter
图形化界面工具（预编译二进制）

## 配置文件

编辑 `config.yaml` 来自定义工具路径和默认参数：

```yaml
executables:
  las2pcd: "las2pcd/build/las2pcd"
  las2pcd_intensity: "las2pcd/build/las2pcd_intensity"
  pcd_enhancer: "las2pcd/build/pcd_enhancer"
  pointcloud_divider: "pointcloud_divider-master/build/pointcloud_divider"

defaults:
  divider:
    grid_size_x: 20
    grid_size_y: 20
    leaf_size: 0.2
    merge_pcds: false
  output_prefix: "pointcloud_map"
```

## 常见问题

### 安装失败

**问题：** cmake 找不到 PCL
```bash
# 解决方案：检查 PCL 版本
dpkg -l | grep libpcl
# 如果版本不是 1.12，修改 las2pcd/CMakeLists.txt 第3行的版本号
```

**问题：** liblas 库找不到
```bash
# 解决方案：手动检查库是否安装
ls /usr/local/lib/liblas*
# 运行 ldconfig 更新库缓存
sudo ldconfig
```

**问题：** 权限不足
```bash
# setup.sh 需要 sudo 权限来安装系统依赖和库
# 确保你的用户在 sudoers 中
```

### 运行失败

**问题：** pointcloud-converter 无法运行
```bash
# 检查依赖
ldd pointcloud-converter
# 如果有缺失的库，重新运行 setup.sh
```

**问题：** 找不到配置文件
```bash
# config.yaml 会在首次运行时自动生成
# 如果被删除，可以手动复制
cp config.yaml.example config.yaml
```

## 目录结构

```
map.zh/
├── config.yaml              # 配置文件（自动生成）
├── config.yaml.example      # 配置模板
├── setup.sh                 # 安装脚本
├── run.sh                   # 启动脚本
├── pointcloud-converter     # GUI 工具（预编译）
├── requirements.txt         # Python 依赖
├── las2pcd/                 # LAS 转 PCD 工具源码
│   ├── CMakeLists.txt
│   ├── las2pcd.cpp
│   ├── las2pcd_intensity.cpp
│   └── pcd_enhancer.cpp
├── pointcloud_divider-master/  # 点云分割工具源码
│   ├── CMakeLists.txt
│   ├── src/
│   └── include/
└── libLAS-master/           # libLAS 库源码
    └── build/
```

## 开发说明

### 重新编译工具

如果修改了源码，需要重新编译：

```bash
# 重新编译 las2pcd
cd las2pcd/build
cmake ..
make -j$(nproc)
cd ../..

# 重新编译 pointcloud_divider
cd pointcloud_divider-master/build
cmake ..
make -j$(nproc)
cd ../..
```

### 修改 GUI

GUI 使用 Python + PyQt5 开发，如需修改：
1. 确保安装了 Python 依赖：`pip3 install -r requirements.txt`
2. 修改源码后重新打包为二进制

## 许可证

请查看各子项目的 LICENSE 文件。

## 贡献

欢迎提交 Issue 和 Pull Request！

## 联系方式

如有问题，请在 GitHub 上提 Issue。
