# las2pcd


## Ubuntu 安装

``` bash
sudo apt-get install -y libpcl-dev liblas-dev liblas-c-dev
git clone https://github.com/pixmoving-moveit/las2pcd.git
cd las2pcd
mkdir build
cd build
cmake ..
make
```

## 使用
编译完成后会生成2个可执行文件， `las2pcd`与`las2pcd_intensity`，分别是将las转换成RGB点云与intensity点云，可根据实际需求选择可执行文件
使用方式
```bash
./las2pcd[_intensity] \
[las文件绝对路径，如`/home/ahua20/pix_factory.las`] \
[pcd输出文件绝对路径，如`/home/ahua20/pix_factory.pcd`] \
[x0, 如43141.0431] \
[y0, 如34151265.3] \ 
[z0, 如312454135.1341]
```
