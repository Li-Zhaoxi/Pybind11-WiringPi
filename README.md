# Pybind11-WiringPi
The python version of WiringPi, that is packaged by Pybind11


## Install

```
git clone --recursive https://github.com/Li-Zhaoxi/Pybind11-WiringPi
cd Pybind11-WiringPi/3rdparty/WiringPi-RDK
./build
cd ..
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j3（已编辑）
```
