# Pybind11-WiringPi
The python version of WiringPi, that is packaged by Pybind11


## Installation

Firstly, you need to run the following codes
```
git clone --recursive https://github.com/Li-Zhaoxi/Pybind11-WiringPi
cd Pybind11-WiringPi
```

### 1. Build Pybind11

```
cd 3rdparty/pybind11
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ..
sudo make install
```


### 2. Build Pybind11-WiringPi

Please make sure your current path is `Pybind11-WiringPi`, and then run actual compilation process

```
cd 3rdparty/WiringPi-RDK
./build
cd ..
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j3（已编辑）
```

A file named like "WiringPi.cpython-38-aarch64-linux-gnu.so" will be built in the folder `build`.

If you can `import WiringPi` correctly,  it means that you have successfully compiled this project.
