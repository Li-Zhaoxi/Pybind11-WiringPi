# Pybind11-WiringPi
The python version of WiringPi, that is packaged by Pybind11


## Installation

Firstly, you need to run the following codes
```
pip3 install sphinx
sudo apt-get install python3-sphinx

sudo pip3 install pybind11-stubgen

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


### Build doc

```
pip3 install sphinx
```
<table>
	<tr>
		<td  rowspan = "2"> 单元格1 </td>
		<td> 单元格2 </td>
	</tr>
	<tr>
		<td> 单元格4 </td>
	</tr>
​</table>

<table>
	<tr>
		<td  colspan = "2"> 单元格1 </td>
	</tr>
	<tr>
		<td> 单元格3 </td>
		<td> 单元格4 </td>
	</tr>
</table>

| Headers | Module | Declaration: Python | Declaration: C++ |
|:-------:|:-------:|:-------:|:-------:|
| `ads1115.h` | `WiringPi.adc` | `def ads1115Setup(pinBase: int, i2cAddress: int) -> bool` | `int ads1115Setup (int pinBase, int i2cAddress)` |
| `max31855.h` | `WiringPi.adc` | `def max31855Setup(pinBase: int, spiChannel: int) -> bool` | `int max31855Setup (int pinBase, int spiChannel)` |
| `max5322.h` | `WiringPi.adc` | `def max5322Setup(pinBase: int, spiChannel: int) -> bool` | `int max5322Setup (int pinBase, int spiChannel)` |

|-------|-------|-------| -------|
| 单元格1 | 单元格2 | 单元格3 | 单元格3 |
| 单元格4 | 单元格5 | 单元格6 | 单元格6 |

 | 表头1 | 表头2 |
 | --- | --- |
 | <td colspan="2">合并单元格</td>|
 | 单元格1 | 单元格2 |

