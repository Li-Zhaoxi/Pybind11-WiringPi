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
	<tr align = "center">
    <td  rowspan = "2"> Header </td>
    <td  rowspan = "2"> Module </td>
		<td  colspan = "2"> Declaration </td>
	</tr>
  <tr align = "center">
    <td> Python </td>
    <td> C++ </td>
	</tr>
  <tr align = "center">
    <td> <code> ads1115.h</code></td>
    <td> <code> WiringPi.adc</code></td>
    <td align = "left"> <code> def ads1115Setup(<br>&nbsp;&nbsp;pinBase: int, <br>&nbsp;&nbsp;i2cAddress: int<br>)-> bool</code></td>
    <td align = "left"> <code> int ads1115Setup(<br>&nbsp;&nbsp;pinBase: int, <br>&nbsp;&nbsp;i2cAddress: int<br>);</code></td>
  </tr>
  <tr align = "center">
    <td> <code> max31855.h</code></td>
    <td> <code> WiringPi.adc</code></td>
    <td align = "left"> 
      <code> 
        def max31855Setup(
        <br>&nbsp;&nbsp;pinBase: int, 
        <br>&nbsp;&nbsp;spiChannel: int
        <br>) -> bool
      </code>
    </td>
    <td align = "left"> 
      <code> 
        int max31855Setup(
        <br>&nbsp;&nbsp;int pinBase, 
        <br>&nbsp;&nbsp;int spiChannel);
      </code>
    </td>
  </tr>
  <tr align = "center">
    <td> <code> max5322.h</code></td>
    <td> <code> WiringPi.adc</code></td>
    <td align = "left"> 
      <code> 
        def max5322Setup(
        <br>&nbsp;&nbsp;pinBase: int, 
        <br>&nbsp;&nbsp;spiChannel: int
        <br>) -> bool
      </code>
    </td>
    <td align = "left"> 
      <code> 
        int max5322Setup(
        <br>&nbsp;&nbsp;int pinBase, 
        <br>&nbsp;&nbsp;int spiChannel);
      </code>
    </td>
  </tr>

</table>