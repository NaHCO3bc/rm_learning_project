# CAN总线学习报告

**CAN：控制器局域网络，是一种能够实现分布式实时控制的串行通信网络**

**CAN总线的ID不能重复**

## CAN总线分类：

片内总线

片外总线

## 常见片外总线：

CAN(Flex CAN、FD CAN、Classic CAN)、IIC、RS485、SPI、USB、PCI、PCIe

## 总线的优点：

1.作为总线可以整合、控制所有通过CAN总线连接的设备

2.速度高（可达到1Mbps）、通信距离远（最远达到10km），容错性强

3.无损位仲裁机制，多主结构

4.低成本：ECUs通过单个CAN接口进行通信，布线成本低。

5.高集成：CAN总线系统允许在所有ECUs上进行集中错误诊断和配置

## CAN总线网络：

**CAN总线网络主要挂在CAN_H（high）和CAN_L（low），各个节点通过这两条线实现信号的串行差分传输，为了避免信号的的反射和干扰，还需要在CAN_H和CAN_L之间接上120欧姆的终端电阻**

*因为电缆的特性阻抗为120欧姆，为了模拟无限远的传输线，选择120欧姆的终端电阻*

![img](https://imgconvert.csdnimg.cn/aHR0cDovL3d3dy5ocXlqLmNvbS91cGxvYWRzL2FsbGltZy8xODAzMTUvMi0xUDMxNTE2MlExNE8ucG5n)

## CAN收发器

**CAN收发器的作用是负责逻辑电平和信号电平之间的转换**

**即从CAN控制芯片输出逻辑电平到CAN收发器，然后经过CAN收发器内部转换将逻辑电平转换为差分信号输出到CAN总线上，CAN总线上的节点都可以决定自己是否需要总线上的数据。**

![img](https://imgconvert.csdnimg.cn/aHR0cDovL3d3dy5ocXlqLmNvbS91cGxvYWRzL2FsbGltZy8xODAzMTUvMi0xUDMxNTE2MjkxQTU3LnBuZw)



## ![can总线原理](https://imgconvert.csdnimg.cn/aHR0cDovL3d3dy5ocXlqLmNvbS91cGxvYWRzL2FsbGltZy8xODAzMTUvMi0xUDMxNTE2MzAzNFUyLnBuZw)

## CAN信号表示：

**CAN总线采用不归零码位填充技术**

*CAN总线上的信号有两种不同的状态，分别是显性的逻辑0和隐性的逻辑1，信号每一次传输完后不需要返回到逻辑0（显性）的电平*

![img](https://imgconvert.csdnimg.cn/aHR0cDovL3d3dy5ocXlqLmNvbS91cGxvYWRzL2FsbGltZy8xODAzMTUvMi0xUDMxNTE2MzE0Mzk1NS5wbmc)

## CAN电平分类：

**显性电平，隐性电平（类比于生物中的显性性状和隐性性状）**

CAN的数据总线有两条：黄色的CAN_High,绿色的CAN_Low。没有数据发送时，两条线的电平都为2.5V，称为静电平（**隐性电平**）。当有信号发送时，CAN_High的电平升高1V，即3.5V，CAN_Low的电平降低1V，即1.5V。

**CAN_H-CAN_L<0.5V时为隐性的，逻辑信号表现为*逻辑1*-高电平**（隐性电平）

**CAN_H-CAN_L>0.9V时为显性的，逻辑信号表现为*逻辑0*-低电平**（显性电平）

## CAN信号传输：

**发送过程：CAN控制器将CPU传来的信号转换为逻辑电平（显/隐性电平），CAN发射器接受逻辑电平之后，再将其转换成差分电平输出到CAN总线上。**

![img](https://img-blog.csdn.net/20180324160205758?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MDUyODQxNw==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

**接受过程：CAN接受器将CAN_H和CAN_L线上传来的差分电平转换为逻辑电平输出到CAN控制器，CAN控制器再把逻辑电平转化为相应的信号发送到CPU上。**

![img](https://img-blog.csdn.net/20180324160233631?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MDUyODQxNw==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

## CAN编码方式：

*NRZ不归零编码*

## 同步类型：

*硬同步，重同步*

## CANS数据传输：

**CAN总线传输的是CAN帧**

### CAN通信帧类型：

**数据帧：包含用于传输的节点数据的帧**

*1.基本数据帧：11位ID*

*2.扩展数据帧：29位ID*

远程帧、错误帧、过载帧、帧间隔

*数据帧根据仲裁段长度不同分为标准帧（2.0A）和扩展帧（2.0B）*



## ![img](https://imgconvert.csdnimg.cn/aHR0cDovL2ltYWdlczAuY25ibG9ncy5jb20vYmxvZy83MTgxNjEvMjAxNTA4LzE0MTIxNjE1MTYwODczNy5wbmc)

### 帧起始

由一个显性位（低电平）组成，发送节点发送帧起始，其他节点同步于帧起始。

### 帧结束

由7个隐性位（高电平）组成。

![img](https://img-blog.csdn.net/20170122155013926)

### 仲裁段

**CAN仲裁：决定多个ID之间的传输顺序**

**ID在单条CAN总线上必须是唯一的，否则两个节点将在仲裁位（ID）传输结束后继续传输，造成错误**

总线空闲时，总线上任何节点都可以发送报文，如果有两个及以上的节点传送报文，则可能导致总线访问冲突。**CAN使用了标识符的逐位仲裁方法解决**

CAN总线控制器在发送数据的同时监控总线电平，如果电平不同，则停止发送并进行其他处理。如果该位位于仲裁段，则退出总线竞争；如果位于其他段，则发生错误事件。

## ![img](https://imgconvert.csdnimg.cn/aHR0cDovL2ltYWdlczAuY25ibG9ncy5jb20vYmxvZy83MTgxNjEvMjAxNTA4LzE0MTIxNjE1NTY3OTY3OS5wbmc)

**帧的ID越小，优先级越高。**

由于数据帧的RTR位为显性电平，远程帧为隐性电平，所以帧格式和帧ID相同的情况下，数据帧优先于远程帧；由于标准帧的IDE 位为显性电平，扩展帧的IDE位为隐性电平，对于前11位的ID相同的标准帧和扩展帧，标准帧优先级比扩展帧高。

![img](https://img-blog.csdn.net/20170122155014314)

### 数据段

数据量小，发送和接受时间短，实时性高，被干扰的概率小，抗干扰能力强。

## CAN的校验15位CRC方式

*循环冗余校验*

# can-utils工具

**candump：显示、过滤和记录CAN数据到文件；从CAN总线接口接受数据并以十六进制形式打印到标准输出，也可以输出到指定文件**

**cansend：发送单帧；往指定的CAN总线接口发送指定的数据**

## 测试CAN总线：

 查看是否检测到CAN设备：

```bash
$ ifconfig -a
# 如果在列表中发现含有can名称的设备就说明该设备可以被识别到了
```

用**直连线**将两个CAN口连接起来，便于进行回环测试。

初始化CAN0和CAN1设备，比特率为1Mbits：

```bash
$ sudo ip link set can0 up type can bitrate 1000000
$ sudo ip link set can1 up type can bitrate 1000000
```

打开一个新终端用于监测can0接收到的信息：

```bash
$ candump can0
```

在一个终端中使用can1发送信息：

```bash
$ cansend can1 200#5A5A5A5A5A5A5A5A
```

若能在监测can0的终端中看到can1发来的信息如下，说明通信成功，模块工作正常。

```bash
can0 200 [8] 5A 5A 5A 5A 5A 5A 5A 5A
```