# PID学习报告

### *引子*

*棒棒控制（启停式控制）*

*缺点：控制不稳定，起伏大，误差大*

## PID控制（P：比例，I：积分，D：微分）：

![bd496fd269c4be06174ba103f57d7c3](C:\Users\bc\AppData\Local\Temp\WeChat Files\bd496fd269c4be06174ba103f57d7c3.png)

![这里写图片描述](https://img-blog.csdn.net/20180712104043488?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzI1MzUyOTgx/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### 比例控制：

**u(t)=kp*e(t)**

*静态误差难以消除*

### PI控制：

Ki=Kp * 1/Ti

![7528628beef17cd4b3183f0f2eea96a](C:\Users\bc\AppData\Local\Temp\WeChat Files\7528628beef17cd4b3183f0f2eea96a.jpg)

### PID控制：

![fc3f89c53e3e4887151c0101162a335](C:\Users\bc\AppData\Local\Temp\WeChat Files\fc3f89c53e3e4887151c0101162a335.jpg)

## PID手动整合

*Kp，Ki，Kd变化过程*

![7dfbcf22096328228240e29db6b30ae](C:\Users\bc\AppData\Local\Temp\WeChat Files\7dfbcf22096328228240e29db6b30ae.png)

口诀：

![65afce5a3a5207892c5d08cae552d5f](C:\Users\bc\AppData\Local\Temp\WeChat Files\65afce5a3a5207892c5d08cae552d5f.png)

* **第一句：先将Ki，Kd设成0，调Kp再调Ki，Kd**

* **第三句：振荡频繁就要加大Kp**

* **第四句：振荡上下幅度太大就要调小Kp**

* **第五句：调大Ki（调小Ti）；第六句：调小Ki**

## PID衰减曲线法整合：

![addfc872a16f13902de040d0a2c281d](C:\Users\bc\AppData\Local\Temp\WeChat Files\addfc872a16f13902de040d0a2c281d.png)

## PID自动整定法：

### 模式识别法（专家系统）

* *识别手册中的记录* 

* **进行曲线预整定**

* *在暂态过程中取一些特征的向量，根据方法去修正PID的值*

### 自校正PID（极点配置法）：

**调整PID控制器的结构和参数，让闭环系统的特征多项式接近于预定的式子**

### 模糊自适应整定PID：

*对于非线性时不变（非LTI）*

**需要一个基础PID的值，通过模糊规则调整PID**

### 神经网络PID：

**一般用来调整神经网络的学习率以及平滑因子α等**

*基于bp反向传播神经网络的自学习PID控制器*

### 其他参数优化算法

## PI过冲与抗积分饱和：

**抗积分饱和算法：**在计算U(k)的时候，先判断上一时刻的控制量U(k-1)是否已经超出了限制范围

*1.若U(k-1)>Umax，则只累加负误差*

*2.若U(k-1)<Umin，则只累加正误差*

![acbdeb3347b53028b92b7648604d95a](C:\Users\bc\AppData\Local\Temp\WeChat Files\acbdeb3347b53028b92b7648604d95a.png)

## 不完全微分PID：

微分项有引入高频干扰的风险，但若在控制算法中加入低通滤波器，则可使系统性能得到改善

![9121d0feb56114f177bce9f3d9bb3e9](C:\Users\bc\AppData\Local\Temp\WeChat Files\9121d0feb56114f177bce9f3d9bb3e9.png)

## 微分先行PID控制：

只对**输出量**进行微分，而对给定指令不起微分作用，因此它适合于给定指令频繁升降的场合，可以**避免指令的改变导致超调过大**

## 代码：位置式PID

typedef struct{

float Kp,Ki,Kd;

float summary;//总合

float expect;//用户期望值

float last_error;//上次的错误值

}PID_Structure_t;

float pid_calculate_result(PID_Struture_t*pid_handler,float collect){

float current_result;

float error = pid_handler -> expect - collect;//期望值 - 传感器此刻的值

pid_handler -> summary = pid_handler -> summary + error;//误差累加到积分项

//PID公式计算此时的PID的输出量

current_result = pid_handler -> Kp* error+ //比例控制项

​                             pid_handler ->Ki * pid_handler -> summary + //积分控制项

​                             pid_handler ->Kd *(error - pid_handler -> last_error); //微分控制项

pid_handler -> last_error=error;//更新上一时刻的误差

return current_result;//返回计算值

}

