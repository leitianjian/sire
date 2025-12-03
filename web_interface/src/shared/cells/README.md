### Cells都应该为连接组件，需要直接连接Redux Store 和 Actions， 获取数据和解发动作。


### 怎么创建Cell

1. 编写每个组件完成后都需要在index.js中进行调用

### cell中怎么发起action

1. 在获取store中的内容时需要调用mapStateToProps来将数据导入到组件的props中，mapStateToProps调用action向store中请求数据

2. dispatchProps需要使用dispatchToProps来进行请求的分发，如进行api请求等

### 每个cell的作用介绍：

#### BlocklyEditor文件夹：

1. 可视化编程组件，详细介绍见BlocklyEditor文件夹中的README.md文档

#### ControlSwitchButton文件夹：

1. 用于舵机部分的控制，与点动控制开关组件类似，主要区别是能够对舵机点动按钮进行切换

#### EthercatCatConfiguration文件夹：

1. 从站配置组件，在没有连接后端程序时先从kaanh.xml中读取从站数据，并显示页面上，kaanh.xml的解析步骤代码在ethercat文件中

#### JogCoordinate文件夹：

1. 手动笛卡尔坐标系Jog组件，控制机器人在笛卡尔坐标系下运动

#### JogJoint文件夹：

1. 手动关节坐标系Jog组件，控制机器人关节运动

#### locale文件夹：

1. 存放跟国际化相关的组件，使用react-intl包来进行国际化

#### LogDisplay文件夹：

1. 存放log日志功能组件

#### MotorDebug文件夹：

1. 电机调试组件

#### RealtimeChart文件夹：

1. 存放与实时绘图相关组件

#### RealTimeData文件夹：

1. 实时绘图数据显示组件

#### RobotArm文件夹：

1. Robot图像显示组件，组件与舵机相关，主要用于舵机控制时切换机器人坐标系显示

#### SavePoint文件夹：

1. 保存示教点组件

#### steeringControl文件夹：

1. 舵机控制jog组件，用来方便对舵机控制，能够进行点动控制

#### steeringStateDisplay文件夹：

1. 显示舵机当前状态

#### switchButton文件夹：

1. 用来对点动按钮进行模式切换，可以有关节点动、笛卡尔坐标系点动和舵机点动

#### ViewThree文件夹：

1. 三维视图组件，通过get指令中的四元数和位置信息来使模型可以运动起来

#### cellBase.js：

1. 基类，组件的类都继承自这个基类，有sendCmd等常用操作

#### CodeEditor.js

1. 代码编辑组件

#### CommandButton.js

1. 无

#### ConfigBase.js

1. 基类，有保存数据到xml等常用操作

#### Debugger.js

1. 调试面板组件，用于调试时给机器人发送原生指令

#### ExampleCell.js

1. 实例组件，无实质作用，仅仅用于演示

#### index.js

1. 每个组件都需要从index.js中导入才能在界面上显示

#### JogJoint.js

1. 关节点动组件，用于关节点动控制

#### LoadCalib.js

1. 末端负载标定组件，此组件与力控部分相关

#### Logger.js

1. 日志信息部分，待完善

#### ManualSwitch.js

1. 手动控制开关，对机器人进行使能，手动自动，回零点等操作

#### ModelConfigeration.js

1. 三维模型配置组件，可以对三维模型的路径进行更改，更换三维模型

#### MotionCalib.js

1. 整机动力学标定，与力控部分相关，进行动力学标定

#### RobotStatus.js

1. 显示机器人错误信息和连接状态等信息

#### RosControl.js

1. 进行Ros操作

#### setDH.js

1. 构型参数配置，在配置完从站之后需要进行构型参数配置

#### ShowGetContent.js

1. get数据显示组件

#### steeringEngine.js

1. 舵机状态显示组件

#### ToolCalib.js

1. 工具坐标系标定组件，用于对工具坐标系进行标定

#### Unknown.js

1. 用于当有未识别的组件时程序不发生错误信息

#### View3D.js

1. 三维显示组件，已弃用

#### zeroCalib.js

1. 零点标定组件





