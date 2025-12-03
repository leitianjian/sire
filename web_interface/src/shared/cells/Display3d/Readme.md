# 3D显示重新开发文档

## 依赖、技术路径

**使用的第三方库**

1. [react-three-fiber](https://docs.pmnd.rs/react-three-fiber)

**技术路径**

1. 因为React-three-fiber使用了大量hooks api，计划在该模块中以Function component为主，使用hooks完成主要逻辑，并且使用redux进行状态管理
2. 使用typescript，以增加代码的可维护性

## 实现规划

1. 计划先实现一个能动的，将老的显示模块替换掉，将老的功能全部迁移过来（机械臂的stl文件加载与显示）
2. 实现一个能根据geometry的属性自己生成对应的3d模型的三维显示。
