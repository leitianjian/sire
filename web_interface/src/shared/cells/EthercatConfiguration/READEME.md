### Ethercat从站配置组件

#### EthercatConfiguration.js

1. 从站配置，通过ethercatController从redux中获取xml，xml通过mapstateToProps传入到组件的props属性中，然后在shouldComponentUpdate中将this.props.ethercatController注入到组件的state中保存，对state中的slave进行增删改等操作，点击保存按钮会调用EthercatController.FromJSON(this.state.slave).toXML方法，将state中的slave转成xml格式，发送给控制器后端程序进行保存。

#### HeaderBar.js

1. 每个slave中的头标签，可以选择品牌，型号等

##### index.js

1. 导入EthercatConfiguration.js中的内容

#### MyCheckBox.js

1. is_tx选中

#### selectors.js

1. 从站配置中的selectors,防止重复渲染

#### SideBar.js

1. 从站配置中的侧面选择slave栏

#### SlaveInput.js

1. 无