### ethercat文件夹说明

1. 此文件夹中的所有文件都是为了解析ehercat从站配置的xml文件所写的js文件，每个文件都代表一个节点，每个节点都有FromXml、FromJson、toXml、toJson四个函数

2. FromXml代表将xml解析成对象

3. FromJson代表将json解析成对象

4. toXml代表将对象解析成xml格式

5. toJson代表将对象转化成Json格式