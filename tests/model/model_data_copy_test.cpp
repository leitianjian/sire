#include <gtest/gtest.h>

#include <aris/dynamic/model.hpp>

/**
* 这个测试用来测试Model的数据保存问题，需要保存的数据可能有
* 1. 每个Part的状态（p v a）
* 2. 每个Motion的状态（p v a）
* 3. 每个GeneralMotion的状态
* 4. ForcePool的值和对应的旋量的关系
* 是不是除了1和4其他的都可以先不管，只要被复制的Model保证数据的一致性，
* 然后使用Part更新所有Pool的数据
* 尝试一下
*/

