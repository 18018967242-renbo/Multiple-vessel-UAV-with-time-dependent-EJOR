#ifndef cvrpsol_hpp
#define cvrpsol_hpp

#include <iostream>
#include "Params.h"
#include <ctime>

class Solution {

public:

    double revenue;//表示解的收益
    Customer_schedule SRoute;

    int nodes_count;    //任务点请求数目
    std::list<int> nodes_bank;  //存储destroy中待移除的job索引号
    std::set<int> removedCustomers;  // 作为解的一部分

    Solution();//是默认构造函数，用于创建一个空的 Solution 对象。
    Solution(const Solution& sol);//拷贝构造函数，用于创建一个 Solution 对象的副本。
    Solution(Customer_schedule Sche);//是一个构造函数，可能用于根据给定的 Route_Type 对象创建一个 Solution 对象。
    bool valid();//bool valid() 是一个函数，它可能用于检查 Solution 对象的有效性或合法性，根据特定的问题或约束定义。返回值是布尔类型，表示对象是否有效
};

#endif