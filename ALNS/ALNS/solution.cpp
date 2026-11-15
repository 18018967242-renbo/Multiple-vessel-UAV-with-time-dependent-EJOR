#include "solution.hpp"
#include <iostream>
#include <list> 
#include <cmath>

#include "Params.h"

using namespace std;


//这是默认构造函数，它不带任何参数。
//它用于创建一个空的 Solution 对象，通常用于在稍后的代码中初始化对象，以便稍后将数据填充到对象中。
Solution::Solution() 
{
    revenue = 0.0;
    nodes_count = 0;
    SRoute.Total_Profit = 0.0;
    nodes_bank.clear();      // 清空节点银行
    removedCustomers.clear(); // 初始化移除客户集
}


//这个复制构造函数的目的是创建一个新的 Solution 对象，该对象包含与另一个 Solution 对象 sol 相同的节点数量、成本、路径总长度和路径节点信息。
//通过复制这些信息，可以在不影响原始对象的情况下创建一个具有相同状态的新对象，以便进行进一步的操作和计算
Solution::Solution(const Solution& sol) 
{
    cout << "11111111111" << endl;
    this->SRoute.Routes.clear();//这行代码清空了新对象的 SRoute 中的路径信息，以确保新对象的路径是空的。
    for (int j = 0; j < sol.SRoute.Routes.size(); ++j)
        this->SRoute.Routes.push_back(sol.SRoute.Routes[j]);
   
    this->nodes_count = sol.nodes_count;//这行代码将新创建的对象的 nodes_count 成员变量设置为 sol 对象的 nodes_count 成员变量的值。这是在复制 sol 对象的节点数量信息到新对象中。
    this->revenue = sol.revenue;
    this->SRoute.Total_Profit = sol.SRoute.Total_Profit;

    //每次迭代都将 sol 对象路径中的一个节点添加到新对象的 SRoute 中。
    //这样，循环完成后，新对象的路径将包含与原始对象相同的节点顺序。

    for (auto it = sol.nodes_bank.begin(); it != sol.nodes_bank.end(); it++) 
    {
        nodes_bank.push_back(*it);
        //每次迭代都将 sol 对象 nodes_bank 中的一个节点添加到新对象的 nodes_bank 中。
        //这样，循环完成后，新对象的 nodes_bank 将包含与原始对象相同的节点信息。
    }

    // 复制removedCustomers集合
    this->removedCustomers = sol.removedCustomers;


    // 添加调试输出，确认复制是否成功
    cout << "拷贝构造函数：原始解的removedCustomers大小: " << sol.removedCustomers.size() << endl;
    cout << "拷贝构造函数：新解的removedCustomers大小: " << this->removedCustomers.size() << endl;
    if (!this->removedCustomers.empty())
    {
        cout << "拷贝构造函数：新解的removedCustomers内容: ";
        for (int customer : this->removedCustomers)
        {
            cout << customer << " ";
        }
        cout << endl;
    }

}


//总之，这个构造函数的目的是创建一个新的 Solution 对象，该对象包含与参数 R 对象相同的成本、节点数量、路径总长度和路径节点信息。
//通过使用参数 R 的信息，可以轻松地创建一个具有相同状态的新对象，以便进一步的操作和计算。
Solution::Solution(Customer_schedule Sche)
{     
    this->SRoute.Routes.clear();
    for (int j = 0; j < Sche.Routes.size(); ++j)
        this->SRoute.Routes.push_back(Sche.Routes[j]);


    this->revenue = Sche.Total_Profit;
    this->nodes_count = 0;
    this->SRoute.Total_Profit = Sche.Total_Profit;

    //// 初始化其他成员变量
    //this->nodes_count = 0;  // 或者根据你的逻辑设置
    //this->nodes_bank.clear();
    //this->removedCustomers.clear(); // 初始化为空集合
}


//通过检查 nodes_bank 容器是否为空来判断 Solution 对象是否有效。具体来说，
// //如果 nodes_bank 容器为空，那么表示所有的节点都已经被正确地安排到路径中，没有剩余的节点，因此 Solution 对象是有效的，函数返回 true；
//否则，如果 nodes_bank 容器不为空，表示还有节点没有被安排到路径中，那么 Solution 对象是无效的，函数返回 false。
bool Solution::valid() {
    return !nodes_bank.size();
}
