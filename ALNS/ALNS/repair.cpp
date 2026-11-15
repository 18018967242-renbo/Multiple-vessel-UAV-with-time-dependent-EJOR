#include "solution.hpp"
#include <ctime>

#include "Params.h"
#include <limits>


using namespace std;

typedef std::vector<int> v1d;   //一维向量
typedef std::vector<v1d> v2d;	//二维向量
typedef std::vector<v2d> v3d;	//三维向量

typedef std::vector<double> v1do;   //一维向量double
typedef std::vector<v1do> v2do;	//二维向量
typedef std::vector<v2do> v3do;	//三维向量
void bubblesort_vec(v1d& vec, v1do& a);


//这个函数的主要目标是以贪婪的方式将尚未插入的节点插入到路径中，以尽量降低总成本。
Solution* insertion_greedy(Solution& sol)
{

    cout << "***********************************   insertion_greedy1  Start*****************************************" << endl << endl;

    //cout << "repair1" << endl;
    Solution* nSol = new Solution(sol);//创建一个新的 Solution 对象 nSol，作为原解 sol 的备份
    //
    double higher, bianCost;//声明变量 higher 和 bianCost，用于存储成本的比较和计算。
    int j, i, xid, k;
  
    std::list<int> list_2;//用于存储已经插入的节点，防止重复插入。

    int size = nSol->nodes_bank.size();//获取 sol 对象中剩余节点的数量，存储在变量 size 中
   // cout << "repair中node_bank剩余节点的数量 = " << size << endl;


    //当size不等于0时，进入循环
    while (size)
    {       
        for (auto it = nSol->nodes_bank.begin(); it != nSol->nodes_bank.end(); it++)
        {
            higher = -100000;//将 higher 初始化为正无穷，用于记录插入当前节点后的成本。

            int Qk = -1;
            int Qi = -1;


            //cout << "先------*it=" << *it << endl;
            if (std::count(list_2.begin(), list_2.end(), *it))
            {
                continue;
            }
            cout << "实------*it=" << *it << endl;
            for (int k = 0; k < Nk; k++)
            {
                for (i = 0; i <= Route_current.Routes[k].V.size(); i++)
                {
                    cout << "i= " << i << endl;
                    Route_current.Routes[k].V.insert(Route_current.Routes[k].V.begin() + i, Customer[*it]);

                    bianCost = Calculation(Route_current);

                    if (bianCost > 0)
                    {
                       cout << "repair1bianCost=" << bianCost << endl;
                        if (bianCost > higher)
                        {
                            higher = bianCost;
                            Qk = k;
                            Qi = i;
                            xid = *it;
                        }
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i);     //删除刚插入的点
                    }
                    else
                    {
                        cout << "表明任务点" << *it << "插入此处会导致无解" << endl;
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i);      //删除刚插入的点
                        continue;   
                    }
                   
                }
            }

            if (Qk != -1)
            {

                cout << "最终插入" << Qk << "的" << Qi << "位置" << endl;
               // system("pause");
                Route_current.Routes[Qk].V.insert(Route_current.Routes[Qk].V.begin() + Qi, Customer[xid]);

                Route_current.Total_Profit = Calculation(Route_current);

                nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
                nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue

                size--;
                nSol->nodes_count++;

                //从待移除的列表中删除已经加入路径上的节点
                auto itt = std::find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), xid);
                if (itt != nSol->nodes_bank.end())
                {
                    nSol->nodes_bank.erase(itt);
                }
               // system("pause");
            }
            else
            {
                cout << "节点 " << *it << " 无法插入任何路径，跳过该节点。" << endl;

                nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
                nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue

                size--;
                nSol->nodes_count++;

                // 从待移除的列表中删除已经加入路径上的节点
                auto itt = std::find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), *it);
                if (itt != nSol->nodes_bank.end())
                {
                    nSol->nodes_bank.erase(itt);
                }

                sol.removedCustomers.insert(*it);  // 直接加入移除列表
                //system("pause");
            }



        }     

       cout << "查看插入之后的路径" << endl;
        for (int k = 1; k <= Nk; k++)
        {
            cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
            for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
                cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
            cout << endl;
        }
        
    }
    
    cout << "Route_current.Total_Profit = " << Route_current.Total_Profit << endl;
    nSol->revenue = Route_current.Total_Profit;

    cout << endl << endl << "***********************************   insertion_greedy  End   *****************************************" << endl;

    return nSol;

}


Solution* insertion_greedy_with_noise(Solution& sol)
{
    cout << "***********************************   insertion_greedy_with_noise  Start*****************************************" << endl << endl;

    Solution* nSol = new Solution(sol); // 创建一个新的 Solution 对象 nSol，作为原解 sol 的备份
    double higher, bianCost; // 声明变量 higher 和 bianCost，用于存储成本的比较和计算。
    int j, i, xid, k;
    

    std::list<int> list_2; // 用于存储已经插入的节点，防止重复插入。
    double max_travel_time = 0.0; // 最大旅行时间，用于噪声计算

    // 计算最大旅行时间
    for (int i = 0; i < nSol->nodes_bank.size(); i++)
    {
        for (int j = 0; j < nSol->nodes_bank.size(); j++)
        {
            if (i != j)
            {
                double travel_time = sqrt((Customer[i].px - Customer[j].px) * (Customer[i].px - Customer[j].py) + (Customer[i].py - Customer[j].py) * (Customer[i].py - Customer[j].py))/vv; // 假设有一个函数 getTravelTime 计算旅行时间
                if (travel_time > max_travel_time)
                {
                    max_travel_time = travel_time;
                }
            }
        }
    }

    double noise_factor = 0.1; // 噪声控制参数，可以根据需要调整

    int size = nSol->nodes_bank.size(); // 获取 sol 对象中剩余节点的数量，存储在变量 size 中
    //cout << "repair中node_bank剩余节点的数量 = " << size << endl;

    // 当 size 不等于 0 时，进入循环
    while (size)
    {
        for (auto it = nSol->nodes_bank.begin(); it != nSol->nodes_bank.end(); it++)
        {
            higher = -100000;//将 higher 初始化为正无穷，用于记录插入当前节点后的成本。
            int Qk = -1;
            int Qi = -1;

            //cout << "先------*it=" << *it << endl;
            if (std::count(list_2.begin(), list_2.end(), *it))
            {
                continue;
            }
            //cout << "实------*it=" << *it << endl;
            for (int k = 0; k < Nk; k++)
            {
                for (i = 0; i <= Route_current.Routes[k].V.size(); i++)
                {
                    // cout << "i= " << i << endl;
                    Route_current.Routes[k].V.insert(Route_current.Routes[k].V.begin() + i, Customer[*it]);

                    double noise = max_travel_time * noise_factor * (static_cast<double>(rand()) / RAND_MAX * 2 - 1); // 生成噪声[-1,1]之间的随机数
                    bianCost = Calculation(Route_current) + noise; // 加入噪声

                    if (bianCost >= 0)
                    {
                       // cout << "repair1bianCost=" << bianCost << endl;
                        if (bianCost > higher)
                        {
                            higher = bianCost;
                            Qk = k;
                            Qi = i;
                            xid = *it;
                        }
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i); // 删除刚插入的点
                    }
                    else
                    {
                       // cout << "表明任务点" << *it << "插入此处会导致无解" << endl;
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i); // 删除刚插入的点
                        continue;
                    }
                }
            }

            if (Qk != -1)
            {

                cout << "最终插入" << Qk << "的" << Qi << "位置" << endl;
                // system("pause");
                Route_current.Routes[Qk].V.insert(Route_current.Routes[Qk].V.begin() + Qi, Customer[xid]);

                Route_current.Total_Profit = Calculation(Route_current);

                nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
                nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue

                size--;
                nSol->nodes_count++;

                //从待移除的列表中删除已经加入路径上的节点
                auto itt = std::find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), xid);
                if (itt != nSol->nodes_bank.end())
                {
                    nSol->nodes_bank.erase(itt);
                }
                // system("pause");
            }
            else
            {
                cout << "节点 " << *it << " 无法插入任何路径，跳过该节点。" << endl;

                nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
                nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue

                size--;
                nSol->nodes_count++;

                // 从待移除的列表中删除已经加入路径上的节点
                auto itt = std::find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), *it);
                if (itt != nSol->nodes_bank.end())
                {
                    nSol->nodes_bank.erase(itt);
                }

                sol.removedCustomers.insert(*it);  // 直接加入移除列表
                //system("pause");
            }
        }

       cout << "查看插入之后的路径" << endl;
        for (int k = 1; k <= Nk; k++)
        {
            cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
            for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
                cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
            cout << endl;
        }
    }

    nSol->revenue = Route_current.Total_Profit;

    cout << endl << endl << "***********************************   insertion_greedy_with_noise  End   *****************************************" << endl;

    return nSol;
}


Solution* insertion_best(Solution& sol) 
{
    cout << "***********************************   insertion_best  Start*****************************************" << endl << endl;


    Solution* nSol = new Solution(sol); // 创建一个新的 Solution 对象 nSol，作为原解 sol 的备份

    double higher, bianCost; // 声明变量 higher 和 bianCost，用于存储成本的比较和计算
    int j, i, xid, k;
    

    list<int> list_2; // 用于存储已经插入的节点，防止重复插入

    int size = nSol->nodes_bank.size(); // 获取 sol 对象中剩余节点的数量，存储在变量 size 中
    //cout << "repair中node_bank剩余节点的数量 = " << size << endl;

    // 设置随机数种子
    srand(static_cast<unsigned>(time(0)));

    // 当 size 不等于 0 时，进入循环
    while (size) 
    {
        // 随机选择或时间窗优先选择
        vector<pair<int, int>> candidates; // 存储候选节点及其时间窗宽度
        for (auto it = nSol->nodes_bank.begin(); it != nSol->nodes_bank.end(); ++it) 
        {
            if (std::count(list_2.begin(), list_2.end(), *it)) 
            {
                continue;
            }
            // 计算时间窗宽度
            double time_window_width = Customer[*it].duedate - Customer[*it].earlist;
            candidates.push_back({ *it, time_window_width });
        }

        // 随机选择
        if (rand() % 2 == 0) 
        {
            random_shuffle(candidates.begin(), candidates.end());
        }
        // 时间窗优先选择
        else 
        {
            sort(candidates.begin(), candidates.end(), [](const pair<int, int>& a, const pair<int, int>& b) {
                return a.second < b.second;
                });
        }

        for (const auto& candidate : candidates) 
        {
            higher = -100000;//将 higher 初始化为正无穷，用于记录插入当前节点后的成本。
            int Qk = -1;
            int Qi = -1;

            xid = candidate.first;
            for (int k = 0; k < Nk; k++) 
            {
                for (i = 0; i <= Route_current.Routes[k].V.size(); i++) 
                {
                    Route_current.Routes[k].V.insert(Route_current.Routes[k].V.begin() + i, Customer[xid]);

                    bianCost = Calculation(Route_current);

                    if (bianCost >= 0) 
                    {
                       // cout << "repair1bianCost=" << bianCost << endl;
                        if (bianCost > higher) 
                        {
                            higher = bianCost;
                            Qk = k;
                            Qi = i;
                        }
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i); // 删除刚插入的点
                    }
                    else 
                    {
                        //cout << "表明任务点" << xid << "插入此处会导致无解" << endl;
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i); // 删除刚插入的点
                        continue;
                    }
                }
            }

            if (Qk != -1) 
            {
                Route_current.Routes[Qk].V.insert(Route_current.Routes[Qk].V.begin() + Qi, Customer[xid]);
                Route_current.Total_Profit = Calculation(Route_current);

                nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
                nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue

                size--;
                nSol->nodes_count++;

                // 从待移除的列表中删除已经加入路径上的节点
                auto itt = std::find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), xid);
                if (itt != nSol->nodes_bank.end()) 
                {
                    nSol->nodes_bank.erase(itt);
                }

                break; // 退出当前循环，进入下一个节点的插入
            }
            else
            {
                cout << "节点 " << xid << " 无法插入任何路径，跳过该节点。" << endl;

                nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
                nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue

                size--;
                nSol->nodes_count++;

                // 从待移除的列表中删除已经加入路径上的节点
                auto itt = std::find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), xid);
                if (itt != nSol->nodes_bank.end())
                {
                    nSol->nodes_bank.erase(itt);
                }

                sol.removedCustomers.insert(xid);  // 直接加入移除列表
                //system("pause");
            }
			
        }
    }

    cout << "查看插入之后的路径" << endl;
    for (int k = 1; k <= Nk; k++)
    {
        cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
        for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
            cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
        cout << endl;
    }

    nSol->revenue = Route_current.Total_Profit;

    cout << endl << endl << "***********************************   insertion_best  End   *****************************************" << endl;

    return nSol;
}


Solution* insertion_best_with_noise(Solution& sol)
{
    cout << "***********************************   insertion_best_with_noise  Start*****************************************" << endl << endl;

    Solution* nSol = new Solution(sol); // 创建一个新的 Solution 对象 nSol，作为原解 sol 的备份

    double higher, bianCost; // 声明变量 higher 和 bianCost，用于存储成本的比较和计算
    int j, i, xid, k;

    list<int> list_2; // 用于存储已经插入的节点，防止重复插入
    double max_travel_time = 0.0; // 最大旅行时间，用于噪声计算

    // 计算最大旅行时间
    for (int i = 0; i < nSol->nodes_bank.size(); i++)
    {
        for (int j = 0; j < nSol->nodes_bank.size(); j++)
        {
            if (i != j)
            {
                double travel_time = sqrt((Customer[i].px - Customer[j].px) * (Customer[i].px - Customer[j].py) + (Customer[i].py - Customer[j].py) * (Customer[i].py - Customer[j].py)) / vv; // 假设有一个函数 getTravelTime 计算旅行时间
                if (travel_time > max_travel_time)
                {
                    max_travel_time = travel_time;
                }
            }
        }
    }

    double noise_factor = 0.1; // 噪声控制参数，可以根据需要调整

    int size = nSol->nodes_bank.size(); // 获取 sol 对象中剩余节点的数量，存储在变量 size 中
    //cout << "repair中node_bank剩余节点的数量 = " << size << endl;

    // 设置随机数种子
    srand(static_cast<unsigned>(time(0)));

    // 当 size 不等于 0 时，进入循环
    while (size)
    {
        // 随机选择或时间窗优先选择
        vector<pair<int, int>> candidates; // 存储候选节点及其时间窗宽度
        for (auto it = nSol->nodes_bank.begin(); it != nSol->nodes_bank.end(); ++it)
        {
            if (std::count(list_2.begin(), list_2.end(), *it))
            {
                continue;
            }
            // 计算时间窗宽度
            double time_window_width = Customer[*it].duedate - Customer[*it].earlist;
            candidates.push_back({ *it, time_window_width });
        }

        // 随机选择
        if (rand() % 2 == 0)
        {
            random_shuffle(candidates.begin(), candidates.end());
        }
        // 时间窗优先选择
        else
        {
            sort(candidates.begin(), candidates.end(), [](const pair<int, int>& a, const pair<int, int>& b) {
                return a.second < b.second;
                });
        }

        for (const auto& candidate : candidates)
        {
            higher = -100000;//将 higher 初始化为正无穷，用于记录插入当前节点后的成本。
            xid = candidate.first;
            int Qk = -1;
            int Qi = -1;
            cout << "xid = " << xid << endl;
            for (int k = 0; k < Nk; k++)
            {
                for (i = 0; i <= Route_current.Routes[k].V.size(); i++)
                {
                    Route_current.Routes[k].V.insert(Route_current.Routes[k].V.begin() + i, Customer[xid]);

                    double noise = max_travel_time * noise_factor * (static_cast<double>(rand()) / RAND_MAX * 2 - 1); // 生成噪声[-1,1]之间的随机数
                    bianCost = Calculation(Route_current) + noise; // 加入噪声

                    if (bianCost >= 0)
                    {
                        cout << "repair1bianCost=" << bianCost << endl;
                        if (bianCost > higher)
                        {
                            higher = bianCost;
                            Qk = k;
                            Qi = i;
                        }
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i); // 删除刚插入的点
                    }
                    else
                    {
                        //cout << "表明任务点" << xid << "插入此处会导致无解" << endl;
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i); // 删除刚插入的点
                        continue;
                    }
                }
            }

            if (Qk != -1)
            {
                Route_current.Routes[Qk].V.insert(Route_current.Routes[Qk].V.begin() + Qi, Customer[xid]);
                Route_current.Total_Profit = Calculation(Route_current);

                nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
                nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue

                size--;
                nSol->nodes_count++;

                // 从待移除的列表中删除已经加入路径上的节点
                auto itt = std::find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), xid);
                if (itt != nSol->nodes_bank.end())
                {
                    nSol->nodes_bank.erase(itt);
                }

                continue; // 退出当前循环，进入下一个节点的插入
            }
            else
            {
                cout << "节点 " << xid << " 无法插入任何路径，跳过该节点。" << endl;

                nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
                nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue

                size--;

                // 从待移除的列表中删除已经加入路径上的节点
                auto itt = std::find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), xid);
                if (itt != nSol->nodes_bank.end())
                {
                    nSol->nodes_bank.erase(itt);
                }

                nSol->removedCustomers.insert(xid); // 无可行插入位置，保留为未服务任务；批量运行不暂停。
            }
        }
    }

    /*cout << "查看插入之后的路径" << endl;
    for (int k = 1; k <= Nk; k++)
    {
        cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
        for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
            cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
        cout << endl;
    }*/
    nSol->revenue = Route_current.Total_Profit;

    cout << endl << endl << "***********************************   insertion_best_with_noise  End   *****************************************" << endl;

    return nSol;
}


Solution* insertion_regret(Solution& sol)
{
    cout << "***********************************   insertion_regret  Start*****************************************" << endl << endl;


    //cout << "repair1" << endl;
    Solution* nSol = new Solution(sol);//创建一个新的 Solution 对象 nSol，作为原解 sol 的备份
    
    struct Position
    {
        int Number;
        int Ass_k;	//路径
        int Ass_i;	//路径中的任务点的位置

    };
    Position PO[200];

    //存储第*it个工件的regret,qid,rid,rkd;
    v1do Profit_11(0);
    v1d vec(0);
    double regret[200] = { 0 };
    int qid[200] = { 0 };
    int rkd[200] = { 0 };
    int rid[200] = { 0 };
    

    int j, i, xid, Qk, Qi;
    std::list<int> list_3;
    int size = nSol->nodes_bank.size();

    while (size)
    {
        int iit = 0;
        for (auto it = nSol->nodes_bank.begin(); it != nSol->nodes_bank.end(); it++)
        {
            Profit_11.clear();
            vec.clear();

            //cout << "先------*it=" << *it << endl;
            if (std::count(list_3.begin(), list_3.end(), *it))
            {
                continue;
            }

            int xu = 0;
            for (int k = 0; k < Nk; k++)
            {
                for (i = 0; i <= Route_current.Routes[k].V.size(); i++)
                {
                    //cout << "i= " << i << endl;
                    Route_current.Routes[k].V.insert(Route_current.Routes[k].V.begin() + i, Customer[*it]);
                    double bianCost = Calculation(Route_current);

                    if (bianCost >= 0)
                    {
                        Profit_11.push_back(Calculation(Route_current));
                        PO[xu].Number = xu;
                        PO[xu].Ass_k = k;
                        PO[xu].Ass_i = i;
                        vec.push_back(xu);
                        xu++;
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i);     //删除刚插入的点  
                    }
                    else
                    {
                        cout << "表明任务点" << *it << "插入此处会导致无解" << endl;
                        Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i);      //删除刚插入的点
                        continue;
                    }                            
                }
                
            }

            //按照降序排列
            bubblesort_vec(vec, Profit_11);

           /* cout << "降序排列后结果" << endl;
            for (i = 0; i < vec.size(); i++)
            {
                cout << vec[i] << "(" << Profit_11[vec[i]] << ")" << endl;
            }*/

            //计算regret
            // 目标是最大化利润，排序后应从末端取最佳和次佳值。
            // 只有一个可行位置时也必须避免访问 vec[1] 越界。
            if (vec.empty())
                continue;
            const int best_index = vec.back();
            const int second_index = vec.size() >= 2 ? vec[vec.size() - 2] : best_index;
            regret[iit] = Profit_11[best_index] - Profit_11[second_index];
            qid[iit] = *it;
            rid[iit] = PO[best_index].Ass_i;
            rkd[iit] = PO[best_index].Ass_k;
            iit++;
          
        }

        double low = -100000000;
        for (iit = 0; iit < size; iit++)
        {
            if (regret[iit] > low)
            {
                low = regret[iit];
                xid = qid[iit];
                Qk = rkd[iit];
                Qi = rid[iit];
            }
        }

        Route_current.Routes[Qk].V.insert(Route_current.Routes[Qk].V.begin() + Qi, Customer[xid]);

        Route_current.Total_Profit = Calculation(Route_current);

        nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
        nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue

        size--;
        nSol->nodes_count++;

        //从待移除的列表中删除已经加入路径上的节点
        auto itt = std::find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), xid);
        if (itt != nSol->nodes_bank.end())
        {
            nSol->nodes_bank.erase(itt);
        }

    }
   /* cout << "查看插入之后的路径" << endl;
    for (int k = 1; k <= Nk; k++)
    {
        cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
        for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
            cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
        cout << endl;
    }*/
    cout << "***********************************   insertion_regret  End*****************************************" << endl << endl;

    return nSol;
}


void bubblesort_vec(v1d& vec, v1do& a)//升序排列
{
    float temp;
    for (float pass = 1; pass < vec.size(); pass++)
    {
        for (float k = 0; k < vec.size() - pass; k++)
        {
            if (a[vec[k]] > a[vec[k + 1]])
            {
                temp = vec[k];
                vec[k] = vec[k + 1];
                vec[k + 1] = temp;
            }

        }
    }


}


// ===================== 跨船 regret 修复 =====================
// 普通 regret insertion 比较同一任务的最佳与次佳“插入位置”。
// 本算子要求次佳选项来自另一艘船，从而直接衡量任务的跨船分配紧迫性。
Solution* insertion_cross_vessel_regret(Solution& sol)
{
	Solution* nSol = new Solution(sol);
	Route_current = sol.SRoute;
	if (Route_current.Routes.size() < static_cast<size_t>(Nk))
		Route_current.Routes.resize(Nk);

	struct VesselOption
	{
		bool feasible = false;
		double objective = -numeric_limits<double>::infinity();
		int route = -1;
		int position = -1;
	};

	while (!nSol->nodes_bank.empty())
	{
		const double current_objective = Route_current.Total_Profit;
		bool selected = false;
		double largest_regret = -numeric_limits<double>::infinity();
		int selected_customer = -1;
		VesselOption selected_option;

		for (int customer_id : nSol->nodes_bank)
		{
			vector<VesselOption> best_by_vessel(Nk);
			for (int k = 0; k < Nk; ++k)
			{
				for (int position = 0; position <= static_cast<int>(Route_current.Routes[k].V.size()); ++position)
				{
					auto& route = Route_current.Routes[k].V;
					route.insert(route.begin() + position, Customer[customer_id]);
					const double objective = Calculation(Route_current);
					route.erase(route.begin() + position);

					if (objective >= 0.0 && objective > best_by_vessel[k].objective)
					{
						best_by_vessel[k] = { true, objective, k, position };
					}
				}
			}

			vector<VesselOption> feasible_options;
			for (const auto& option : best_by_vessel)
				if (option.feasible)
					feasible_options.push_back(option);
			sort(feasible_options.begin(), feasible_options.end(), [](const VesselOption& lhs, const VesselOption& rhs)
			{
				return lhs.objective > rhs.objective;
			});

			if (feasible_options.empty())
				continue;

			const double best_value = feasible_options[0].objective;
			// 只有一艘船可接收时，给予大 regret，优先保留这个稀缺选项。
			const double second_vessel_value = feasible_options.size() >= 2
				? feasible_options[1].objective : current_objective - Profit_cost;
			const double regret = (best_value - second_vessel_value)
				+ 0.05 * max(0.0, best_value - current_objective);

			if (!selected || regret > largest_regret)
			{
				selected = true;
				largest_regret = regret;
				selected_customer = customer_id;
				selected_option = feasible_options[0];
			}
		}

		if (!selected)
		{
			// 剩余任务在所有船舶上均不可行，纳入未服务集，保证修复过程有限终止。
			for (int customer_id : nSol->nodes_bank)
				nSol->removedCustomers.insert(customer_id);
			nSol->nodes_bank.clear();
			break;
		}

		auto& selected_route = Route_current.Routes[selected_option.route].V;
		selected_route.insert(selected_route.begin() + selected_option.position, Customer[selected_customer]);
		Route_current = Calculation_removal(Route_current);

		auto bank_position = find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), selected_customer);
		if (bank_position != nSol->nodes_bank.end())
			nSol->nodes_bank.erase(bank_position);
		nSol->removedCustomers.erase(selected_customer);
		++nSol->nodes_count;
	}

	nSol->SRoute = Route_current;
	nSol->revenue = Route_current.Total_Profit;
	return nSol;
}


// ===================== 同步裕度-时间收益跨船 regret 修复 =====================
// 该算子保留“最佳方案与次佳船舶方案之差”的跨船 regret，同时利用固定路线
// SOCP 返回的续航裕度和实际任务收益评价候选。这样优先处理可选船舶稀缺、
// 容易离开最高收益区间、且插入后同步裕度较小的任务。
Solution* insertion_sync_profit_regret(Solution& sol)
{
	Solution* nSol = new Solution(sol);
	Route_current = Calculation_removal(sol.SRoute);
	if (Route_current.Routes.size() < static_cast<size_t>(Nk))
		Route_current.Routes.resize(Nk);

	struct VesselOption
	{
		bool feasible = false;
		double objective = -numeric_limits<double>::infinity();
		double desirability = -numeric_limits<double>::infinity();
		double minimum_slack_ratio = 0.0;
		double inserted_task_profit = 0.0;
		int route_index = -1;
		int position = -1;
		Route_Type evaluated_route;
	};

	while (!nSol->nodes_bank.empty())
	{
		const double current_objective = Route_current.Total_Profit;
		bool selected = false;
		double largest_priority = -numeric_limits<double>::infinity();
		int selected_customer = -1;
		VesselOption selected_option;

		for (int customer_id : nSol->nodes_bank)
		{
			vector<VesselOption> best_by_vessel(Nk);
			const double maximum_task_profit = max(MY_EPSILON,
				Profit_cost * Customer[customer_id].weight);

			for (int k = 0; k < Nk; ++k)
			{
				const Route_Type& base_route = Route_current.Routes[k];
				for (int position = 0; position <= static_cast<int>(base_route.V.size()); ++position)
				{
					Route_Type candidate_route = base_route;
					Customer_Type inserted_customer = Customer[customer_id];
					inserted_customer.vessel = k;
					candidate_route.V.insert(candidate_route.V.begin() + position, inserted_customer);

					bool feasible = false;
					const double candidate_profit = EvaluateRouteCached(candidate_route, feasible);
					if (!feasible)
						continue;

					const double objective = current_objective - base_route.Profit + candidate_profit;
					double minimum_slack_ratio = 1.0;
					for (int j = 0; j < static_cast<int>(candidate_route.V.size()); ++j)
					{
						double used_endurance = candidate_route.V[j].service_time;
						if (j < static_cast<int>(candidate_route.outbound_times.size()) &&
							j < static_cast<int>(candidate_route.return_times.size()))
						{
							used_endurance += candidate_route.outbound_times[j]
								+ candidate_route.return_times[j];
						}
						minimum_slack_ratio = min(minimum_slack_ratio,
							max(0.0, a - used_endurance) / max(MY_EPSILON, a));
					}

					int degraded_tasks = 0;
					for (int j = 0; j < static_cast<int>(candidate_route.V.size()); ++j)
					{
						const double maximum_profit = max(MY_EPSILON,
							Profit_cost * candidate_route.V[j].weight);
						if (j < static_cast<int>(candidate_route.task_profits.size()) &&
							candidate_route.task_profits[j] < maximum_profit - MY_EPSILON)
							++degraded_tasks;
					}
					const double degraded_ratio = candidate_route.V.empty() ? 0.0
						: static_cast<double>(degraded_tasks) / candidate_route.V.size();
					const double inserted_profit = position < static_cast<int>(candidate_route.task_profits.size())
						? candidate_route.task_profits[position] : 0.0;

					// 主项仍是实际总收益；同步裕度与收益分段只在相近候选之间提供前瞻性。
					const double desirability = objective
						+ 0.04 * maximum_task_profit * minimum_slack_ratio
						- 0.03 * maximum_task_profit * degraded_ratio
						+ 0.02 * inserted_profit;

					if (!best_by_vessel[k].feasible ||
						desirability > best_by_vessel[k].desirability)
					{
						VesselOption option;
						option.feasible = true;
						option.objective = objective;
						option.desirability = desirability;
						option.minimum_slack_ratio = minimum_slack_ratio;
						option.inserted_task_profit = inserted_profit;
						option.route_index = k;
						option.position = position;
						option.evaluated_route = candidate_route;
						best_by_vessel[k] = option;
					}
				}
			}

			vector<VesselOption> feasible_options;
			for (const auto& option : best_by_vessel)
				if (option.feasible)
					feasible_options.push_back(option);
			sort(feasible_options.begin(), feasible_options.end(), [](const VesselOption& lhs,
				const VesselOption& rhs)
			{
				return lhs.desirability > rhs.desirability;
			});
			if (feasible_options.empty())
				continue;

			const double second_value = feasible_options.size() >= 2
				? feasible_options[1].desirability
				: current_objective - maximum_task_profit;
			const double vessel_regret = feasible_options[0].desirability - second_value;
			const double scarcity_bonus = 0.15 * maximum_task_profit
				/ feasible_options.size();
			const double synchronization_urgency = 0.10 * maximum_task_profit
				* (1.0 - feasible_options[0].minimum_slack_ratio);
			const double profit_recovery = 0.05 * max(0.0,
				maximum_task_profit - feasible_options[0].inserted_task_profit);
			const double priority = vessel_regret + scarcity_bonus
				+ synchronization_urgency + profit_recovery;

			if (!selected || priority > largest_priority)
			{
				selected = true;
				largest_priority = priority;
				selected_customer = customer_id;
				selected_option = feasible_options[0];
			}
		}

		if (!selected)
		{
			// 不能插入的任务保留为未服务任务，防止修复循环无法终止。
			for (int customer_id : nSol->nodes_bank)
				nSol->removedCustomers.insert(customer_id);
			nSol->nodes_bank.clear();
			break;
		}

		Route_current.Routes[selected_option.route_index] = selected_option.evaluated_route;
		Route_current.Total_Profit = selected_option.objective;
		Customer[selected_customer].vessel = selected_option.route_index;
		auto bank_position = find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), selected_customer);
		if (bank_position != nSol->nodes_bank.end())
			nSol->nodes_bank.erase(bank_position);
		nSol->removedCustomers.erase(selected_customer);
		++nSol->nodes_count;
	}

	nSol->SRoute = Route_current;
	nSol->revenue = Route_current.Total_Profit;
	return nSol;
}
