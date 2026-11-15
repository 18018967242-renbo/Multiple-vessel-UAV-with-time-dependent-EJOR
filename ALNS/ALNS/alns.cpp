#include "alns.hpp"

#include <iostream>
#include <list> 
#include <cmath>
#include <stdlib.h>
#include <ctime>
#include <windows.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <random>
#include <chrono>
#include <limits>
#include <memory>

using namespace std;

ALNS::ALNS(Params* params) : params(params) {}


int rwSelection(vector<float>& weights) 
{
   
    float total = 0, lun = 0;
    for (float x : weights) 
    {
        total += x;
    }
    //cout << "total=" << total << endl;

    std::uniform_real_distribution<> dis(0, total);
    // 生成随机数
    float v = static_cast<float>(dis(AlnsRandomEngine()));
   
    int idx = 0;
    for (float x : weights) 
    {
        lun += x;
        if (lun >= v)
            return idx;
        idx++;
    }
    return -1;
}

// 根据成对权重选择 destroy-repair 组合，保留两个算子间的交互效应。
pair<int, int> rwPairSelection(const vector<vector<float>>& weights)
{
    double total = 0.0;
    for (const auto& row : weights)
        for (float value : row)
            total += max(0.0001f, value);

    std::uniform_real_distribution<double> distribution(0.0, total);
    const double target = distribution(AlnsRandomEngine());
    double cumulative = 0.0;
    for (int d = 0; d < static_cast<int>(weights.size()); ++d)
    {
        for (int r = 0; r < static_cast<int>(weights[d].size()); ++r)
        {
            cumulative += max(0.0001f, weights[d][r]);
            if (cumulative >= target)
                return { d, r };
        }
    }
    return { static_cast<int>(weights.size()) - 1,
        static_cast<int>(weights.back().size()) - 1 };
}

struct SearchStateIndicators
{
    double vessel_imbalance = 0.0;
    double synchronization_pressure = 0.0;
    double temporal_profit_loss = 0.0;
};

// 从当前解提取与本问题直接相关的状态，而不是只依赖算子的历史平均得分。
// 三个指标分别对应多船分配、船-UAV 续航同步和到达时间相关收益。
SearchStateIndicators MeasureSearchState(const Customer_schedule& schedule)
{
    SearchStateIndicators state;
    if (schedule.Routes.empty())
        return state;

    int total_tasks = 0;
    int minimum_route_size = (numeric_limits<int>::max)();
    int maximum_route_size = 0;
    double synchronization_sum = 0.0;
    double maximum_profit_sum = 0.0;
    double actual_profit_sum = 0.0;

    for (const auto& route : schedule.Routes)
    {
        const int route_size = static_cast<int>(route.V.size());
        total_tasks += route_size;
        minimum_route_size = min(minimum_route_size, route_size);
        maximum_route_size = max(maximum_route_size, route_size);
        for (int i = 0; i < route_size; ++i)
        {
            const double maximum_profit = max(MY_EPSILON, Profit_cost * route.V[i].weight);
            const double actual_profit = i < static_cast<int>(route.task_profits.size())
                ? route.task_profits[i] : maximum_profit;
            maximum_profit_sum += maximum_profit;
            actual_profit_sum += actual_profit;

            double used_endurance = route.V[i].service_time;
            if (i < static_cast<int>(route.outbound_times.size()) &&
                i < static_cast<int>(route.return_times.size()))
            {
                used_endurance += route.outbound_times[i] + route.return_times[i];
            }
            synchronization_sum += min(1.0, used_endurance / max(MY_EPSILON, a));
        }
    }

    const double average_route_size = static_cast<double>(total_tasks)
        / max(1, static_cast<int>(schedule.Routes.size()));
    state.vessel_imbalance = total_tasks > 0
        ? min(1.0, (maximum_route_size - minimum_route_size) / max(1.0, average_route_size)) : 0.0;
    state.synchronization_pressure = total_tasks > 0
        ? synchronization_sum / total_tasks : 0.0;
    state.temporal_profit_loss = maximum_profit_sum > MY_EPSILON
        ? min(1.0, max(0.0, maximum_profit_sum - actual_profit_sum) / maximum_profit_sum) : 0.0;
    return state;
}

// 在保留自适应算子对权重的基础上，根据当前解状态临时放大相应算子族。
// 环境变量可关闭该机制，以便与纯历史权重选择进行公平消融。
vector<vector<float>> BuildStateConditionedWeights(
    const vector<vector<float>>& base_weights, const Customer_schedule& schedule)
{
    vector<vector<float>> adjusted = base_weights;
    const char* disabled = getenv("ALNS_DISABLE_STATE_CONDITIONING");
    if (disabled != nullptr && std::string(disabled) == "1")
        return adjusted;

    const SearchStateIndicators state = MeasureSearchState(schedule);
    for (int d = 0; d < static_cast<int>(adjusted.size()); ++d)
    {
        for (int r = 0; r < static_cast<int>(adjusted[d].size()); ++r)
        {
            double multiplier = 1.0;
            if (d == 4) // 多船时间压力移除
                multiplier *= 1.0 + 1.5 * state.vessel_imbalance;
            else if (d == 5) // 时间收益外部性移除
                multiplier *= 1.0 + 2.0 * state.temporal_profit_loss;
            else if (d == 6) // 同步临界字符串移除
                multiplier *= 1.0 + 2.0 * state.synchronization_pressure;

            if (r == 4) // 原跨船 regret
                multiplier *= 1.0 + state.vessel_imbalance;
            else if (r == 5) // 同步裕度-时间收益跨船 regret
                multiplier *= 1.0 + state.synchronization_pressure
                    + state.temporal_profit_loss;
            adjusted[d][r] = static_cast<float>(max(0.0001, adjusted[d][r] * multiplier));
        }
    }
    return adjusted;
}

void updatePairWeights(vector<vector<float>>& scores,
    vector<vector<float>>& weights, vector<vector<float>>& times)
{
    for (int d = 0; d < static_cast<int>(weights.size()); ++d)
    {
        for (int r = 0; r < static_cast<int>(weights[d].size()); ++r)
        {
            weights[d][r] = static_cast<float>((1.0 - rho) * weights[d][r]
                + rho * scores[d][r] / max(1.0f, times[d][r]));
            scores[d][r] = 0.0f;
            times[d][r] = 1.0f;
        }
    }
}



void gengxin_weight(vector<float>& scores, vector<float>& weights, vector<float>& times) 
{
    for (int i = 0; i < weights.size(); i++)
    {
        //cout << "weights[i]= " << weights[i] << "scores[i]=" << scores[i] << "times[i]" << times[i] << endl;
        //weights[i] = (1 - rho) * weights[i] + rho * scores[i] * (times[i]/ time_segment);
        weights[i] = static_cast<float>((1 - rho) * weights[i] + rho * scores[i] / times[i]);
    }
}

void resetScore(vector<float>& scores) 
{
    for (auto it = scores.begin(); it != scores.end(); it++) 
    {
        *it = 0;//    *it 表示迭代器 it 指向的当前元素，即向量 scores 中的一个元素。
        //  = 0 将该元素的值设置为0，即将其重置为0。
    }
}

void resetTime(vector<float>& times) 
{
    for (auto it = times.begin(); it != times.end(); it++) 
    {
        *it = 1;//* 符号用于解引用指针或迭代器，即获取指针或迭代器所指向的对象或元素。
    }
}

void ALNS::runStep() 
{
    Solution sol;
    Route_current = sol_curr->SRoute; // 每次迭代从当前解恢复全局路线，避免上次候选解残留。
    const EvaluationStats stats_before = GetEvaluationStats();
    const auto pair_start_time = std::chrono::steady_clock::now();
    float outcome_score = 0.0f;

    //cout << "进到了runStep" << endl;
   
    const vector<vector<float>> contextual_pair_weights =
        BuildStateConditionedWeights(pair_weights, Route_current);
    pair<int, int> selected_pair = rwPairSelection(contextual_pair_weights);
    int dt_idx = selected_pair.first;
    int rp_idx = selected_pair.second;
    const char* independent_selection = getenv("ALNS_INDEPENDENT_OPERATOR_SELECTION");
    if (independent_selection != nullptr && std::string(independent_selection) == "1")
    {
        dt_idx = rwSelection(weights_destroy);
        rp_idx = rwSelection(weights_repair);
    }
    // 消融/回归测试开关：可用环境变量强制执行某个算子对，正常实验不设置即可。
    const char* forced_destroy = getenv("ALNS_FORCE_DESTROY");
    const char* forced_repair = getenv("ALNS_FORCE_REPAIR");
    if (forced_destroy != nullptr)
        dt_idx = min(static_cast<int>(m_destroy.size()) - 1, max(0, atoi(forced_destroy)));
    if (forced_repair != nullptr)
        rp_idx = min(static_cast<int>(m_repair.size()) - 1, max(0, atoi(forced_repair)));
    cout << "Selected operator pair: destroy=" << dt_idx << ", repair=" << rp_idx << endl;


    //Select destroy/repair repair
    times_destroy[dt_idx] += 1;//更新使用次数
    times_repair[rp_idx] += 1;//更新使用次数
    pair_times[dt_idx][rp_idx] += 1.0f;


    SH f_dt = m_destroy[dt_idx], f_rp = m_repair[rp_idx];
    
    //cout << "destory and repair type: " << dt_idx << ", " << rp_idx << endl;

    // 每轮的破坏解和修复解只在本轮使用，离开 runStep 时自动释放。
    std::unique_ptr<Solution> destroyed(f_dt(*sol_curr));
    /*cout << "After destroy cost: " << destroyed->revenue<< endl;
    for (int k = 0; k < Nk; k++)
    {
        cout << "第" << k << "条路径:" << endl;
        for (int i = 0; i < destroyed->SRoute.Routes[k].V.size(); i++)
            cout << destroyed->SRoute.Routes[k].V[i].customer_id << "->";
        cout << endl;
    }
    cout << "dt: " << destroyed->nodes_count << endl;*/

    //system("pause");

    std::unique_ptr<Solution> repaired(f_rp(*destroyed));
    Solution* s = repaired.get();
   /* cout << "rp: " << repaired->nodes_count << endl;
    for (int k = 0; k < Nk; k++)
    {
        cout << "第" << k << "条路径:" << endl;
        for (int i = 0; i < repaired->SRoute.Routes[k].V.size(); i++)
            cout << repaired->SRoute.Routes[k].V[i].customer_id << "->";
        cout << endl;
    }
    cout << "After repair cost: " << repaired->revenue << endl;*/

   // system("pause");
    ////======================这儿尝试把初始解移除的任务点重新插入路径中====================
    //double higher, bianCost;//声明变量 higher 和 bianCost，用于存储成本的比较和计算。
    //int xid;
    //int Qk = 110000;
    //int Qi = 110000;
    // 
    //// 遍历 removedCustomers 中的每个任务点，尝试插入
    //
    //for (int it : sol.removedCustomers)
    //{
    //    bool foundFeasiblePosition = false; // 添加标志位，判断是否找到可行位置
    //    higher = -1000;//将 higher 初始化为正无穷，用于记录插入当前节点后的成本。
    //    // 插入任务点
    //    for (int k = 0; k < Nk; k++)
    //    {
    //        for (int i = 0; i <= Route_current.Routes[k].V.size(); i++)
    //        {
    //            cout << "尝试将" << it << "插入[" << k << "]的位置" << i << "处" << endl;
    //            Route_current.Routes[k].V.insert(Route_current.Routes[k].V.begin() + i, Customer[it]);
    //
    //            cout << "查看临时插入的位置" << endl << endl;
    //            for (int k = 0; k < Nk; k++)
    //            {
    //                cout << "第" << k << "条路径:" << endl;
    //                for (int i = 0; i < Route_current.Routes[k].V.size(); i++)
    //                    cout << Route_current.Routes[k].V[i].customer_id << "->";
    //                cout << endl;
    //            }
    //
    //            bianCost = Calculation(Route_current);
    //            if (bianCost >= 0)
    //            {                    
    //                //cout << "repair1bianCost=" << bianCost << endl;
    //                if (bianCost > higher)
    //                {
    //                    foundFeasiblePosition = true; // 找到了可行位置
    //                    higher = bianCost;
    //                    Qk = k;
    //                    Qi = i;
    //                    xid = it;
    //                }
    //                Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i);     //删除刚插入的点
    //            }
    //            else
    //            {
    //                //cout << "表明任务点" << it << "插入此处会导致无解" << endl;
    //                Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + i);      //删除刚插入的点
    //                continue;
    //            }
    //            
    //        }
    //    }
    //
    //    // 只有当找到可行位置时才执行插入操作
    //    if (foundFeasiblePosition)
    //    {
    //        //cout << xid << "最终插入了第" << Qk << "船的第" << Qi << "个位置" << endl;
    //        Route_current.Routes[Qk].V.insert(Route_current.Routes[Qk].V.begin() + Qi, Customer[xid]);
    //        Route_current.Total_Profit = Calculation(Route_current);
    //
    //        // 更新解决方案
    //        Solution* nSol = new Solution(*repaired);
    //        nSol->SRoute = Route_current;
    //        nSol->revenue = Route_current.Total_Profit;
    //
    //        // 从 removedCustomers 中移除该任务点
    //        sol.removedCustomers.erase(xid);
    //
    //        /*cout << "查看移除之后的888888888888" << endl;
    //        for (int customer : sol.removedCustomers)
    //         {
    //             cout << customer << " ";
    //        }
    //        cout << endl;
    //        system("pause");*/
    //
    //        cout << "任务点" << it << "成功插入并从待插入列表中移除" << endl;
    //    }
    //    else
    //    {
    //        cout << "任务点" << xid << "无法插入到任何可行位置，跳过该任务点" << endl;
    //
    //        continue;
    //        // 不执行插入操作，继续处理下一个任务点
    //    }
    //
    //}
    //cout << "重新插入之后的路径" << endl << endl;
    //for (int k = 0; k < Nk; k++)
    //{
    //    cout << "第" << k << "条路径:" << endl;
    //    for (int i = 0; i < Route_current.Routes[k].V.size(); i++)
    //        cout << Route_current.Routes[k].V[i].customer_id << "->";
    //    cout << endl;
    //}
    //system("pause");
    //==============================end====================================

    if (s->valid())       
    {
        
        double newCost = Route_current.Total_Profit;       
        cout << endl << "newCost= " << newCost << "bestCost= " << sol_best->revenue << "currCost= " << sol_curr->revenue << endl;
       // system("pause");
        if (newCost > sol_best->revenue)           
        {
            cout << "*****************最优******************" << endl; 
            //如果 newCost 小于 sol_best->cost，则将 sol_curr 和 sol_best 更新为 s，并更新相关的分数信息 scores_destroy 和 scores_repair。
            *sol_curr = *s;
            *sol_best = *s;

            sol_best->revenue = newCost;
            sol_curr->revenue = newCost;

            scores_destroy[dt_idx] += score1;
            scores_repair[rp_idx] += score1;
            outcome_score = score1;


            // 整体拷贝避免 clear() 后通过 Routes[k] 越界访问，也防止总利润被单船利润覆盖。
            sol_curr->SRoute = Route_current;
            sol_best->SRoute = Route_current;


        }
        else if ((newCost <= sol_best->revenue) && (newCost >= sol_curr->revenue)) 
        {

            //如果 newCost 大于等于 sol_best->cost 且小于等于 sol_curr->cost，则将 sol_curr 更新为 s，并更新相关的分数信息。
            cout << "#########################较优" << endl;
            *sol_curr = *s;
            sol_curr->revenue = newCost;
            scores_destroy[dt_idx] += score2;
            scores_repair[rp_idx] += score2;
            outcome_score = score2;
            sol_curr->SRoute = Route_current;
        }
        else 
        {

            //如果 newCost 大于 sol_best->cost 且大于 sol_curr->cost，则以一定概率决定是否接受不优解，概率计算使用了模拟退火算法的思想。
            //如果接受不优解，则将 sol_curr 更新为 s，并更新相关的分数信息。如果不接受，则保持当前解不变。
            std::uniform_real_distribution<> dis2(0, 1);
            float fire = static_cast<float>(dis2(AlnsRandomEngine()));
            //cout << "W= " << W << endl;
            //cout << "fire= " << fire << endl;
            //cout << "exp((sol_curr->cost - newCost) / W= " << exp((sol_curr->cost - newCost) / W) << endl;
            if (exp((sol_curr->revenue - newCost) / Initial_W) > fire) 
            {
                cout << "+++++++++++++++++不优接受+++++++++++++" << endl;
                *sol_curr = *s;
                sol_curr->revenue = newCost;
                scores_destroy[dt_idx] += score3;
                scores_repair[rp_idx] += score3;
                outcome_score = score3;
                sol_curr->SRoute = Route_current;
            }
            else 
            {
                cout << "%%%%%%%%%%%%%%%%不优放弃%%%%%%%%%%%%%%%%" << endl;
                Route_current.Total_Profit = sol_curr->SRoute.Total_Profit;
              

                Route_current.Routes.clear();
                for (int k = 0; k < Nk; k++)
                {
                    Route_current.Routes.push_back(sol_curr->SRoute.Routes[k]);
                }             
            }
        }

        Initial_W = static_cast<float>(jw * Initial_W);//����
    }

    // 成本感知奖励：同样的改进下，优先选择用更少锥子问题和更少时间的算子对。
    const EvaluationStats stats_after = GetEvaluationStats();
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - pair_start_time).count();
    const long long conic_calls = stats_after.conic_solves - stats_before.conic_solves;
    const double cost = 1.0 + 0.10 * conic_calls + 0.001 * milliseconds;
    pair_scores[dt_idx][rp_idx] += static_cast<float>(outcome_score / cost);
}

//这段代码实现了 ALNS 算法的核心逻辑，通过迭代搜索来寻找最佳解。
//在每次迭代中，执行一系列操作，包括更新权重、分数、时间信息，并根据一定的条件来控制迭代次数和时间限制。
//最终，输出找到的最佳解的成本，并更新 Route 对象以反映最佳解的路线。
void ALNS::run(Solution* base) 
{    
    ResetEvaluationCache();
    SetAlnsRandomSeed(20250914u + static_cast<unsigned int>(1009 * Nj + 9176 * Nk));

    // 首次运行前补齐 SOCP 诊断信息，使第一轮专属算子也能读取真实同步与收益状态。
    base->SRoute = Calculation_removal(base->SRoute);
    base->revenue = base->SRoute.Total_Profit;
    Initial_W = static_cast<float>(max(1.0, 0.05 * abs(base->revenue)));
    //重置名为 scores_repair 和 scores_destroy 的向量中的元素值为0。这个函数可能用于初始化或清零分数信息。
    resetScore(scores_repair);
    resetScore(scores_destroy);

    //sol_best 和 sol_curr 分别表示当前最佳解和当前解，而 cost_best 表示最佳解的成本。
    //这些变量被初始化为 base 参数传递的解，即初始解。
    sol_best = new Solution(*base);
    sol_curr = new Solution(*base);
    
    //cout << "base->revenue = " << base->revenue << endl;
    //cout << "Start cost: " << sol_best->revenue << endl;

    //count 用于计数迭代次数，limit 用于设置迭代的上限次数，这里的设置方式似乎根据 Customer_Number 的不同而有所不同。
    int count = 0;
    // 原实现只迭代 50/100 次，对大规模多船实例无法形成有效的自适应学习。
    const char* iteration_env = getenv("ALNS_MAX_ITERATIONS");
    const int limit = iteration_env != nullptr
        ? max(1, atoi(iteration_env)) : max(500, 20 * max(1, Customer_Number));
    const char* stagnation_env = getenv("ALNS_STAGNATION_LIMIT");
    const int stagnation_limit = stagnation_env != nullptr
        ? max(1, atoi(stagnation_env)) : max(100, 5 * max(1, Customer_Number));
    const char* time_env = getenv("ALNS_TIME_LIMIT_SECONDS");
    const double time_limit_seconds = time_env != nullptr
        ? max(1.0, atof(time_env)) : 3600.0;

    int newseed = limit;
    auto STime = std::chrono::steady_clock::now();

    int noImproveCount = 0;  // 连续未改进计数器
    double bestRevenue = sol_best->revenue;  // 记录当前最优解


    while (count++ < limit) 
    { //criteria clock() - params->startTime <= params->timeLimit

        cout << "*************************************" << count << "次迭代***************************************" << endl;
       //system("pause");

        --newseed; // 保留原迭代计数含义；实际随机数由 AlnsRandomEngine 统一生成。

        this->runStep();//调用 this->runStep() 来执行 ALNS 算法的一个迭代步骤。


       // cout << "run中的Current best: " << sol_best->revenue << endl;   


        // 检查是否改进
        if (sol_best->revenue > bestRevenue) 
        {
            bestRevenue = sol_best->revenue;
            noImproveCount = 0;  // 重置未改进计数
        }
        else
        {
            noImproveCount++;
        }

        // 如果连续30次没有改进，退出循环
        if (noImproveCount >= stagnation_limit) 
        {
            //cout << "连续30次未改进，提前终止。" << endl;
            break;
        }


        //当 count 的值达到 time_segment 的整数倍时，
        if (count % time_segment == 0) 
        {
            cout << "Current best: " << sol_best->revenue << endl;
            gengxin_weight(scores_repair, weights_repair, times_repair);//更新下一段权重
            gengxin_weight(scores_destroy, weights_destroy, times_destroy);//更新下一段权重
            resetScore(scores_repair);//把该段分数归0
            resetScore(scores_destroy);//把该段分数归0
            resetTime(times_repair);//把该段操作使用次数归1
            resetTime(times_destroy);//把该段操作使用次数归1
            updatePairWeights(pair_scores, pair_weights, pair_times);
            //Calculation2(sol_best->SRoute);
        }
        auto nowTime = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(nowTime - STime).count() > time_limit_seconds) 
        {
            break;
        }
    }
    const EvaluationStats final_stats = GetEvaluationStats();
    cout << "ALNS evaluation statistics: schedule_calls=" << final_stats.schedule_calls
        << ", conic_solves=" << final_stats.conic_solves
        << ", cache_hits=" << final_stats.cache_hits
        << ", fast_rejections=" << final_stats.fast_rejections << endl;
    //cout << "Best found: " << sol_best->revenue << endl;

    //最后，将 Route 对象的属性更新为最佳解的路线信息，以便后续操作中使用。
    Route_best.Total_Profit = sol_best->revenue;
   
    
   /* for (int k = 0; k < Nk; k++)
    {
        cout << "第" << k << "条路径:" << endl;
        for (int i = 0; i < sol_best->SRoute.Routes[k].V.size(); i++)
            cout << sol_best->SRoute.Routes[k].V[i].customer_id << "->";
        cout << endl;
    }*/

    Route_best.Routes.clear();
    for (int k = 0; k < Nk; k++)
    {
       // Route_best.Routes[k].Profit = sol_best->SRoute.Routes[k].Profit;
       // cout << "Route_best.Routes[k].Profit = " << Route_best.Routes[k].Profit << endl;
       // Route_best.Routes[k].Profit = sol_best->SRoute.Routes[k].Profit;
        Route_best.Routes.push_back(sol_best->SRoute.Routes[k]);        
    }

   // system("pause");
    Solution sol;
    //======================这儿尝试把初始解移除的任务点重新插入路径中====================
    double higher, bianCost;//声明变量 higher 和 bianCost，用于存储成本的比较和计算。
    int xid;
   

    // 遍历 removedCustomers 中的每个任务点，尝试插入

    for (int it : sol.removedCustomers)
    {
        bool foundFeasiblePosition = false; // 添加标志位，判断是否找到可行位置
        higher = -1000;//将 higher 初始化为正无穷，用于记录插入当前节点后的成本。
        int Qk = -1;
        int Qi = -1;


        // 插入任务点
        for (int k = 0; k < Nk; k++)
        {
            for (int i = 0; i <= Route_best.Routes[k].V.size(); i++)
            {
               // cout << "尝试将" << it << "插入[" << k << "]的位置" << i << "处" << endl;
                Route_best.Routes[k].V.insert(Route_best.Routes[k].V.begin() + i, Customer[it]);

               /* cout << "查看临时插入的位置" << endl << endl;
                for (int k = 0; k < Nk; k++)
                {
                    cout << "第" << k << "条路径:" << endl;
                    for (int i = 0; i < Route_best.Routes[k].V.size(); i++)
                        cout << Route_best.Routes[k].V[i].customer_id << "->";
                    cout << endl;
                }*/

                bianCost = Calculation(Route_best);
                if (bianCost > 0)
                {
                    //cout << "repair1bianCost=" << bianCost << endl;
                    if (bianCost > higher)
                    {
                        foundFeasiblePosition = true; // 找到了可行位置
                        higher = bianCost;
                        Qk = k;
                        Qi = i;
                        xid = it;
                    }
                    Route_best.Routes[k].V.erase(Route_best.Routes[k].V.begin() + i);     //删除刚插入的点
                }
                else
                {
                    //cout << "表明任务点" << it << "插入此处会导致无解" << endl;
                    Route_best.Routes[k].V.erase(Route_best.Routes[k].V.begin() + i);      //删除刚插入的点
                    continue;
                }

            }
        }

        // 只有当找到可行位置时才执行插入操作
        if (foundFeasiblePosition)
        {
            //cout << xid << "最终插入了第" << Qk << "船的第" << Qi << "个位置" << endl;
            Route_best.Routes[Qk].V.insert(Route_best.Routes[Qk].V.begin() + Qi, Customer[xid]);
            Route_best.Total_Profit = Calculation(Route_best);

            // 从 removedCustomers 中移除该任务点
            sol.removedCustomers.erase(xid);

           /* cout << "查看移除之后的888888888888" << endl;
            for (int customer : sol.removedCustomers)
            {
                cout << customer << " ";
            }
            cout << endl;
            system("pause");*/

            //cout << "任务点" << it << "成功插入并从待插入列表中移除" << endl;
        }
        else
        {
           // cout << "任务点" << xid << "无法插入到任何可行位置，跳过该任务点" << endl;

            continue;
            // 不执行插入操作，继续处理下一个任务点
        }

    }
    /*cout << "重新插入之后的路径" << endl << endl;
    for (int k = 0; k < Nk; k++)
    {
        cout << "第" << k << "条路径:" << endl;
        for (int i = 0; i < Route_best.Routes[k].V.size(); i++)
            cout << Route_best.Routes[k].V[i].customer_id << "->";
        cout << endl;
    }
    system("pause");*/
    //==============================end====================================
	// 最佳解已复制到 Route_best；释放本算例保留的两个解对象。
	delete sol_curr;
	delete sol_best;
	sol_curr = nullptr;
	sol_best = nullptr;

}

void ALNS::addRepair(SH method) 
{
    m_repair.push_back(method);//将传递给函数的 method 参数添加到名为 m_repair 的类成员向量中。
    weights_repair.push_back(1);//向名为 weights_repair 的另一个向量中添加了一个值1。这个向量可能用于存储修复方法的权重。
    scores_repair.push_back(0);//向名为 scores_repair 的另一个向量中添加了一个值0。这个向量可能用于存储修复方法的分数。
    times_repair.push_back(1);//向名为 times_repair 的另一个向量中添加了一个值1。这个向量可能用于存储修复方法的时间信息。
    pair_weights.assign(m_destroy.size(), vector<float>(m_repair.size(), 1.0f));
    pair_scores.assign(m_destroy.size(), vector<float>(m_repair.size(), 0.0f));
    pair_times.assign(m_destroy.size(), vector<float>(m_repair.size(), 1.0f));
}

void ALNS::addDestroy(SH method) 
{
    m_destroy.push_back(method);
    weights_destroy.push_back(1);
    scores_destroy.push_back(0);
    times_destroy.push_back(1);
    pair_weights.assign(m_destroy.size(), vector<float>(m_repair.size(), 1.0f));
    pair_scores.assign(m_destroy.size(), vector<float>(m_repair.size(), 0.0f));
    pair_times.assign(m_destroy.size(), vector<float>(m_repair.size(), 1.0f));
}
