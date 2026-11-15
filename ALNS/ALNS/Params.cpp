
#include "solution.hpp"
#include "Params.h"
#include "Solve_SOCP.h"


#include <iostream>
#include <unordered_map>
#include <sstream>


int cpu_time = 3600;
float Initial_W;

namespace
{
	struct CachedRouteValue
	{
		bool feasible;
		double profit;
		Result diagnostics;
	};

	std::unordered_map<std::string, CachedRouteValue> route_cache;
	EvaluationStats evaluation_stats;
	std::mt19937 alns_rng(20250914u);

	std::string RouteSignature(const Route_Type& route)
	{
		// 缓存在每个算例开始时清空，因此序列 ID 足以唯一表示当前子问题。
		std::ostringstream out;
		for (const auto& customer : route.V)
			out << customer.customer_id << ',';
		return out.str();
	}

	bool ViolatesNecessaryConditions(const Route_Type& route)
	{
		double total_service = 0.0;
		for (const auto& customer : route.V)
		{
			total_service += customer.service_time;
			// 服务时间单独已超过 UAV 续航，则不可能存在可行起降点。
			if (customer.service_time > a + MY_EPSILON)
				return true;
			if (customer.earlist - Delta_1 > T + MY_EPSILON ||
				customer.duedate + Delta_2 < -MY_EPSILON)
				return true;
		}
		// 巡逻周期至少要容纳所有服务时间；这是一个安全的必要条件。
		return total_service > T + MY_EPSILON;
	}

	void ClearRouteDiagnostics(Route_Type& route)
	{
		route.arrival_times.clear();
		route.task_profits.clear();
		route.outbound_times.clear();
		route.return_times.clear();
		route.vessel_sync_times.clear();
		route.takeoff_x.clear();
		route.takeoff_y.clear();
		route.landing_x.clear();
		route.landing_y.clear();
	}

	void ApplyDiagnostics(Route_Type& route, const Result& result)
	{
		route.arrival_times = result.arrival_times;
		route.task_profits = result.task_profits;
		route.outbound_times = result.outbound_times;
		route.return_times = result.return_times;
		route.vessel_sync_times = result.vessel_sync_times;
		route.takeoff_x = result.takeoff_x;
		route.takeoff_y = result.takeoff_y;
		route.landing_x = result.landing_x;
		route.landing_y = result.landing_y;
	}
}

void ResetEvaluationCache()
{
	// 每个算例结束时真正释放哈希表和 SOCP 诊断向量的容量。
	std::unordered_map<std::string, CachedRouteValue>().swap(route_cache);
	evaluation_stats = EvaluationStats();
}

EvaluationStats GetEvaluationStats()
{
	return evaluation_stats;
}

std::mt19937& AlnsRandomEngine()
{
	return alns_rng;
}

void SetAlnsRandomSeed(unsigned int seed)
{
	alns_rng.seed(seed);
}

double EvaluateRouteCached(Route_Type& route, bool& feasible)
{
	if (route.V.empty())
	{
		feasible = true;
		route.Profit = 0.0;
		ClearRouteDiagnostics(route);
		return 0.0;
	}

	const std::string key = RouteSignature(route);
	const char* disable_cache = getenv("ALNS_DISABLE_CACHE");
	const bool cache_enabled = disable_cache == nullptr || std::string(disable_cache) != "1";
	// 单个大算例也可能产生大量不同路线；限制缓存规模，避免内存无限增长。
	if (cache_enabled && route_cache.size() >= 5000)
		std::unordered_map<std::string, CachedRouteValue>().swap(route_cache);
	auto cached = route_cache.find(key);
	if (cache_enabled && cached != route_cache.end())
	{
		++evaluation_stats.cache_hits;
		feasible = cached->second.feasible;
		route.Profit = cached->second.profit;
		ApplyDiagnostics(route, cached->second.diagnostics);
		return route.Profit;
	}

	if (ViolatesNecessaryConditions(route))
	{
		++evaluation_stats.fast_rejections;
		feasible = false;
		route.Profit = -99999999999999.0;
		ClearRouteDiagnostics(route);
		if (cache_enabled)
		{
			Result rejected;
			rejected.success = false;
			rejected.obj = route.Profit;
			route_cache[key] = { false, route.Profit, rejected };
		}
		return route.Profit;
	}

	vector<int> sequence;
	sequence.reserve(route.V.size());
	for (const auto& customer : route.V)
		sequence.push_back(customer.customer_id);

	++evaluation_stats.conic_solves;
	Result result = cplex_solve(static_cast<IloInt>(sequence.size()), sequence, route,
		vv, vd, a, Profit_cost, Delta_1, Delta_2, T, profit_early, profit_late);
	feasible = result.success;
	route.Profit = feasible ? result.obj : -99999999999999.0;
	if (feasible)
		ApplyDiagnostics(route, result);
	else
		ClearRouteDiagnostics(route);
	if (cache_enabled)
		route_cache[key] = { feasible, route.Profit, result };
	return route.Profit;
}


Params::Params()
{
	// 初始化随机种子
	//generator.seed(seedRNG);

}

double Calculation(Customer_schedule Sche)
{
	// 增量评价：每艘船的子问题按路线序列独立缓存。
	// 候选解只改变一条路线时，其余船舶将直接命中缓存。
	{
		++evaluation_stats.schedule_calls;
		double total_profit = 0.0;
		for (int k = 0; k < Nk; ++k)
		{
			bool feasible = false;
			double route_profit = EvaluateRouteCached(Sche.Routes[k], feasible);
			if (!feasible)
				return -999999.0;
			total_profit += route_profit;
		}
		return total_profit;
	}

	/*cout << "中间计算过程———————————————最优路径信息 : " << endl;
	for (int k = 0; k < Nk; k++)
	{
		cout << "第" << k << "条路径:" << endl;
		for (int i = 0; i < Sche.Routes[k].V.size(); i++)
			cout << Sche.Routes[k].V[i].customer_id << "->";
		cout << endl;
	}*/

	// 4. 计算每条船舶的收益
	// 初始化二维向量 y_ik
	vector<vector<int>> y_ik(Nj + 1, vector<int>(Nk + 1, 0));

	// 初始化三维向量 x_ijk
	vector<vector<vector<int>>> x_ijk(Nj + 1, vector<vector<int>>(Nj + 1, vector<int>(Nk + 1, 0)));

	// 填充 y_ik 和 x_ijk
	for (int k = 0; k < Nk; k++)
	{
		for (int j = 0; j < Sche.Routes[k].V.size(); j++)
		{
			int customer_id = Sche.Routes[k].V[j].customer_id;
			y_ik[customer_id][k+1] = 1; // 任务点 i 分配到船舶 k 上

			if (j > 0)
			{
				int prev_customer_id = Sche.Routes[k].V[j - 1].customer_id;
				x_ijk[prev_customer_id][customer_id][k+1] = 1; // 任务点 i 在任务点 j 前面
			}
		}

		// 设置 x[i][0][k] 和 x[0][i][k]
		if (!Sche.Routes[k].V.empty())
		{
			int first_customer_id = Sche.Routes[k].V[0].customer_id;
			if (first_customer_id != 0)
			{
				x_ijk[0][first_customer_id][k+1] = 1; // 从港口到第一个任务点
			}

			int last_customer_id = Sche.Routes[k].V.back().customer_id;
			if (last_customer_id != 0)
			{
				x_ijk[last_customer_id][0][k+1] = 1; // 从最后一个任务点回到港口
			}
		}
	}


	// 输出 y_ik
	//cout << "y_ik:" << endl;
	//for (int i = 1; i <= Nj; i++)
	//{
	//	for (int k = 1; k <= Nk; k++)
	//	{
	//		cout << "y[" << i << "][" << k << "] = " << y_ik[i][k] << endl;
	//	}
	//}


	/*for (int k = 1; k <= Nk; k++)
	{
		cout << "k = " << k << ":";
		for (int i = 1; i <= Nj; i++)
			if (y_ik[i][k] >= 0.001)
				cout << i << " ";
		cout << endl;
	}*/


	for (int k = 0; k < Nk; k++)
	{

		int ass_num = 0;

		vector<int> sortpoint;//store vessel k上每一个trip访问的node
		sortpoint.clear();

		//cout << seq[k].size() << endl;		
		//cout << "sortpoint.size() = " << sortpoint.size() << endl;

		for (int j = 0; j < Sche.Routes[k].V.size(); j++)
		{
			sortpoint.push_back(Sche.Routes[k].V[j].customer_id);
			ass_num++;
		}

		/*cout << "看一看" << endl;
		for (int j = 0; j < sortpoint.size(); j++)
			cout << sortpoint[j] << " ";
		cout << endl;*/



		/*cout << "[";
		for (int i = 0; i < sortpoint.size(); i++)
		{
			cout << Customer[sortpoint[i]].px << "," << Customer[sortpoint[i]].py << endl;
		}
		cout << "],";*/



		int size = sortpoint.size();

		if (ass_num >= 0.1)
		{
			//cout << "第[" << k << "]个vessel上有访问点" << endl;
			Result result_sp;
			result_sp = cplex_solve(size, sortpoint, Sche.Routes[k], vv, vd, a, Profit_cost, Delta_1, Delta_2, T, profit_early, profit_late);				

			if (result_sp.success) //如果成功求解
			{
				Sche.Routes[k].Profit = result_sp.obj;
				//cout << "result_sp.obj = " << result_sp.obj << endl;
								
			}
			else
			{
				cout << "--------------没有求解成功，说明无解--------------------" << endl;
				Sche.Total_Profit = -999999;
				return Sche.Total_Profit;
			}
			
		}
		else
		{
			Sche.Routes[k].Profit = 0;
		}
	}

	Sche.Total_Profit = 0;
	for (int k = 0; k < Nk; k++)
	{
		Sche.Total_Profit += Sche.Routes[k].Profit;
	}

	//cout << "Total_cost = " << Sche.Total_Profit << endl;
	return Sche.Total_Profit;
	//return Sche;
}



Customer_schedule Calculation_removal(Customer_schedule Sche)
{
	// 移除操作后同样使用船舶级缓存，并保留每条路线的利润。
	{
		++evaluation_stats.schedule_calls;
		Sche.Total_Profit = 0.0;
		for (int k = 0; k < Nk; ++k)
		{
			bool feasible = false;
			EvaluateRouteCached(Sche.Routes[k], feasible);
			Sche.Total_Profit += Sche.Routes[k].Profit;
		}
		return Sche;
	}

	/*cout << "中间计算过程———————————————最优路径信息 : " << endl;
	for (int k = 0; k < Nk; k++)
	{
		cout << "第" << k << "条路径:" << endl;
		for (int i = 0; i < Sche.Routes[k].V.size(); i++)
			cout << Sche.Routes[k].V[i].customer_id << "->";
		cout << endl;
	}*/

	// 4. 计算每条船舶的收益
	// 初始化二维向量 y_ik
	vector<vector<int>> y_ik(Nj + 1, vector<int>(Nk + 1, 0));

	// 初始化三维向量 x_ijk
	vector<vector<vector<int>>> x_ijk(Nj + 1, vector<vector<int>>(Nj + 1, vector<int>(Nk + 1, 0)));

	// 填充 y_ik 和 x_ijk
	for (int k = 0; k < Nk; k++)
	{
		for (int j = 0; j < Sche.Routes[k].V.size(); j++)
		{
			int customer_id = Sche.Routes[k].V[j].customer_id;
			y_ik[customer_id][k + 1] = 1; // 任务点 i 分配到船舶 k 上

			if (j > 0)
			{
				int prev_customer_id = Sche.Routes[k].V[j - 1].customer_id;
				x_ijk[prev_customer_id][customer_id][k + 1] = 1; // 任务点 i 在任务点 j 前面
			}
		}

		// 设置 x[i][0][k] 和 x[0][i][k]
		if (!Sche.Routes[k].V.empty())
		{
			int first_customer_id = Sche.Routes[k].V[0].customer_id;
			if (first_customer_id != 0)
			{
				x_ijk[0][first_customer_id][k + 1] = 1; // 从港口到第一个任务点
			}

			int last_customer_id = Sche.Routes[k].V.back().customer_id;
			if (last_customer_id != 0)
			{
				x_ijk[last_customer_id][0][k + 1] = 1; // 从最后一个任务点回到港口
			}
		}
	}


	// 输出 y_ik
	//cout << "y_ik:" << endl;
	//for (int i = 1; i <= Nj; i++)
	//{
	//	for (int k = 1; k <= Nk; k++)
	//	{
	//		cout << "y[" << i << "][" << k << "] = " << y_ik[i][k] << endl;
	//	}
	//}


	for (int k = 1; k <= Nk; k++)
	{
		cout << "k = " << k << ":";
		for (int i = 1; i <= Nj; i++)
			if (y_ik[i][k] >= 0.001)
				cout << i << " ";
		cout << endl;
	}


	for (int k = 0; k < Nk; k++)
	{

		int ass_num = 0;

		vector<int> sortpoint;//store vessel k上每一个trip访问的node
		sortpoint.clear();

		//cout << seq[k].size() << endl;		
		//cout << "sortpoint.size() = " << sortpoint.size() << endl;

		for (int j = 0; j < Sche.Routes[k].V.size(); j++)
		{
			sortpoint.push_back(Sche.Routes[k].V[j].customer_id);
			ass_num++;
		}

		/*cout << "看一看" << endl;
		for (int j = 0; j < sortpoint.size(); j++)
			cout << sortpoint[j] << " ";
		cout << endl;

		cout << "[";
		for (int i = 0; i < sortpoint.size(); i++)
		{
			cout << Customer[sortpoint[i]].px << "," << Customer[sortpoint[i]].py << endl;
		}
		cout << "],";*/

		int size = sortpoint.size();

		if (ass_num >= 0.01)
		{
			//cout << "第[" << k << "]个vessel上有访问点" << endl;
			Result result_sp;
			result_sp = cplex_solve(size, sortpoint, Sche.Routes[k], vv, vd, a, Profit_cost, Delta_1, Delta_2, T, profit_early, profit_late);			

			//cout << "result_sp.obj = " << result_sp.obj << endl;
			if (result_sp.success) //如果成功求解，添加opt cut
			{
				Sche.Routes[k].Profit = result_sp.obj;			
			}
			else
			{
				cout << "--------------没有求解成功，说明无解--------------------" << endl;
				Sche.Routes[k].Profit = -99999999999999;
			}

		}
		else
		{
			Sche.Routes[k].Profit = 0;
		}
	}

	Sche.Total_Profit = 0;
	for (int k = 0; k < Nk; k++)
	{
		Sche.Total_Profit += Sche.Routes[k].Profit;
	}

	//cout << "Total_cost = " << Sche.Total_Profit << endl;
	return Sche;
}

