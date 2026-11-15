#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <io.h>
#include <process.h>
//0#include <unistd.h>
#include <direct.h>
#include <cmath>
#include "alns.hpp"
#include <ilcplex/ilocplex.h>
#include <ilcplex/ilocplexi.h>
#include <stdlib.h>
#include <time.h>
#include <iomanip>
#include <math.h>
#include <algorithm> 
#include <set>
#include <vector>
#include <windows.h>
#include <tchar.h>
#include <shellapi.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "Params.h"
#include "solution.hpp"
#include "Solve_SOCP.h"


using namespace std;


//LKH_read用来生成一个tspshunxu，将他传递给CVP,用作固定油田点的访问顺序
//CVP作为单船单无人机模型，求解每个油田点的起飞点和降落点，用作初始解


int num1, num2, num3, Iter_Max, m;
int Nj, Nk, T, Profit_cost, vd, vv, profit_early, profit_late, Customer_Number;
double a, Delta_1, Delta_2;

//// 记录已移除的客户点集合
//set<int> removedCustomers;


//************************ALNS算法************************************
Solution* baseSolution;//这是一个声明，它创建了一个名为baseSolution的指针，该指针可能用于存储算法中的基本解决方案。

//这里应该属于函数的声明
Solution* removal_random(Solution& sol);//定义了函数，该函数接受一个Solution对象的引用作为参数，并返回一个指向Solution对象的指针。该函数可能用于从给定解决方案中随机移除一些元素。
Solution* removal_revenue_based(Solution& sol);//定义了函数，它也接受一个Solution对象的引用作为参数，并返回一个指向Solution对象的指针。该函数可能用于从给定解决方案中移除最差的元素。
Solution* removal_time_window(Solution& sol);//定义了函数，它接受一个Solution对象的引用作为参数，并返回一个指向Solution对象的指针。该函数可能用于根据某种比率从解决方案中移除元素。
Solution* removal_shaw(Solution& sol);
Solution* removal_inter_vessel_pressure(Solution& sol);
Solution* removal_temporal_profit_externality(Solution& sol);
Solution* removal_synchronization_critical_string(Solution& sol);

Solution* insertion_greedy(Solution& sol);
Solution* insertion_greedy_with_noise(Solution& sol);
Solution* insertion_best(Solution& sol);
Solution* insertion_best_with_noise(Solution& sol);
Solution* insertion_regret(Solution& sol);//定义了函数，它接受一个Solution对象的引用作为参数，并返回一个指向Solution对象的指针。该函数可能用于根据某种遗憾值策略向解决方案中插入元素。
Solution* insertion_cross_vessel_regret(Solution& sol);
Solution* insertion_sync_profit_regret(Solution& sol);


void ReadData();
void Initialization();
Solution Construction();
void runALNS(Params* params);


int main()
{
	// 源码按 UTF-8 编译，控制台也使用 UTF-8，避免中文输出乱码。
	SetConsoleOutputCP(CP_UTF8);
	// 直接修改三个循环的起止值，即可连续读取任务数、船舶数和算例编号。
	for (num1 = 50; num1 <= 100; num1 += 5)
		for (num2 = 6; num2 <= 8; num2 += 2)
			for (num3 = 1; num3 <= 1; num3 += 1)
				for(int num4=1; num4 <= 1; num4++)
			{
				// 不存在的算例直接跳过，继续运行后面的算例。
				char fname_check[200];
				snprintf(fname_check, sizeof(fname_check), "..\\Data(4.4)\\Example%d_%d_%d.dat", num1, num2, num3);
				ifstream file_check(fname_check);
				if (!file_check)
				{
					cerr << "Missing instance, skip: " << fname_check << endl;
					continue;
				}
				file_check.close();
				//Resultfile2 << num1 << "_" << num2 << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;				

				//每次循环开始时，代码还会根据循环变量的值来设置一些全局变量，如Customer_Number和Iter_Max。然后，代码调用ReadData()函数来读取数据，并创建一个名为Params的参数对象。
				//Customer_Number = num2 - 1;
				//cout << "Customer_Number: " << Customer_Number << endl;
				Iter_Max = num2;
				srand((unsigned)time(NULL));
				clock_t Start, Finish;
				Start = clock();
				ReadData();  //读取数据
				Customer_Number = Nj; // 将迭代规模与实际任务数同步，避免使用未赋值的全局变量。

				//cout << "num_j = " << Nj << endl;
				//cout << "vessel = " << Nk << endl;
				//cout << "cycle = " << T << endl;
				//cout << "profit_cost = " << Profit_cost << endl;
				//cout << "drone_speed = " << vd << endl;
				//cout << "vessel_speed = " << vv << endl;
				//cout << "drone_capacity = " << a << endl;	
				//cout << "Delta_1 = " << Delta_1 << endl;
				//cout << "Delta_2 = " << Delta_2 << endl;
				//cout << "profit_early = " << profit_early << endl;
				//cout << "profit_late = " << profit_late << endl;

				//cout << "px[j] = ";//忽略到等于号
				//for (int j = 0; j <= Nj; j++)
				//{
				//	cout << Customer[j].px << " ";
				//}
				//cout << endl;

				//cout << "py[j] = ";
				//for (int j = 0; j <= Nj; j++)
				//{
				//	cout << Customer[j].py << " ";
				//}
				//cout << endl;


				//cout << "service_time[j] = ";
				//for (int j = 0; j <= Nj; j++)
				//{
				//	cout << Customer[j].service_time << " ";
				//}
				//cout << endl;

				//cout << "earlist[j] = ";
				//for (int j = 0; j <= Nj; j++)
				//{
				//	cout << Customer[j].earlist << " ";
				//}
				//cout << endl;

				//cout << "duedate[j] = ";
				//for (int j = 0; j <= Nj; j++)
				//{
				//	cout << Customer[j].duedate << " ";
				//}
				//cout << endl;


				//cout << "weight[j] = ";
				//for (int j = 0; j <= Nj; j++)
				//{
				//	cout << Customer[j].weight << " ";
				//}
				//cout << endl;

				Params params;//创建一个对象


				Initialization();  //路径初始化,调用Initialization()函数来执行路径的初始化操作

				// 新的调用方式
				Solution initialSolution = Construction();  // 获取包含完整信息的Solution对象

				
				//Construction();   //构造初始路径,调用Construction()函数来构建初始路径。

				double initial_objective = Route_current.Total_Profit;

				cout << "initial_objective =" << initial_objective;

				baseSolution = new Solution(initialSolution);//创建了一个名为baseSolution的解对象

				// 关键：确保正确设置Solution对象的所有信息
				baseSolution->SRoute = Route_current;           // 设置路径信息
				baseSolution->revenue = initial_objective;           // 设置收益 - 重要！


				//// 验证baseSolution是否包含正确的removedCustomers
				//cout << "=== 验证baseSolution ===" << endl;
				//cout << "baseSolution的removedCustomers大小: " << baseSolution->removedCustomers.size() << endl;
				//if (!baseSolution->removedCustomers.empty()) {
				//	cout << "baseSolution中的removedCustomers: ";
				//	for (int customer : baseSolution->removedCustomers) {
				//		cout << customer << " ";
				//	}
				//	cout << endl;
				//}		

				runALNS(&params);//ALNS搜索,调用runALNS(&params)函数来执行ALNS搜索算法，尝试改进初始路径。

				//详细结果输出
				char instance[200];
				sprintf(instance, "..\\results\\detail results\\Example_%d_%d_%d.txt", num1, num2, num3);
				ofstream Foutfile(instance, ios::app);

				Foutfile << "Route.Total_profit : " << Route_best.Total_Profit << endl;
				Foutfile << "最优路径信息 : " << endl;
				for (int k = 0; k < Nk; k++)
				{
					Foutfile << "第" << k + 1 << "条路径:" << endl;
					for (int i = 0; i < Route_best.Routes[k].V.size(); i++)
						Foutfile << Route_best.Routes[k].V[i].customer_id << "->";
					Foutfile << endl;
				}
				

				double final_obj = Calculation(Route_best);

				// 输出检查 Route 对象的内容
				//cout << "Output之后的Route.Dis: " << final_obj << endl;
				
				Finish = clock();
				Resultfile << "Example" << num1 << "_" << num2 << "_" << num3 << "	" << Nj << "	" << Nk << "	" << initial_objective << "	" << final_obj << "	" << (Finish - Start) / 1000.0 << endl; // 依次在一个txt写出解

				cout << "Example" << num1 << "_" << num2 << "_" << num3 << "	" << Nj << "	" << Nk << "	" << initial_objective << "	" << final_obj << "	" << (Finish - Start) / 1000.0 << endl; // 依次在一个txt写出解
				cout << "运行时间：" << (Finish - Start) / 1000.0 << endl;

				//释放向量
				// 直接清空容器；原实现在 clear() 后继续访问 Routes[k]，存在越界。
				Route_best.Routes.clear();
				Route_best.Total_Profit = 0.0;
				Route_current.Routes.clear();
				Route_current.Total_Profit = 0.0;
				// 本算例的路线缓存与初始解不再被后续算例使用。
				ResetEvaluationCache();
				delete baseSolution;
				baseSolution = nullptr;

			}



	return 0;
}



void ReadData()
{
	char fname[1000];
	snprintf(fname, sizeof(fname), "..\\Data(4.4)\\Example%d_%d_%d.dat", num1, num2, num3);
	ifstream fin(fname);
	if (!fin)
	{
		cerr << "ERROR: could not open file '" << fname << "' for reading" << endl;
		throw(-1);
	}


	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> Nj;


	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> Nk;

	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> T;


	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> Profit_cost;


	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> vd;


	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> vv;

	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> a;	//


	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> Delta_1;	//


	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> Delta_2;	//

	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> profit_early;	//


	fin.ignore(256, '=');// 跳过直到读到 =
	fin >> profit_late;	//


	//===================导入数据信息===============

	char temp;
	fin.ignore(256, '=');
	//cout << "px[j] = ";//忽略到等于号
	for (int j = 0; j <= Nj; j++)
	{
		double temp;
		fin >> temp;
		Customer[j].px = temp;
		//cout << Customer[j].px << " ";
	}
	//cout << endl;

	fin.ignore(256, '=');       //忽略到等于号
	//cout << "py[j] = ";
	for (int j = 0; j <= Nj; j++)
	{
		double temp;
		fin >> temp;
		Customer[j].py = temp;
		//cout << Customer[j].py << " ";
	}
	//cout << endl;

	fin.ignore(256, '=');       //忽略到等于号

	//cout << "service_time[j] = ";
	for (int j = 0; j <= Nj; j++)
	{
		double temp;
		fin >> temp;
		Customer[j].service_time = temp;
		//cout << Customer[j].service_time << " ";
	}
	//cout << endl;

	fin.ignore(256, '=');       //忽略到等于号

	//cout << "earlist[j] = ";
	for (int j = 0; j <= Nj; j++)
	{
		double temp;
		fin >> temp;
		Customer[j].earlist = temp;
		//cout << Customer[j].earlist << " ";
	}
	//cout << endl;

	fin.ignore(256, '=');       //忽略到等于号
	//cout << "duedate[j] = ";
	for (int j = 0; j <= Nj; j++)
	{
		double temp;
		fin >> temp;
		Customer[j].duedate = temp;
		//cout << Customer[j].duedate << " ";
	}
	//cout << endl;

	fin.ignore(256, '=');       //忽略到等于号
	//cout << "weight[j] = ";
	for (int j = 0; j <= Nj; j++)
	{
		double temp;
		fin >> temp;
		Customer[j].weight = temp;
		//cout << Customer[j].weight << " ";
	}
	//cout << endl;

	for (int j = 0; j <= Nj; j++)
	{
		Customer[j].customer_id = j;
		//cout << Customer[j].weight << " ";
	}
}



void Initialization()
{
	for (int k = 1; k <= Nk; ++k)
	{
		if (!Route[k].V.empty())//检查名为Route的对象的成员变量V是否为空
			Route[k].V.clear();//如果Route.V不为空，那么这行代码将清空Route对象的成员变量V，即删除所有已经存在的元素。	
		Route[k].Profit = 0;// 将名为Route的对象的成员变量Profit的值设置为0。
	}				
}




Solution Construction()
{
	//=============移除的任务点要放置在中，然后再nSol->nodes_banks中随机选择几个插入，随机选择几个呢，做一个local search,这样就能将移除掉的点尽可能插入到
	

	cout << "****************************************Construction Start************************************" << endl;
	Solution sol;
	bool solutionFound = false;
	int removedCount = 0;
	int maxRemovalAttempts = Nj/4+1; // 最多可移除的客户点数量


	// 定义分配优先级计算函数
	auto calculateAssignmentPriority = [&](int customer_id) -> double
	{
		double timeWindowSize = Customer[customer_id].duedate - Customer[customer_id].earlist;
		double weight = Customer[customer_id].weight;
		double serviceTime = Customer[customer_id].service_time;

		// 权重系数，可以根据实际情况调整
		double alpha = 0.0;  // 时间窗紧迫度权重
		double beta = 1.0;   // 任务权重
		double gamma = 0.0;  // 服务时间权重

		return alpha * (1.0 / (timeWindowSize + 0.001)) + beta * weight + gamma * (1.0 / (serviceTime + 0.001));
	};

	// 尝试不同的移除客户点数量
	while (!solutionFound && removedCount <= maxRemovalAttempts)
	{
		cout << "尝试移除 " << removedCount << " 个客户点进行贪心构造..." << endl;

		// 1. 清空所有船舶路径
		for (int k = 1; k <= Nk; k++)
		{
			Route[k].V.clear();
			Route[k].Profit = 0;
		}

		// 2. 创建可用客户列表并按优先级排序
		vector<pair<int, double>> customerPriorities;
		for (int i = 1; i <= Nj; ++i)
		{
			if (sol.removedCustomers.find(i) == sol.removedCustomers.end())
			{
				double priority = calculateAssignmentPriority(i);
				customerPriorities.push_back({ i, priority });
			}
		}


		// 按优先级从高到低排序
		sort(customerPriorities.begin(), customerPriorities.end(),
			[](const pair<int, double>& a, const pair<int, double>& b) {
				return a.second > b.second;  // 优先级高的在前
			});

		/*cout << "按优先级排序后的前10个客户: ";
		for (int i = 0; i < min(24, (int)customerPriorities.size()); i++) {
			cout << customerPriorities[i].first << "(" << customerPriorities[i].second << ") ";
		}
		cout << endl;*/


		// 3. 贪心分配：逐个任务分配并检查可行性
		bool constructionSuccess = true;
		int currentRoundRemoved = 0;  // 记录本轮构造中移除的任务数
		int taskIndex = 0;  // 用于轮询分配的任务计数器

		for (auto& customerPair : customerPriorities)
		{
			int customer_id = customerPair.first;
			bool assigned = false;

			// 轮询分配：从预定的船舶开始尝试，如果不行再试其他船舶
			int startVessel = (taskIndex % Nk) + 1;  // 轮询起始船舶
			taskIndex++;

			// 先尝试轮询分配的船舶
			for (int offset = 0; offset < Nk; offset++)
			{
				int k = ((startVessel - 1 + offset) % Nk) + 1;  // 从startVessel开始轮询

				/*cout << "尝试分配任务 " << customer_id << " 给船舶 " << k;
				if (offset == 0)
					cout << " (轮询首选)";
				cout << endl;*/

				// 临时分配
				Route[k].V.push_back(Customer[customer_id]);
				Customer[customer_id].vessel = k;

				// 按earliest重新排序该船舶的任务
				sort(Route[k].V.begin(), Route[k].V.end(),
					[&](const Customer_Type& a, const Customer_Type& b)
					{
						return a.earlist < b.earlist;
					});

				// 立即检查该船舶的可行性
				vector<int> sortpoint;

				for (int j = 0; j < Route[k].V.size(); j++) 
				{
					sortpoint.push_back(Route[k].V[j].customer_id);
				}

				Result result_sp;
				if (sortpoint.size() > 0)
				{
					result_sp = cplex_solve(sortpoint.size(), sortpoint, Route[k], vv, vd, a,
						Profit_cost, Delta_1, Delta_2, T, profit_early, profit_late);
				}
				else {
					result_sp.success = true;
					result_sp.obj = 0;
				}

				if (result_sp.success)
				{
					// 分配成功
					Route[k].Profit = result_sp.obj;
					assigned = true;
					//cout << "任务 " << customer_id << " 成功分配给船舶 " << k << endl;
					break;
				}
				else 
				{
					// 分配失败，回退
					//cout << "船舶 " << k << " 分配失败，回退" << endl;

					Route[k].V.erase(
						std::remove_if(Route[k].V.begin(), Route[k].V.end(),
							[&customer_id](const Customer_Type& customer) {   // 按引用捕获
								return customer.customer_id == customer_id;
							}),
						Route[k].V.end()
					);


					// 查找第一个值为8的元素
					auto it = std::find(sortpoint.begin(), sortpoint.end(), customer_id);
					if (it != sortpoint.end()) {
						sortpoint.erase(it);  // 删除找到的第一个8
						/*cout << "删除第一个8后: ";
						for (int val : sortpoint) cout << val << " ";
						cout << endl;*/
					}

				}
			}

			if (!assigned) 
			{
				//cout << "任务 " << customer_id << " 无法分配给任何船舶，直接移除" << endl;
				sol.removedCustomers.insert(customer_id);  // 直接加入移除列表
				currentRoundRemoved++;  // 本轮移除计数
			}

			//system("pause");
		}



		// 4. 检查是否所有已分配的船舶都有可行解
		bool allRoutesValid = true;
		for (int k = 1; k <= Nk; k++)
		{
			if (Route[k].V.size() > 0) 
			{
				vector<int> sortpoint;
				for (int j = 0; j < Route[k].V.size(); j++)
				{
					sortpoint.push_back(Route[k].V[j].customer_id);
				}

				Result result_sp = cplex_solve(sortpoint.size(), sortpoint, Route[k], vv, vd, a,
					Profit_cost, Delta_1, Delta_2, T, profit_early, profit_late);

				if (result_sp.success)
				{
					Route[k].Profit = result_sp.obj;
				}
				else
				{
					allRoutesValid = false;
					break;
				}
			}
		}

		if (allRoutesValid)
		{
			solutionFound = true;
			/*cout << "贪心构造成功找到可行解！" << endl;
			cout << "本轮构造中直接移除的任务数量: " << currentRoundRemoved << endl;*/
			// 更新总移除计数
			removedCount += currentRoundRemoved;

			/*cout << "最终分配的路径信息: " << endl;
			for (int k = 1; k <= Nk; k++)
			{
				cout << "船舶 " << k << " (收益: " << Route[k].Profit << "): ";
				for (int j = 0; j < Route[k].V.size(); j++)
					cout << Route[k].V[j].customer_id << " -> ";
				cout << endl;
			}*/
			break;
		}
		//system("pause");

	}

	//// 输出已移除的客户点
	//if (!sol.removedCustomers.empty())
	//{
	//	cout << "为了找到可行解，已移除以下客户点: ";
	//	for (int customer : sol.removedCustomers)
	//	{
	//		cout << customer << " ";
	//	}
	//	cout << endl;
	//}
	//system("pause");

	// 计算总收益
	double final_Profit = 0;
	for (int k = 1; k <= Nk; k++)
	{
		final_Profit += Route[k].Profit;
	}

	// 保存当前解
	Route_current.Total_Profit = final_Profit;
	Route_current.Routes.clear(); // 先清空，避免重复添加

	for (int k = 1; k <= Nk; k++)
	{
		Route_current.Routes.push_back(Route[k]);
	}

	// 输出初始解信息
	cout << "初始解信息:" << endl;
	for (int k = 1; k <= Nk; k++)
	{
		cout << "第" << k << "条路径上的收益: " << Route_current.Routes[k - 1].Profit << endl;
		cout << "路径: ";
		for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
			cout << Route_current.Routes[k - 1].V[i].customer_id << " -> ";
		cout << endl;
	}

	cout << "总收益 = " << Route_current.Total_Profit << endl;
	//system("pause");

	cout << "****************************************Construction End************************************" << endl;

	return sol;
}

//ALNS算法
void runALNS(Params* params)//该函数接受一个指向Params对象的指针作为参数
{
	ALNS fwork(params); // 算例结束时自动释放算子权重等容器。
	const char* generic_only_env = getenv("ALNS_GENERIC_ONLY");
	const bool generic_only = generic_only_env != nullptr
		&& std::string(generic_only_env) == "1";
	fwork.addDestroy(&removal_random);
	fwork.addDestroy(&removal_revenue_based);
	fwork.addDestroy(&removal_time_window);
	fwork.addDestroy(&removal_shaw);
	fwork.addDestroy(&removal_inter_vessel_pressure);
	// 索引5：利用任务对下游到达时间和分段收益的外部性进行移除。
	fwork.addDestroy(&removal_temporal_profit_externality);
	// 索引6：围绕 UAV 续航裕度小的任务释放连续同步片段。
	fwork.addDestroy(&removal_synchronization_critical_string);
	

	fwork.addRepair(&insertion_greedy);
	fwork.addRepair(&insertion_greedy_with_noise);
	fwork.addRepair(&insertion_best);
	fwork.addRepair(&insertion_best_with_noise);
	// 使用跨船 regret，而不是仅比较同一船舶上的两个插入位置。
	fwork.addRepair(&insertion_cross_vessel_regret);
	// 索引5：同时考虑跨船机会稀缺性、同步裕度和时间收益恢复。
	fwork.addRepair(&insertion_sync_profit_regret);
	

	fwork.run(baseSolution);
}

