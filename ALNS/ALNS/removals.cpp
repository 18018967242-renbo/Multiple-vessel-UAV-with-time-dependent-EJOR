#include "solution.hpp"
#include <ctime>

#include "Params.h"
#include <limits>
#include <numeric>
#include <random>
#define largeNum 9999999



using namespace std;


Solution* removal_random(Solution& sol)
{
	Solution* nSol = new Solution(sol);

    cout << "***********************************   removal_random  Start*****************************************" << endl << endl;

    //cout << "destroy1" << endl;
    //首先创建了一个新的 Solution 对象 nSol，并将其初始化为与输入解 sol 相同的状态。
    //这是为了不修改原始解 sol，而在新解 nSol 上进行操作。
   /*cout << "查看初始解的分配" << endl;
    for (int k = 1; k <= Nk; k++)
    {
        cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
        for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
            cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
        cout << endl;
    }
    cout << "final_Profit = " << Route_current.Total_Profit << endl;
	cout << "检查 removedCustomers 是否有值：" << endl;
	cout << "原始解中的 removedCustomers: ";
	for (int customer : sol.removedCustomers)
	{
		cout << customer << " ";
	}
	cout << endl;
	system("pause");*/

	int length = sol.removedCustomers.size();//将初始解构造时移除的任务点数量除去
	int newsize = Nj - length;



    //确定q，即要移除的节点数量
    int q = 0; //q代表移除的Job个数
    if (Nj < 20)
    {
        std::uniform_int_distribution<int> dis1(1, newsize * 1 / 5);
        q = dis1(AlnsRandomEngine());
    }
    else 
    {
        std::uniform_int_distribution<int> dis1(1, newsize * 1 / 8);
        q = dis1(AlnsRandomEngine());
    }
    //cout << "要移除的任务点数量：" << q << endl;   

    int qid;//用于存储要选择的油田点的标识符。
    int Pk, Pi;//Pk用于存储节点位于那一条路径上，也就是哪一个船舶上。Pi用于存储某条路径上的某节点
    

    //使用一个 while 循环，循环的次数由外部的 q 控制。循环的目的是随机选择要移除的节点，并检查节点是否已存在于 nSol->nodes_bank 中
    while (q--)
    {
        std::uniform_int_distribution<int> dis(1, Nj);
        // 生成随机数,将其赋给 qid。这个操作随机选择了一个节点标识符。
        qid = dis(AlnsRandomEngine());


       // cout << "要移除的任务点的标号： " << qid << endl;

        //使用 std::count 函数检查 qid 是否已经存在于 nSol->nodes_bank 向量中。如果节点已存在，则会执行以下操作：
        if (std::count(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), qid))
        {
            q = q + 1;//将 q 增加1，这样循环将继续选择下一个随机节点。
			//cout << "已经被移除了，不能重复移除";
            continue;
        }

		// 检查 qid 是否已经被移除（即是否在 removedCustomers 中）
		if (nSol->removedCustomers.find(qid) != nSol->removedCustomers.end()) 
		{
			//cout << "已经在初始解中被移除了，不能重复移除";
			continue;
		}
        
        //我们要在这里找到Route.V中所有Number为qid的数据，并且要记录他们在Route.V中的位置
        for (int k = 0; k < Nk; k++)
        {
            for (int j = 0; j < Route_current.Routes[k].V.size(); ++j)
            {
                //cout << "Route_current.Routes[" << k+1 << "].V.Number，即油田点的编号: " << Route_current.Routes[k].V[j].customer_id << endl;
                if (Route_current.Routes[k].V[j].customer_id == qid)
                {
                    Pk = k;
					Pi = j;
                }
            }
        }
       
        // 从路径Pk中移除点qid
		if (Pk >= 0 && Pk < Route_current.Routes.size())
		{
			auto& route = Route_current.Routes[Pk].V;
			route.erase(route.begin() + Pi);
			//cout << "Removed task " << qid << " from Route " << Pk + 1 << endl;
		}
		else 
		{
			cout << "No task with id " << qid << " found in any route" << endl;
		}

        Route_current = Calculation_removal(Route_current);

        nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
        nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue       
        nSol->nodes_count--;
        nSol->nodes_bank.push_back(qid);              
    }

	/*cout << "查看移除之后的路径" << endl;
	for (int k = 1; k <= Nk; k++)
	{
		cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
		for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
			cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
		cout << endl;
	}
	cout << "final_Profit = " << Route_current.Total_Profit << endl;

	cout << "查看已经移除的任务点：" << endl;
	for (auto it = nSol->nodes_bank.begin(); it != nSol->nodes_bank.end(); it++)
	{
		cout << *it << ",";
	}*/


	nSol->revenue = Route_current.Total_Profit;

	cout << endl << "***********************************  removal_random  End   *****************************************" << endl;

	return nSol;
}



Solution* removal_revenue_based(Solution& sol)
{
    cout << "*********************************** Revenue-Based Removal Start ***********************************" << endl;

    /*cout << "查看初始解的分配" << endl;
    for (int k = 1; k <= Nk; k++)
    {
        cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
        for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
            cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
        cout << endl;
    }
    cout << "final_Profit = " << Route_current.Total_Profit << endl;*/

	int length = sol.removedCustomers.size();
	int newsize = Nj - length;

    // 1. 创建新解并初始化
    Solution* nSol = new Solution(sol);

    // 2. 确定要移除的任务点数量 q
    int q = 0;
    if (Nj < 20) 
    {
        q = max(1, newsize / 5); // 至少移除1个点
    }
    else          
    {
        q = max(1, newsize / 8); // 至少移除1个点
    }
    //cout << "要移除的任务点数量：" << q << endl;

	//system("pause");


    //3.计算每个任务点的收益
    //============================下面这部分是计算每个任务点的收益， 目的是为了优先移除收益最小的任务点================================

    for (int k = 1; k <= Nk; k++)
    {
        int ass_num = 0;

        vector<int> sortpoint;//store vessel k上每一个trip访问的node
        sortpoint.clear();

        for (int j = 0; j < Route_current.Routes[k-1].V.size(); j++)
        {
            sortpoint.push_back(Route_current.Routes[k-1].V[j].customer_id);
            ass_num++;
        }
        int size = sortpoint.size();

		/*for (int j = 0; j < sortpoint.size(); j++)
		{
			cout << "sortpoint[" << j << "] = " << sortpoint[j] << ",";
		}
		cout << endl;*/


		if (size > 0)
		{
			IloEnv env_cpx;
			//获取每个点的坐标位置
			IloInt Nr_cpx = sortpoint.size();

			IloNumArray px_socp(env_cpx, Nr_cpx + 1); // 
			IloNumArray py_socp(env_cpx, Nr_cpx + 1); // 
			IloNumArray service_time_socp(env_cpx, Nr_cpx + 1); // 服务时间
			IloNumArray earlist_socp(env_cpx, Nr_cpx + 1); // 最早服务时间
			IloNumArray duedate_socp(env_cpx, Nr_cpx + 1); // 最迟服务时间
			IloNumArray weight_socp(env_cpx, Nr_cpx + 1); // 权重

			//cout << "Nr_cps = " << Nr_cpx << endl;
			for (int i = 0; i < Nr_cpx; i++)
			{
				px_socp[i + 1] = Customer[sortpoint[i]].px;
				py_socp[i + 1] = Customer[sortpoint[i]].py;
				service_time_socp[i + 1] = Customer[sortpoint[i]].service_time;
				earlist_socp[i + 1] = Customer[sortpoint[i]].earlist;
				duedate_socp[i + 1] = Customer[sortpoint[i]].duedate;
				weight_socp[i + 1] = Customer[sortpoint[i]].weight;
			}
			//坐标变量
			IloNumVarArray Ptx(env_cpx, Nr_cpx + 1);   //无人机访问第i个油田点的起飞横坐标
			for (int i = 0; i <= Nr_cpx; i++)
			{
				Ptx[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray Pty(env_cpx, Nr_cpx + 1);   //无人机访问第i个油田点的起飞纵坐标
			for (int i = 0; i <= Nr_cpx; i++)
			{
				Pty[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray Plx(env_cpx, Nr_cpx + 1);   //无人机访问第i个油田点的降落横坐标
			for (int i = 0; i <= Nr_cpx; i++)
			{
				Plx[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray Ply(env_cpx, Nr_cpx + 1);   //无人机访问第i个油田点的降落纵坐标
			for (int i = 0; i <= Nr_cpx; i++)
			{
				Ply[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray t_total(env_cpx, Nr_cpx + 1);   //无人机访问任务点i起飞,船只航行的时间
			for (int i = 0; i <= Nr_cpx; i++)
			{
				t_total[i] = IloNumVar(env_cpx, 0, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray t1(env_cpx, Nr_cpx + 1);   //无人机从起飞点到目标点的飞行时间
			for (int i = 0; i <= Nr_cpx; i++)
			{
				t1[i] = IloNumVar(env_cpx, 0, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray t2(env_cpx, Nr_cpx + 1);   //无人机从目标点到降落点的飞行时间
			for (int i = 0; i <= Nr_cpx; i++)
			{
				t2[i] = IloNumVar(env_cpx, 0, IloInfinity, ILOFLOAT);
			}


			IloNumVarArray D_cpx(env_cpx, size + 2, 0, IloInfinity, ILOFLOAT);   //船在i点降落点和j点起飞点之间的巡航距离，包含港口点，所以总共有


			//编程辅助变量
			IloNumVarArray z1_sp(env_cpx, Nr_cpx + 1);//上一个油田点的降落点和下一个油田点的起飞点横坐标之间的差值
			for (int i = 1; i <= Nr_cpx; i++)
			{
				z1_sp[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray z2_sp(env_cpx, Nr_cpx + 1);//上一个油田点的降落点和下一个油田点的起飞点纵坐标之间的差值
			for (int i = 1; i <= Nr_cpx; i++)
			{
				z2_sp[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray z3_sp(env_cpx, Nr_cpx + 1);//油田点降落点和起飞点横坐标之间的差值
			for (int i = 1; i <= Nr_cpx; i++)
			{
				z3_sp[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray z4_sp(env_cpx, Nr_cpx + 1);//油田点降落点和起飞点纵坐标之间的差值
			for (int i = 1; i <= Nr_cpx; i++)
			{
				z4_sp[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray z7_sp(env_cpx, Nr_cpx + 1);//油田点和降落点横坐标之间的差值
			for (int i = 1; i <= Nr_cpx; i++)
			{
				z7_sp[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray z8_sp(env_cpx, Nr_cpx + 1);//油田点和降落点纵坐标之间的差值
			for (int i = 1; i <= Nr_cpx; i++)
			{
				z8_sp[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray z9_sp(env_cpx, Nr_cpx + 1);//上一个油田点的降落点和下一个油田点的起飞点横坐标之间的差值
			for (int i = 1; i <= Nr_cpx; i++)
			{
				z9_sp[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray z10_sp(env_cpx, Nr_cpx + 1);//上一个油田点的降落点和下一个油田点的起飞点纵坐标之间的差值
			for (int i = 1; i <= Nr_cpx; i++)
			{
				z10_sp[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT);
			}

			IloNumVarArray z11_sp(env_cpx, Nr_cpx + 1);
			for (int i = 1; i <= Nr_cpx; i++)
			{
				z11_sp[i] = IloNumVar(env_cpx, -IloInfinity, IloInfinity, ILOFLOAT); //完成定义二维变量
			}

			IloNumVar Gammak(env_cpx, 0, IloInfinity, ILOFLOAT); //收益


			IloNumVarArray total_profit11(env_cpx, Nr_cpx + 1);
			for (int i = 0; i <= Nr_cpx; i++)
			{
				total_profit11[i] = IloNumVar(env_cpx, 0, IloInfinity, ILOFLOAT); //完成定义二维变量
			}

			IloNumVarArray arrival_time_sp(env_cpx, Nr_cpx + 1);
			for (int i = 0; i <= Nr_cpx; i++)
			{
				arrival_time_sp[i] = IloNumVar(env_cpx, 0, IloInfinity, ILOFLOAT); //完成定义二维变量
			}


			IloModel model_sp(env_cpx);
			IloObjective obj_sp(env_cpx, Gammak, IloObjective::Maximize);
			model_sp.add(obj_sp);


			//约束（60），每个vechile上收益的范围
			IloExpr temp1(env_cpx);
			for (int i = 1; i <= Nr_cpx; i++)
			{
				temp1 += total_profit11[i] * weight_socp[i];

			}
			model_sp.add(Gammak <= temp1);
			temp1.end();

			//约束（61），针对任务点i无人机起飞和降落之间的距离和时间限制
			for (int i = 1; i <= Nr_cpx; i++)
			{
				model_sp.add(z1_sp[i] == Ptx[i] - Plx[i]);
				model_sp.add(z2_sp[i] == Pty[i] - Ply[i]);
				model_sp.add(z1_sp[i] * z1_sp[i] + z2_sp[i] * z2_sp[i] <= t_total[i] * t_total[i] * vv * vv);
			}

			//约束（62），i目标点和i目标点的起飞点之间的距离和时间限制
			for (int i = 1; i <= Nr_cpx; i++)
			{
				model_sp.add(z3_sp[i] == px_socp[i] - Ptx[i]);
				//cout << "1-----------------------------------------------------------------" << endl;
				model_sp.add(z4_sp[i] == py_socp[i] - Pty[i]);
				//cout << "2-----------------------------------------------------------------" << endl;
				model_sp.add(z3_sp[i] * z3_sp[i] + z4_sp[i] * z4_sp[i] <= vd * vd * t1[i] * t1[i]);
				//cout << "3-----------------------------------------------------------------" << endl;
			}

			//约束（63），i目标点和i目标点降落点之间的距离和时间限制
			for (int i = 1; i <= Nr_cpx; i++)
			{
				model_sp.add(z7_sp[i] == px_socp[i] - Plx[i]);
				model_sp.add(z8_sp[i] == py_socp[i] - Ply[i]);
				model_sp.add(z7_sp[i] * z7_sp[i] + z8_sp[i] * z8_sp[i] <= vd * vd * t2[i] * t2[i]);
			}


			//约束（64），港口和第一个起飞点之间的距离和时间限制
			{
				model_sp.add(Ptx[1] * Ptx[1] + Pty[1] * Pty[1] <= D_cpx[1] * D_cpx[1]);
			}


			//约束（65），返回港口和最后一个降落点之间的距离和时间限制
			{
				model_sp.add(Plx[Nr_cpx] * Plx[Nr_cpx] + Ply[Nr_cpx] * Ply[Nr_cpx] <= D_cpx[Nr_cpx + 1] * D_cpx[Nr_cpx + 1]);
			}


			//约束（66），第i-1个任务点降落点和第i个任务点起飞点之间的距离
			for (int i = 2; i <= Nr_cpx; i++)
			{
				model_sp.add(z9_sp[i] == Plx[i - 1] - Ptx[i]);
				model_sp.add(z10_sp[i] == Ply[i - 1] - Pty[i]);
				model_sp.add(z9_sp[i] * z9_sp[i] + z10_sp[i] * z10_sp[i] <= D_cpx[i] * D_cpx[i]);
			}



			//约束(67), 无人机每次起飞的飞行时间+服务时间不能超过续航
			for (int i = 1; i <= Nr_cpx; i++)
			{
				model_sp.add(t1[i] + t2[i] + service_time_socp[i] <= a);
			}

			//(68)
			for (int i = 1; i <= Nr_cpx; i++)
			{
				model_sp.add(t1[i] + t2[i] + service_time_socp[i] >= t_total[i]);
			}



			//(69) 巡航周期限制
			{
				IloExpr temp(env_cpx);
				for (int i = 1; i <= Nr_cpx; i++)
				{
					temp += D_cpx[i] / vv + t1[i] + t2[i] + service_time_socp[i];
				}
				model_sp.add(temp + D_cpx[Nr_cpx + 1] / vv <= T);	//周期T=24
				temp.end();

			}

			//(63)到达时间的限制	
			{
				model_sp.add(arrival_time_sp[1] >= D_cpx[1] / vv + t1[1]);
			}



			//(64)两个任务点到达时间之间的约束关系
			for (int i = 2; i <= Nr_cpx; i++)
			{
				model_sp.add(arrival_time_sp[i] >= arrival_time_sp[i - 1] + service_time_socp[i - 1] + t2[i - 1] + D_cpx[i] / vv + t1[i]);
			}


			//这个是后面补充的
			{
				model_sp.add(arrival_time_sp[Nr_cpx] + service_time_socp[Nr_cpx] + t2[Nr_cpx] + D_cpx[Nr_cpx + 1] / vv <= T);
			}




			//补充几个有效不等式
			for (int i = 1; i <= Nr_cpx; i++)
			{
				model_sp.add(t1[i] * vd <= t2[i] * vd + t_total[i] * vv);
			}


			for (int i = 1; i <= Nr_cpx; i++)
			{
				model_sp.add(t2[i] * vd <= t1[i] * vd + t_total[i] * vv);
			}


			for (int i = 1; i <= Nr_cpx; i++)
			{
				model_sp.add(z11_sp[i] == t1[i] - t2[i]);
				model_sp.add(z11_sp[i] * z11_sp[i] <= (a * a * vv * vv) / vd * vd);
			}


			//(22)
			//下面是用来表示分段函数, 其实可以不需要task_chosen[i]这个变量，但是由于下面的分段函数用y_ik表示一直有错误，所以只能引入task_chosen[i]变量
			// 定义 zz1[i], zz2[i], zz3[i], zz4[i]为二进制变量，用于分段条件
			IloIntVarArray zzz2(env_cpx, Nr_cpx + 1, 0, 1);
			IloIntVarArray zzz3(env_cpx, Nr_cpx + 1, 0, 1);
			IloIntVarArray zzz4(env_cpx, Nr_cpx + 1, 0, 1);
			for (int i = 1; i <= Nr_cpx; i++)
			{
				zzz2[i] = IloIntVar(env_cpx, 0, 1);
				zzz3[i] = IloIntVar(env_cpx, 0, 1);
				zzz4[i] = IloIntVar(env_cpx, 0, 1);

				model_sp.add(zzz2[i]);
				model_sp.add(zzz3[i]);
				model_sp.add(zzz4[i]);
			}

			double epsilon = 1e-8;

			for (int i = 1; i <= Nr_cpx; i++)
			{

				// 条件2：earliest[i]-Delta_1 <= arrival_time[i] < earliest[i] 用 zz2[i] = 1 表示
				model_sp.add(arrival_time_sp[i] >= earlist_socp[i] - Delta_1 - largeNum * (1 - zzz2[i]));
				model_sp.add(arrival_time_sp[i] <= earlist_socp[i] - epsilon + largeNum * (1 - zzz2[i]));


				// 条件3：earliest[i] <= arrival_time[i] <= duedate[i] 用 zz3[i] = 1 表示
				model_sp.add(arrival_time_sp[i] >= earlist_socp[i] - largeNum * (1 - zzz3[i]));
				model_sp.add(arrival_time_sp[i] <= duedate_socp[i] + largeNum * (1 - zzz3[i]));


				// 条件4：duedate[i] < arrival_time[i] <= duedate[i]+Delta_2 用 zz4[i] = 1 表示
				model_sp.add(arrival_time_sp[i] >= duedate_socp[i] + epsilon - largeNum * (1 - zzz4[i]));
				model_sp.add(arrival_time_sp[i] <= duedate_socp[i] + Delta_2 + largeNum * (1 - zzz4[i]));


				model_sp.add(zzz2[i] + zzz3[i] + zzz4[i] == 1);
			}

			double big_MM = 0.0;
			for (int i = 1; i <= Nr_cpx; i++)
			{
				big_MM += Nr_cpx * Profit_cost * weight_socp[i]; // 或者更大的值（如果可能）
			}


			// 约束分段函数的实现
			for (int i = 1; i <= Nr_cpx; i++)
			{
				// // 条件2：earliest[i]-Delta_1 <= arrival_time[i] < earliest[i]
				model_sp.add(total_profit11[i] >= profit_early + (Profit_cost - profit_early) / Delta_1 * (arrival_time_sp[i] - (earlist_socp[i] - Delta_1)) - 10 * big_MM * (1 - zzz2[i]));
				model_sp.add(total_profit11[i] <= profit_early + (Profit_cost - profit_early) / Delta_1 * (arrival_time_sp[i] - (earlist_socp[i] - Delta_1)) + 10 * big_MM * (1 - zzz2[i]));


				// 条件3：earliest[i] <= arrival_time[i] <= duedate[i]
				model_sp.add(total_profit11[i] >= Profit_cost - 10 * big_MM * (1 - zzz3[i]));
				model_sp.add(total_profit11[i] <= Profit_cost + 10 * big_MM * (1 - zzz3[i]));


				// 条件4：条件4：duedate[i] < arrival_time[i] <= duedate[i]+Delta_2
				model_sp.add(total_profit11[i] >= Profit_cost - (Profit_cost - profit_late) / Delta_2 * (arrival_time_sp[i] - duedate_socp[i]) - 10 * big_MM * (1 - zzz4[i]));
				model_sp.add(total_profit11[i] <= Profit_cost - (Profit_cost - profit_late) / Delta_2 * (arrival_time_sp[i] - duedate_socp[i]) + 10 * big_MM * (1 - zzz4[i]));



			}


			IloCplex mycplex_sp(model_sp);
			//mycplex_sp.setParam(IloCplex::Threads, 1); // 设置参数
			mycplex_sp.setParam(IloCplex::ClockType, 1);
			mycplex_sp.setParam(IloCplex::EpInt, 0);
			mycplex_sp.setParam(IloCplex::EpGap, 0.00);
			mycplex_sp.setParam(IloCplex::TiLim, 3600);
			mycplex_sp.setParam(IloCplex::WorkMem, 1024);
			mycplex_sp.setParam(IloCplex::NodeFileInd, 3);
			mycplex_sp.setParam(IloCplex::Param::MIP::Limits::TreeMemory, 2048);

			const bool route_solved = mycplex_sp.solve();
			/*cout << endl;
			for (int i = 1; i <= Nr_cpx; i++)
			{
				cout << "收益 " << i << "	" << mycplex_sp.getValue(total_profit11[i]) << endl;
			}*/

			for (int i = 0; i < Nr_cpx; i++)
			{
				if (route_solved)
					Customer[sortpoint[i]].single_profit = mycplex_sp.getValue(total_profit11[i + 1]);
				else if (i < static_cast<int>(Route_current.Routes[k - 1].task_profits.size()))
					Customer[sortpoint[i]].single_profit = Route_current.Routes[k - 1].task_profits[i];
			}
			// 通用收益移除算子也会独立调用 CPLEX，不能遗漏环境清理。
			mycplex_sp.end();
			env_cpx.end();
		}
		

		
    }
	
	
    // 4. 按收益升序排序（优先移除收益低的任务点）
	
	// 1. 将任务点索引和收益存入vector
	vector<pair<int, double>> profit_list;
	for (int j = 1; j <= Nj; j++)
	{
		profit_list.emplace_back(j, Customer[j].single_profit);
	}

	// 2. 按收益升序排序
	sort(profit_list.begin(), profit_list.end(),
		[](const pair<int, double>& a, const pair<int, double>& b) 
		{
			return a.second < b.second;
		});

	//// 3. 输出排序结果
	//for (const auto& item : profit_list) 
	//{
	//	cout << "任务点 " << item.first << ": " << item.second << endl;
	//}
	
	int qid;//用于存储要选择的油田点的标识符。
	int Pk, Pi;//Pk用于存储节点位于那一条路径上，也就是哪一个船舶上。Pi用于存储某条路径上的某节点


	//cout << "profit_list.size() = " << profit_list.size() << endl;
	// 4. 移除收益最低的q个任务点
	//cout << "\n正在移除以下" << q << "个收益最低的任务点：" << endl;


	int removedCount = 0;  // 已成功移除的任务数量
	int i = 0;             // profit_list的索引

	while (removedCount < q && i < profit_list.size())
	{
		qid = profit_list[i].first;
		//cout << "qid = " << qid << endl;

		i++;  // 无论是否移除，都移动到下一个候选项

		//使用 std::count 函数检查 qid 是否已经存在于 nSol->nodes_bank 向量中。
		if (std::count(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), qid))
		{
			//cout << "任务 " << qid << " 已经在nodes_bank中，跳过" << endl;
			continue;
		}

		// 检查 qid 是否已经被移除（即是否在 removedCustomers 中）
		if (sol.removedCustomers.find(qid) != sol.removedCustomers.end())
		{
			//cout << "任务 " << qid << " 已经被移除了，跳过" << endl;
			continue;
		}

		// 重置路径和位置变量
		Pk = -1;
		Pi = -1;

		//我们要在这里找到Route.V中所有Number为qid的数据，并且要记录他们在Route.V中的位置
		for (int k = 0; k < Nk; k++)
		{
			for (int j = 0; j < Route_current.Routes[k].V.size(); ++j)
			{
				if (Route_current.Routes[k].V[j].customer_id == qid)
				{
					Pk = k;
					Pi = j;
					break;  // 找到后跳出内层循环
				}
			}
			if (Pk != -1) break;  // 找到后跳出外层循环
		}

		// 检查是否找到了任务
		if (Pk == -1 || Pi == -1)
		{
			//cout << "任务 " << qid << " 不在任何路径中，跳过" << endl;
			continue;
		}

		//从路径中移除任务点
		if (Pk >= 0 && Pk < Route_current.Routes.size())
		{
			auto& route = Route_current.Routes[Pk].V;
			if (Pi >= 0 && Pi < route.size())
			{
				route.erase(route.begin() + Pi);
				//cout << "成功移除任务 " << qid << " 从路径 " << Pk + 1 << endl;

				Route_current = Calculation_removal(Route_current);
				// 更新解的状态
				nSol->SRoute = Route_current;
				nSol->revenue = Route_current.Total_Profit;
				nSol->nodes_bank.push_back(qid);
				nSol->nodes_count--;

				removedCount++;  // 只有成功移除才增加计数
			}
			else
			{
				cout << "任务 " << qid << " 索引越界，跳过" << endl;
			}
		}
		else
		{
			cout << "任务 " << qid << " 路径索引无效，跳过" << endl;
		}
	}

	// 检查是否移除了足够的任务
	if (removedCount < q)
	{
		cout << "警告：只成功移除了 " << removedCount << " 个任务，目标是 " << q << " 个" << endl;
	}
	

 //   // 5. 输出移除后的路径信息
	//cout << "查看移除之后的路径" << endl;
	//for (int k = 1; k <= Nk; k++)
	//{
	//	cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
	//	for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
	//		cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
	//	cout << endl;
	//}
	//cout << "final_Profit = " << Route_current.Total_Profit << endl;

	//cout << "查看已经移除的任务点：" << endl;
	//for (auto it = nSol->nodes_bank.begin(); it != nSol->nodes_bank.end(); it++)
	//{
	//	cout << *it << ",";
	//}


	nSol->revenue = Route_current.Total_Profit;

    cout <<endl << "*********************************** Revenue-Based Removal End ***********************************" << endl;
    return nSol;
}



Solution* removal_time_window(Solution& sol) 
{
	cout << "***********************************   removal_time_window  Start*****************************************" << endl << endl;

	/*cout << "查看初始解的分配" << endl;
	for (int k = 1; k <= Nk; k++) 
	{
		cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
		for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
			cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
		cout << endl;
	}
	cout << "final_Profit = " << Route_current.Total_Profit << endl;*/

	Solution* nSol = new Solution(sol);

	// 计算每个任务的时间窗口宽度
	vector<pair<int, double>> time_windows;
	for (int k = 0; k < Nk; k++) 
	{
		for (int i = 0; i < Route_current.Routes[k].V.size(); i++) 
		{
			int customer_id = Route_current.Routes[k].V[i].customer_id;
			double earliest_time = Route_current.Routes[k].V[i].earlist;
			double latest_time = Route_current.Routes[k].V[i].duedate;
			double window_width = latest_time - earliest_time;
			time_windows.push_back({ customer_id, window_width });
		}
	}

	// 按时间窗口宽度从小到大排序
	sort(time_windows.begin(), time_windows.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
		return a.second < b.second;
		});


	int length = sol.removedCustomers.size();
	int newsize = Nj - length;


	// 确定要移除的任务数量
	int q = 0; // q 代表移除的任务数量
	if (Nj < 20) 
	{
		std::uniform_int_distribution<int> dis1(1, newsize * 1 / 5);
		q = dis1(AlnsRandomEngine());
	}
	else {
		std::uniform_int_distribution<int> dis1(1, newsize * 1 / 8);
		q = dis1(AlnsRandomEngine());
	}

	//cout << "要移除的任务点数量：" << q << endl;

	// 移除时间窗口最紧的任务


	int removedCount = 0;  // 已成功移除的任务数量
	int i = 0;             // profit_list的索引

	while (removedCount < q && i < time_windows.size())
	{
		int qid = time_windows[i].first;

		//cout << "要移除的任务点的标号： " << qid << endl;

		// 检查任务是否已经移除
		if (std::count(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), qid)) {
			++i; // 跳过已移除任务，避免重复检查同一元素导致死循环。
			continue;
		}

		// 检查 qid 是否已经被移除（即是否在 removedCustomers 中）
		if (sol.removedCustomers.find(qid) != sol.removedCustomers.end())
		{
			//cout << "已经被移除了，不能重复移除";
			++i;
			continue;
		}

		// 找到任务所在的路径
		int Pk = -1;
		int Pi = -1;
		for (int k = 0; k < Nk; k++)
		{
			for (int j = 0; j < Route_current.Routes[k].V.size(); ++j)
			{
				if (Route_current.Routes[k].V[j].customer_id == qid)
				{
					Pk = k;
					Pi = j;
					break;
				}
			}
			if (Pk != -1)
				break;
		}

		// 从路径中移除任务
		if (Pk >= 0 && Pk < Route_current.Routes.size())
		{
			auto& route = Route_current.Routes[Pk].V;

			if (Pi >= 0 && Pi < route.size())
			{
				route.erase(route.begin() + Pi);				
				Route_current = Calculation_removal(Route_current);

				nSol->SRoute = Route_current; // 更新 nSol 的 SRoute
				nSol->revenue = Route_current.Total_Profit; // 同步更新 revenue
				nSol->nodes_bank.push_back(qid);
				nSol->nodes_count--;				
				removedCount ++;
			}
			
		}
		else
		{
			cout << "No task with id " << qid << " found in any route" << endl;
		}

		++i;
	}
	
	/*cout << "查看移除之后的路径" << endl;
	for (int k = 1; k <= Nk; k++) 
	{
		cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
		for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
			cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
		cout << endl;
	}
	cout << "final_Profit = " << Route_current.Total_Profit << endl;

	cout << "查看已经移除的任务点：" << endl;
	for (auto it = nSol->nodes_bank.begin(); it != nSol->nodes_bank.end(); it++)
	{
		cout << *it << ",";
	}*/

	//system("pause");
	cout << endl << "***********************************  removal_time_window  End   *****************************************" << endl;
	return nSol;
}



Solution* removal_shaw(Solution& sol)
{
	cout << "***********************************   removal_shaw  Start*****************************************" << endl << endl;

	// 查看初始解的分配

	/*cout << "查看初始解的分配" << endl;
	for (int k = 1; k <= Nk; k++)
	{
		cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
		for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
			cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
		cout << endl;
	}
	cout << "final_Profit = " << Route_current.Total_Profit << endl;*/

	// 创建一个新的 Solution 对象 nSol，并将其初始化为与输入解 sol 相同的状态
	Solution* nSol = new Solution(sol);


	int length = sol.removedCustomers.size();
	int newsize = Nj - length;

	// 确定 q，即要移除的节点数量
	int q = 0; // q 代表移除的 Job 个数
	if (Nj < 20)
	{
		std::uniform_int_distribution<int> dis1(1, newsize * 1 / 5);
		q = dis1(AlnsRandomEngine());
	}
	else
	{
		std::uniform_int_distribution<int> dis1(1, newsize * 1 / 8);
		q = dis1(AlnsRandomEngine());
	}

	//cout << "要移除的任务点数量：" << q << endl;

	int qid;//用于存储要选择的油田点的标识符。
	int Pk, Pi;//Pk用于存储节点位于那一条路径上，也就是哪一个船舶上。Pi用于存储某条路径上的某节点
	Pk = -1;
	Pi = -1;

	// 随机选择种子客户
	std::uniform_int_distribution<int> dis(1, Nj);

	int seed_customer_id;
	do
	{
		seed_customer_id = dis(AlnsRandomEngine());
	} while (sol.removedCustomers.find(seed_customer_id) != sol.removedCustomers.end() ||
		std::find(sol.nodes_bank.begin(), sol.nodes_bank.end(), seed_customer_id) != sol.nodes_bank.end());


	//cout << "Selected seed customer: " << seed_customer_id << endl;

	for (int k = 0; k < Nk; k++)
	{
		for (int j = 0; j < Route_current.Routes[k].V.size(); ++j)
		{
			if (Route_current.Routes[k].V[j].customer_id == seed_customer_id)
			{
				Pi = j;
				Pk = k;
				break;
			}
		}
	}

	// 从路径中移除种子客户
	Route_current.Routes[Pk].V.erase(Route_current.Routes[Pk].V.begin() + Pi);
	//cout << "Removed seed customer " << seed_customer_id << " from Route " << Pk + 1 << endl;

	Route_current = Calculation_removal(Route_current);

	// 更新 nSol
	nSol->SRoute = Route_current;
	nSol->revenue = Route_current.Total_Profit;
	nSol->nodes_count--;
	nSol->nodes_bank.push_back(seed_customer_id);


	// 计算相似度并选择要移除的客户
	vector<pair<int, double>> similarity_scores;
	for (int k = 0; k < Nk; k++)
	{
		if (Route_current.Routes[k].V.size() > 0)
		{
			for (int j = 0; j < Route_current.Routes[k].V.size(); ++j)
			{
				int customer_id = Route_current.Routes[k].V[j].customer_id;
				if (customer_id == seed_customer_id)
					continue;

				// 计算加权函数 R(i, j)

				double distance = sqrt((Customer[seed_customer_id].px - Customer[customer_id].px) * (Customer[seed_customer_id].px - Customer[customer_id].px) + (Customer[seed_customer_id].py - Customer[customer_id].py) * (Customer[seed_customer_id].py - Customer[customer_id].py)); /* 计算种子客户与当前客户的距离 */
				double time_window_diff = abs(Customer[seed_customer_id].duedate - Customer[customer_id].duedate);
				double weight_diff = abs(Customer[seed_customer_id].weight - Customer[customer_id].weight);

				double similarity = 0.0001 * distance + 0.04 * time_window_diff + 0.001 * weight_diff;
				similarity_scores.push_back({ customer_id, similarity });
			}
		}
	}

	// 按相似度排序
	sort(similarity_scores.begin(), similarity_scores.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
		return a.second < b.second;
		});

	// 选择相似度最高的 q-1 个客户
	vector<int> to_remove;
	for (int i = 0; i < q - 1; i++)
	{
		// 计算相似度并选择要移除的客户
		vector<pair<int, double>> similarity_scores;
		for (int k = 0; k < Nk; k++)
		{
			if (Route_current.Routes[k].V.size() > 0)
			{
				for (int j = 0; j < Route_current.Routes[k].V.size(); ++j)
				{
					int customer_id = Route_current.Routes[k].V[j].customer_id;
					if (customer_id == seed_customer_id)
						continue;

					// 计算加权函数 R(i, j)

					double distance = sqrt((Customer[seed_customer_id].px - Customer[customer_id].px) * (Customer[seed_customer_id].px - Customer[customer_id].px) + (Customer[seed_customer_id].py - Customer[customer_id].py) * (Customer[seed_customer_id].py - Customer[customer_id].py)); /* 计算种子客户与当前客户的距离 */
					double time_window_diff = abs(Customer[seed_customer_id].duedate - Customer[customer_id].duedate);
					double weight_diff = abs(Customer[seed_customer_id].weight - Customer[customer_id].weight);

					double similarity = 0.0001 * distance + 0.04 * time_window_diff + 0.001 * weight_diff;
					similarity_scores.push_back({ customer_id, similarity });
				}
			}
		}


		// 按相似度排序
		sort(similarity_scores.begin(), similarity_scores.end(), [](const pair<int, double>& a, const pair<int, double>& b)
			{
				return a.second < b.second;
			});


		if (similarity_scores.empty()) 
			break;

		// 选择相似度最高的客户
		to_remove.push_back(similarity_scores[0].first);

		// 方案1：检查是否在nodes_bank中，如果在，再检查是否在removedCustomers中
		if (std::count(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), similarity_scores[0].first) > 0)
		{
			if (nSol->removedCustomers.find(similarity_scores[0].first) != nSol->removedCustomers.end())
			{
				cout << "oooooooooooooo" << endl;
			}
		}
		else
		{
			// 从路径中移除选中的客户
			for (int k = 0; k < Nk; k++)
			{
				if (Route_current.Routes[k].V.size() > 0)
				{
					for (int j = 0; j < Route_current.Routes[k].V.size(); ++j)
					{
						if (Route_current.Routes[k].V[j].customer_id == similarity_scores[0].first)
						{
							Route_current.Routes[k].V.erase(Route_current.Routes[k].V.begin() + j);
						}
					}
				}
			}

			Route_current = Calculation_removal(Route_current);

			// 更新 nSol
			nSol->SRoute = Route_current;
			nSol->revenue = Route_current.Total_Profit;
			nSol->nodes_count--;
			nSol->nodes_bank.push_back(similarity_scores[0].first);
		}

		
	}


	Route_current = Calculation_removal(Route_current);

	//// 查看移除之后的路径
	//cout << "查看移除之后的路径" << endl;
	//for (int k = 1; k <= Nk; k++) {
	//	cout << "第" << k << "条路径上的收益" << Route_current.Routes[k - 1].Profit << endl;
	//	for (int i = 0; i < Route_current.Routes[k - 1].V.size(); i++)
	//		cout << Route_current.Routes[k - 1].V[i].customer_id << "->";
	//	cout << endl;
	//}
	//cout << "final_Profit = " << Route_current.Total_Profit << endl;

	//cout << "查看已经移除的任务点：" << endl;
	//for (auto it = nSol->nodes_bank.begin(); it != nSol->nodes_bank.end(); it++)
	//{
	//	cout << *it << ",";
	//}

	cout << endl << "***********************************  removal_shaw  End   *****************************************" << endl;

	return nSol;
}


// ===================== 多船时间压力协同移除 =====================
// 与普通 Shaw removal 仅根据客户相似度不同，本算子先识别“高压力船舶”和
// “低压力接收船舶”，再同时打开两条路线，为后续的跨船重分配创造空间。
Solution* removal_inter_vessel_pressure(Solution& sol)
{
	Solution* nSol = new Solution(sol);
	Route_current = sol.SRoute;

	if (Nk <= 0 || Route_current.Routes.empty())
		return nSol;

	// 所有压力分量均转换为无量纲指标，避免时间单位由小时改为分钟时
	// 改变路线排序或任务移除顺序。
	const double safe_horizon = max(MY_EPSILON, static_cast<double>(T));
	const double safe_endurance = max(MY_EPSILON, static_cast<double>(a));
	double maximum_task_weight = MY_EPSILON;
	for (int i = 1; i <= Nj; ++i)
		maximum_task_weight = max(maximum_task_weight, static_cast<double>(Customer[i].weight));
	auto normalized_window_tightness = [&](const Customer_Type& customer)
	{
		const double window_width = max(0.0, customer.duedate - customer.earlist);
		return 1.0 - min(1.0, window_width / safe_horizon);
	};

	vector<double> pressure(Nk, 0.0);
	int scheduled = 0;
	for (int k = 0; k < Nk; ++k)
	{
		double service_load = 0.0;
		double tightness = 0.0;
		double maximum_profit = 0.0;
		for (const auto& customer : Route_current.Routes[k].V)
		{
			service_load += customer.service_time;
			tightness += normalized_window_tightness(customer);
			maximum_profit += Profit_cost * customer.weight;
			++scheduled;
		}

		const double lost_profit_ratio = maximum_profit > MY_EPSILON
			? max(0.0, maximum_profit - Route_current.Routes[k].Profit) / maximum_profit : 0.0;
		const double average_tightness = Route_current.Routes[k].V.empty()
			? 0.0 : tightness / Route_current.Routes[k].V.size();
		pressure[k] = service_load / safe_horizon + lost_profit_ratio + 0.10 * average_tightness;
	}

	if (scheduled == 0)
		return nSol;

	int high_pressure_route = static_cast<int>(max_element(pressure.begin(), pressure.end()) - pressure.begin());
	int receiving_route = -1;
	if (Nk > 1)
	{
		double lowest = numeric_limits<double>::infinity();
		for (int k = 0; k < Nk; ++k)
		{
			// 空路线没有可供交换的任务，不作为低压力伙伴路线。
			if (k != high_pressure_route && !Route_current.Routes[k].V.empty()
				&& pressure[k] < lowest)
			{
				lowest = pressure[k];
				receiving_route = k;
			}
		}
	}

	struct Candidate
	{
		int customer_id;
		double score;
	};
	vector<Candidate> high_pressure_candidates;
	vector<Candidate> low_pressure_candidates;
	auto collect_candidates = [&](int route_index, vector<Candidate>& candidates)
	{
		for (const auto& customer : Route_current.Routes[route_index].V)
		{
			const double normalized_weight = customer.weight / maximum_task_weight;
			const double window_tightness = normalized_window_tightness(customer);
			const double endurance_share = customer.service_time / safe_endurance;
			// 路线已在第一阶段选定；第二阶段仅按无量纲任务属性进行路线内排序。
			const double score = normalized_weight + window_tightness + endurance_share;
			candidates.push_back({ customer.customer_id, score });
		}
	};
	collect_candidates(high_pressure_route, high_pressure_candidates);
	if (receiving_route >= 0)
		collect_candidates(receiving_route, low_pressure_candidates);

	auto rank_candidates = [](const Candidate& lhs, const Candidate& rhs)
	{
		return lhs.score > rhs.score;
	};
	sort(high_pressure_candidates.begin(), high_pressure_candidates.end(), rank_candidates);
	sort(low_pressure_candidates.begin(), low_pressure_candidates.end(), rank_candidates);

	const int candidate_count = static_cast<int>(high_pressure_candidates.size()
		+ low_pressure_candidates.size());
	// 船舶数越多，分配空间越大，适度提高破坏比例以便跨船搜索。
	const double destroy_ratio = 0.08 + 0.02 * min(4, max(0, Nk - 1));
	const int q = min(candidate_count,
		max(1, static_cast<int>(ceil(scheduled * destroy_ratio))));

	// 当存在非空伙伴路线且 q >= 2 时，强制两条路线都贡献任务。
	// eta=0.70 将更多破坏预算用于高压力路线，同时为双向交换保留空间。
	int q_high = q;
	int q_low = 0;
	if (!low_pressure_candidates.empty() && q >= 2)
	{
		const double eta = 0.70;
		q_high = min(q - 1, max(1, static_cast<int>(ceil(eta * q))));
		q_low = q - q_high;
	}

	q_high = min(q_high, static_cast<int>(high_pressure_candidates.size()));
	q_low = min(q_low, static_cast<int>(low_pressure_candidates.size()));
	int unallocated = q - q_high - q_low;
	if (unallocated > 0)
	{
		const int extra_high = min(unallocated,
			static_cast<int>(high_pressure_candidates.size()) - q_high);
		q_high += extra_high;
		unallocated -= extra_high;
	}
	if (unallocated > 0)
	{
		const int extra_low = min(unallocated,
			static_cast<int>(low_pressure_candidates.size()) - q_low);
		q_low += extra_low;
	}
	const char* trace_pressure = getenv("ALNS_TRACE_PRESSURE_REMOVAL");
	if (trace_pressure != nullptr && string(trace_pressure) == "1")
	{
		cout << "Pressure removal: high_route=" << high_pressure_route
			<< ", low_route=" << receiving_route
			<< ", rho_high=" << pressure[high_pressure_route]
			<< ", rho_low=" << (receiving_route >= 0 ? pressure[receiving_route] : -1.0)
			<< ", q_high=" << q_high << ", q_low=" << q_low << endl;
	}

	set<int> removal_ids;
	for (int i = 0; i < q_high; ++i)
		removal_ids.insert(high_pressure_candidates[i].customer_id);
	for (int i = 0; i < q_low; ++i)
		removal_ids.insert(low_pressure_candidates[i].customer_id);

	for (auto& route : Route_current.Routes)
	{
		route.V.erase(remove_if(route.V.begin(), route.V.end(), [&](const Customer_Type& customer)
		{
			return removal_ids.find(customer.customer_id) != removal_ids.end();
		}), route.V.end());
	}

	for (int customer_id : removal_ids)
		nSol->nodes_bank.push_back(customer_id);

	Route_current = Calculation_removal(Route_current);
	nSol->SRoute = Route_current;
	nSol->revenue = Route_current.Total_Profit;
	nSol->nodes_count = sol.nodes_count - static_cast<int>(removal_ids.size());
	return nSol;
}


// ===================== 时间收益外部性移除 =====================
// 普通 worst removal 关注任务自身贡献；本算子进一步度量任务 i 对同一路线后续
// 任务的时间相关收益挤压。若删除 i 后其余任务恢复的收益较多，说明 i 具有较高
// 的时间收益外部性，应优先释放并交由修复算子重新分配。
Solution* removal_temporal_profit_externality(Solution& sol)
{
	Solution* nSol = new Solution(sol);
	Route_current = Calculation_removal(sol.SRoute);

	struct Candidate
	{
		int customer_id;
		double score;
	};
	struct ScreeningCandidate
	{
		int route_index;
		int position;
		double quick_score;
	};
	vector<Candidate> candidates;
	vector<ScreeningCandidate> screening_candidates;
	int scheduled = 0;
	double penalized_ratio_sum = 0.0;

	// 第一阶段只读取当前 SOCP 解中的任务收益和续航占用，筛出最可能具有
	// 时间外部性的任务；第二阶段才进行 leave-one-out 精确路线评价。
	for (int k = 0; k < static_cast<int>(Route_current.Routes.size()); ++k)
	{
		const Route_Type& base_route = Route_current.Routes[k];
		for (int position = 0; position < static_cast<int>(base_route.V.size()); ++position)
		{
			const Customer_Type& customer = base_route.V[position];
			const double maximum_profit = max(MY_EPSILON, Profit_cost * customer.weight);
			const double task_profit = position < static_cast<int>(base_route.task_profits.size())
				? base_route.task_profits[position] : maximum_profit;
			const double own_profit_loss = max(0.0, maximum_profit - task_profit);
			double endurance_load = customer.service_time / max(MY_EPSILON, a);
			if (position < static_cast<int>(base_route.outbound_times.size()) &&
				position < static_cast<int>(base_route.return_times.size()))
			{
				endurance_load = (base_route.outbound_times[position]
					+ base_route.return_times[position] + customer.service_time) / max(MY_EPSILON, a);
			}
			const double quick_score = own_profit_loss
				+ 0.20 * maximum_profit * min(1.5, endurance_load);
			screening_candidates.push_back({ k, position, quick_score });
			penalized_ratio_sum += own_profit_loss / maximum_profit;
			++scheduled;
		}
	}

	if (screening_candidates.empty())
		return nSol;
	const double average_penalized_ratio = penalized_ratio_sum / max(1, scheduled);
	const double destroy_ratio = min(0.24, 0.08 + 0.12 * average_penalized_ratio);
	const int q = max(1, static_cast<int>(ceil(scheduled * destroy_ratio)));
	sort(screening_candidates.begin(), screening_candidates.end(),
		[](const ScreeningCandidate& lhs, const ScreeningCandidate& rhs)
	{
		return lhs.quick_score > rhs.quick_score;
	});
	// 精确外部性评价预算随问题规模次线性增长，同时保证不少于最终移除数量。
	const int evaluation_budget = min(static_cast<int>(screening_candidates.size()),
		max(q, static_cast<int>(ceil(2.0 * sqrt(static_cast<double>(scheduled))))));

	for (int index = 0; index < evaluation_budget; ++index)
	{
		const ScreeningCandidate& screened = screening_candidates[index];
		const Route_Type& base_route = Route_current.Routes[screened.route_index];
		const Customer_Type& customer = base_route.V[screened.position];
		const double maximum_profit = max(MY_EPSILON, Profit_cost * customer.weight);
		const double task_profit = screened.position < static_cast<int>(base_route.task_profits.size())
			? base_route.task_profits[screened.position] : maximum_profit;

		Route_Type route_without = base_route;
		route_without.V.erase(route_without.V.begin() + screened.position);
		bool feasible = false;
		const double remaining_profit = EvaluateRouteCached(route_without, feasible);
		if (!feasible)
			continue;

		// externality > 0 表示删除该任务后，除该任务以外的路线收益得到恢复。
		const double externality = remaining_profit - (base_route.Profit - task_profit);
		std::uniform_real_distribution<double> jitter(0.0, 0.01 * maximum_profit);
		const double score = externality + 0.35 * max(0.0, maximum_profit - task_profit)
			+ 0.05 * screened.quick_score + jitter(AlnsRandomEngine());
		candidates.push_back({ customer.customer_id, score });
	}
	if (candidates.empty())
		return nSol;

	sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs)
	{
		return lhs.score > rhs.score;
	});

	set<int> removal_ids;
	for (int i = 0; i < min(q, static_cast<int>(candidates.size())); ++i)
		removal_ids.insert(candidates[i].customer_id);

	for (auto& route : Route_current.Routes)
	{
		route.V.erase(remove_if(route.V.begin(), route.V.end(), [&](const Customer_Type& customer)
		{
			return removal_ids.find(customer.customer_id) != removal_ids.end();
		}), route.V.end());
	}
	for (int customer_id : removal_ids)
	{
		if (find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), customer_id) == nSol->nodes_bank.end())
			nSol->nodes_bank.push_back(customer_id);
	}

	Route_current = Calculation_removal(Route_current);
	nSol->SRoute = Route_current;
	nSol->revenue = Route_current.Total_Profit;
	nSol->nodes_count = max(0, sol.nodes_count - static_cast<int>(removal_ids.size()));
	return nSol;
}


// ===================== 船-UAV 同步临界字符串移除 =====================
// 一个任务的改变会联动前序回收点、后序起飞点和下游到达时间。因此本算子不是
// 随机删除相邻节点，而是从续航裕度小、实际收益受损且下游传播风险高的任务出发，
// 释放一个连续同步片段，让修复阶段共同重构该片段的访问顺序和船舶归属。
Solution* removal_synchronization_critical_string(Solution& sol)
{
	Solution* nSol = new Solution(sol);
	Route_current = Calculation_removal(sol.SRoute);

	int scheduled = 0;
	int seed_route = -1;
	int seed_position = -1;
	double best_criticality = -numeric_limits<double>::infinity();
	double criticality_sum = 0.0;
	vector<vector<double>> criticality(Route_current.Routes.size());

	for (int k = 0; k < static_cast<int>(Route_current.Routes.size()); ++k)
	{
		const Route_Type& route = Route_current.Routes[k];
		criticality[k].resize(route.V.size(), 0.0);
		for (int i = 0; i < static_cast<int>(route.V.size()); ++i)
		{
			const Customer_Type& customer = route.V[i];
			const double maximum_profit = max(MY_EPSILON, Profit_cost * customer.weight);
			const double actual_profit = i < static_cast<int>(route.task_profits.size())
				? route.task_profits[i] : maximum_profit;
			double endurance_load = customer.service_time / max(MY_EPSILON, a);
			if (i < static_cast<int>(route.outbound_times.size()) &&
				i < static_cast<int>(route.return_times.size()))
			{
				endurance_load = (route.outbound_times[i] + route.return_times[i]
					+ customer.service_time) / max(MY_EPSILON, a);
			}

			int degraded_downstream = 0;
			for (int j = i + 1; j < static_cast<int>(route.V.size()); ++j)
			{
				const double downstream_max = max(MY_EPSILON, Profit_cost * route.V[j].weight);
				if (j < static_cast<int>(route.task_profits.size()) &&
					route.task_profits[j] < downstream_max - MY_EPSILON)
					++degraded_downstream;
			}
			const double downstream_ratio = route.V.size() - i - 1 > 0
				? static_cast<double>(degraded_downstream) / (route.V.size() - i - 1) : 0.0;
			const double profit_loss_ratio = max(0.0, maximum_profit - actual_profit) / maximum_profit;
			const double value = 0.50 * min(1.5, endurance_load)
				+ 0.35 * profit_loss_ratio + 0.15 * downstream_ratio;
			criticality[k][i] = value;
			criticality_sum += value;
			++scheduled;
			if (value > best_criticality)
			{
				best_criticality = value;
				seed_route = k;
				seed_position = i;
			}
		}
	}

	if (scheduled == 0 || seed_route < 0)
		return nSol;

	const double average_criticality = criticality_sum / scheduled;
	const double destroy_ratio = min(0.25, 0.07 + 0.10 * average_criticality);
	const int route_size = static_cast<int>(Route_current.Routes[seed_route].V.size());
	const int target = min(route_size, max(1, static_cast<int>(ceil(scheduled * destroy_ratio))));

	// 从临界种子向两侧扩展，每次纳入相邻临界度较高的一侧，确保删除集合连续。
	int left = seed_position;
	int right = seed_position;
	while (right - left + 1 < target)
	{
		const double left_value = left > 0 ? criticality[seed_route][left - 1]
			: -numeric_limits<double>::infinity();
		const double right_value = right + 1 < route_size ? criticality[seed_route][right + 1]
			: -numeric_limits<double>::infinity();
		if (left_value >= right_value && left > 0)
			--left;
		else if (right + 1 < route_size)
			++right;
		else
			break;
	}

	set<int> removal_ids;
	for (int i = left; i <= right; ++i)
		removal_ids.insert(Route_current.Routes[seed_route].V[i].customer_id);
	Route_current.Routes[seed_route].V.erase(
		Route_current.Routes[seed_route].V.begin() + left,
		Route_current.Routes[seed_route].V.begin() + right + 1);

	for (int customer_id : removal_ids)
	{
		if (find(nSol->nodes_bank.begin(), nSol->nodes_bank.end(), customer_id) == nSol->nodes_bank.end())
			nSol->nodes_bank.push_back(customer_id);
	}
	Route_current = Calculation_removal(Route_current);
	nSol->SRoute = Route_current;
	nSol->revenue = Route_current.Total_Profit;
	nSol->nodes_count = max(0, sol.nodes_count - static_cast<int>(removal_ids.size()));
	return nSol;
}
