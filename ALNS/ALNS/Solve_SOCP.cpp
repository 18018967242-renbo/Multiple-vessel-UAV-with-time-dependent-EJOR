#include "Solve_SOCP.h"

#include <ilcplex/ilocplex.h>
#include <ilcplex/ilocplexi.h>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <iomanip>
#include <string>
#include <math.h>
#include <algorithm> 
#include <set>
#include <vector>
#include <windows.h>
#include <tchar.h>
#include <shellapi.h>
#define largeNum  100000

using namespace std;

typedef std::vector<int> v1d;
typedef std::vector<v1d> v2d;
typedef std::vector<v2d> v3d;

typedef std::vector<double> v1do;   //一维向量
typedef std::vector<v1do> v2do;	//二维向量
typedef std::vector<v2do> v3do;	//三维向量



Result cplex_solve(IloInt size, v1d& sortpoint, Route_Type Route, IloInt vv, IloInt vd, IloNum R, IloNum Profit_cost, IloNum Delta_1, IloNum Delta_2, IloNum T, IloNum profit_early, IloNum profit_late)
{

	IloEnv env_cpx;


	//获取每个点的坐标位置
	IloInt Nr_cpx = sortpoint.size();

	/*cout << endl << "1111111111[" << endl;
	for (int i = 0; i < sortpoint.size(); i++)
	{
		cout << "sortpoint[i] = " << sortpoint[i] << "," << Customer[sortpoint[i]].px << ", " << Customer[sortpoint[i]].py << endl;
	}
	cout << "],";


	cout << "9999999999" << endl;
	for (int i = 0; i < Route.V.size(); i++)
	{
		cout << "D_cpx[i] = " << Route.V[i].customer_id << endl;
	}
	cout << endl << endl;*/

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

	/*cout << "px_socp = " << endl;
	for (int i = 1; i <= size; i++)
	{
		cout << px_socp[i] << endl;
	}
	cout << endl;

	cout << "py_socp = " << endl;
	for (int i = 1; i <= size; i++)
	{
		cout << py_socp[i] << endl;
	}
	cout << endl;

	cout << "service_time_socp = " << endl;
	for (int i = 1; i <= size; i++)
	{
		cout << service_time_socp[i] << endl;
	}
	cout << endl;

	cout << "earlist_socp = " << endl;
	for (int i = 1; i <= size; i++)
	{
		cout << earlist_socp[i] << endl;
	}
	cout << endl;

	cout << "duedate_socp = " << endl;
	for (int i = 1; i <= size; i++)
	{
		cout << duedate_socp[i] << endl;
	}
	cout << endl;

	cout << "weight_socp = " << endl;
	for (int i = 1; i <= size; i++)
	{
		cout << weight_socp[i] << endl;
	}
	cout << endl;*/



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
	model_sp.add(Gammak == temp1);
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
		model_sp.add(t1[i] + t2[i] + service_time_socp[i] <= R);
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
		model_sp.add(z11_sp[i] * z11_sp[i] <= (R * R * vv * vv) / vd * vd);
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
	//mycplex_sp.setParam(IloCplex::NodeFileInd, 3);
	mycplex_sp.setParam(IloCplex::Param::MIP::Limits::TreeMemory, 2048);


	Result result_sp;
	result_sp.success = false;

	if (mycplex_sp.solve())
	{
		//cout << "成功求解mycplex_sp" << endl;
		result_sp.success = true;
		IloNum value_LB_sp = mycplex_sp.getBestObjValue();
		IloNum value_obj_sp = mycplex_sp.getObjValue();
		double gap = mycplex_sp.getMIPRelativeGap();
		result_sp.obj = value_obj_sp;
		result_sp.seqence = sortpoint;

		// 将固定路线下的最优连续状态回传给 ALNS。后续算子据此识别
		// 时间收益外部性和续航临界片段，但不人工修改 SOCP 的起降位置。
		for (int i = 1; i <= Nr_cpx; ++i)
		{
			result_sp.arrival_times.push_back(mycplex_sp.getValue(arrival_time_sp[i]));
			result_sp.task_profits.push_back(
				mycplex_sp.getValue(total_profit11[i]) * weight_socp[i]);
			result_sp.outbound_times.push_back(mycplex_sp.getValue(t1[i]));
			result_sp.return_times.push_back(mycplex_sp.getValue(t2[i]));
			result_sp.vessel_sync_times.push_back(mycplex_sp.getValue(t_total[i]));
			result_sp.takeoff_x.push_back(mycplex_sp.getValue(Ptx[i]));
			result_sp.takeoff_y.push_back(mycplex_sp.getValue(Pty[i]));
			result_sp.landing_x.push_back(mycplex_sp.getValue(Plx[i]));
			result_sp.landing_y.push_back(mycplex_sp.getValue(Ply[i]));
		}

		cout << "SOCP求解所得obj_sp=" << value_obj_sp << endl;	
	}
	else
	{
		result_sp.success = false;
		cout << "没有成功求解mycplex_sp" << endl;
		
	}

	// 每次固定路线求解都创建独立的 CPLEX 环境，返回前必须释放。
	mycplex_sp.end();
	env_cpx.end();
	return result_sp;
}
