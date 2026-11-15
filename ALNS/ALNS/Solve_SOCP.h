#ifndef SOLVE_SOCP_H
#define SOLVE_SOCP_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <ilcplex/ilocplex.h>
#include <algorithm>
#include <vector>
#include <set>
#include <atomic>
#include <mutex>
#include <ilcplex/ilocplexi.h>
#include "Params.h"
using namespace std;

typedef IloArray<IloNumArray>    NumMatrix;   // 二维整型参数
typedef IloArray<IloNumArray>    NumMatrix;   // 二维浮点参数
typedef IloArray<NumMatrix>      NumMatrix3d; // 三维浮点参数
typedef IloArray<NumMatrix3d>    NumMatrix4d;

typedef IloArray<IloNumVarArray> NumVarMatrix;   // 二维整型变量
typedef IloArray<IloNumVarArray> NumVarMatrix;   // 二维浮点变量
typedef IloArray<NumVarMatrix>   NumVarMatrix3d; // 三维整型变量
typedef IloArray<NumVarMatrix3d>   NumVarMatrix4d; // 三维浮点变量

typedef std::vector<int> v1d;
typedef std::vector<v1d> v2d;
typedef std::vector<v2d> v3d;

typedef std::vector<double> v1do;   //一维向量
typedef std::vector<v1do> v2do;	//二维向量
typedef std::vector<v2do> v3do;	//三维向量
struct Result
{
	bool success;		//子问题是否求解成功
	double obj;			//子问题的值
	vector<int> seqence;//节点排序
	vector<vector<double>> p; //位置
	vector <vector<double>> t;
	vector<double> T;		//无人机起飞后船舶的航行的时间

	// 面向 ALNS 专属算子的显式诊断量。固定路线仍由原 SOCP 精确优化，
	// 这里只回传最优解状态，不改变模型、约束或目标函数。
	vector<double> arrival_times;
	vector<double> task_profits;
	vector<double> outbound_times;
	vector<double> return_times;
	vector<double> vessel_sync_times;
	vector<double> takeoff_x;
	vector<double> takeoff_y;
	vector<double> landing_x;
	vector<double> landing_y;
};

Result cplex_solve(IloInt Nj, v1d& sortpoint, Route_Type Route, IloInt vv, IloInt vd, IloNum R, IloNum Profit_cost, IloNum Delta_1, IloNum Delta_2, IloNum T, IloNum profit_early, IloNum profit_late);


#endif
