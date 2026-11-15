#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <vector>
#include <list>
#include <ilcplex/ilocplex.h>
#include <ilcplex/ilocplexi.h>
#include <unordered_set>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <math.h>
#include <time.h>
#include <algorithm>
#include <random>
#include <set> // 包含 set 头文件
#include <unordered_map>
#include <sstream>
#define MY_EPSILON 0.00001 // Precision parameter, useful to avoid numerical instabilities
#define snprintf _snprintf
#define INF 0x3ffffff   //无穷大
#define smallNum 0.00001
using namespace std;
ILOSTLBEGIN
typedef IloArray<IloIntArray>    IntMatrix;
typedef IloArray<IntMatrix>      IntMatrix3d;
typedef IloArray<IntMatrix3d>    IntMatrix4d;
typedef IloArray<IloNumArray>    NumMatrix;
typedef IloArray<NumMatrix>      NumMatrix3d;
typedef IloArray<NumMatrix3d>    NumMatrix4d;
typedef IloArray<NumMatrix4d>    NumMatrix5d;
typedef IloArray<IloIntVarArray> IntVarMatrix;
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IntVarMatrix>   IntVarMatrix3d;
typedef IloArray<NumVarMatrix>   NumVarMatrix3d;
typedef IloArray<IntVarMatrix3d> IntVarMatrix4d;
typedef IloArray<NumVarMatrix3d> NumVarMatrix4d;
typedef IloArray<NumVarMatrix4d> NumVarMatrix5d;

__declspec(selectany) ofstream Resultfile("..\\results\\alns_result.txt", ios::app);
__declspec(selectany) ofstream Resultfile2("..\\ALNS_Result\\调diedai_data.txt", ios::app);
__declspec(selectany) ofstream Resultfile3("..\\results\\调xiangxijieguo.txt", ios::app);
/*********************************参数定义****************************************/
extern int Nj;				//目标点的个数
extern int Nk;				//无人机个数
extern int T;				//周期
extern int Profit_cost;		//单个任务点的最大收益
extern int vd;				//无人机速度
extern int vv;				//货运船速度
extern double a;			//无人机续航时间
extern double Delta_1;		//允许访问任务点提前的时间
extern double Delta_2;		//允许访问任务点延迟的时间
extern int profit_early;	//提前访问时最小的收益
extern int profit_late;		//延迟访问时最小的收益
extern int Customer_Number;


/*********************************算法参数定义****************************************/
extern int cpu_time;
//extern int seed;
#define score1 1.5//根据解的情况更新分数
#define score2 1.0
#define score3 0.5
#define rho 0.1//权重调整反应因子
#define time_segment 10//每隔多少次更新权重
#define jw 0.999//降温
extern float Initial_W; //降温初始温度
extern set<int> removedCustomers;		//用于存储初始解过程中移除的任务点


struct Customer_Type	//定义任务点结构体
{		
	int customer_id;    //节点自身编号
	int vessel;			//节点所属巡逻船路径编号
	double px, py;   //节点横纵坐标
	double service_time;      //节点服务时间
	double earlist, duedate;   //节点最早服务时间和最晚服务时间
	double weight;			//节点的权重系数
	double single_profit;		//这个是用来记录每个任务点的收益的，辅助基于收益的任务点移除算子
};   


//这个是当前的解
struct Route_Type 
{
	double Profit;   //单条路径总长度
	vector<Customer_Type> V;   //单条路径上顾客节点序列

	// ===================== SOCP 路线诊断信息 =====================
	// 这些量不参与路线编码，也不改变连续子问题；它们仅把固定序列下的最优
	// 到达、收益与无人机飞行状态反馈给 ALNS，用于问题特定算子的选择。
	vector<double> arrival_times;       // 每个任务的最优到达时间
	vector<double> task_profits;        // 每个任务对目标函数的加权实际收益
	vector<double> outbound_times;      // UAV 起飞点到任务点的飞行时间 t1
	vector<double> return_times;        // UAV 任务点到降落点的飞行时间 t2
	vector<double> vessel_sync_times;   // UAV 执行任务期间船舶航行时间
	vector<double> takeoff_x;
	vector<double> takeoff_y;
	vector<double> landing_x;
	vector<double> landing_y;
};   //车辆路径及搜索到最优的车辆路径



struct Customer_schedule 
{
	vector<Route_Type> Routes; // 多条路径集合
	double Total_Profit;       // 所有路径总利润
};




//__declspec(selectany)，确保在多个源文件中包含这些声明时，只有一个源文件实际分配内存并初始化这些全局变量。这有助于避免重复定义和冲突问题
__declspec(selectany) struct Customer_Type Customer[110];   //原点编号为1，油田点编号为2-9
__declspec(selectany) struct Route_Type Route[110];   //原点编号为1，油田点编号为2-9


__declspec(selectany) struct Customer_schedule Route_current, Route_best;   //车辆路径及搜索到最优的车辆路径



double Calculation(Customer_schedule sche);

Customer_schedule Calculation_removal(Customer_schedule sche);

// ===================== 分解感知的增量评价器 =====================
// 固定一艘船的任务顺序后，UAV 起降点优化是独立的锥优化子问题。
// 因此以“任务序列”为键缓存子问题结果，避免每次插入都重算未改变的船舶。
struct EvaluationStats
{
	long long schedule_calls = 0;       // 完整方案评价次数
	long long conic_solves = 0;         // 真正调用 CPLEX 锥子问题的次数
	long long cache_hits = 0;           // 船舶路线缓存命中次数
	long long fast_rejections = 0;      // 被安全必要条件直接判为不可行的次数
};

void ResetEvaluationCache();
EvaluationStats GetEvaluationStats();
double EvaluateRouteCached(Route_Type& route, bool& feasible);

// 可复现随机数源：消除在每个算子内重复构造 random_device 造成的不可复现性。
std::mt19937& AlnsRandomEngine();
void SetAlnsRandomSeed(unsigned int seed);


class Params
{
public:
	// Constructor
	Params();//声明了一个构造函数 Params()，但没有提供函数体，因此这是一个空构造函数。构造函数通常用于初始化对象的成员变量或执行其他必要的初始化工作。
};
#endif
