"""Multi-vessel/UAV patrol model: CPLEX C++ -> Gurobi Python.

对应的 C++ 源文件：
    Multi-Vessel-UAV patrol(cplex)/Vessel-UAV patrol together/model.cpp

索引约定与 C++ 完全一致：
    i, j = 0, ..., Nj，其中 0 是港口，1..Nj 是任务点；
    k = 1, ..., Nk，表示船舶编号。

代码中 C03、C04 ... 的编号对应 C++ 源码注释中的公式编号。
只对 C++ 中两个明显的实现问题做了必要的等价处理：
1. C++ 的 big_MM 未初始化就使用；Python 从 0 开始求和，避免未定义行为。
2. C++ 的部分有效不等式误写为 k < Nk；Python 对所有 k=1..Nk 添加。

Gurobi 与 CPLEX 的主要 API 对应：
    IloNumVar / IloBoolVar  -> Model.addVar(s)
    model.add(...)          -> Model.addConstr / Model.addQConstr
    IloObjective::Maximize  -> Model.setObjective(..., GRB.MAXIMIZE)
    mycplex.solve()         -> Model.optimize()
"""

import csv
import math
import os
import re

import gurobipy as gp
import matplotlib.pyplot as plt
from gurobipy import GRB


SMALL_NUM = 1e-5


# ===================== 基础模型实验设置 =====================
# 本轮实验不添加任何有效不等式
# 约束建模代码仍保留在下方，但由于对应开关全为 False，
# Python 运行时不会将它们加入 Gurobi 模型。以后只需把某项改为 True 即可恢复。
USE_CYCLE = False             # 注释/关闭有效不等式 (44)
USE_PROFIT_CAP = False        # 同时关闭有效不等式区块中的单任务收益上界
USE_DEPOT_OUT = False         # 注释/关闭有效不等式 (45)
USE_DEPOT_IN = False          # 注释/关闭有效不等式 (46)
USE_INTER_TASK = False        # 注释/关闭有效不等式 (47)
USE_TIME_TRIANGLE_1 = True   # 注释/关闭有效不等式 (48)
USE_TIME_TRIANGLE_2 = True   # 注释/关闭有效不等式 (49)
USE_TIME_DIFFERENCE = True   # 注释/关闭有效不等式 (54)

# Give each experiment a different name, for example: baseline, only_cycle, all.
EXPERIMENT_LABEL = "baseline_no_valid_equalities"
# False 表示本轮结果直接放入 RESULT_ROOT，不再套一层实验名子目录。
USE_EXPERIMENT_SUBDIRECTORY = False

# Instance range: Example{num1}_{num2}_{num3}.dat
NUM1_START = 13
NUM1_END = 16
NUM1_STEP = 1
NUM2_START = 2
NUM2_END = 8
NUM2_STEP = 2

NUM3_START = 1
NUM3_END = 2
NUM3_STEP = 1

DATA_DIR = r"F:\Projects\Multi Vessel UAV\Multi-Vessel-UAV patrol(cplex)\Data(4.4)"
# 结果始终写入本 Python 文件同级的指定目录，避免 PyCharm/命令行
# 的工作目录不同时，结果被写到其他地方。
RESULT_ROOT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "gurobi_results_base_model+48,49,54",
)
TIME_LIMIT = 3600
THREADS = 1
RANDOM_SEED = 0
# 批量基准测试不导出体积较大的 LP 文件；需调试模型时可改为 True。
WRITE_MODEL_LP = False


def read_data(filename):
    """读取 C++ ``readParameter`` + ``readData`` 使用的 .dat 格式。

    前 11 个等号后是标量参数，之后 6 组分别是 px、py、
    service_time、earlist、duedate 和 weight。每组应有 Nj+1 个数，
    第 0 个数对应港口。
    """
    rows = []
    current_row = None

    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    for line in lines:
        if "=" in line:
            current_row = []
            rows.append(current_row)
            line = line.split("=", 1)[1]

        if current_row is None:
            continue

        nums = re.findall(r"[-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?", line)
        if nums:
            current_row.extend(float(x) for x in nums)

    if len(rows) < 17:
        raise ValueError(
            f"{filename}: 需要 11 个标量参数和 6 个数组，"
            f"实际只读到 {len(rows)} 组数据。"
        )

    Nj = int(rows[0][0])
    Nk = int(rows[1][0])
    n = Nj + 1

    array_names = ("px", "py", "service_time", "earlist", "duedate", "weight")
    for name, values in zip(array_names, rows[11:17]):
        if len(values) < n:
            raise ValueError(
                f"{filename}: {name} 应有 {n} 个元素，实际只有 {len(values)} 个。"
            )

    data = {
        "Nj": Nj,
        "Nk": Nk,
        "T": rows[2][0],
        "Profit_cost": rows[3][0],
        "vd": rows[4][0],
        "vv": rows[5][0],
        "R": rows[6][0],
        "Delta_1": rows[7][0],
        "Delta_2": rows[8][0],
        "profit_early": rows[9][0],
        "profit_late": rows[10][0],
        "px": rows[11][:n],
        "py": rows[12][:n],
        "service_time": rows[13][:n],
        "earlist": rows[14][:n],
        "duedate": rows[15][:n],
        "weight": rows[16][:n],
    }

    # 源 C++ 的 (6)(7) 直接用 p1X^2+p1Y^2 表示到港口的距离，
    # 因此它隐含假设港口坐标为 (0, 0)。不满足时立即报错，
    # 防止静默求解一个与 C++ 含义不一致的模型。
    if abs(data["px"][0]) > SMALL_NUM or abs(data["py"][0]) > SMALL_NUM:
        raise ValueError(
            f"{filename}: C++ 模型要求港口坐标 px[0]=py[0]=0，"
            f"当前为 ({data['px'][0]}, {data['py'][0]})。"
        )
    if data["vd"] <= 0 or data["vv"] <= 0:
        raise ValueError(f"{filename}: 无人机和船舶速度必须为正数。")
    if data["Delta_1"] <= 0 or data["Delta_2"] <= 0:
        raise ValueError(f"{filename}: Delta_1 和 Delta_2 必须为正数。")
    return data


def main() -> None:
    enabled_cuts = {
        name
        for name, enabled in (
            ("cycle", USE_CYCLE),
            ("profit_cap", USE_PROFIT_CAP),
            ("depot_out", USE_DEPOT_OUT),
            ("depot_in", USE_DEPOT_IN),
            ("inter_task", USE_INTER_TASK),
            ("time_triangle_1", USE_TIME_TRIANGLE_1),
            ("time_triangle_2", USE_TIME_TRIANGLE_2),
            ("time_difference", USE_TIME_DIFFERENCE),
        )
        if enabled
    }
    experiment_label = EXPERIMENT_LABEL
    data_dir = DATA_DIR
    result_root = RESULT_ROOT
    if USE_EXPERIMENT_SUBDIRECTORY:
        result_dir = os.path.join(result_root, experiment_label)
    else:
        result_dir = result_root
    figure_dir = os.path.join(result_dir, "figures")
    summary_file = os.path.join(result_dir, "summary_results.txt")
    ablation_summary_file = os.path.join(result_root, "valid_inequality_results.csv")
    os.makedirs(result_root, exist_ok=True)
    os.makedirs(result_dir, exist_ok=True)
    os.makedirs(figure_dir, exist_ok=True)

    if not os.path.exists(summary_file):
        with open(summary_file, "w", encoding="utf-8") as f:
            f.write("Example\tNj\tNk\tBestBound\tObjective\tGap\tTime\tStatus\n")
    elif os.path.getsize(summary_file) == 0:
        with open(summary_file, "w", encoding="utf-8") as f:
            f.write("Example\tNj\tNk\tBestBound\tObjective\tGap\tTime\tStatus\n")

    print_data = True
    fix_cycle3 = False

    # These loops are written like C++:
    # for (num1 = 5; num1 <= 15; num1 += 1)
    for num1 in range(NUM1_START, NUM1_END + 1, NUM1_STEP):
        for num2 in range(NUM2_START, NUM2_END + 1, NUM2_STEP):
            for num3 in range(NUM3_START, NUM3_END + 1, NUM3_STEP):
                case_name = f"Example{num1}_{num2}_{num3}"
                file_name = f"Example{num1}_{num2}_{num3}.dat"
                data_file = data_dir + "\\" + file_name

                if not os.path.exists(data_file):
                    print(f"Skip missing file: {data_file}")
                    continue

                output_file = result_dir + "\\" + f"Example{num1}_{num2}_{num3}_result.txt"
                print(f"Reading data file: {data_file}")
                data = read_data(data_file)
                Nj = data["Nj"]
                Nk = data["Nk"]
                T = data["T"]
                Profit_cost = data["Profit_cost"]
                vd = data["vd"]
                vv = data["vv"]
                R = data["R"]
                Delta_1 = data["Delta_1"]
                Delta_2 = data["Delta_2"]
                profit_early = data["profit_early"]
                profit_late = data["profit_late"]
                px = data["px"]
                py = data["py"]
                service_time = data["service_time"]
                earlist = data["earlist"]
                duedate = data["duedate"]
                weight = data["weight"]

                if print_data:
                    print("========== Data check ==========")
                    print(f"file = {file_name}")
                    print(f"Nj = {Nj}")
                    print(f"Nk = {Nk}")
                    print(f"T = {T}")
                    print(f"Profit_cost = {Profit_cost}")
                    print(f"vd = {vd}")
                    print(f"vv = {vv}")
                    print(f"R = {R}")
                    print(f"Delta_1 = {Delta_1}")
                    print(f"Delta_2 = {Delta_2}")
                    print(f"profit_early = {profit_early}")
                    print(f"profit_late = {profit_late}")
                    print(f"px = {px}")
                    print(f"py = {py}")
                    print(f"service_time = {service_time}")
                    print(f"earlist = {earlist}")
                    print(f"duedate = {duedate}")
                    print(f"weight = {weight}")
                    print("================================")

                nodes = range(Nj + 1)       # 0, 1, ..., Nj
                tasks = range(1, Nj + 1)    # 1, 2, ..., Nj
                vessels = range(1, Nk + 1)  # 1, 2, ..., Nk

                big_M_time = 100 * T
                epsilon = 1e-8

                c = [[0.0 for j in nodes] for i in nodes]
                for i in nodes:
                    for j in nodes:
                        c[i][j] = math.sqrt((px[i] - px[j]) ** 2 + (py[i] - py[j]) ** 2)

                # ===================== 创建 Gurobi 模型 =====================
                model = gp.Model("vessel_uav_profit_model")

                # ===================== 决策变量：对应 C++ Define Variables =====================
                # p1=(p1X,p1Y)：船 k 执行任务 i 时的 UAV 起飞点；
                # p2=(p2X,p2Y)：同一架次的 UAV 降落/回收点。
                p1X = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="p1X")
                p1Y = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="p1Y")
                p2X = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="p2X")
                p2Y = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="p2Y")

                # y[i,k]=1：任务 i 由船 k 服务；x[i,j,k]=1：船 k 紧接着从 i 到 j。
                # C++ 用 [0,Nj]x[0,Nj]x[1,Nk]，这里保留同样的有效索引。
                y = model.addVars(tasks, vessels, vtype=GRB.BINARY, name="y")
                x = model.addVars(nodes, nodes, vessels, vtype=GRB.BINARY, name="x")

                # D[i,j,k]：船 k 从任务 i 的 UAV 回收点到任务 j 起飞点的距离。
                # t：船从起飞点到回收点的航行时间；t1/t2：UAV 去/返航时间。
                D = model.addVars(nodes, nodes, vessels, lb=0, name="D")
                t = model.addVars(tasks, vessels, lb=0, name="t")
                t1 = model.addVars(tasks, vessels, lb=0, name="t1")
                t2 = model.addVars(tasks, vessels, lb=0, name="t2")
                arrival_time = model.addVars(tasks, lb=0, name="arrival_time")
                total_profit = model.addVars(tasks, lb=0, name="total_profit")
                # u[i] 是 C++ 中共享于所有船舶的 MTZ 顺序变量。
                # 当 Nj=1 时 C++ 的上下界会冲突，max(1,Nj-1) 只处理这个边界情况。
                u = model.addVars(tasks, lb=1, ub=max(1, Nj - 1), vtype=GRB.INTEGER, name="u")

                # z1..z11 是把欧式距离写成二阶锥/二次约束时的辅助变量。
                # z7/z8/z11 等于“非负距离 + 非负 Big-M 项”，因而数学上必然 >=0。
                # 显式设置 lb=0 不改变 C++ 可行域，并帮助 Gurobi 识别凸二阶锥结构。
                z1 = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="z1")
                z2 = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="z2")
                z3 = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="z3")
                z4 = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="z4")
                z5 = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="z5")
                z6 = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="z6")
                z7 = model.addVars(tasks, vessels, lb=0, name="z7")
                z8 = model.addVars(tasks, vessels, lb=0, name="z8")
                z9 = model.addVars(tasks, tasks, vessels, lb=-GRB.INFINITY, name="z9")
                z10 = model.addVars(tasks, tasks, vessels, lb=-GRB.INFINITY, name="z10")
                z11 = model.addVars(tasks, tasks, vessels, lb=0, name="z11")
                z16 = model.addVars(tasks, vessels, lb=-GRB.INFINITY, name="z16")

                # C++ 还声明了 z14/z15，但它们从未出现在目标函数或任何约束中。
                # 创建完全游离的变量只会增加模型体积，因此 Python 版有意不创建它们。

                zz2 = model.addVars(tasks, vtype=GRB.BINARY, name="zz2")
                zz3 = model.addVars(tasks, vtype=GRB.BINARY, name="zz3")
                zz4 = model.addVars(tasks, vtype=GRB.BINARY, name="zz4")

                # ===================== 目标函数：C++ 第 470--476 行 =====================
                # 最大化被服务任务的加权总收益。total_profit[i] 是单位收益，
                # weight[i] 是任务权重，两者乘积与 C++ temp 表达式逐项对应。
                model.setObjective(
                    gp.quicksum(total_profit[i] * weight[i] for i in tasks),
                    GRB.MAXIMIZE,
                )

                # ===================== 约束：按 C++ 原编号排列 =====================
                # 对角元素不允许自环，且自环距离固定为 0。
                for k in vessels:
                    for i in nodes:
                        model.addConstr(x[i, i, k] == 0, name=f"C_self_x[{i},{k}]")
                        model.addConstr(D[i, i, k] == 0, name=f"C_self_D[{i},{k}]")


                # (2) 船从 UAV 起飞点 p1 航行到回收点 p2。
                # ||p1-p2||_2 <= vv*t；vv 是船速，t 是船航行时间。
                for i in tasks:
                    for k in vessels:
                        model.addConstr(z1[i, k] == p1X[i, k] - p2X[i, k], name=f"C03_dx[{i},{k}]")
                        model.addConstr(z2[i, k] == p1Y[i, k] - p2Y[i, k], name=f"C03_dy[{i},{k}]")
                        model.addQConstr(
                            z1[i, k] * z1[i, k] + z2[i, k] * z2[i, k]
                            <= (vv * t[i, k]) * (vv * t[i, k]),
                            name=f"C03_vessel_distance[{i},{k}]",
                        )


                # (3) UAV 从起飞点 p1 飞到任务点 (px[i],py[i])：
                # ||p1-task_i||_2 <= vd*t1。
                for i in tasks:
                    for k in vessels:
                        model.addConstr(z3[i, k] == p1X[i, k] - px[i], name=f"C04_dx[{i},{k}]")
                        model.addConstr(z4[i, k] == p1Y[i, k] - py[i], name=f"C04_dy[{i},{k}]")
                        model.addQConstr(
                            z3[i, k] * z3[i, k] + z4[i, k] * z4[i, k]
                            <= (vd * t1[i, k]) * (vd * t1[i, k]),
                            name=f"C04_uav_outbound[{i},{k}]",
                        )


                # (4) UAV 从任务点飞到回收点 p2：||p2-task_i||_2 <= vd*t2。
                for i in tasks:
                     for k in vessels:
                        model.addConstr(z5[i, k] == p2X[i, k] - px[i], name=f"C05_dx[{i},{k}]")
                        model.addConstr(z6[i, k] == p2Y[i, k] - py[i], name=f"C05_dy[{i},{k}]")
                        model.addQConstr(
                            z5[i, k] * z5[i, k] + z6[i, k] * z6[i, k]
                            <= (vd * t2[i, k]) * (vd * t2[i, k]),
                            name=f"C05_uav_inbound[{i},{k}]",
                        )


                for i in tasks:
                     for k in vessels:
                        # (5-1) 港口到第一个起飞点的船行距离。x[0,i,k]=1 时紧约束为
                        # ||p1||_2 <= D[0,i,k]；否则用 c[0][i]+vd*R 松弛。
                        model.addQConstr(
                            p1X[i, k] * p1X[i, k] + p1Y[i, k] * p1Y[i, k]
                            <= z7[i, k] * z7[i, k],
                            name=f"C06_depot_to_launch[{i},{k}]",
                        )
                        model.addConstr(
                            z7[i, k] == D[0, i, k] + (1 - x[0, i, k]) * (c[0][i] + vd * R),
                            name=f"C06_radius[{i},{k}]",
                        )

                        # (5-2) 最后一个回收点到港口的船行距离。
                        model.addQConstr(
                            p2X[i, k] * p2X[i, k] + p2Y[i, k] * p2Y[i, k]
                            <= z8[i, k] * z8[i, k],
                            name=f"C07_landing_to_depot[{i},{k}]",
                        )
                        model.addConstr(
                            z8[i, k] == D[i, 0, k] + (1 - x[i, 0, k]) * (c[i][0] + vd * R),
                            name=f"C07_radius[{i},{k}]",
                        )


                for k in vessels:
                    for i in tasks:
                        for j in tasks:
                            if i == j:
                                continue
                            # (5-3) 相邻任务 i->j 之间，船从 i 的回收点 p2_i 到 j 的起飞点 p1_j。
                            model.addConstr(z9[i, j, k] == p2X[i, k] - p1X[j, k], name=f"C08_dx[{i},{j},{k}]")
                            model.addConstr(z10[i, j, k] == p2Y[i, k] - p1Y[j, k], name=f"C08_dy[{i},{j},{k}]")
                            model.addQConstr(
                                z9[i, j, k] * z9[i, j, k] + z10[i, j, k] * z10[i, j, k]
                                <= z11[i, j, k] * z11[i, j, k],
                                name=f"C08_inter_task_distance[{i},{j},{k}]",
                            )
                            model.addConstr(
                                z11[i, j, k]
                                == D[i, j, k] + (1 - x[i, j, k]) * (c[i][j] + 2 * vd * R),
                                name=f"C08_radius[{i},{j},{k}]",
                            )

                # (6) UAV 续航：去程+服务+返程不超过 R。
                # (7) 船从 p1 到 p2 的时间不能超过 UAV 任务时长，避免 UAV 等船。
                for k in vessels:
                    for i in tasks:
                        model.addConstr(
                             t1[i, k] + t2[i, k] + service_time[i] * y[i, k] <= R,
                              name=f"C17_endurance[{i},{k}]",
                            )
                        model.addConstr(
                           t1[i, k] + t2[i, k] + service_time[i] * y[i, k] >= t[i, k],
                             name=f"C18_no_uav_wait[{i},{k}]",
                           )


                # (8) 每个任务最多由一艘船服务；允许任务不被选中。
                for i in tasks:
                    model.addConstr(
                        gp.quicksum(y[i, k] for k in vessels) <= 1,
                        name=f"C10_at_most_one_vessel[{i}]",
                    )


                for k in vessels:
                    # (9) 每艘船最多从港口出发一次（因此最多形成一条路线）。
                    model.addConstr(
                        gp.quicksum(x[0, i, k] for i in tasks) <= 1,
                        name=f"C14_at_most_one_route[{k}]",
                    )
                    # (10) 船从港口出发的次数 = 回到港口的次数。
                    model.addConstr(
                        gp.quicksum(x[0, i, k] for i in tasks)
                        == gp.quicksum(x[i, 0, k] for i in tasks),
                        name=f"C13_depot_balance[{k}]",
                    )


                # (11) 若船 k 服务任务 j，则 j 恰好有一条入弧。
                for j in tasks:
                    for k in vessels:
                        model.addConstr(
                            gp.quicksum(x[i, j, k] for i in nodes if i != j) == y[j, k],
                            name=f"C11_in_degree[{j},{k}]",
                        )

                # (12) 若船 k 服务任务 i，则 i 恰好有一条出弧。
                for i in tasks:
                    for k in vessels:
                        model.addConstr(
                            gp.quicksum(x[i, j, k] for j in nodes if i != j) == y[i, k],
                            name=f"C12_out_degree[{i},{k}]",
                        )


                # (13) 若 i 是船 k 的第一个任务，到达 i 的时间至少为
                # 船从港口到起飞点的时间 + UAV 去程时间。
                for k in vessels:
                    for i in tasks:
                        model.addConstr(
                            arrival_time[i]
                            >= D[0, i, k] / vv + t1[i, k]
                            - (1 - x[0, i, k]) * big_M_time,
                            name=f"C19_first_arrival[{i},{k}]",
                        )

                # (14)访问最后一个任务点的到达时间限制
                # 这里也不加 Big-M，与 C++ 第 723 行严格一致。
                for k in vessels:
                    for i in tasks:
                        model.addConstr(
                            T >= arrival_time[i] + service_time[i] * y[i, k]
                              + t2[i, k] + D[i, 0, k] / vv,
                              name=f"C20_return_before_T[{i},{k}]",
                        )

                # (15) 若船 k 紧接着从 i 到 j，则 j 的到达时间要覆盖：
                # i 的到达时间、服务时间、UAV 返程、船间转移、j 的 UAV 去程。
                for k in vessels:
                    for i in tasks:
                        for j in tasks:
                            if i != j:
                                model.addConstr(
                                    arrival_time[j]
                                    >= arrival_time[i]
                                    + service_time[i] * y[i, k]
                                    + t2[i, k]
                                    + D[i, j, k] / vv
                                    + t1[j, k]
                                    - big_M_time * (1 - x[i, j, k])
                                , name=f"C20_time_sequence[{i},{j},{k}]")


                # (16-17) 分段收益函数。zz2/zz3/zz4 分别表示早到、准时、迟到区间。
                # C++ 的 IloNum big_MM; 没有初始化便用 +=，属于未定义行为。
                # 这里等价实现其明显意图：big_MM = Profit_cost * sum(weight)。
                big_MM = Profit_cost * sum(weight[i] for i in tasks)
                if big_MM <= 0:
                    big_MM = 1
                big_profit_M = 10 * Nj * big_MM

                for i in tasks:
                    sum_y = gp.quicksum(y[i, k] for k in vessels)

                    # 区间 2：earlist-Delta_1 <= arrival_time < earlist。
                    model.addConstr(
                        arrival_time[i] >= earlist[i] - Delta_1 - big_M_time * (1 - zz2[i]),
                        name=f"C21_early_lb[{i}]",
                    )
                    model.addConstr(
                        arrival_time[i] <= earlist[i] - epsilon + big_M_time * (1 - zz2[i]),
                        name=f"C21_early_ub[{i}]",
                    )

                    # 区间 3：earlist <= arrival_time <= duedate。
                    model.addConstr(
                        arrival_time[i] >= earlist[i] - big_M_time * (1 - zz3[i]),
                        name=f"C21_on_time_lb[{i}]",
                    )
                    model.addConstr(
                        arrival_time[i] <= duedate[i] + big_M_time * (1 - zz3[i]),
                        name=f"C21_on_time_ub[{i}]",
                    )

                    # 区间 4：duedate < arrival_time <= duedate+Delta_2。
                    model.addConstr(
                        arrival_time[i] >= duedate[i] + epsilon - big_M_time * (1 - zz4[i]),
                        name=f"C21_late_lb[{i}]",
                    )
                    model.addConstr(
                        arrival_time[i] <= duedate[i] + Delta_2 + big_M_time * (1 - zz4[i]),
                        name=f"C21_late_ub[{i}]",
                    )

                    early_profit = profit_early + (Profit_cost - profit_early) / Delta_1 * (arrival_time[i] - (earlist[i] - Delta_1))
                    late_profit = Profit_cost - (Profit_cost - profit_late) / Delta_2 * (arrival_time[i] - duedate[i])

                    # 被选中的区间通过上、下两条 Big-M 约束将 total_profit 固定在该段函数上。
                    model.addConstr(total_profit[i] >= early_profit - big_profit_M * (1 - zz2[i]), name=f"C21_early_profit_lb[{i}]")
                    model.addConstr(total_profit[i] <= early_profit + big_profit_M * (1 - zz2[i]), name=f"C21_early_profit_ub[{i}]")

                    model.addConstr(total_profit[i] >= Profit_cost - big_profit_M * (1 - zz3[i]), name=f"C21_on_time_profit_lb[{i}]")
                    model.addConstr(total_profit[i] <= Profit_cost + big_profit_M * (1 - zz3[i]), name=f"C21_on_time_profit_ub[{i}]")

                    model.addConstr(total_profit[i] >= late_profit - big_profit_M * (1 - zz4[i]), name=f"C21_late_profit_lb[{i}]")
                    model.addConstr(total_profit[i] <= late_profit + big_profit_M * (1 - zz4[i]), name=f"C21_late_profit_ub[{i}]")

                    # 任务没有分配给任何船时 sum_y=0，结合 total_profit>=0 得 total_profit=0。
                    model.addConstr(total_profit[i] <= big_profit_M * sum_y, name=f"C21_unselected_profit_zero[{i}]")
                    if USE_PROFIT_CAP:
                        model.addConstr(
                            total_profit[i] <= Profit_cost,
                            name=f"VI_profit_cap[{i}]",
                        )
                    model.addConstr(zz2[i] <= sum_y, name=f"C21_early_only_if_selected[{i}]")
                    model.addConstr(zz3[i] <= sum_y, name=f"C21_on_time_only_if_selected[{i}]")
                    model.addConstr(zz4[i] <= sum_y, name=f"C21_late_only_if_selected[{i}]")
                    model.addConstr(
                        zz2[i] + zz3[i] + zz4[i] == sum_y,
                        name=f"C21_exactly_one_profit_segment[{i}]",
                    )


                # (19-20) MTZ 子环消除：任务点之间不能形成与港口脱节的闭环。
                for k in vessels:
                     for i in tasks:
                        for j in tasks:
                            if i != j:
                               model.addConstr(
                                   u[i] - u[j] + Nj * x[i, j, k] <= Nj - 1,
                                     name=f"C15_MTZ[{i},{j},{k}]",
                                )


                # ===================== C++ 有效不等式 =====================
                # (44)有效不等式，各开关仅为消融实验服务；默认全部开启，与 C++ 完整模型一致。
                if USE_CYCLE:
                    for k in vessels:
                        model.addConstr(
                            gp.quicksum(t1[i, k] + t2[i, k] + service_time[i] * y[i, k] for i in tasks)
                            + gp.quicksum((D[0, i, k] + D[i, 0, k]) / vv for i in tasks)
                            + gp.quicksum(D[i, j, k] / vv for i in tasks for j in tasks if i != j)
                            <= T,
                            name=f"VI_cycle[{k}]",
                        )

                # C++ 在以下不等式的船舶循环中误用 k < Nk，导致最后一艘船遗漏。
                # Python 使用 vessels=range(1,Nk+1)，将同一条有效不等式正确加到每艘船上。
                for k in vessels:
                    for i in tasks:
                        if USE_DEPOT_OUT:
                            #(45)有效不等式，
                            model.addConstr(
                                x[0, i, k] * c[0][i] <= D[0, i, k] + vd * t1[i, k],
                                name=f"VI_depot_out[{i},{k}]",
                            )
                        #(46)有效不等式，
                        if USE_DEPOT_IN:
                            model.addConstr(
                                x[i, 0, k] * c[i][0] <= D[i, 0, k] + vd * t2[i, k],
                                name=f"VI_depot_in[{i},{k}]",
                            )
                        # (47)有效不等式，
                        if USE_INTER_TASK:
                            for j in tasks:
                                if i != j:
                                    model.addConstr(
                                        x[i, j, k] * c[i][j]
                                        <= D[i, j, k] + vd * t2[i, k] + vd * t1[j, k],
                                        name=f"VI_inter_task[{i},{j},{k}]",
                                    )

                        #(48)
                        if USE_TIME_TRIANGLE_1:
                            model.addConstr(
                                t1[i, k] * vd <= t2[i, k] * vd + t[i, k] * vv,
                                name=f"VI_time_triangle_1[{i},{k}]",
                            )
                        #(49)
                        if USE_TIME_TRIANGLE_2:
                            model.addConstr(
                                t2[i, k] * vd <= t1[i, k] * vd + t[i, k] * vv,
                                name=f"VI_time_triangle_2[{i},{k}]",
                            )
                        #(54)
                        if USE_TIME_DIFFERENCE:
                            model.addConstr(
                                z16[i, k] == t1[i, k] - t2[i, k],
                                name=f"VI_time_difference_def[{i},{k}]",
                            )
                            # 严格保留 C++ 的运算顺序：(R^2*vv^2)/vd*vd，该式会约去 vd。
                            # 如论文原式实为 (R*vv/vd)^2，则应另行将右端改为
                            # (R*R*vv*vv)/(vd*vd)；本次不擅自改动 C++ 数学式。
                            model.addQConstr(
                                z16[i, k] * z16[i, k] <= (R * R * vv * vv) / vd * vd,
                                name=f"VI_time_difference_bound[{i},{k}]",
                            )

                # Optional fixed assignments from the C++ "cycle 3" block.
                if fix_cycle3:
                    model.addConstr(y[4, 1] == 1)
                    model.addConstr(y[2, 1] == 1)
                    model.addConstr(y[3, 2] == 1)
                    model.addConstr(y[1, 2] == 1)

                # ===================== Gurobi 求解参数 =====================
                # Threads/MIPGap/TimeLimit 分别对应 CPLEX Threads/EpGap/TiLim。
                # CPLEX EpInt=0 在 Gurobi 中没有合法的精确对应，Gurobi IntFeasTol 最小为 1e-9。
                model.Params.Threads = THREADS
                model.Params.Seed = RANDOM_SEED
                model.Params.MIPGap = 1e-6
                model.Params.TimeLimit = TIME_LIMIT
                model.Params.IntFeasTol = 1e-9
                # 二阶锥约束已通过非负右端建模；NonConvex=2 作为兼容性保障，
                # 以免不同 Gurobi 版本对二次形式的自动识别方式不同。
                model.Params.NonConvex = 2

                # CPLEX 源码总是导出 model_debug.lp。这里按算例分文件导出，
                # 避免批量运行时后一个算例覆盖前一个。
                if WRITE_MODEL_LP:
                    model.update()
                    model.write(os.path.join(result_dir, f"{case_name}_model.lp"))

                # ===================== solve =====================
                model.optimize()

                write_header = (
                    not os.path.exists(ablation_summary_file)
                    or os.path.getsize(ablation_summary_file) == 0
                )
                with open(ablation_summary_file, "a", newline="", encoding="utf-8-sig") as f:
                    fieldnames = [
                        "instance", "experiment", "cuts", "status", "sol_count",
                        "objective", "best_bound", "mip_gap", "runtime", "node_count",
                        "linear_constraints", "quadratic_constraints", "threads", "seed",
                        "time_limit",
                    ]
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    if write_header:
                        writer.writeheader()
                    has_solution = model.SolCount > 0
                    writer.writerow(
                        {
                            "instance": case_name,
                            "experiment": experiment_label,
                            "cuts": "+".join(sorted(enabled_cuts)) if enabled_cuts else "none",
                            "status": model.Status,
                            "sol_count": model.SolCount,
                            "objective": model.ObjVal if has_solution else "",
                            "best_bound": model.ObjBound,
                            "mip_gap": model.MIPGap if has_solution else "",
                            "runtime": model.Runtime,
                            "node_count": model.NodeCount,
                            "linear_constraints": model.NumConstrs,
                            "quadratic_constraints": model.NumQConstrs,
                            "threads": THREADS,
                            "seed": RANDOM_SEED,
                            "time_limit": TIME_LIMIT,
                        }
                    )

                # ===================== output =====================
                output_lines = []
                output_lines.append(f"Experiment: {experiment_label}")
                output_lines.append(
                    "Cuts: " + ("+".join(sorted(enabled_cuts)) if enabled_cuts else "none")
                )

                if model.SolCount == 0:
                    output_lines.append(f"Nj: {Nj}")
                    output_lines.append(f"Nk: {Nk}")
                    output_lines.append(f"Status: {model.Status}")
                    output_lines.append("No feasible solution was found.")
                    with open(output_file, "w", encoding="utf-8") as f:
                        f.write("\n".join(output_lines))
                    with open(summary_file, "a", encoding="utf-8") as f:
                        f.write(f"{case_name}\t{Nj}\t{Nk}\tNA\tNA\tNA\t{model.Runtime:.6g}\t{model.Status}\n")
                    print("\n".join(output_lines))
                    continue

                routes = {}
                for k in vessels:
                    route = [0]
                    current_node = 0
                    for step in nodes:
                        next_node = -1
                        for j in nodes:
                            if current_node != j and x[current_node, j, k].X >= 0.01:
                                next_node = j
                                break
                        if next_node == -1:
                            break
                        route.append(next_node)
                        if next_node == 0:
                            break
                        current_node = next_node
                    routes[k] = route

                output_lines.append(f"Nj: {Nj}")
                output_lines.append(f"Nk: {Nk}")
                output_lines.append(f"LB: {model.ObjBound:.6g}")
                output_lines.append(f"Objective: {model.ObjVal:.6g}")
                output_lines.append(f"Gap: {model.MIPGap:.6g}")
                output_lines.append(f"Time: {model.Runtime:.6g}")

                with open(summary_file, "a", encoding="utf-8") as f:
                    f.write(
                        f"{case_name}\t{Nj}\t{Nk}\t{model.ObjBound:.6g}\t"
                        f"{model.ObjVal:.6g}\t{model.MIPGap:.6g}\t{model.Runtime:.6g}\t{model.Status}\n"
                    )

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239y[i][k]: ")
                    output_lines.append("  ".join("0" if abs(y[i, k].X) < SMALL_NUM else f"{y[i, k].X:.6g}" for i in tasks) + "  ")

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239p1X[i][k]: ")
                    output_lines.append("  ".join(f"{p1X[i, k].X:.6g}" for i in tasks if y[i, k].X >= SMALL_NUM) + "  ")

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239p1Y[i][k]: ")
                    output_lines.append("  ".join(f"{p1Y[i, k].X:.6g}" for i in tasks if y[i, k].X >= SMALL_NUM) + "  ")

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239p2X[i][k]: ")
                    output_lines.append("  ".join(f"{p2X[i, k].X:.6g}" for i in tasks if y[i, k].X >= SMALL_NUM) + "  ")

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239p2Y[i][k]: ")
                    output_lines.append("  ".join(f"{p2Y[i, k].X:.6g}" for i in tasks if y[i, k].X >= SMALL_NUM) + "  ")

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239t[i][k]: ")
                    output_lines.append("  ".join(f"{t[i, k].X:.6g}" for i in tasks if y[i, k].X >= SMALL_NUM) + "  ")

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239t1[i][k]: ")
                    output_lines.append("  ".join(f"{t1[i, k].X:.6g}" for i in tasks if y[i, k].X >= SMALL_NUM) + "  ")

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239t2[i][k]: ")
                    output_lines.append("  ".join(f"{t2[i, k].X:.6g}" for i in tasks if y[i, k].X >= SMALL_NUM) + "  ")

                output_lines.append("\u6bcf\u76ee\u6807\u70b9\u7684\u7b49\u5f85\u65f6\u95f4service_time[i]: ")
                for i in tasks:
                    output_lines.append(f"service_time[{i}] = {service_time[i]:.6g}")

                output_lines.append("\u6bcf\u76ee\u6807\u70b9\u7684\u6536\u76catotal_profit[i]: ")
                for i in tasks:
                    output_lines.append(f"total_profit[{i}] = {total_profit[i].X:.6g}")

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239x[i][j][k]\u60c5\u51b5: ")
                    for i in nodes:
                        one_row = []
                        for j in nodes:
                            if abs(x[i, j, k].X) < SMALL_NUM:
                                one_row.append("0")
                            else:
                                one_row.append(f"{x[i, j, k].X:.6g}")
                        output_lines.append(f"[{i}]: " + "  ".join(one_row) + "  ")
                    output_lines.append("")

                for k in vessels:
                    output_lines.append(f"\u7b2c{k}\u8258\u8239D[i][j][k]\u60c5\u51b5: ")
                    for i in nodes:
                        one_row = []
                        for j in nodes:
                            if abs(D[i, j, k].X) < SMALL_NUM:
                                one_row.append("0       ")
                            else:
                                one_row.append(f"{D[i, j, k].X:.6g}")
                        output_lines.append(f"[{i}]: " + "  ".join(one_row) + "  ")
                    output_lines.append("")

                output_lines.append("\u5230\u8fbe\u4efb\u52a1\u70b9\u7684\u65f6\u95f4\u70b9\uff1a")
                for i in tasks:
                    output_lines.append(f"\u4efb\u52a1\u70b9[{i}]: {arrival_time[i].X:.6g}")
                    output_lines.append("")

                output_lines.append("#=================\u7ed8\u56fe\u6240\u9700\u6570\u636e================")
                output_lines.append("#\u6bcf\u8258\u8239\u8bbf\u95ee\u7684\u4efb\u52a1\u70b9\uff1a")
                for k in vessels:
                    output_lines.append(f"#\u7b2c{k}\u8258\u8239\u8bbf\u95ee\u7684\u4efb\u52a1\u70b9: " + "\t".join(str(node) for node in routes[k]))
                output_lines.append("")
                output_lines.append("")

                output_lines.append("#\u6e2f\u53e3\u548c\u4efb\u52a1\u70b9\u7684\u5750\u6807\uff08\u6563\u70b9\u56fe\uff09\uff1a")
                output_lines.append("task_x = [" + ",".join(f"{px[j]:.6g}" for j in nodes) + "]")
                output_lines.append("task_y = [" + ",".join(f"{py[j]:.6g}" for j in nodes) + "]")
                output_lines.append("")

                output_lines.append("#\u65e0\u4eba\u673a\u7684\u8d77\u98de\u964d\u843d\u70b9\u7684\u5750\u6807(\u6563\u70b9\u56fe)")
                for k in vessels:
                    uav_x = []
                    uav_y = []
                    for i in tasks:
                        if y[i, k].X >= SMALL_NUM:
                            uav_x.append(f"{p1X[i, k].X:.6g}")
                            uav_x.append(f"{p2X[i, k].X:.6g}")
                            uav_y.append(f"{p1Y[i, k].X:.6g}")
                            uav_y.append(f"{p2Y[i, k].X:.6g}")
                    output_lines.append(f"uav{k}_p1x = [" + ",".join(uav_x) + "]")
                    output_lines.append(f"uav{k}_p1y = [" + ",".join(uav_y) + "]")
                output_lines.append("")
                output_lines.append("")

                output_lines.append("#\u65e0\u4eba\u673a\u7684\u8def\u5f84(\u6298\u7ebf\u56fe\uff0c\u65e0\u9700\u8c03\u6574\u4efb\u52a1\u70b9\u7684\u987a\u5e8f)")
                for k in vessels:
                    iikk = 0
                    output_lines.append(f"#Vessel {k}")
                    output_lines.append(f"#Vessel {k}-UAV\uff1a\u65e0\u4eba\u673a\u8bbf\u95ee\u4efb\u52a1\u70b9")
                    for i in tasks:
                        if y[i, k].X >= SMALL_NUM:
                            iikk += 1
                            output_lines.append(f"vessel{k}_uav_x_{iikk} = [{p1X[i, k].X:.6g}, {px[i]:.6g}, {p2X[i, k].X:.6g}]")
                            output_lines.append(f"vessel{k}_uav_y_{iikk} = [{p1Y[i, k].X:.6g},{py[i]:.6g},{p2Y[i, k].X:.6g}]")
                    output_lines.append("")
                output_lines.append("")
                output_lines.append("")

                for k in vessels:
                    output_lines.append(f"#\u7b2c{k}\u8258\u8239\u8bbf\u95ee\u987a\u5e8f: " + "\u2192".join(str(node) for node in routes[k]))
                output_lines.append("")

                output_lines.append("#\u8239\u7684\u8def\u5f84(\u6298\u7ebf\u56fe)")
                for k in vessels:
                    vessel_x = ["0.0"]
                    vessel_y = ["0.0"]
                    for node in routes[k]:
                        if node != 0:
                            vessel_x.append(f"{p1X[node, k].X:.6g}")
                            vessel_x.append(f"{p2X[node, k].X:.6g}")
                            vessel_y.append(f"{p1Y[node, k].X:.6g}")
                            vessel_y.append(f"{p2Y[node, k].X:.6g}")
                    vessel_x.append("0.0")
                    vessel_y.append("0.0")
                    output_lines.append(f"vessel{k}_x = [" + ",".join(vessel_x) + "]")
                    output_lines.append(f"vessel{k}_y = [" + ",".join(vessel_y) + "]")

                # ===================== draw figure =====================
                figure_file = figure_dir + "\\" + f"Example{num1}_{num2}_{num3}_path.png"

                plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "Arial Unicode MS"]
                plt.rcParams["axes.unicode_minus"] = False

                fig, ax = plt.subplots(figsize=(10, 8), dpi=150)

                ax.scatter(px[0], py[0], marker="s", s=90, color="black", label="\u6e2f\u53e3")
                ax.scatter([px[i] for i in tasks], [py[i] for i in tasks], marker="o", s=55, color="tab:blue", label="\u4efb\u52a1\u70b9")

                ax.text(px[0], py[0], " 0", fontsize=10, color="black")
                for i in tasks:
                    ax.text(px[i], py[i], f" {i}", fontsize=9, color="tab:blue")

                colors = ["tab:red", "tab:green", "tab:orange", "tab:purple", "tab:brown", "tab:pink", "tab:gray", "tab:olive"]

                for k in vessels:
                    color = colors[(k - 1) % len(colors)]
                    vessel_x_plot = [0.0]
                    vessel_y_plot = [0.0]
                    for node in routes[k]:
                        if node != 0:
                            vessel_x_plot.append(p1X[node, k].X)
                            vessel_x_plot.append(p2X[node, k].X)
                            vessel_y_plot.append(p1Y[node, k].X)
                            vessel_y_plot.append(p2Y[node, k].X)
                    vessel_x_plot.append(0.0)
                    vessel_y_plot.append(0.0)
                    ax.plot(vessel_x_plot, vessel_y_plot, "-", linewidth=2, color=color, label=f"\u7b2c{k}\u8258\u8239\u8def\u5f84")
                    ax.scatter(vessel_x_plot, vessel_y_plot, s=25, color=color)
                    for i in tasks:
                        if y[i, k].X >= SMALL_NUM:
                            ax.plot([p1X[i, k].X, px[i], p2X[i, k].X], [p1Y[i, k].X, py[i], p2Y[i, k].X], "--", linewidth=1.5, color=color, alpha=0.75)
                            ax.scatter([p1X[i, k].X, p2X[i, k].X], [p1Y[i, k].X, p2Y[i, k].X], marker="x", s=40, color=color)

                ax.set_title(f"{file_name} \u8def\u5f84\u56fe")
                ax.set_xlabel("x")
                ax.set_ylabel("y")
                ax.grid(True, linestyle="--", alpha=0.35)
                ax.axis("equal")
                ax.legend()
                plt.tight_layout()
                plt.savefig(figure_file)
                plt.close(fig)

                output_lines.append("")
                output_lines.append(f"#\u8def\u5f84\u56fe\u6587\u4ef6: {figure_file}")

                with open(output_file, "w", encoding="utf-8") as f:
                    f.write("\n".join(output_lines))
                print(f"Finished. Result written to {output_file}")


if __name__ == "__main__":
    main()
