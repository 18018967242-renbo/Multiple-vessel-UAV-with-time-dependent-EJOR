#ifndef alns_hpp
#define alns_hpp

#include "solution.hpp"

#include <ctime>

#include <list> 
#include <iostream>
#include <vector>

// Function pointer for repair/removal heuristic用于修复/删除启发式的函数指针
//这一行声明了一个名为SH的类型别名，用于函数指针。函数指针类型SH被定义为指向一个以Solution&引用为参数并返回Solution*指针的函数
typedef Solution* (*SH)(Solution&);


//这一行开始声明一个名为ALNS的C++类。
class ALNS 
{

    Params* params;//这一行声明了一个指向Params类对象的指针。很可能Params类包含了用于配置ALNS算法的各种参数或设置

    //这些行声明了两个向量，m_repair和m_destroy，它们将存储类型为SH的函数指针。这些向量用于存储修复（repair）和破坏（destroy）启发式方法
    std::vector< SH > m_repair;
    std::vector< SH > m_destroy;

    //这些行声明了多个向量，用于存储与修复和破坏启发式方法相关的各种得分、权重和时间。这些值很可能用于控制ALNS算法的行为。
    std::vector<float> scores_repair;
    std::vector<float> scores_destroy;
    std::vector<float> weights_repair;
    std::vector<float> weights_destroy;
    std::vector<float> times_repair;
    std::vector<float> times_destroy;

    // 成对学习 destroy-repair 的协同效果，而不是将两类算子完全独立评分。
    // 奖励同时考虑解质量和锥子问题调用成本。
    std::vector<std::vector<float>> pair_weights;
    std::vector<std::vector<float>> pair_scores;
    std::vector<std::vector<float>> pair_times;

    //这些行声明了两个指向Solution对象的指针，sol_curr和sol_best。这些指针很可能表示算法找到的当前解和最佳解。
    Solution* sol_curr;
    Solution* sol_best;

    //这一行声明了一个double类型的变量profit_best，很可能用于存储算法找到的最佳解的成本
    double profit_best;

public:

    //addRepair和addDestroy函数很可能用于向ALNS算法中添加修复和破坏启发式方法，以允许自定义算法的行为。
    void addRepair(SH method);
    void addDestroy(SH method);

    //run和runStep函数用于执行ALNS算法，其中run可能运行整个算法，而runStep执行算法的单个步骤。
    void run(Solution* base);
    void runStep();

    //构造函数ALNS很可能用于初始化ALNS对象，可能需要一个Params对象作为参数来配置算法。
    ALNS(Params* params);
};

#endif
