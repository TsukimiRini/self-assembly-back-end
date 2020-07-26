//
// Created by 褚文杰 on 2019/10/21.
//

#ifndef ASSEMBLYSHAPE_AGENT_H
#define ASSEMBLYSHAPE_AGENT_H

#include<unordered_map>
#include<tuple>
#include<cstdlib>
#include<ctime>
#include <chrono>
#include <mutex>
#include "Cloud.h"

//extern Cloud center;//main中定义的量,说明agent在定义之前需要创建main,(同理cloud在agent前)而main中又引了agent,造成duplicate
//可能可行的方法是将main.cpp与agent.cpp合并,agent.cpp中需要用到的关于cloud的量就直接成为全局量不需要extern
//或者是将Cloud center的定义写在agent.h里,main在使用center的时候是通过include访问的(必要的话加extern)
//Cloud center = Cloud(a_num, dec_m, width, height, dim);
//Agent* swarm = new Agent[a_num];


//struct KEY {
//    int index;
//    int px;
//    int py;
//    KEY(int index, int x, int y) :index(index), px(x), py(y) {}
//};
//
//namespace std {
//    template<>
//    struct hash<KEY> {
//    public:
//        std::size_t operator()(const KEY &key) const {
//            using std::size_t;
//            using std::hash;
//
//            return hash<int>()(key.index) ^ hash<int>()(key.px) ^ hash<int>()(key.py); //这样的hash函数可能导致不同agent针对同样的key值进行信息采集并存入local_view的时候,指向同样的地址
//        }
//    };
//
//    template<>
//    struct equal_to<KEY>{
//    public:
//        bool operator()(const KEY &key1, const KEY &key2) const{
//            return key1.index == key2.index && key1.px == key2.px && key1.py == key2.py;
//        }
//    };
//}

class Agent {
public:
    int index;
    int prior;
    int pos_x;
    int pos_y;
    int sense_r;
    std::unique_lock<std::mutex> guard;
//    std::unordered_map<KEY,std::tuple<bool,int,double,double>> local_view;

    Agent(int id, int px, int py, int r = 2);

    Agent();

    void get_local_info();

    void set_config(int id, int px, int py, int r = 2);

    void get_best_action(int para_set_mode, int W, bool strict_, bool is_parallel = false, bool local=false);

    //multiple threads for agents: require linear outer conflict negotiation
    void get_stop_wait_action(int terminal = -1, double prob = 0.2);

    void avoid_expel();

    //multiple threads for agents: do not require outer conflict negotiation, but thread synchronization of
    //the former operation (i.e., get_best_action) is needed
    void get_no_conflict_action(int terminal = -1);

    //a single thread for all agents(step1+adjust step2 by updating next action list)
    void take_action();

    void parallel_running(bool incremental, double decay_ratio, int method, int para_set_mode, int W, bool strict_, bool is_parallel,
                          int terminal, double prob, bool increment_update_light, bool local);

private:
    std::vector<int> conflicts;
    std::vector<int> inferior_neighbors;
    std::vector<std::vector<int>> rand_bias;
    std::vector<std::vector<int>> rand_less;
};


#endif //ASSEMBLYSHAPE_AGENT_H
