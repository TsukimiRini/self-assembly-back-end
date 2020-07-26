#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <algorithm>
#include <thread>

#include "Agent.h"

using namespace std;
#define TEMP_THREAD_NUM 7
#define PI acos(-1)
const string RUN_ENV = "WIN";
//#define TEMP_THREAD_NUM 64
//#define PI acos(-1)
//const string RUN_ENV = "LINUX";
const int max_agent = 50000;
const int light_intensity = 1000;
const int width = 43;
const int height = 43;
mutex locks[width][height];
mutex cloud_mutex;
enum Agent_num_mode
{
    less_a = -1,
    equal_a = 0,
    more_a = 1
};
Agent_num_mode a_mode = more_a;

Cloud center;
vector<Agent> swarm;
vector<vector<vector<int>>> agent_init_pos_in_shapes; //7*agent_num*2

struct Priors
{
public:
    int a_id;
    double red_light;
    double blue_light;
    int p;

    Priors()
    {
        a_id = -1, p = -1;
        red_light = 0;
        blue_light = 0;
    }
};

bool comp(const Priors &a, const Priors &b)
{
    //prior值越小的越优先,comp返回true说明a<b,也就是a比b优先执行
    return a.blue_light > b.blue_light || (a.blue_light == b.blue_light && a.red_light < b.red_light);
}

bool comp1(const Priors &a, const Priors &b)
{
    //prior值越小的越优先,comp返回true说明a<b,也就是a比b优先执行
    return a.blue_light > b.blue_light || (a.blue_light == b.blue_light && a.red_light > b.red_light);
}

struct ThreadArg
{
    int thread_id;
    int s_id;
    int e_id;

    ThreadArg()
    {
        thread_id = -1;
        s_id = -1;
        e_id = -1;
    };
};

void updateFeild(int t_id, int s_id, int e_id, bool incremental, double decay_ratio, int method = 5)
{
    if (incremental)
    {
        for (int i = s_id; i <= e_id; i++)
        {
            center.multi_update_light_field(swarm[i].pos_x, swarm[i].pos_y, decay_ratio);
        }
    }
    else
    {
        for (int i = s_id; i <= e_id; i++)
        {
            center.cal_sum_light(swarm[i].pos_x, swarm[i].pos_y, decay_ratio, method);
        }
    }
}

void getActionPriorQueue(int t_id, int s_id, int e_id, int para_set_mode, int W, bool strict_, bool is_parallel, bool local = false)
{
    for (int i = s_id; i <= e_id; i++)
    {
        swarm[i].get_best_action(para_set_mode, W, strict_, is_parallel, local);
    }
}

void takeActions(int t_id, int s_id, int e_id)
{
    for (int i = s_id; i <= e_id; i++)
    {
        swarm[i].take_action();
    }
}

void parallel_swarms(int t_id, int s_id, int e_id, bool incremental, double decay_ratio, int method, int para_set_mode, int W, bool strict_,
                     bool is_parallel, int terminal, double prob, bool increment_update_light, bool local)
{
    for (int i = s_id; i <= e_id; i++)
    {
        swarm[i].parallel_running(incremental, decay_ratio, method, para_set_mode, W, strict_, is_parallel, terminal, prob,
                                  increment_update_light, local);
    }
}

double getMold(const vector<vector<int>> &vec)
{ //求向量的模长
    int n = vec.size();
    double sum = 0.0;
    for (int i = 0; i < n; ++i)
    {
        int m = vec[i].size();
        for (int j = 0; j < m; ++j)
        {
            sum += vec[i][j] * vec[i][j];
        }
    }
    return sqrt(sum);
}

double getSimilarity(const vector<vector<int>> &lhs, const vector<vector<int>> &rhs)
{
    int n = lhs.size();
    if (n == rhs.size())
    {
        double tmp = 0.0; //内积
        for (int i = 0; i < n; ++i)
        {
            int m = lhs[i].size();
            if (m == rhs[i].size())
            {
                for (int j = 0; j < m; ++j)
                {
                    tmp += lhs[i][j] * rhs[i][j];
                }
            }
            else
            {
                return -1;
            }
        }
        return tmp / (getMold(lhs) * getMold(rhs));
    }
    else
    {
        return -1;
    }
}

Agent::Agent(int id, int px, int py, int r)
{
    index = id;
    pos_x = px;
    pos_y = py;
    if (guard.owns_lock())
    {
        guard.unlock();
        guard.release();
    }
    guard = unique_lock<mutex>(locks[pos_x][pos_y], defer_lock);
    guard.lock();
    sense_r = r;
    prior = -1;
}

Agent::Agent()
{
    index = -1;
    prior = -1;
}

void Agent::set_config(int id, int px, int py, int r)
{
    index = id;
    pos_x = px;
    pos_y = py;
    if (guard.owns_lock())
    {
        //        cout<<index<<" owns."<<endl;
        guard.unlock();
        //        guard.release();
    }
    //    else{
    //        cout<<index<<" doesn't own."<<endl;
    //        guard.release();
    //    }
    guard = unique_lock<mutex>(locks[pos_x][pos_y], defer_lock);
    guard.lock();
    sense_r = r;
}

void Agent::get_local_info()
{

    for (int i = pos_x - 2; i < pos_x + 3; i++)
    {
        for (int j = pos_y - 2; j < pos_y + 3; j++)
        {
            if (i >= 0 && i < center.lf.gr_scale[0] && j >= 0 && j < center.lf.gr_scale[1])
            {
                int a_id = -1;
                if (center.agent_poses[i][j] >= 0)
                {
                    a_id = center.agent_poses[i][j];
                    if (swarm[a_id].prior < prior)
                    {
                        this->conflicts.push_back(a_id);
                    }
                    else if (swarm[a_id].prior > prior && (i == pos_x - 1 || i == pos_x + 1 || i == pos_x) &&
                             (j == pos_y - 1 || j == pos_y + 1 || j == pos_y))
                    {
                        this->inferior_neighbors.push_back(a_id);
                    }
                }
            }
        }
    }
}

void Agent::get_best_action(int para_set_mode, int W, bool strict_, bool is_parallel, bool local)
{
    if (!is_parallel)
    {
        get_local_info();
    }
    if (center.lf.grids[pos_x][pos_y] == 0)
    {
        //        std::vector<double> light_is;
        //        if(center.info.ac_tar>10000000) {
        if (local)
        {
            std::vector<std::pair<double, double>> light_is;
            for (int i = pos_x - 1; i <= pos_x + 1 && i < center.lf.gr_scale[0]; i++)
            {
                for (int j = pos_y - 1; j <= pos_y + 1 && j < center.lf.gr_scale[1]; j++)
                {
                    if (i >= 0 && j >= 0)
                    {
                        if (center.lf.blue_light[i][j] >= center.lf.blue_light[pos_x][pos_y])
                        {
                            std::vector<int> v = {i, j};
                            pair<double, double> light = {center.lf.red_light[i][j], center.lf.blue_light[i][j]};

                            auto pos = lower_bound(light_is.begin(), light_is.end(), light, [](pair<double, double> lhs, pair<double, double> rhs) -> bool {
                                return lhs.first < rhs.first || (lhs.first == rhs.first && lhs.second > rhs.second);
                            });
                            //                        auto pos = lower_bound(light_is.begin(), light_is.end(), center.lf.blue_light[i][j]);
                            //                        auto pos = lower_bound(light_is.begin(), light_is.end(), center.lf.red_light[i][j]);
                            int i_pos = pos - light_is.begin();
                            //                        if(center.lf.red_light[i][j]<center.lf.red_light[pos_x][pos_y]) {
                            //                            light_is.insert(pos, center.lf.blue_light[i][j]);
                            //                            this->rand_bias.insert(this->rand_bias.begin() + i_pos, 1, v);
                            //                        }
                            light_is.insert(pos, light);
                            this->rand_bias.insert(this->rand_bias.begin() + i_pos, 1, v);
                        }
                        else
                        {
                            std::vector<int> v = {i, j};
                            this->rand_less.push_back(v);
                        }
                    }
                }
            }
        }
        else
        {
            std::vector<double> light_is;
            for (int i = pos_x - 1; i <= pos_x + 1 && i < center.lf.gr_scale[0]; i++)
            {
                for (int j = pos_y - 1; j <= pos_y + 1 && j < center.lf.gr_scale[1]; j++)
                {
                    if (i >= 0 && j >= 0)
                    {
                        if (center.lf.blue_light[i][j] >= center.lf.blue_light[pos_x][pos_y])
                        {
                            std::vector<int> v = {i, j};
                            auto pos = lower_bound(light_is.begin(), light_is.end(), center.lf.blue_light[i][j]);
                            int i_pos = pos - light_is.begin();
                            light_is.insert(pos, center.lf.blue_light[i][j]);
                            this->rand_bias.insert(this->rand_bias.begin() + i_pos, 1, v);
                        }
                        else
                        {
                            std::vector<int> v = {i, j};
                            this->rand_less.push_back(v);
                        }
                    }
                }
            }
        }
    }
    else
    {
        std::vector<std::pair<double, double>> light_is;
        double prob = 1;
        if (para_set_mode == 2 || para_set_mode == 3 || para_set_mode == 4)
        {
            srand((unsigned)time(0));
            prob = rand() / double(RAND_MAX);
            if (prob <= W)
            {
                strict_ = false;
            }
            else
            {
                strict_ = true;
            }
        }
        for (int i = pos_x - 1; i <= pos_x + 1 && i < center.lf.gr_scale[0]; i++)
        {
            for (int j = pos_y - 1; j <= pos_y + 1 && j < center.lf.gr_scale[1]; j++)
            {
                if (i >= 0 and j >= 0)
                {
                    if (((para_set_mode == 0 || para_set_mode == 1) && center.info.ac_tar > W) || ((para_set_mode == 2 || para_set_mode == 3 || para_set_mode == 4) && prob <= W))
                    {
                        bool cond = false;
                        if (strict_)
                        {
                            cond = center.lf.blue_light[i][j] >= center.lf.blue_light[pos_x][pos_y] &&
                                   center.lf.grids[i][j] == 1;
                        }
                        else
                        {
                            cond = center.lf.blue_light[i][j] >= center.lf.blue_light[pos_x][pos_y];
                        }
                        if (cond)
                        {
                            std::vector<int> v = {i, j};
                            pair<double, double> light = {center.lf.red_light[i][j], center.lf.blue_light[i][j]};

                            auto pos = lower_bound(light_is.begin(), light_is.end(), light, [](pair<double, double> lhs, pair<double, double> rhs) -> bool {
                                return lhs.second > rhs.second || (lhs.second == rhs.second && lhs.first < rhs.first);
                            });
                            int i_pos = pos - light_is.begin();
                            light_is.insert(pos, light);
                            this->rand_bias.insert(this->rand_bias.begin() + i_pos, 1, v);
                        }
                        else
                        {
                            if (strict_)
                            {
                                if (center.lf.grids[i][j] == 1)
                                {
                                    std::vector<int> v = {i, j};
                                    this->rand_less.push_back(v);
                                }
                            }
                            else
                            {
                                std::vector<int> v = {i, j};
                                this->rand_less.push_back(v);
                            }
                        }
                    }
                    else
                    {
                        if (center.lf.red_light[i][j] <= center.lf.red_light[pos_x][pos_y] &&
                            center.lf.grids[i][j] == 1)
                        {
                            std::vector<int> v = {i, j};
                            pair<double, double> light = {center.lf.red_light[i][j], center.lf.blue_light[i][j]};
                            auto pos = lower_bound(light_is.begin(), light_is.end(), light, [](pair<double, double> lhs, pair<double, double> rhs) -> bool {
                                return lhs.first < rhs.first;
                            });
                            int i_pos = pos - light_is.begin();
                            light_is.insert(pos, light);
                            this->rand_bias.insert(this->rand_bias.begin() + i_pos, 1, v);
                        }
                        else if (center.lf.grids[i][j] == 1)
                        {
                            std::vector<int> v = {i, j};
                            this->rand_less.push_back(v);
                        }
                    }
                }
            }
        }
    }
}

void Agent::get_stop_wait_action(int terminal, double prob)
{
    vector<vector<int>> surround;
    for (int i = pos_x - 1; i <= pos_x + 1; i++)
    {
        surround.emplace_back();
        for (int j = pos_y - 1; j <= pos_y + 1; j++)
        {
            surround[i - (pos_x - 1)].push_back(1);
        }
    }
    for (int i = 0; i < conflicts.size(); i++)
    {
        int cfl_x = center.nx_poses[conflicts[i]][0];
        int cfl_y = center.nx_poses[conflicts[i]][1];
        if (cfl_x - pos_x >= -1 and cfl_x - pos_x <= 1 and cfl_y - pos_y >= -1 and cfl_y - pos_y <= 1)
        {
            surround[1 + cfl_x - pos_x][1 + cfl_y - pos_y] = 0;
        }
    }
    for (int i = 0; i < inferior_neighbors.size(); i++)
    {
        int cfl_x = swarm[inferior_neighbors[i]].pos_x;
        int cfl_y = swarm[inferior_neighbors[i]].pos_y;
        if (cfl_x - pos_x >= -1 and cfl_x - pos_x <= 1 and cfl_y - pos_y >= -1 and cfl_y - pos_y <= 1)
        {
            surround[1 + cfl_x - pos_x][1 + cfl_y - pos_y] = 0;
        }
    }

    int next_x = pos_x;
    int next_y = pos_y;

    if (center.lf.grids[pos_x][pos_y] == 0)
    {
        for (int i = this->rand_bias.size() - 1; i >= 0; i--)
        {
            if (surround[1 + this->rand_bias[i][0] - pos_x][1 + this->rand_bias[i][1] - pos_y] == 1)
            {
                next_x = this->rand_bias[i][0];
                next_y = this->rand_bias[i][1];
                break;
            }
        }
    }
    else
    {
        for (int i = 0; i <= this->rand_bias.size() - 1; i++)
        {
            if (surround[1 + this->rand_bias[i][0] - pos_x][1 + this->rand_bias[i][1] - pos_y] == 1)
            {
                next_x = this->rand_bias[i][0];
                next_y = this->rand_bias[i][1];
                break;
            }
        }
    }

    if (next_x == pos_x && next_y == pos_y)
    {
        vector<vector<int>> tmp_less;
        if (rand_less.size() > 0)
        {
            for (int t = 0; t <= (rand_less.size() - 1); t++)
            {
                if (surround[1 + rand_less[t][0] - pos_x][1 + this->rand_less[t][1] - pos_y] == 1)
                {
                    int x = rand_less[t][0];
                    int y = rand_less[t][1];
                    vector<int> tmp = {x, y};
                    tmp_less.push_back(tmp);
                }
            }
        }
        double p = (float)rand() / RAND_MAX;
        if (tmp_less.size() > 0 && p < prob)
        {
            int rand_p = rand() % tmp_less.size();
            center.nx_poses[index][0] = tmp_less[rand_p][0];
            center.nx_poses[index][1] = tmp_less[rand_p][1];
        }
        else
        {
            center.nx_poses[index][0] = next_x;
            center.nx_poses[index][1] = next_y;
        }
    }
    else
    {
        center.nx_poses[index][0] = next_x;
        center.nx_poses[index][1] = next_y;
    }
}

void Agent::avoid_expel()
{
    bool stop = false;
    for (int i = 0; i < this->inferior_neighbors.size(); i++)
    {
        int next_x = center.nx_poses[index][0];
        int next_y = center.nx_poses[index][1];
        int cfl_x = center.nx_poses[this->inferior_neighbors[i]][0];
        int cfl_y = center.nx_poses[this->inferior_neighbors[i]][1];
        if (center.nx_poses[this->inferior_neighbors[i]][0] == swarm[this->inferior_neighbors[i]].pos_x &&
            center.nx_poses[this->inferior_neighbors[i]][1] == swarm[this->inferior_neighbors[i]].pos_y &&
            next_x == cfl_x && next_y == cfl_y)
        {
            stop = true;
            break;
        }
    }
    if (stop)
    {
        center.nx_poses[index][0] = pos_x;
        center.nx_poses[index][1] = pos_y;
    }
}

void Agent::get_no_conflict_action(int terminal)
{
    get_local_info();
    int next_x = pos_x;
    int next_y = pos_y;
    bool exist_op = false;
    std::vector<std::vector<int>> rand_pos;
    std::vector<std::vector<int>> rand_neg;
    if (center.lf.grids[pos_x][pos_y] == 0)
    {
        double max_blue = center.lf.blue_light[pos_x][pos_y];
        for (int i = pos_x - 1; i <= pos_x + 1; i++)
        {
            for (int j = pos_y - 1; j <= pos_y + 1; j++)
            {
                bool any_cfl = false;
                for (auto k = conflicts.begin(); k != conflicts.end(); k++)
                {
                    int cfl_x = center.nx_poses[*k][0];
                    int cfl_y = center.nx_poses[*k][1];
                    if (cfl_x == i and cfl_y == j)
                    {
                        any_cfl = true;
                        k = conflicts.erase(k);
                        //                        if(k == cfl_agents.end()) break;
                        break;
                    }
                }
                if (!any_cfl)
                {
                    if (center.lf.blue_light[i][j] > max_blue)
                    {
                        exist_op = true;
                        max_blue = center.lf.blue_light[i][j];
                        next_x = i;
                        next_y = j;
                        std::vector<int> v = {i, j};
                        rand_pos.push_back(v);
                    }
                    else if (center.lf.blue_light[i][j] == max_blue)
                    {
                        std::vector<int> v = {i, j};
                        rand_pos.push_back(v);
                    }
                    else
                    {
                        std::vector<int> v = {i, j};
                        rand_neg.push_back(v);
                    }
                }
            }
        }
    }
    else
    {
        double min_red = center.lf.red_light[pos_x][pos_y];
        for (int i = pos_x - 1; i <= pos_x + 1; i++)
        {
            for (int j = pos_y - 1; j <= pos_y + 1; j++)
            {
                bool any_cfl = false;
                for (auto k = conflicts.begin(); k != conflicts.end(); k++)
                {
                    int cfl_x = center.nx_poses[*k][0];
                    int cfl_y = center.nx_poses[*k][1];
                    if (cfl_x == i and cfl_y == j)
                    {
                        any_cfl = true;
                        k = conflicts.erase(k);
                        break;
                    }
                }
                if (!any_cfl)
                {
                    if (center.lf.red_light[i][j] < min_red && center.lf.grids[i][j] == 1)
                    {
                        exist_op = true;
                        min_red = center.lf.red_light[i][j];
                        next_x = i;
                        next_y = j;
                        std::vector<int> v = {i, j};
                        rand_pos.push_back(v);
                    }
                    else if (center.lf.red_light[i][j] == min_red && center.lf.grids[i][j] == 1)
                    {
                        std::vector<int> v = {i, j};
                        rand_pos.push_back(v);
                    }
                    else if (center.lf.red_light[i][j] > min_red && center.lf.grids[i][j] == 1)
                    {
                        std::vector<int> v = {i, j};
                        rand_neg.push_back(v);
                    }
                }
            }
        }
    }
    if (exist_op)
    {
        center.nx_poses[index][0] = next_x;
        center.nx_poses[index][1] = next_y;
    }
    else
    {
        srand((unsigned)time(0));
        double prob = rand() / double(RAND_MAX);
        if (rand_pos.size() > 0)
        {
            int rand_p = rand() % rand_pos.size();
            center.nx_poses[index][0] = rand_pos[rand_p][0];
            center.nx_poses[index][1] = rand_pos[rand_p][1];
        }
        else if (rand_neg.size() > 0 and prob <= 0.1)
        {
            int rand_p = rand() % rand_pos.size();
            center.nx_poses[index][0] = rand_neg[rand_p][0];
            center.nx_poses[index][1] = rand_neg[rand_p][1];
        }
        else
        {
            center.nx_poses[index][0] = pos_x;
            center.nx_poses[index][1] = pos_y;
        }
    }
}

// if multiple threads, before take actions, center.agent_poses must be cleared
void Agent::take_action()
{
    pos_x = center.nx_poses[index][0];
    pos_y = center.nx_poses[index][1];
    center.agent_poses[pos_x][pos_y] = index;
    center.nx_poses[index][0] = 0;
    center.nx_poses[index][1] = 0;
    this->conflicts.clear();
    this->inferior_neighbors.clear();
    this->rand_bias.clear();
    this->rand_less.clear();
    //local_view unordered_map释放空间,移动到新位置之后local_view中的值都要重新计算,原本的key下的值不清理直接替换有可能导致内存泄漏,而且local_view越来越大
}

void Agent::parallel_running(bool incremental, double decay_ratio, int method, int para_set_mode,
                             int W, bool strict_, bool is_parallel,
                             int terminal, double prob, bool increment_update_light, bool local)
{
    // update surrounding light field
    if (incremental)
    {
        center.multi_update_light_field(pos_x, pos_y, decay_ratio);
    }
    else
    {
        center.cal_sum_light(pos_x, pos_y, decay_ratio, method, local);
    }
    // get action priority queue
    get_best_action(para_set_mode, W, strict_, is_parallel, local);

    // conflict avoidance
    int next_x = pos_x;
    int next_y = pos_y;

    unique_lock<mutex> temp_guard;
    if (center.lf.grids[pos_x][pos_y] == 0)
    {
        if (not local)
        {
            for (int i = this->rand_bias.size() - 1; i >= 0; i--)
            {
                temp_guard = unique_lock<mutex>(locks[this->rand_bias[i][0]][this->rand_bias[i][1]], defer_lock);
                temp_guard.try_lock();
                if (temp_guard.owns_lock())
                {
                    next_x = this->rand_bias[i][0];
                    next_y = this->rand_bias[i][1];
                    center.agent_poses[pos_x][pos_y] = -1;
                    center.agent_poses[next_x][next_y] = index;
                    guard.unlock();
                    guard.swap(temp_guard);
                    break;
                }
                else
                {
                    continue;
                }
            }
        }
        else
        {
            for (int i = 0; i <= this->rand_bias.size() - 1; i++)
            {
                temp_guard = unique_lock<mutex>(locks[this->rand_bias[i][0]][this->rand_bias[i][1]], defer_lock);
                temp_guard.try_lock();
                if (temp_guard.owns_lock())
                {
                    next_x = this->rand_bias[i][0];
                    next_y = this->rand_bias[i][1];
                    center.agent_poses[pos_x][pos_y] = -1;
                    center.agent_poses[next_x][next_y] = index;
                    guard.unlock();
                    guard.swap(temp_guard);
                    break;
                }
                else
                {
                    continue;
                }
            }
        }
    }
    else
    {
        for (int i = 0; i <= this->rand_bias.size() - 1; i++)
        {
            temp_guard = unique_lock<mutex>(locks[this->rand_bias[i][0]][this->rand_bias[i][1]], defer_lock);
            temp_guard.try_lock();
            if (temp_guard.owns_lock())
            {
                next_x = this->rand_bias[i][0];
                next_y = this->rand_bias[i][1];
                center.agent_poses[pos_x][pos_y] = -1;
                center.agent_poses[next_x][next_y] = index;
                guard.unlock();
                guard.swap(temp_guard);
                break;
            }
            else
            {
                continue;
            }
        }
    }

    if (next_x == pos_x && next_y == pos_y)
    {
        double p = (float)rand() / RAND_MAX;
        if (p < prob)
        {
            if (rand_less.size() > 0)
            {
                default_random_engine generator{random_device{}()};
                shuffle(rand_less.begin(), rand_less.end(), generator);
                for (int t = 0; t <= (rand_less.size() - 1); t++)
                {
                    temp_guard = unique_lock<mutex>(locks[this->rand_less[t][0]][this->rand_less[t][1]], defer_lock);
                    temp_guard.try_lock();
                    if (temp_guard.owns_lock())
                    {
                        next_x = rand_less[t][0];
                        next_y = rand_less[t][1];
                        center.agent_poses[pos_x][pos_y] = -1;
                        center.agent_poses[next_x][next_y] = index;
                        guard.unlock();
                        guard.swap(temp_guard);
                        break;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
    }
    this->rand_bias.clear();
    this->rand_less.clear();

    //record new state(can be combined into get_stop_wait_action)
    {
        unique_lock<mutex> cloud_guard(cloud_mutex, defer_lock);
        cloud_guard.lock();
        if (cloud_guard.owns_lock())
        {
            if (increment_update_light)
            {
                center.multi_shared_update(pos_x, pos_y, next_x, next_y);
            }
            else
            {
                center.simple_update_tar(pos_x, pos_y, next_x, next_y);
            }
        }
        cloud_guard.unlock();
        cloud_guard.release();
    }

    //transmit to new positions and record new state
    pos_x = next_x;
    pos_y = next_y;
}

bool initialize_no_seed_agent_positions(int f, int min_i, int min_j, int shape_agent_num)
{
    int cnt = 4;
    bool is_enough = false;
    if (min_j - 3 < 4 or min_i < 4 or min_i >= width - 4)
    {
        cout << "Too narrow space for edge following." << endl;
        return is_enough;
    }

    for (int j = min_j - 3; j >= 4; j--)
    {
        //        swarm[cnt].set_config(min_i,j,2*sqrt(2));
        agent_init_pos_in_shapes[f].push_back({min_i, j});
        cnt++;
        if (cnt == shape_agent_num)
        {
            is_enough = true;
            break;
        }
    }
    int max_height = int(0.6 * height);
    vector<vector<int>> incre_options;
    if (not is_enough)
    {
        for (int i = min_i + 1; i < width - 4; i++)
        {
            for (int j = 4; j <= max_height; j++)
            {
                if (j > min_j - 3)
                {
                    bool is_valid = true;
                    for (int px = i - 2; px <= i + 2 && px < width - 1; px++)
                    {
                        for (int py = j - 2; py <= j + 2 && py < height - 1; py++)
                        {
                            //                            cout<<px<<" "<<py<<endl;
                            if (center.lf.grids[px][py] == 0)
                            {
                                is_valid = is_valid && true;
                            }
                            else
                            {
                                is_valid = is_valid && false;
                                break;
                            }
                        }
                        if (not is_valid)
                        {
                            break;
                        }
                    }
                    if (is_valid)
                    {
                        //                        swarm[cnt].set_config(i,j,2*sqrt(2));
                        agent_init_pos_in_shapes[f].push_back({i, j});
                        cnt++;
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    //                    swarm[cnt].set_config(i,j,2*sqrt(2));
                    agent_init_pos_in_shapes[f].push_back({i, j});
                    cnt++;
                }
                if (cnt == shape_agent_num)
                {
                    is_enough = true;
                    break;
                }
                if (j == max_height)
                {
                    incre_options.push_back({i, j});
                }
            }
            if (is_enough)
            {
                break;
            }
        }
    }
    if (not is_enough)
    {
        for (int i = min_i - 1; i >= 4; i--)
        {
            for (int j = 4; j <= max_height; j++)
            {
                if (j > min_j - 3)
                {
                    bool is_valid = true;
                    for (int px = i - 2; px <= i + 3 && px < width - 1; px++)
                    {
                        for (int py = j - 2; py <= j + 2 && py < height - 1; py++)
                        {
                            //                            cout<<px<<" "<<py<<endl;
                            if (center.lf.grids[px][py] == 0)
                            {
                                is_valid = is_valid && true;
                            }
                            else
                            {
                                is_valid = is_valid && false;
                                break;
                            }
                        }
                        if (not is_valid)
                        {
                            break;
                        }
                    }
                    if (is_valid)
                    {
                        //                        swarm[cnt].set_config(i,j,2*sqrt(2));
                        agent_init_pos_in_shapes[f].push_back({i, j});
                        cnt++;
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    //                    swarm[cnt].set_config(i,j,2*sqrt(2));
                    agent_init_pos_in_shapes[f].push_back({i, j});
                    cnt++;
                }
                if (cnt == shape_agent_num)
                {
                    is_enough = true;
                    break;
                }
                if (j == max_height)
                {
                    incre_options.push_back({i, j});
                }
            }
            if (is_enough)
            {
                break;
            }
        }
    }
    if (not is_enough)
    {
        for (int o = 0; o < incre_options.size(); o++)
        {
            int i = incre_options[o][0];
            for (int j = incre_options[o][1] + 1; j < height - 2; j++)
            {
                if (j > min_j - 3)
                {
                    bool is_valid = true;
                    for (int px = i - 2; px <= i + 2 && px < width - 1; px++)
                    {
                        for (int py = j - 2; py <= j + 2 && py < height - 1; py++)
                        {
                            //                            cout<<px<<" "<<py<<endl;
                            if (center.lf.grids[px][py] == 0)
                            {
                                is_valid = is_valid && true;
                            }
                            else
                            {
                                is_valid = is_valid && false;
                                break;
                            }
                        }
                        if (not is_valid)
                        {
                            break;
                        }
                    }
                    if (is_valid)
                    {
                        //                        swarm[cnt].set_config(i,j,2*sqrt(2));
                        agent_init_pos_in_shapes[f].push_back({i, j});
                        cnt++;
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    //                    swarm[cnt].set_config(i,j,2*sqrt(2));
                    agent_init_pos_in_shapes[f].push_back({i, j});
                    cnt++;
                }
                if (cnt == shape_agent_num)
                {
                    is_enough = true;
                    break;
                }
            }
            if (is_enough)
            {
                break;
            }
        }
    }

    if (not is_enough)
    {
        cout << "There isn't enough space for initialization." << endl;
    }
    return is_enough;
}

/*specific initialization*/
//int main() {
//    string dictionary, out_dict;
//    if (RUN_ENV == "WIN") {
//        dictionary =
//                "D:\\projects\\CLionProjects\\InitSettingGenerator\\tmp_edge_display\\" + to_string(width) + '_' + to_string(height);
//        out_dict = "D:\\projects\\CLionProjects\\AssemblyShape_C_Version\\exp";
//    } else if (RUN_ENV == "MAC") {
//        dictionary =
//                "/Users/chuwenjie/CLionProjects/InitSettingGenerator/tmp_edge_display/" + to_string(width) + '*' + to_string(height);
//        out_dict = "/Users/chuwenjie/CLionProjects/AssemblyShape/exp";
//    } else {
//        dictionary = "./exp/" + to_string(width) + '*' + to_string(height);
//        out_dict = "./exp";
//    }
//    DIR *dir;
//    struct dirent *ptr;
//    vector<string> name_posts;
//    vector<int> shape_nums;
//    vector<string> filelist;
//    vector<int> a_num_s;
//    const char *p = dictionary.c_str();
//    if ((dir = opendir(p)) == NULL) {
//        perror("Open dir error...");
//        exit(1);
//    }
//    while ((ptr = readdir(dir)) != NULL) {
//        if (string(ptr->d_name).compare(0, 4, "grid") == 0)    //file
//        {
//            string temp;
//            if (RUN_ENV == "WIN") {
//                temp = dictionary + '\\' + ptr->d_name;
//            } else {
//                temp = dictionary + '/' + ptr->d_name;
//            }
//            filelist.push_back(temp);
//            string _post = ptr->d_name;
//            int shape_num = _post[5] - '0';
//            shape_nums.push_back(shape_num);
//            int a_num = atoi(_post.substr(7, _post.size() - 11).c_str());
//            a_num_s.push_back(a_num);
//            _post = _post.substr(4, _post.size() - 4);
//            name_posts.push_back(_post);
//        }
//    }
//    closedir(dir);
//
//    //TODO: Update Args
//    vector<int> para_set_modes = {0};//0: man set; 1: feedback set; 2: Simulate Anneal
//    //Best W
//    //40
////    int W1_s[10] = {50, 50, 50, 50, 80, 40, width * height, 200, 50, 80};
//    //80
//    int W1_s[10] = {50, 300, 300, 300, 300, 180, width * height, 200, 300, 300};//strategy选择(>W1 1,else 2)
//    bool strict__s[10] = {true, true, true, false, true, true, true, false, true, true};
//    double decay_ratio_s[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//    double prob_s[10] = {0.2, 0.2, 0.5, 0.5, 0.2, 0.6, 0.5, 0.6, 0.2, 0.2};
//    bool increment_update_light_s[10] = {false, false, false, false, false, false, false, false, false, false};
//    int method_s[10] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
//
//    for (int w = 0; w < 1; w++) {
//        int para_set_mode = para_set_modes[0];
//        for (int f = 0; f < filelist.size(); f++) {
//            int shape_agent_num;
//            for (int k = 0; k < a_num_s[f]; k++) {
//                swarm.push_back(Agent());
//            }
//            srand(time(NULL));
//            int rand_more = 4;
//            if (a_mode == more_a) {
//                //如果是多,则为N+3~N+0.05N+3
//                for (int i = 0; i < rand_more + 3; i++) {
//                    swarm.push_back(Agent());
//                }
//            } else if (a_mode == equal_a) {
//                //如果是正好,由于seed robot中只有v3在目标内,所以至少为N+3
//                for (int i = 0; i < 3; i++) {
//                    swarm.push_back(Agent());
//                }
//            } else {
//                rand_more = rand_more - 3;
//                //如果是少,则为N+3-0.05N~N+3
//                if (rand_more > 0) {
//                    for (int i = 0; i < rand_more; i++) {
//                        swarm.erase(swarm.end());
//                    }
//                } else if (rand_more < 0) {
//                    for (int i = 0; i < -rand_more; i++) {
//                        swarm.push_back(Agent());
//                    }
//                }
//            }
//            shape_agent_num = swarm.size();
//
//            center = Cloud(shape_agent_num, width, height);
//            //read the grid environment
//            fstream infile;
//            infile.open(filelist[f], ios::in);
//            if (!infile) {
//                cout << "open failed" << endl;
//                exit(1);
//            }
//            int i = 0;
//            int j = height - 1;
//            int min_i = width;
//            int min_j = height;
//            while (!infile.eof() and j >= 0) {
//                i = 0;
//                while (!infile.eof() and i < width) {
//                    infile >> center.lf.grids[i][j];
//                    if (center.lf.grids[i][j] == 1) {
//                        if (j <= min_j) {
//                            if (j < min_j) {
//                                min_j = j;
//                                min_i = i;
//                            } else {
//                                if (i < min_i) {
//                                    min_i = i;
//                                }
//                            }
//                        }
//                    }
//                    i++;
//                }
//                j--;
//            }
//            infile.close();
//
//            vector<vector<int>> agent_init_pos_in_a_shape;
//            agent_init_pos_in_shapes.push_back(agent_init_pos_in_a_shape);
//            //initialize seed robot v0-v3, noting that only v3 is within the target shape,
//            //so the total number of agent for n-n shape is at least n+3 agent.
//            agent_init_pos_in_shapes[f].push_back({min_i, min_j - 1});
//            agent_init_pos_in_shapes[f].push_back({min_i + 1, min_j - 1});
//            agent_init_pos_in_shapes[f].push_back({min_i, min_j - 2});
//            agent_init_pos_in_shapes[f].push_back({min_i, min_j});
//            //初始化剩余agent位置
//            bool is_enough = initialize_no_seed_agent_positions(f, min_i, min_j, shape_agent_num);
//            if (not is_enough) {
//                cout << "Experiment failed!" << endl;
//                continue;
//            }
//
//            int exp_num = 20;
//            const int THREAD_NUM = TEMP_THREAD_NUM>shape_agent_num?shape_agent_num:TEMP_THREAD_NUM;
//
//            double exp_avg_iter = 0;
//            double exp_avg_iter_t = 0;
//            double exp_avg_similarity = 0;
//            int valid_exp = 0;
//            int minor_valid_exp = 0;
//            double minor_exp_avg_iter = 0;
//            double minor_exp_avg_iter_t = 0;
//            string out_arg;
//
//            if (RUN_ENV == "WIN") {
//                out_arg = out_dict + "\\args_" + to_string(w) + "_" + to_string(width) + "_" +
//                          to_string(height) + name_posts[f];
//            } else {
//                out_arg = out_dict + "/args_" + to_string(w) + "_" + to_string(width) + "_" +
//                          to_string(height) + name_posts[f];
//            }
//            ofstream outarg(out_arg, ios::app);
//
//            for (int e = 0; e < exp_num; e++) {
//                //在每轮实验开始初始化所有agent的位置
//                swarm[0].set_config(0,agent_init_pos_in_shapes[f][0][0], agent_init_pos_in_shapes[f][0][1], 2);
//                swarm[1].set_config(1,agent_init_pos_in_shapes[f][1][0], agent_init_pos_in_shapes[f][1][1], 2);
//                swarm[2].set_config(2, agent_init_pos_in_shapes[f][2][0], agent_init_pos_in_shapes[f][2][1], 2);
//                swarm[3].set_config(3, agent_init_pos_in_shapes[f][3][0], agent_init_pos_in_shapes[f][3][1], 2);
//                for (int s = 4; s < agent_init_pos_in_shapes[f].size(); s++) {
//                    swarm[s].set_config(s, agent_init_pos_in_shapes[f][s][0], agent_init_pos_in_shapes[f][s][1], 2);
//                }
//                for(int i=0; i<swarm.size();i++){
//                    center.agent_poses[swarm[i].pos_x][swarm[i].pos_y] = swarm[i].index;
//                }
//
//                //record the initialization
//                string out_name;
//                if (RUN_ENV == "WIN") {
//                    out_name = out_dict + "\\poses_" + to_string(w) + "_" + to_string(e) + "_" +
//                               to_string(width) + "_" + to_string(height) +
//                               name_posts[f];
//                } else {
//                    out_name = out_dict + "/poses_" + to_string(w) + "_" + to_string(e) + "_" +
//                               to_string(width) + "_" + to_string(height) +
//                               name_posts[f];
//                }
//                ofstream outfile(out_name, ios::app);
//                outfile << "arguments: " << width << ' ' << height << ' ' << shape_agent_num << ' ' << THREAD_NUM
//                        << endl;
//                outfile << "agent positions:" << endl;
//                for (int k = 0; k < shape_agent_num; k++) {
//                    if (k < shape_agent_num - 1) {
//                        outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
//                    } else {
//                        outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
//                    }
//                }
//                outfile << endl;
//
//                //arguments that will affect the formation process
//                //1+1:光源中心汇聚 2+2:驱散,根据相对位置体现为向内部驱赶/向凸出边缘驱赶 2+3:在随机性有利于打破少数分散点形成的相对震荡状态
//                //根据问题规模以及具体形状,可在不同的时间节点调控策略与优先级的组合
//                //对于极度分散的目标采用随机优先级即可,优先级的设定主要是针对filling目标,防止出现拥塞和整体左右摆动,
//                //在所有agent之间极度分散或目标极度分散的情况下啊无需考虑优先级
//
//                double W1 = 0;
//                if(para_set_mode==0) {
//                    W1 = W1_s[shape_nums[f]];//strategy选择(>W1 1,else 2)
//                }else if(para_set_mode==1){
//                    W1 = 0;
//                }else{
//                    W1 = 1;
//                }
//
//                //强制不离开区域内的策略有利于快速收敛
//                //但针对多目标时,该策略可能形成"路障",因此在多目标早期(中心汇聚阶段)可以放宽要求,在基本按照各图案需求完成中心汇聚分配之后,
//                //通过强制不离开解除边缘不稳定状态,加上驱散作用快速填补各图案边缘地区
//                bool strict_ = strict__s[shape_nums[f]];
//                //单目标中影响因素主要是距离,因此采用线性方法是可以的,并可以采用增量更新的方式
//                //多目标(包括离散线形)中必须综合考虑距离与不同目标中数量之间的平衡关系;实验证明采用inverse decay的形式能形成更好的feild
//                bool increment_update_light = increment_update_light_s[shape_nums[f]];
//                double decay_ratio = decay_ratio_s[shape_nums[f]];
//                // 0-5,linear & Man,linear & O,inverse & Man,inverse & O,inverse & improved Man,inverse square & O
//                int method = method_s[shape_nums[f]]; //only useful when increment_update_light=true
//                if (increment_update_light) {
//                    center.multi_shared_calculation();
//                } else {
//                    center.info.ac_tar = shape_agent_num-1;
//                }
//                //一定概率下不动为最优动作的agent可选择坏的动作
//                double prob = prob_s[shape_nums[f]];
//                bool local = false; //表征在agent未进入目标区域前除了blue_light还需不需要计算local内的red_light
//
//                //supplementary variables for recording & execution
//                vector<thread> threads;
//                //default_random_engine generator{random_device{}()};
//                int terminal = 0;
//                int minor_terminal = 0;
//                clock_t startT, endT;
//                vector<double> dec_times;
//                vector<int> ac_tar_decay;
//                ac_tar_decay.push_back(center.info.ac_tar);
//                vector<int> mv_agent_records;
//
//                while (center.info.ac_tar > rand_more+3 && terminal < 1000 and minor_terminal < 500) {
//                    center.info.mv_agent = 0;
//                    cout << center.info.ac_tar << endl;
//                    terminal += 1;
//                    startT = clock();
//                    int left = int(shape_agent_num % THREAD_NUM);
//                    int alloc = 0;
//                    int s_ids[THREAD_NUM];
//                    int e_ids[THREAD_NUM];
//                    srand(time(NULL));
//                    for (int k = 0; k < THREAD_NUM; k++) {
//                        s_ids[k] = -1, e_ids[k] = -1;
//                        if (left > alloc) {
//                            s_ids[k] = k * (int(shape_agent_num / THREAD_NUM) + 1);
//                            e_ids[k] = s_ids[k] + int(shape_agent_num / THREAD_NUM);
//                            alloc++;
//                        } else {
//                            s_ids[k] = alloc * (int(shape_agent_num / THREAD_NUM) + 1) +
//                                       (k - alloc) * int(shape_agent_num / THREAD_NUM);
//                            e_ids[k] = s_ids[k] + int(shape_agent_num / THREAD_NUM) - 1;
//                        }
//                    }
//
//                    bool is_parallel = true;
//                    for (int k = 0; k < THREAD_NUM; k++) {
//                        threads.emplace_back(parallel_swarms, k, s_ids[k], e_ids[k], increment_update_light,
//                                             decay_ratio,
//                                             method, para_set_mode, W1, strict_, is_parallel, terminal, prob, increment_update_light, local);
//                    }
//                    // 等待其他线程join
//                    for (int k = 0; k < THREAD_NUM; k++) {
//                        threads[k].join();
//                    }
//                    threads.clear();
//
//                    endT = clock();
//                    dec_times.push_back((double) (endT - startT));
//
//                    //record new positions for all agents
//                    outfile << "agent positions:" << endl;
//                    for (int k = 0; k < shape_agent_num; k++) {
//                        if (k < shape_agent_num - 1) {
//                            outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
//                        } else {
//                            outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
//                        }
//                    }
//                    outfile << endl;
//                    ac_tar_decay.push_back(center.info.ac_tar);
//                    if(para_set_mode == 2){
//                        W1 = double(center.info.ac_tar) / shape_agent_num;
//                    }else if(para_set_mode==3) {
//                        W1 = log((center.info.ac_tar+1))/log(shape_agent_num+1);
//                    }else if(para_set_mode == 4){
//                        W1 = 0.5*sin(double(center.info.ac_tar)/shape_agent_num*PI-PI*0.5)+0.5;
//                    }else if(para_set_mode == 1 && W1==0){
//                        int mm = ac_tar_decay.size()-10;
//                        int min_pos = max(0,mm);
//                        double avg_decay = double(ac_tar_decay[min_pos]-ac_tar_decay[ac_tar_decay.size()-1])/(ac_tar_decay.size()-1-min_pos);
//                        int cnt = 0;
//                        for(int l=min_pos;l<ac_tar_decay.size()-1;l++){
//                            if(ac_tar_decay[l]-ac_tar_decay[l+1]<=0){
//                                cnt += 1;
//                            }
//                        }
//                        if(avg_decay<=10 && cnt>2){
//                            W1 = center.info.ac_tar;
//                            cout<<"W1:"<<min_pos<<" "<<avg_decay<<" "<<W1<<endl;
////                            W1 = ac_tar_decay[min_pos];
//                        }
//                    }
//                    if (center.info.ac_tar <= rand_more+6) {
//                        minor_terminal += 1;
//                    }
//                    if (center.info.ac_tar <= rand_more+6 && minor_terminal > 2800) {
//                        increment_update_light = false;
//                    }
//                    if(center.info.ac_tar<=rand_more+13){
//                        local=false;
//                    }
//                    int mv_agent_num = center.info.mv_agent;
//                    mv_agent_records.push_back(mv_agent_num);
//                }
//                double avg_t = 0;
//                for (int k = 0; k < dec_times.size(); k++) {
//                    avg_t += dec_times[k];
//                }
//                double avg_iteration = avg_t / (dec_times.size() * CLOCKS_PER_SEC);
//                outfile.flush();
//                outfile.close();
//
//                vector<vector<int>> formed_shape;
//                for(int g=0;g<width;g++){
//                    formed_shape.push_back(vector<int>());
//                    for(int h=0;h<height;h++){
//                        if(center.agent_poses[g][h]>=0){
//                            formed_shape[g].push_back(1);
//                        }else{
//                            formed_shape[g].push_back(0);
//                        }
//                    }
//                }
//                double mse_similarity = getSimilarity(center.lf.grids,formed_shape);
//                exp_avg_similarity = exp_avg_similarity + mse_similarity;
//                outarg << "Experiment " << e << ":" << endl;
//                outarg << "The average decision time for each iteration is: " << avg_iteration << "s." << endl;
//                outarg << "Main: program exiting after " << terminal << " steps, and " << minor_terminal
//                       << " steps for the last 3 positions, the similarity is " <<mse_similarity<< endl;
//                outarg << "Decay line:";
//                for (int k = 0; k < ac_tar_decay.size(); k++) {
//                    outarg << ' ' << ac_tar_decay[k];
//                }
//                outarg<<endl;
//                outarg << "The number of moving agent line:";
//                for (int k = 0; k < mv_agent_records.size(); k++) {
//                    outarg << ' ' << mv_agent_records[k];
//                }
//                ac_tar_decay.clear();
//                mv_agent_records.clear();
//                outarg << endl;
//                outarg << "W1: " << W1 << ", is_increment: " << increment_update_light_s[f] << ", is_strict: "
//                       << strict_ << ", decay_ratio: " << decay_ratio << ", method: " << method
//                       << ", prob: " << prob << "." << endl;
//
//                cout << "End an experiment! Clearing..." << endl;
//                vector<int> line(height, -1);
//                vector<vector<int>> array(width);
//                for (int k = 0; k < array.size(); k++) {
//                    array[k].assign(line.begin(), line.end());
//                }
//                center.agent_poses.swap(array);
//                for (int k = 0; k < swarm.size(); k++) {
//                    if (swarm[k].guard.owns_lock()) {
//                        swarm[k].guard.unlock();
//                    }
//                }
////                cout<<"here1"<<endl;
//
//                if (center.info.ac_tar == rand_more+3) {
//                    valid_exp += 1;
//                    exp_avg_iter += terminal;
//                    exp_avg_iter_t += avg_iteration;
//                }
//                if (center.info.ac_tar <= rand_more+6) {
//                    minor_valid_exp += 1;
//                    minor_exp_avg_iter = minor_exp_avg_iter + (terminal - minor_terminal);
//                    minor_exp_avg_iter_t += avg_iteration;
//                }
////                cout<<"here2"<<endl;
//            }
//            exp_avg_iter = exp_avg_iter / valid_exp;
//            exp_avg_iter_t = exp_avg_iter_t / valid_exp;
//            exp_avg_similarity = exp_avg_similarity / exp_num;
//            minor_exp_avg_iter = minor_exp_avg_iter / minor_valid_exp;
//            minor_exp_avg_iter_t = minor_exp_avg_iter_t / minor_valid_exp;
//            cout << f << ": exp_avg_iter=" << exp_avg_iter << ", exp_avg_iter_time=" << exp_avg_iter_t << endl;
//            outarg << endl << endl;
//            outarg << "After " << exp_num << " experiments, for shape " << f <<", avg_similarity="<<exp_avg_similarity
//                   <<", "<<valid_exp<<" experiments success, exp_avg_iter=" << exp_avg_iter
//                   << ", exp_avg_iter_time=" << exp_avg_iter_t << ", minor_exp_avg_iter=" << minor_exp_avg_iter
//                   << ", minor_exp_avg_iter_time=" << minor_exp_avg_iter_t << endl;
//            outarg.flush();
//            outarg.close();
//            swarm.clear();
//        }
//    }
//    return 0;
//}

/*random initilization*/
int main(int argc, char **argv)
{
    string dictionary, out_dict;
    if (RUN_ENV == "WIN")
    {
        //        dictionary =
        //                "D:\\projects\\CLionProjects\\InitSettingGenerator\\exp\\" + to_string(width) + '_' + to_string(height);
        dictionary = ".\\data\\" + to_string(width) + '_' + to_string(height);
        out_dict = ".\\data\\method";
    }
    else if (RUN_ENV == "MAC")
    {
        dictionary =
            "/Users/chuwenjie/CLionProjects/InitSettingGenerator/supp_display/" + to_string(width) + '_' + to_string(height);
        out_dict = "/Users/chuwenjie/CLionProjects/AssemblyShape/exp";
    }
    else
    {
        dictionary = "./exp/" + to_string(width) + '*' + to_string(height);
        out_dict = "./exp";
    }
    //    string work_root = argv[1];
    //    string dictionary, out_dict;
    //    if (RUN_ENV == "WIN") {
    //        dictionary = work_root +
    //                     "\\inputs\\"+ argv[2]+"\\"+ to_string(width) + '_' + to_string(height);
    //        out_dict = work_root + "\\outputs\\" + argv[3] +"\\"+to_string(width) + '_' + to_string(height);
    //    } else {
    //        dictionary = work_root + "/inputs/" + argv[2]+ "/" + to_string(width) + '_' + to_string(height);
    //        out_dict = work_root + "/outputs/" + argv[3]+ "/" + to_string(width) + '_' + to_string(height);
    //    }

    DIR *dir;
    struct dirent *ptr;
    vector<string> name_posts;
    vector<int> shape_nums;
    vector<string> filelist;
    vector<int> a_num_s;
    const char *p = dictionary.c_str();
    if ((dir = opendir(p)) == NULL)
    {
        cout << p << endl;
        perror("Open dir error...");
        exit(1);
    }
    while ((ptr = readdir(dir)) != NULL)
    {
        if (string(ptr->d_name).compare(0, 4, "grid") == 0) //file
        {
            string temp;
            if (RUN_ENV == "WIN")
            {
                temp = dictionary + '\\' + ptr->d_name;
            }
            else
            {
                temp = dictionary + '/' + ptr->d_name;
            }
            filelist.push_back(temp);
            string _post = ptr->d_name;
            int shape_num = _post[5] - '0';
            shape_nums.push_back(shape_num);
            int a_num = atoi(_post.substr(7, _post.size() - 11).c_str());
            a_num_s.push_back(a_num);
            _post = _post.substr(4, _post.size() - 4);
            name_posts.push_back(_post);
        }
    }
    closedir(dir);

    //TODO: Update Args
    vector<int> para_set_modes = {0}; //0: man set; 1: feedback set; 2: Simulate Anneal
    //Best W
    int W1_s[10] = {300, 600, 700, 310, 580, 190, width * height, 200, 450, 580}; //strategy选择(>W1 1,else 2)
                                                                                  //95    int W1_s[10] = {200, 350, 350, 350, 300, 180, width * height, 200, 300, 350};//strategy选择(>W1 1,else 2)
                                                                                  //90    int W1_s[10] = {200, 300, 350, 350, 300, 180, width * height, 200, 300, 350};//strategy选择(>W1 1,else 2)
                                                                                  //80    int W1_s[10] = {50, 300, 300, 300, 300, 180, width * height, 200, 300, 300};//strategy选择(>W1 1,else 2)
                                                                                  //70    int W1_s[10] = {50, 250, 200, 190, 250, 170, width * height, 150, 200, 200};//strategy选择(>W1 1,else 2)
                                                                                  //60    int W1_s[10] = {50, 250, 200, 150, 250, 150, width * height, 150, 200, 200};//strategy选择(>W1 1,else 2)
                                                                                  //16    int W1_s[10] = {10, 10, 10, 20, 10, 20, width * height, 20, 10, 25};//strategy选择(>W1 1,else 2)
                                                                                  //25    int W1_s[10] = {30, 50, 40, 50, 35, 20, width * height, 20, 30, 40};//strategy选择(>W1 1,else 2)
                                                                                  //40    int W1_s[10] = {60, 100, 75, 90, 50, 20, width * height, 20, 90, 50};//strategy选择(>W1 1,else 2)

    //    int choice_W[10][7]={{0,50,80,150,300,600,width*height},
    //                     {0,50,150,300,450,600,width*height},
    //                     {0,50,150,300,450,600,width*height},
    //                     {0,50,150,300,450,600,width*height},
    //                     {0,10,20,300,450,600,width*height},
    //                     {0,150,300,450,600,700,width*height},
    //                     {0,100,150,width*height,width*height,width*height,width*height},
    //                     {0,50,100,200,250,450,width*height},
    //                     {0,50,150,300,450,600,width*height},
    //                     {0,50,150,300,450,600,width*height}};

    bool strict__s[10] = {true, true, true, false, true, false, true, false, true, true};
    double decay_ratio_s[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    //    double prob_s[10] = {0.2, 0.2, 0.5, 0.5, 0.2, 0.6, 0.5, 0.6, 0.2, 0.2};
    double prob_s[10] = {0.2, 0.2, 0.5, 0.2, 0.2, 0.6, 0.5, 0.6, 0.2, 0.2};
    bool increment_update_light_s[10] = {false, false, false, false, false, false, false, false, false, false};
    int method_s[10] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4};

    //    for (int w = 0; w < para_set_modes.size(); w++) {
    //        int para_set_mode = para_set_modes[w];
    int para_set_mode = para_set_modes[0];
    //        int W1_s[10]= {choice_W[0][w],choice_W[1][w],choice_W[2][w],choice_W[3][w],choice_W[4][w],
    //                       choice_W[5][w],choice_W[6][w],choice_W[7][w],choice_W[8][w],choice_W[9][w]};
    for (int f = 0; f < filelist.size(); f++)
    {
        center = Cloud(a_num_s[f], width, height);
        const int THREAD_NUM = TEMP_THREAD_NUM > a_num_s[f] ? a_num_s[f] : TEMP_THREAD_NUM;
        for (int k = 0; k < a_num_s[f]; k++)
        {
            swarm.push_back(Agent());
        }

        int exp_num = 20;
        double exp_avg_iter = 0;
        double exp_avg_iter_t = 0;
        double exp_avg_similarity = 0;
        int valid_exp = 0;
        int minor_valid_exp = 0;
        double minor_exp_avg_iter = 0;
        double minor_exp_avg_iter_t = 0;
        string out_arg;
        if (RUN_ENV == "WIN")
        {
            out_arg = out_dict + "\\args_" + to_string(0) + "_" + to_string(width) + "_" +
                      to_string(height) + name_posts[f];
        }
        else
        {
            out_arg = out_dict + "/args_" + to_string(0) + "_" + to_string(width) + "_" +
                      to_string(height) + name_posts[f];
        }
        ofstream outarg(out_arg, ios::app);

        //read the grid environment
        fstream infile;
        infile.open(filelist[f], ios::in);
        if (!infile)
        {
            cout << "open failed" << endl;
            exit(1);
        }
        //                int i = 0;
        //                vector<int> valid_l;
        //                while (!infile.eof() and i < width) {
        //                    int j = 0;
        //                    while (j < height) {
        //                        infile >> center.lf.grids[i][j];
        //                        if (center.lf.grids[i][j] == 0) {
        //                            valid_l.push_back(i * height + j);
        //                        }
        //                        j++;
        //                    }
        //                    i++;
        //                }
        int i = 0;
        int j = height - 1;
        vector<int> valid_l;
        while (!infile.eof() and j >= 0)
        {
            i = 0;
            while (!infile.eof() and i < width)
            {
                infile >> center.lf.grids[i][j];
                if (center.lf.grids[i][j] == 0)
                {
                    valid_l.push_back(i * height + j);
                }
                i++;
            }
            j--;
        }
        infile.close();
        //                cout<<"swarm init"<<endl;
        //initialize the positions of agents
        int max_size = valid_l.size();
        srand((unsigned)time(NULL));
        for (int r = 0; r < a_num_s[f]; r++)
        {
            int rand_p = rand() % max_size;
            int px = (int)valid_l[rand_p] / height;
            int py = (int)valid_l[rand_p] % height;
            swarm[r].set_config(r, px, py, 2);
            center.agent_poses[px][py] = r;
            int tmp = valid_l[max_size - 1];
            valid_l[max_size - 1] = valid_l[rand_p];
            valid_l[rand_p] = tmp;
            max_size--;
        }
        //                cout<<"swarm refresh"<<endl;
        //record the initialization
        string out_name;
        if (RUN_ENV == "WIN")
        {
            out_name = out_dict + "\\poses_" + to_string(0) + "_" + to_string(0) + "_" +
                       to_string(width) + "_" + to_string(height) +
                       name_posts[f];
        }
        else
        {
            out_name = out_dict + "/poses_" + to_string(0) + "_" + to_string(0) + "_" +
                       to_string(width) + "_" + to_string(height) +
                       name_posts[f];
        }
        ofstream outfile(out_name, ios::app);
        outfile << "arguments: " << width << ' ' << height << ' ' << a_num_s[f] << ' ' << THREAD_NUM
                << endl;
        outfile << "agent positions:" << endl;
        for (int k = 0; k < a_num_s[f]; k++)
        {
            if (k < a_num_s[f] - 1)
            {
                outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
            }
            else
            {
                outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
            }
        }
        outfile << endl;

        //arguments that will affect the formation process
        //1+1:光源中心汇聚 2+2:驱散,根据相对位置体现为向内部驱赶/向凸出边缘驱赶 2+3:在随机性有利于打破少数分散点形成的相对震荡状态
        //根据问题规模以及具体形状,可在不同的时间节点调控策略与优先级的组合
        //对于极度分散的目标采用随机优先级即可,优先级的设定主要是针对filling目标,防止出现拥塞和整体左右摆动,
        //在所有agent之间极度分散或目标极度分散的情况下啊无需考虑优先级

        double W1 = 0;
        if (para_set_mode == 0)
        {
            W1 = W1_s[shape_nums[f]]; //strategy选择(>W1 1,else 2)
        }
        else if (para_set_mode == 1)
        {
            W1 = 0;
        }
        else
        {
            W1 = 1;
        }

        //强制不离开区域内的策略有利于快速收敛
        //但针对多目标时,该策略可能形成"路障",因此在多目标早期(中心汇聚阶段)可以放宽要求,在基本按照各图案需求完成中心汇聚分配之后,
        //通过强制不离开解除边缘不稳定状态,加上驱散作用快速填补各图案边缘地区
        bool strict_ = strict__s[shape_nums[f]];
        //单目标中影响因素主要是距离,因此采用线性方法是可以的,并可以采用增量更新的方式
        //多目标(包括离散线形)中必须综合考虑距离与不同目标中数量之间的平衡关系;实验证明采用inverse decay的形式能形成更好的feild
        bool increment_update_light = increment_update_light_s[shape_nums[f]];
        double decay_ratio = decay_ratio_s[shape_nums[f]];
        // 0-5,linear & Man,linear & O,inverse & Man,inverse & O,inverse & improved Man,inverse square & O
        int method = method_s[shape_nums[f]]; //only useful when increment_update_light=true
        if (increment_update_light)
        {
            center.multi_shared_calculation();
        }
        else
        {
            center.info.ac_tar = a_num_s[f];
        }
        //一定概率下不动为最优动作的agent可选择坏的动作
        double prob = prob_s[shape_nums[f]];
        bool local = false; //表征在agent未进入目标区域前除了blue_light还需不需要计算local内的red_light

        //supplementary variables for recording & execution
        vector<thread> threads;
        //default_random_engine generator{random_device{}()};
        int terminal = 0;
        int minor_terminal = 0;
        clock_t startT, endT;
        vector<double> dec_times;

        vector<int> ac_tar_decay;
        ac_tar_decay.push_back(center.info.ac_tar);
        vector<int> mv_agent_records;
        //                cout << "At beginning: "<<center.info.ac_tar << endl;
        //                while (center.info.ac_tar > 0 && terminal < 6000 and minor_terminal < 4000) {
        while (center.info.ac_tar > 0 && terminal < 1000 and minor_terminal < 500)
        {
            center.info.mv_agent = 0;
            cout << center.info.ac_tar << endl;
            terminal += 1;
            startT = clock();
            int left = int(a_num_s[f] % THREAD_NUM);
            int alloc = 0;
            int s_ids[THREAD_NUM];
            int e_ids[THREAD_NUM];
            srand(time(NULL));
            for (int k = 0; k < THREAD_NUM; k++)
            {
                s_ids[k] = -1, e_ids[k] = -1;
                if (left > alloc)
                {
                    s_ids[k] = k * (int(a_num_s[f] / THREAD_NUM) + 1);
                    e_ids[k] = s_ids[k] + int(a_num_s[f] / THREAD_NUM);
                    alloc++;
                }
                else
                {
                    s_ids[k] = alloc * (int(a_num_s[f] / THREAD_NUM) + 1) +
                               (k - alloc) * int(a_num_s[f] / THREAD_NUM);
                    e_ids[k] = s_ids[k] + int(a_num_s[f] / THREAD_NUM) - 1;
                }
            }

            bool is_parallel = true;
            for (int k = 0; k < THREAD_NUM; k++)
            {
                threads.emplace_back(parallel_swarms, k, s_ids[k], e_ids[k], increment_update_light,
                                     decay_ratio,
                                     method, para_set_mode, W1, strict_, is_parallel, terminal, prob, increment_update_light, local);
            }
            // 等待其他线程join
            for (int k = 0; k < THREAD_NUM; k++)
            {
                threads[k].join();
            }
            threads.clear();

            endT = clock();
            dec_times.push_back((double)(endT - startT));

            //record new positions for all agents
            outfile << "agent positions:" << endl;
            for (int k = 0; k < a_num_s[f]; k++)
            {
                if (k < a_num_s[f] - 1)
                {
                    outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
                }
                else
                {
                    outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
                }
            }
            outfile << endl;
            ac_tar_decay.push_back(center.info.ac_tar);
            if (para_set_mode == 2)
            {
                W1 = double(center.info.ac_tar) / a_num_s[f];
            }
            else if (para_set_mode == 3)
            {
                //                        W1 = sqrt(double(center.info.ac_tar) / a_num_s[f]);
                W1 = log((center.info.ac_tar + 1)) / log(a_num_s[f] + 1);
            }
            else if (para_set_mode == 4)
            {
                W1 = 0.5 * sin(double(center.info.ac_tar) / a_num_s[f] * PI - PI * 0.5) + 0.5;
            }
            else if (para_set_mode == 1 && W1 == 0)
            {
                int mm = ac_tar_decay.size() - 10;
                int min_pos = max(0, mm);
                double avg_decay = double(ac_tar_decay[min_pos] - ac_tar_decay[ac_tar_decay.size() - 1]) / (ac_tar_decay.size() - 1 - min_pos);
                int cnt = 0;
                for (int l = min_pos; l < ac_tar_decay.size() - 1; l++)
                {
                    if (ac_tar_decay[l] - ac_tar_decay[l + 1] <= 0)
                    {
                        cnt += 1;
                    }
                }
                if (avg_decay <= 10 && cnt > 2)
                {
                    W1 = center.info.ac_tar;
                    cout << "W1:" << min_pos << " " << avg_decay << " " << W1 << endl;
                    //                            W1 = ac_tar_decay[min_pos];
                }
            }
            if (center.info.ac_tar <= 3)
            {
                minor_terminal += 1;
            }
            if (center.info.ac_tar <= 3 && minor_terminal > 2800)
            {
                increment_update_light = false;
            }
            if (center.info.ac_tar <= 10)
            {
                local = false;
            }
            int mv_agent_num = center.info.mv_agent;
            mv_agent_records.push_back(mv_agent_num);
        }
        double avg_t = 0;
        for (int k = 0; k < dec_times.size(); k++)
        {
            avg_t += dec_times[k];
        }
        double avg_iteration = avg_t / (dec_times.size() * CLOCKS_PER_SEC);
        outfile.flush();
        outfile.close();

        vector<vector<int>> formed_shape;
        for (int g = 0; g < width; g++)
        {
            formed_shape.push_back(vector<int>());
            for (int h = 0; h < height; h++)
            {
                if (center.agent_poses[g][h] >= 0)
                {
                    formed_shape[g].push_back(1);
                }
                else
                {
                    formed_shape[g].push_back(0);
                }
            }
        }
        double mse_similarity = getSimilarity(center.lf.grids, formed_shape);
        exp_avg_similarity = exp_avg_similarity + mse_similarity;
        outarg << "Experiment " << 0 << ":" << endl;
        outarg << "The average decision time for each iteration is: " << avg_iteration << "s." << endl;
        outarg << "Main: program exiting after " << terminal << " steps, and " << minor_terminal
               << " steps for the last 3 positions, the similarity is " << mse_similarity << endl;
        outarg << "Decay line:";
        for (int k = 0; k < ac_tar_decay.size(); k++)
        {
            outarg << ' ' << ac_tar_decay[k];
        }
        outarg << endl;
        outarg << "The number of moving agent line:";
        for (int k = 0; k < mv_agent_records.size(); k++)
        {
            outarg << ' ' << mv_agent_records[k];
        }
        ac_tar_decay.clear();
        mv_agent_records.clear();
        outarg << endl;
        outarg << "W1: " << W1 << ", is_increment: " << increment_update_light_s[f] << ", is_strict: "
               << strict_ << ", decay_ratio: " << decay_ratio << ", method: " << method
               << ", prob: " << prob << "." << endl;

        cout << "End an experiment! Clearing..." << endl;
        vector<int> line(height, -1);
        vector<vector<int>> array(width);
        for (int k = 0; k < array.size(); k++)
        {
            array[k].assign(line.begin(), line.end());
        }
        center.agent_poses.swap(array);
        for (int k = 0; k < swarm.size(); k++)
        {
            if (swarm[k].guard.owns_lock())
            {
                swarm[k].guard.unlock();
            }
        }
        //                cout<<"here1"<<endl;

        if (center.info.ac_tar == 0)
        {
            valid_exp += 1;
            exp_avg_iter += terminal;
            exp_avg_iter_t += avg_iteration;
        }
        if (center.info.ac_tar <= 3)
        {
            minor_valid_exp += 1;
            minor_exp_avg_iter = minor_exp_avg_iter + (terminal - minor_terminal);
            minor_exp_avg_iter_t += avg_iteration;
        }
        //                cout<<"here2"<<endl;

        exp_avg_iter = exp_avg_iter / valid_exp;
        exp_avg_iter_t = exp_avg_iter_t / valid_exp;
        exp_avg_similarity = exp_avg_similarity / exp_num;
        minor_exp_avg_iter = minor_exp_avg_iter / minor_valid_exp;
        minor_exp_avg_iter_t = minor_exp_avg_iter_t / minor_valid_exp;
        cout << f << ": exp_avg_iter=" << exp_avg_iter << ", exp_avg_iter_time=" << exp_avg_iter_t << endl;
        outarg << endl
               << endl;
        outarg << "After " << exp_num << " experiments, for shape " << f << ", avg_similarity=" << exp_avg_similarity
               << ", " << valid_exp << " experiments success, exp_avg_iter=" << exp_avg_iter
               << ", exp_avg_iter_time=" << exp_avg_iter_t << ", minor_exp_avg_iter=" << minor_exp_avg_iter
               << ", minor_exp_avg_iter_time=" << minor_exp_avg_iter_t << endl;
        outarg.flush();
        outarg.close();
        swarm.clear();
    }
    return 0;
}