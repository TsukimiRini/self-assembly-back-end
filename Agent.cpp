////
//// Created by 褚文杰 on 2019/10/21.
////
//
//#include "Agent.h"
//
////using namespace std;
//
//int a_num = 113;
//int dim = 2;
//int dec_m = 0;
//const int width = 40;
//const int height = 40;
//Cloud center;
//Agent * swarm;
//
////extern Agent *swarm;
//
//Agent::Agent(int id, int px, int py, int r) {
//    index = id;
//    pos_x = px;
//    pos_y = py;
//    sense_r = r;
//    prior = -1;
//}
//
//Agent::Agent() {
//    index = -1;
//    prior = -1;
//}
//
//void Agent::set_config(int id, int px, int py, int r) {
//    index = id;
//    pos_x = px;
//    pos_y = py;
//    sense_r = r;
//}
//
//std::vector<int> Agent::get_local_info() {
//    std::vector<int> cfl_agents;
//    for (int i = pos_x - 2; i < pos_x + 3; i++) {
//        for (int j = pos_y - 2; j < pos_y + 3; j++) {
//            if (0 <= i < center.lf.gr_scale[0] && 0 <= j < center.lf.gr_scale[1]) {
//                int a_id = -1;
//                if (center.agent_poses[i][j] >= 0) {
//                    a_id = center.agent_poses[i][j];
//                    if ((swarm+a_id)->prior < prior) {
//                        cfl_agents.push_back(a_id);
//                    }
//                }
//                if (i >= pos_x - 1 && i <= pos_x + 1 && j >= pos_y - 1 && j <= pos_y + 1) {
//                    KEY p = KEY(i, j);
//                    local_view[p] = std::tuple<bool, int, double, double>(center.lf.grids[i][j], a_id, center.lf.blue_light[i][j],
//                                                                     center.lf.red_light[i][j]);
//                }
//            }
//        }
//    }
//    return cfl_agents;
//}
//
//void Agent::get_best_action(std::vector<int> &conflicts, int terminal, bool bias) {
//    std::vector<int> cfl_agents = get_local_info();
//    int next_x = pos_x;
//    int next_y = pos_y;
//    std::vector<std::vector<int>> rand_bias;
//    if(center.lf.grids[pos_x][pos_y]==0) {
//        double max_blue = center.lf.blue_light[pos_x][pos_y];
//        for (int i = pos_x - 1; i <= pos_x + 1;i++ ) {
//            for (int j = pos_y - 1;j <= pos_y + 1; j++ ) {
//                if(center.lf.blue_light[i][j]>max_blue){
//                    max_blue = center.lf.blue_light[i][j];
//                    next_x = i;
//                    next_y = j;
//                    if(bias) {
//                        std::vector<int> v = {i, j};
//                        rand_bias.push_back(v);
//                    }
//                }
//            }
//        }
//    }else{
//        double min_red = center.lf.red_light[pos_x][pos_y];
//        for (int i = pos_x-1;i<=pos_x+1;i++){
//            for(int j=pos_y-1;j<=pos_y+1;j++){
//                if(center.lf.red_light[i][j]<min_red && center.lf.grids[i][j]==1){
//                    min_red = center.lf.red_light[i][j];
//                    next_x = i;
//                    next_y = j;
//                    if(bias) {
//                        std::vector<int> v = {i, j};
//                        rand_bias.push_back(v);
//                    }
//                }
//            }
//        }
//
//    }
//    if(bias) {
//        srand((unsigned) time(0));
//        double prob = rand() / double(RAND_MAX);
//        if (prob <= 0.1 and rand_bias.size() > 0) {
//            int rand_p = (rand_bias.size() - 1) * rand() / (double(RAND_MAX) + 1);
//            center.nx_poses[index][0] = rand_bias[rand_p][0];
//            center.nx_poses[index][1] = rand_bias[rand_p][1];
//        } else {
//            center.nx_poses[index][0] = next_x;
//            center.nx_poses[index][1] = next_y;
//        }
//    }else{
//        center.nx_poses[index][0] = next_x;
//        center.nx_poses[index][1] = next_y;
//    }
//    conflicts.assign(cfl_agents.begin(),cfl_agents.end());
//}
//
//void Agent::get_stop_wait_action(std::vector<int> & conflicts, int terminal) {
//    bool stop = false;
//    for(int i=0;i<conflicts.size();i++){
//        int next_x = center.nx_poses[index][0];
//        int next_y = center.nx_poses[index][1];
//        int cfl_x = center.nx_poses[conflicts[i]][0];
//        int cfl_y = center.nx_poses[conflicts[i]][1];
//        if(next_x == cfl_x && next_y == cfl_y){
//            stop = true;
//            break;
//        }
//    }
//    if(stop){
//        center.nx_poses[index][0] = pos_x;
//        center.nx_poses[index][1] = pos_y;
//    }
//}
//
//void Agent::get_no_conflict_action(int terminal) {
//    std::vector<int> cfl_agents = get_local_info();
//    int next_x = pos_x;
//    int next_y = pos_y;
//    bool exist_op = false;
//    std::vector<std::vector<int>> rand_pos;
//    std::vector<std::vector<int>> rand_neg;
//    if(center.lf.grids[pos_x][pos_y]==0) {
//        double max_blue = center.lf.blue_light[pos_x][pos_y];
//        for (int i = pos_x - 1;i <= pos_x + 1; i++) {
//            for (int j = pos_y - 1;j <= pos_y + 1; j++) {
//                bool any_cfl = false;
//                for(auto k =cfl_agents.begin();k!=cfl_agents.end();k++){
//                    int cfl_x = center.nx_poses[*k][0];
//                    int cfl_y = center.nx_poses[*k][1];
//                    if (cfl_x == i and cfl_y == j){
//                        any_cfl = true;
//                        k=cfl_agents.erase(k);
////                        if(k == cfl_agents.end()) break;
//                        break;
//                    }
//                }
//                if(!any_cfl) {
//                    if (center.lf.blue_light[i][j] > max_blue) {
//                        exist_op = true;
//                        max_blue = center.lf.blue_light[i][j];
//                        next_x = i;
//                        next_y = j;
//                        std::vector<int> v = {i, j};
//                        rand_pos.push_back(v);
//                    } else if (center.lf.blue_light[i][j] == max_blue) {
//                        std::vector<int> v = {i, j};
//                        rand_pos.push_back(v);
//                    } else {
//                        std::vector<int> v = {i, j};
//                        rand_neg.push_back(v);
//                    }
//                }
//            }
//        }
//    }else{
//        double min_red = center.lf.red_light[pos_x][pos_y];
//        for (int i = pos_x-1;i<=pos_x+1;i++){
//            for(int j=pos_y-1;j<=pos_y+1;j++){
//                bool any_cfl = false;
//                for(auto k =cfl_agents.begin();k!=cfl_agents.end();k++){
//                    int cfl_x = center.nx_poses[*k][0];
//                    int cfl_y = center.nx_poses[*k][1];
//                    if (cfl_x == i and cfl_y == j){
//                        any_cfl = true;
//                        k=cfl_agents.erase(k);
////                        if(k == cfl_agents.end()) break;
//                        break;
//                    }
//                }
//                if(!any_cfl) {
//                    if (center.lf.red_light[i][j] < min_red && center.lf.grids[i][j] == 1) {
//                        exist_op = true;
//                        min_red = center.lf.red_light[i][j];
//                        next_x = i;
//                        next_y = j;
//                        std::vector<int> v = {i, j};
//                        rand_pos.push_back(v);
//                    } else if (center.lf.red_light[i][j] == min_red && center.lf.grids[i][j] == 1) {
//                        std::vector<int> v = {i, j};
//                        rand_pos.push_back(v);
//                    } else if (center.lf.red_light[i][j] > min_red && center.lf.grids[i][j] == 1) {
//                        std::vector<int> v = {i, j};
//                        rand_neg.push_back(v);
//                    }
//                }
//            }
//        }
//
//    }
//    if(exist_op){
//        center.nx_poses[index][0] = next_x;
//        center.nx_poses[index][1] = next_y;
//    }else{
//        srand((unsigned) time(0));
//        double prob = rand() / double(RAND_MAX);
//        if (rand_pos.size() > 0) {
//            int rand_p = (rand_pos.size() - 1) * rand() / (double(RAND_MAX) + 1);
//            center.nx_poses[index][0] = rand_pos[rand_p][0];
//            center.nx_poses[index][1] = rand_pos[rand_p][1];
//        } else if (rand_neg.size()>0 and prob<=0.1) {
//            int rand_p = (rand_neg.size() - 1) * rand() / (double(RAND_MAX) + 1);
//            center.nx_poses[index][0] = rand_neg[rand_p][0];
//            center.nx_poses[index][1] = rand_neg[rand_p][1];
//        }else{
//            center.nx_poses[index][0] = pos_x;
//            center.nx_poses[index][1] = pos_y;
//        }
//    }
//}
//
//// if multiple threads, before take actions, center.agent_poses must be cleared
//void Agent::take_action(bool is_mul) {
//    if(!is_mul){
//        center.agent_poses[pos_x][pos_y] = -1;
//    }
//    pos_x = center.nx_poses[index][0];
//    pos_y = center.nx_poses[index][1];
//    center.agent_poses[pos_x][pos_y] = index;
//    center.nx_poses[index][0]=0;
//    center.nx_poses[index][1]=0;
//}