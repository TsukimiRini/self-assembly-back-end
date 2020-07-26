//
// Created by 褚文杰 on 2019/10/22.
//

#ifndef ASSEMBLYSHAPE_CLOUD_H
#define ASSEMBLYSHAPE_CLOUD_H

#include<vector>
#include<cmath>
#include<iostream>
#include "LightField.h"


struct FieldInfo{
public:
    int ac_tar;
    int mv_agent;
    int sum_x;
    int sum_y;
    std::vector<int> x_sums;
    std::vector<int> y_sums;
    std::vector<std::vector<int>> x_split_pairs;
    std::vector<std::vector<int>> y_split_pairs;
    std::vector<int> x_sums_cnt;
    std::vector<int> y_sums_cnt;
    std::vector<std::vector<int>> x_split_pairs_cnt;
    std::vector<std::vector<int>> y_split_pairs_cnt;
    int a_sum_x;
    int a_sum_y;
    std::vector<int> ax_sums;
    std::vector<int> ay_sums;
    std::vector<std::vector<int>> ax_split_pairs;
    std::vector<std::vector<int>> ay_split_pairs;
    std::vector<int> ax_sums_cnt;
    std::vector<int> ay_sums_cnt;
    std::vector<std::vector<int>> ax_split_pairs_cnt;
    std::vector<std::vector<int>> ay_split_pairs_cnt;
//    int terminal_record;
};

class Cloud {
public:
    LightField lf;
    FieldInfo info;
    int a_num;
    std::vector<std::vector<int>> agent_poses;
    std::vector<std::vector<int>> nx_poses;
    Cloud(int num, int w, int h);
    Cloud();
    void multi_update_light_field(int px, int py,double decay_ratio);
    void multi_shared_calculation();
    void multi_shared_update(int fx, int fy, int tx, int ty);
    void cal_sum_light(int px,int py,double decay_ratio,int method, bool local=false);
    double cal_decay_light(int fx,int fy,int tx,int ty,double decay_ratio,int method);
    void simple_update_tar(int fx,int fy,int tx,int ty);
};


#endif //ASSEMBLYSHAPE_CLOUD_H
