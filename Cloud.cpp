//
// Created by 褚文杰 on 2019/10/22.
//

#include "Cloud.h"

using namespace std;

Cloud::Cloud() = default;

Cloud::Cloud(int num, int w, int h) {
    lf.init_field(w, h);
    a_num = num;
    for (int x = 0; x < w; x++) {
        agent_poses.emplace_back();
        for (int y = 0; y < h; y++) {
            agent_poses[x].push_back(-1);
        }
    }
    for (int a = 0; a < a_num; a++) {
        nx_poses.emplace_back(2, 0);
    }
}

void Cloud::multi_shared_calculation() {
    //在所有iteration开始之前一次性计算
    int min_x = lf.gr_scale[0];
    int min_y = lf.gr_scale[1];
    int sum_y = 0;//所有tar的y值加和
    int sum_x = 0;//所有tar的x值加和
    int a_sum_y = 0;
    int a_sum_x = 0;
    int ac_tar = 0;//未被占用的tar的数量,ac_agent=ac_tar
    vector<vector<int>> y_split_pairs_cnt;//特定y分割下下侧所有tar数量,上侧所有tar数量
    vector<vector<int>> y_split_pairs;//特定y分割下下侧所有y值加和,上侧所有y值加和
    vector<vector<int>> ay_split_pairs_cnt;//特定y分割下下侧所有不在tar内agent的数量,上侧不在tar内agent的数量
    vector<vector<int>> ay_split_pairs;//特定y分割下下侧所有不在tar内agent的y值加和,上侧不在tar内agent的y值加和
    vector<int> y_sums;//特定y下一行的tar的y加和
    vector<int> y_sums_cnt;//特定y下一行的tar数
    vector<int> ay_sums;//特定y下一行的非tar内agent的y加和
    vector<int> ay_sums_cnt;//特定y下一行的非tar内agent数
    vector<vector<int>> x_split_pairs_cnt;//特定x分割下左侧所有tar数量,右侧所有tar数量
    vector<vector<int>> x_split_pairs;//特定x分割下左侧所有x值加和,右侧所有x值加和
    vector<vector<int>> ax_split_pairs_cnt;//特定x分割下左侧所有不在tar内agent的数量,右侧所有不在tar内agent的数量
    vector<vector<int>> ax_split_pairs;//特定x分割下左侧不在tar内agent的x值加和,右侧不在tar内agent的x值加和
    vector<int> x_sums;//特定x下一列的x加和
    vector<int> x_sums_cnt;//特定x下一列的tar数
    vector<int> ax_sums;//特定x下一列非tar内agent的x加和
    vector<int> ax_sums_cnt;//特定x下一列非tar内agent的tar数
    for (int i = 0; i < lf.gr_scale[0]; i++) {
        x_sums.push_back(0);
        x_sums_cnt.push_back(0);
        ax_sums.push_back(0);
        ax_sums_cnt.push_back(0);
        for (int j = 0; j < lf.gr_scale[1]; j++) {
            if (i == 0) {
                y_sums.push_back(0);
                y_sums_cnt.push_back(0);
                ay_sums.push_back(0);
                ay_sums_cnt.push_back(0);
            }
            if (lf.grids[i][j] == 1) {
                if (min_x > i) {
                    min_x = i;
                }
                if (min_y > j) {
                    min_y = j;
                }
                sum_y += j;
                sum_x += i;
                x_sums[i] += i;
                x_sums_cnt[i] += 1;
                y_sums[j] += j;
                y_sums_cnt[j] += 1;
                ac_tar += 1;
            }else if(agent_poses[i][j]!=-1){
                a_sum_x += i;
                a_sum_y += j;
                ax_sums[i] += i;
                ax_sums_cnt[i] += 1;
                ay_sums[j] += j;
                ay_sums_cnt[j] += 1;
            }
        }
    }
    for (int i = 0; i < lf.gr_scale[0]; i++) {
        if (i==0) {
            vector<int> pair={0,sum_x};
            vector<int> cnt;
            if(min_x>0) {
                cnt.push_back(0);
                cnt.push_back(ac_tar);
            }else{
                cnt.push_back(0);
                cnt.push_back(ac_tar-x_sums_cnt[0]);
            }
            x_split_pairs.push_back(pair);
            x_split_pairs_cnt.push_back(cnt);
        } else {
            vector<int> pair = {x_split_pairs[i - 1][0] + x_sums[i - 1], x_split_pairs[i - 1][1] - x_sums[i]};
            vector<int> cnt = {x_split_pairs_cnt[i - 1][0] + x_sums_cnt[i - 1],
                               x_split_pairs_cnt[i - 1][1] - x_sums_cnt[i]};
            x_split_pairs.push_back(pair);
            x_split_pairs_cnt.push_back(cnt);
        }
        if (i==0) {
            vector<int> pair={0,a_sum_x};
            vector<int> cnt={0,ac_tar-ax_sums_cnt[0]};
            ax_split_pairs.push_back(pair);
            ax_split_pairs_cnt.push_back(cnt);
        } else {
            vector<int> pair = {ax_split_pairs[i - 1][0] + ax_sums[i - 1], ax_split_pairs[i - 1][1] - ax_sums[i]};
            vector<int> cnt = {ax_split_pairs_cnt[i - 1][0] + ax_sums_cnt[i - 1],
                               ax_split_pairs_cnt[i - 1][1] - ax_sums_cnt[i]};
            ax_split_pairs.push_back(pair);
            ax_split_pairs_cnt.push_back(cnt);
        }
    }
    for (int j = 0; j < lf.gr_scale[1]; j++) {
        if (j == 0) {
            vector<int> pair = {0, sum_y};
            vector<int> cnt;
            if(min_y>0) {
                cnt.push_back(0);
                cnt.push_back(ac_tar);
            }else{
                cnt.push_back(0);
                cnt.push_back(ac_tar-y_sums_cnt[0]);
            }
            y_split_pairs.push_back(pair);
            y_split_pairs_cnt.push_back(cnt);
        } else {
            vector<int> pair = {y_split_pairs[j - 1][0] + y_sums[j - 1], y_split_pairs[j - 1][1] - y_sums[j]};
            vector<int> cnt = {y_split_pairs_cnt[j - 1][0] + y_sums_cnt[j - 1],
                               y_split_pairs_cnt[j - 1][1] - y_sums_cnt[j]};
            y_split_pairs.push_back(pair);
            y_split_pairs_cnt.push_back(cnt);
        }
        if (j == 0) {
            vector<int> pair={0,a_sum_y};
            vector<int> cnt={0,ac_tar-ay_sums_cnt[0]};
            ay_split_pairs.push_back(pair);
            ay_split_pairs_cnt.push_back(cnt);
        } else {
            vector<int> pair = {ay_split_pairs[j - 1][0] + ay_sums[j - 1], ay_split_pairs[j - 1][1] - ay_sums[j]};
            vector<int> cnt = {ay_split_pairs_cnt[j - 1][0] + ay_sums_cnt[j - 1],
                               ay_split_pairs_cnt[j - 1][1] - ay_sums_cnt[j]};
            ay_split_pairs.push_back(pair);
            ay_split_pairs_cnt.push_back(cnt);
        }
    }
    info.ac_tar = ac_tar;
    info.sum_x = sum_x;
    info.sum_y = sum_y;
    info.x_sums.assign(x_sums.begin(), x_sums.end());
    info.y_sums.assign(y_sums.begin(), y_sums.end());
    info.x_split_pairs.assign(x_split_pairs.begin(), x_split_pairs.end());
    info.y_split_pairs.assign(y_split_pairs.begin(), y_split_pairs.end());
    info.x_sums_cnt.assign(x_sums_cnt.begin(), x_sums_cnt.end());
    info.y_sums_cnt.assign(y_sums_cnt.begin(), y_sums_cnt.end());
    info.x_split_pairs_cnt.assign(x_split_pairs_cnt.begin(), x_split_pairs_cnt.end());
    info.y_split_pairs_cnt.assign(y_split_pairs_cnt.begin(), y_split_pairs_cnt.end());
    info.a_sum_x = a_sum_x;
    info.a_sum_y = a_sum_y;
    info.ax_sums.assign(ax_sums.begin(), ax_sums.end());
    info.ay_sums.assign(ay_sums.begin(), ay_sums.end());
    info.ax_split_pairs.assign(ax_split_pairs.begin(), ax_split_pairs.end());
    info.ay_split_pairs.assign(ay_split_pairs.begin(), ay_split_pairs.end());
    info.ax_sums_cnt.assign(ax_sums_cnt.begin(), ax_sums_cnt.end());
    info.ay_sums_cnt.assign(ay_sums_cnt.begin(), ay_sums_cnt.end());
    info.ax_split_pairs_cnt.assign(ax_split_pairs_cnt.begin(), ax_split_pairs_cnt.end());
    info.ay_split_pairs_cnt.assign(ay_split_pairs_cnt.begin(), ay_split_pairs_cnt.end());
}

void Cloud::multi_shared_update(int fx, int fy, int tx, int ty) {
    //在新的iteration开始之前同步更新(线性,非多线程)
    if(not(fx==tx && fy==ty)){
        info.mv_agent += 1;
    }
    if (lf.grids[tx][ty] == 1) {
        info.ac_tar -= 1;
        info.sum_x -= tx;
        info.sum_y -= ty;
        info.x_sums[tx] -= tx;
        info.x_sums_cnt[tx] -= 1;
        info.y_sums[ty] -= ty;
        info.y_sums_cnt[ty] -= 1;
        for (int i = 0; i < lf.gr_scale[0]; i++) {
            if (i < tx) {
                info.x_split_pairs[i][1] -= tx;
                info.x_split_pairs_cnt[i][1] -= 1;
            } else if (i > tx) {
                info.x_split_pairs[i][0] -= tx;
                info.x_split_pairs_cnt[i][0] -= 1;
            }
        }
        for (int j = 0; j < lf.gr_scale[1]; j++) {
            if (j < ty) {
                info.y_split_pairs[j][1] -= ty;
                info.y_split_pairs_cnt[j][1] -= 1;
            } else if (j > ty) {
                info.y_split_pairs[j][0] -= ty;
                info.y_split_pairs_cnt[j][0] -= 1;
            }
        }
        if (lf.grids[fx][fy] == 1) {
            info.ac_tar += 1;
            info.sum_x = info.sum_x + fx;
            info.sum_y = info.sum_y + fy;
            info.x_sums[fx] += fx;
            info.x_sums_cnt[fx] += 1;
            info.y_sums[fy] += fy;
            info.y_sums_cnt[fy] += 1;
            for (int i = 0; i < lf.gr_scale[0]; i++) {
                if (i < fx) {
                    info.x_split_pairs[i][1] += fx;
                    info.x_split_pairs_cnt[i][1] += 1;
                } else if (i > fx) {
                    info.x_split_pairs[i][0] += fx;
                    info.x_split_pairs_cnt[i][0] += 1;
                }
            }
            for (int j = 0; j < lf.gr_scale[1]; j++) {
                if (j < fy) {
                    info.y_split_pairs[j][1] += fy;
                    info.y_split_pairs_cnt[j][1] += 1;
                } else if (j > fy) {
                    info.y_split_pairs[j][0] += fy;
                    info.y_split_pairs_cnt[j][0] += 1;
                }
            }
        }else{
            info.a_sum_x = info.a_sum_x - fx;
            info.a_sum_y = info.a_sum_y - fy;
            info.ax_sums[fx] -= fx;
            info.ax_sums_cnt[fx] -= 1;
            info.ay_sums[fy] -= fy;
            info.ay_sums_cnt[fy] -= 1;
            for (int i = 0; i < lf.gr_scale[0]; i++) {
                if (i < fx) {
                    info.ax_split_pairs[i][1] -= fx;
                    info.ax_split_pairs_cnt[i][1] -= 1;
                } else if (i > fx) {
                    info.ax_split_pairs[i][0] -= fx;
                    info.ax_split_pairs_cnt[i][0] -= 1;
                }
            }
            for (int j = 0; j < lf.gr_scale[1]; j++) {
                if (j < fy) {
                    info.ay_split_pairs[j][1] -= fy;
                    info.ay_split_pairs_cnt[j][1] -= 1;
                } else if (j > fy) {
                    info.ay_split_pairs[j][0] -= fy;
                    info.ay_split_pairs_cnt[j][0] -= 1;
                }
            }
        }
    }else{
        if(lf.grids[fx][fy] == 0) {
            info.a_sum_x = info.a_sum_x - fx + tx;
            info.a_sum_y = info.a_sum_y - fy + ty;
            info.ax_sums[fx] -= fx;
            info.ax_sums[tx] += tx;
            info.ax_sums_cnt[fx] -= 1;
            info.ax_sums_cnt[tx] += 1;
            info.ay_sums[fy] -= fy;
            info.ay_sums[ty] += ty;
            info.ay_sums_cnt[fy] -= 1;
            info.ay_sums_cnt[ty] += 1;
            int minx = fx < tx ? fx : tx;
            int maxx = fx > tx ? fx : tx;
            int miny = fy < ty ? fy : ty;
            int maxy = fy > ty ? fy : ty;
            for (int i = 0; i < lf.gr_scale[0]; i++) {
                if (i < minx) {
                    info.ax_split_pairs[i][1] = info.ax_split_pairs[i][1] - fx + tx;
                } else if (i > maxx) {
                    info.ax_split_pairs[i][0] = info.ax_split_pairs[i][0] - fx + tx;
                } else {
                    if (fx > tx) {
                        info.ax_split_pairs[fx][0] += tx;
                        info.ax_split_pairs_cnt[fx][0] += 1;
                        info.ax_split_pairs[tx][1] -= fx;
                        info.ax_split_pairs_cnt[tx][1] -= 1;
                    } else if (fx < tx) {
                        info.ax_split_pairs[fx][1] += tx;
                        info.ax_split_pairs_cnt[fx][1] += 1;
                        info.ax_split_pairs[tx][0] -= fx;
                        info.ax_split_pairs_cnt[tx][0] -= 1;
                    }
                }
            }
            for (int j = 0; j < lf.gr_scale[1]; j++) {
                if (j < miny) {
                    info.ay_split_pairs[j][1] = info.ay_split_pairs[j][1] - fy + ty;
                } else if (j > maxy) {
                    info.ay_split_pairs[j][0] = info.ay_split_pairs[j][0] - fy + ty;
                } else {
                    if (fy > ty) {
                        info.ay_split_pairs[fy][0] += ty;
                        info.ay_split_pairs_cnt[fy][0] += 1;
                        info.ay_split_pairs[ty][1] -= fy;
                        info.ay_split_pairs_cnt[ty][1] -= 1;
                    } else if (fy < ty) {
                        info.ay_split_pairs[fy][1] += ty;
                        info.ay_split_pairs_cnt[fy][1] += 1;
                        info.ay_split_pairs[ty][0] -= fy;
                        info.ay_split_pairs_cnt[ty][0] -= 1;
                    }
                }
            }
        }else{
            info.ac_tar+=1;
            info.sum_x = info.sum_x + fx;
            info.sum_y = info.sum_y + fy;
            info.x_sums[fx] += fx;
            info.x_sums_cnt[fx] += 1;
            info.y_sums[fy] += fy;
            info.y_sums_cnt[fy] += 1;
            for (int i = 0; i < lf.gr_scale[0]; i++) {
                if (i < fx) {
                    info.x_split_pairs[i][1] += fx;
                    info.x_split_pairs_cnt[i][1] += 1;
                } else if (i > fx) {
                    info.x_split_pairs[i][0] += fx;
                    info.x_split_pairs_cnt[i][0] += 1;
                }
            }
            for (int j = 0; j < lf.gr_scale[1]; j++) {
                if (j < fy) {
                    info.y_split_pairs[j][1] += fy;
                    info.y_split_pairs_cnt[j][1] += 1;
                } else if (j > fy) {
                    info.y_split_pairs[j][0] += fy;
                    info.y_split_pairs_cnt[j][0] += 1;
                }
            }
            info.a_sum_x = info.a_sum_x + tx;
            info.a_sum_y = info.a_sum_y + ty;
            info.ax_sums[tx] += fx;
            info.ax_sums_cnt[tx] += 1;
            info.ay_sums[ty] += fy;
            info.ay_sums_cnt[ty] += 1;
            for (int i = 0; i < lf.gr_scale[0]; i++) {
                if (i < tx) {
                    info.ax_split_pairs[i][1] += tx;
                    info.ax_split_pairs_cnt[i][1] += 1;
                } else if (i > tx) {
                    info.ax_split_pairs[i][0] += tx;
                    info.ax_split_pairs_cnt[i][0] += 1;
                }
            }
            for (int j = 0; j < lf.gr_scale[1]; j++) {
                if (j < ty) {
                    info.ay_split_pairs[j][1] += ty;
                    info.ay_split_pairs_cnt[j][1] += 1;
                } else if (j > ty) {
                    info.ay_split_pairs[j][0] += fy;
                    info.ay_split_pairs_cnt[j][0] += 1;
                }
            }



        }
    }
}

void Cloud::simple_update_tar(int fx,int fy,int tx,int ty){
    //在新的iteration开始之前同步更新(线性,非多线程)
    if (lf.grids[tx][ty] == 1) {
        info.ac_tar -= 1;
        if (lf.grids[fx][fy] == 1) {
            info.ac_tar += 1;
        }
    }else{
        if(lf.grids[fx][fy]==1){
            info.ac_tar +=1;
        }
    }
    if(not(fx==tx && fy==ty)){
        info.mv_agent +=1;
    }
}

void Cloud::multi_update_light_field(int px, int py, double decay_ratio) {
    //多线程并行;仅适用与线性递减(decay_md=0)
    for (int i = px - 1; i <= px + 1 && i<lf.gr_scale[0]; i++) {
        for (int j = py - 1; j <= py + 1 && j<lf.gr_scale[1]; j++){
            if(i>=0 && j>=0) {
                if (lf.grids[px][py] == 0) {
                    lf.blue_light[i][j] = info.ac_tar * 1000 -
                                          decay_ratio *
                                          ((info.x_split_pairs[i][1] - info.x_split_pairs_cnt[i][1] * i)
                                           + (info.x_split_pairs_cnt[i][0] * i - info.x_split_pairs[i][0])
                                           + (info.y_split_pairs[j][1] - info.y_split_pairs_cnt[j][1] * j)
                                           +
                                           (info.y_split_pairs_cnt[j][0] * j - info.y_split_pairs[j][0]));
                } else {
                    lf.blue_light[i][j] = info.ac_tar * 1000 -
                                          decay_ratio *
                                          ((info.x_split_pairs[i][1] - info.x_split_pairs_cnt[i][1] * i)
                                           + (info.x_split_pairs_cnt[i][0] * i - info.x_split_pairs[i][0])
                                           + (info.y_split_pairs[j][1] - info.y_split_pairs_cnt[j][1] * j)
                                           +
                                           (info.y_split_pairs_cnt[j][0] * j - info.y_split_pairs[j][0]));
                    lf.red_light[i][j] = info.ac_tar * 1000 -
                                         decay_ratio *
                                         ((info.ax_split_pairs[i][1] - info.ax_split_pairs_cnt[i][1] * i)
                                          + (info.ax_split_pairs_cnt[i][0] * i - info.ax_split_pairs[i][0])
                                          + (info.ay_split_pairs[j][1] - info.ay_split_pairs_cnt[j][1] * j)
                                          +
                                          (info.ay_split_pairs_cnt[j][0] * j - info.ay_split_pairs[j][0]));
                }
            }
        }
    }
}

void Cloud::cal_sum_light(int px, int py, double decay_ratio, int method, bool local) {
    for (int i = px - 1; i <= px + 1 && i<lf.gr_scale[0]; i++) {
        for (int j = py - 1; j <= py + 1 && j < lf.gr_scale[1]; j++) {
            if (i >= 0 && j >= 0) {
                if (lf.grids[px][py] == 0) {
                    double result=0;
                    for(int k=0;k<lf.gr_scale[0];k++){
                        for(int t=0;t<lf.gr_scale[1];t++){
                            if(lf.grids[k][t]==1 && agent_poses[k][t]==-1){
                                double res = cal_decay_light(k,t,i,j,decay_ratio,method);
                                result = result+res;
                            }
                        }
                    }
                    lf.blue_light[i][j]=result;
                    if(local) {
                        double r_res = 0;
                        for (int k = px - 10; k <= px + 10 && k < lf.gr_scale[0]; k++) {
                            for (int t = py - 10; t <= py + 10 && t < lf.gr_scale[1]; t++) {
                                if (k >= 0 && t >= 0 && agent_poses[k][t] >= 0) {
                                    double res = cal_decay_light(k, t, i, j, decay_ratio, method);
                                    r_res = r_res + res;
                                }
                            }
                        }
                        lf.red_light[i][j] = r_res;
                    }
                }else{
                    double b_res = 0;
                    double r_res = 0;
                    for(int k=0;k<lf.gr_scale[0];k++){
                        for(int t=0;t<lf.gr_scale[1];t++){
                            if(lf.grids[k][t]==1 && agent_poses[k][t]==-1){
                                double res = cal_decay_light(k,t,i,j,decay_ratio,method);
                                b_res = b_res+res;
                            }
                            if(lf.grids[k][t]==0 && agent_poses[k][t]>=0){
                                double res = cal_decay_light(k,t,i,j,decay_ratio,method);
                                r_res = r_res+res;
                            }
                        }
                    }
                    lf.blue_light[i][j]=b_res;
                    lf.red_light[i][j]=r_res;
                }
            }
        }
    }
}

double Cloud::cal_decay_light(int fx, int fy, int tx, int ty, double decay_ratio, int method) {
    double result=0.0;
    if(method==0){
        //linear & Man
        result = 1000-(fabs(fx-tx)+fabs(fy-ty))*decay_ratio;
    }else if(method==1){
        //linear & O
        result = 1000-sqrt(pow(fx-tx,2)+pow(fy-ty,2))*decay_ratio;
    }else if(method==2){
        //inverse & Man
        result = 1000/(1+fabs(fx-tx)+fabs(fy-ty));
    }else if(method==3){
        //inverse & O
        result = 1000/(1+sqrt(pow(fx-tx,2)+pow(fy-ty,2)));
    }else if(method==4){
        //inverse & improved Man
        result=1000/(1+max(fabs(fx - tx), fabs(fy - ty)));
    }else if(method==5){
        //inverse square & O
        result=1000/(1+pow(fx-tx,2)+pow(fy-ty,2));
    }else if(method==6){
        int cnt = (max(fabs(fx - tx), fabs(fy - ty))*2+1)*4-4;
        if(cnt>0)
            result=1000/cnt;
        else
            result=1000;
    }
    return result;
}

//double Cloud::multi_update_light_field(int px, int py, double decay_ratio) {
//    //多线程并行;仅适用与线性递减(decay_md=0)
//    for (int i = px - 1; i <= px + 1; i++) {
//        for (int j = py - 1; j <= py + 1; j++) {
//            if (0 <= i <= info.min_x and 0 <= j <= info.min_y) {
//                lf.blue_light[i][j]= info.ac_tar * 1000 - decay_ratio * (info.sum_x + info.sum_y - info.ac_tar * (px + py));
//            } else if (0 <= i <= info.min_x and info.min_y < j < info.max_y) {
//                lf.blue_light[i][j]= info.ac_tar * 1000 - decay_ratio * (info.sum_x - info.ac_tar * px + (info.y_split_pairs[py][1] -
//                                                                                            info.y_split_pairs_cnt[py][1] *
//                                                                                            py) +
//                                                           (info.y_split_pairs_cnt[py][0] * py -
//                                                            info.y_split_pairs[py][0]));
//            } else if (0 <= i <= info.min_x and info.max_y <= j < lf.gr_scale[1]) {
//                lf.blue_light[i][j]= info.ac_tar * 1000 -
//                       decay_ratio * (info.sum_x - info.ac_tar * px + info.ac_tar * py - info.sum_y);
//            } else if (info.min_x < i < info.max_x and 0 <= j <= info.min_y) {
//                lf.blue_light[i][j]= info.ac_tar * 1000 - decay_ratio * ((info.x_split_pairs[px][1] -
//                                                            info.x_split_pairs_cnt[px][1] *
//                                                            px) +
//                                                           (info.x_split_pairs_cnt[px][0] * px -
//                                                            info.x_split_pairs[px][0]) + info.sum_y - info.ac_tar * py);
//            } else if (info.min_x < i < info.max_x and info.min_y < j < info.max_y) {
//                lf.blue_light[i][j]= info.ac_tar * 1000 - decay_ratio * ((info.x_split_pairs[px][1] -
//                                                            info.x_split_pairs_cnt[px][1] *
//                                                            px) +
//                                                           (info.x_split_pairs_cnt[px][0] * px -
//                                                            info.x_split_pairs[px][0]) + (info.y_split_pairs[py][1] -
//                                                                                          info.y_split_pairs_cnt[py][1] *
//                                                                                          py) +
//                                                           (info.y_split_pairs_cnt[py][0] * py -
//                                                            info.y_split_pairs[py][0]));
//            } else if (info.min_x < i < info.max_x and info.max_y <= j < lf.gr_scale[1]) {
//                lf.blue_light[i][j]= info.ac_tar * 1000 - decay_ratio * ((info.x_split_pairs[px][1] -
//                                                            info.x_split_pairs_cnt[px][1] *
//                                                            px) +
//                                                           (info.x_split_pairs_cnt[px][0] * px -
//                                                            info.x_split_pairs[px][0]) + info.ac_tar * py - info.sum_y);
//            } else if (info.max_x <= i < lf.gr_scale[0] and j <= info.min_y) {
//                lf.blue_light[i][j]= info.ac_tar * 1000 - decay_ratio * (info.ac_tar-info.sum_x * px-info.ac_tar * py+info.sum_y);
//            } else if (info.max_x <= i < lf.gr_scale[0] and info.min_y < j < info.max_y) {
//                lf.blue_light[i][j]= info.ac_tar * 1000 - decay_ratio * (info.ac_tar * px-info.sum_x + (info.y_split_pairs[py][1] -
//                                                                                            info.y_split_pairs_cnt[py][1] *
//                                                                                            py) +
//                                                           (info.y_split_pairs_cnt[py][0] * py -
//                                                            info.y_split_pairs[py][0]));
//            } else if (info.max_x <= i < lf.gr_scale[0] and info.max_y <= j < lf.gr_scale[1]) {
//                lf.blue_light[i][j]= info.ac_tar * 1000 - decay_ratio * (info.ac_tar * (px + py)-info.sum_x - info.sum_y);
//            }
//        }
//    }
//}

//    if(lf.grids[fx][fy]==0 and lf.grids[tx][ty]==1){
//        info.sum_x-=tx;
//        info.sum_y-=ty;
//        info.x_sums[tx]-=tx;
//        info.y_sums[ty]-=ty;
//        for(int i=0;i<lf.gr_scale[0];i++){
//            if(i<tx) {
//                info.x_split_pairs[i][1]-=tx;
//            }else if(i>tx){
//                info.x_split_pairs[i][0]-=tx;
//            }
//        }
//        for(int j=0;j<lf.gr_scale[1];j++){
//            if(j<ty) {
//                info.x_split_pairs[j][1]-=ty;
//            }else if(j>ty){
//                info.x_split_pairs[j][0]-=ty;
//            }
//        }
//    }else if(lf.grids[fx][fy]==1 and lf.grids[tx][ty]==1){
//        info.sum_x=info.sum_x-tx+fx;
//        info.sum_y=info.sum_y-ty+fy;
//        info.x_sums[fx]+=fx;
//        info.x_sums[tx]-=tx;
//        info.y_sums[fy]+=fy;
//        info.y_sums[ty]-=ty;
//        for(int i=0;i<lf.gr_scale[0];i++){
//            if(i<tx) {
//                info.x_split_pairs[i][1]-=tx;
//            }else if(i>tx){
//                info.x_split_pairs[i][0]-=tx;
//            }
//            if(i<fx) {
//                info.x_split_pairs[i][1]+=fx;
//            }else if(i>fx){
//                info.x_split_pairs[i][0]+=fx;
//            }
//        }
//        for(int j=0;j<lf.gr_scale[1];j++){
//            if(j<ty) {
//                info.x_split_pairs[j][1]-=ty;
//            }else if(j>ty){
//                info.x_split_pairs[j][0]-=ty;
//            }
//            if(j<fy) {
//                info.x_split_pairs[j][1]+=fx;
//            }else if(j>fy){
//                info.x_split_pairs[j][0]+=fx;
//            }
//        }
//    }