//
// Created by 褚文杰 on 2019/10/21.
//

#ifndef ASSEMBLYSHAPE_LIGHTFIELD_H
#define ASSEMBLYSHAPE_LIGHTFIELD_H

# include <vector>


class LightField {
public:
    int gr_scale[2];
    std::vector<std::vector<int>> grids;
    std::vector<std::vector<double>> blue_light;
    std::vector<std::vector<double>> red_light;
    LightField();
    void init_field(int w,int h);
};

#endif //ASSEMBLYSHAPE_LIGHTFIELD_H
