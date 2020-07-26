//
// Created by 褚文杰 on 2019/10/21.
//

#include "LightField.h"

LightField::LightField() {
    gr_scale[0]=0;
    gr_scale[1]=0;
}

void LightField::init_field(int w, int h) {
    gr_scale[0] = w;
    gr_scale[1] = h;
    for (int x = 0; x < w; x++) {
        grids.emplace_back();
        blue_light.emplace_back();
        red_light.emplace_back();
        for (int y = 0; y < h; y++) {
            grids[x].push_back(0);
            blue_light[x].push_back(0);
            red_light[x].push_back(0);
        }
    }
}


