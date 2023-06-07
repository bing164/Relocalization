//
// Created by bing on 2023/6/5.
//

#ifndef REPLACE_MAPPOINT_H
#define REPLACE_MAPPOINT_H

#include "Frame.h"

class Frame;
class MapPoint {
    MapPoint() {}
    MapPoint(const Eigen::Vector3d &P);


private:
    Eigen::Vector3d Pw;
    Eigen::Vector3d Pc;
};

#endif //REPLACE_MAPPOINT_H
