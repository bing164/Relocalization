//
// Created by bing on 2023/6/3.
//

#ifndef REPLACE_ORBMATCHER_H
#define REPLACE_ORBMATCHER_H


#include "opencv2/opencv.hpp"
#include "Frame.h"

class Frame;
class ORBmatcher {
public:
    ORBmatcher(float nnratio = 0.6, bool checkOri = true);

    int SearchForReplace(const std::string &img_path, std::shared_ptr<Frame> Bow_F, std::shared_ptr<Frame> Cur_F);


protected:
    float m_nnratio;
    bool m_checkOri;
};


#endif //REPLACE_ORBMATCHER_H
