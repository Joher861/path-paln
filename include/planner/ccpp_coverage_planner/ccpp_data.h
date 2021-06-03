#ifndef DATA_CENTER_CCPP_DATA_H
#define DATA_CENTER_CCPP_DATA_H

#include <cinttypes>

#include "misc/planning_typedefs.h"
#include "data_center/data.h"

using planning_data::Data;

struct ccppCellData
{
    int index;
    std::vector<cv::Point2f> vertices;
    PosePath current_path;
};

struct missOutPath
{
    int index;
    int cellNum;
    PosePath current_path;
};

DEFINE_DATA(CCPP)
std::vector<ccppCellData> ccppData;
std::vector<missOutPath> missOutData;

END_DATA(CCPP)
#endif