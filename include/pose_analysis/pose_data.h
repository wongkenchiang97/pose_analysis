#ifndef POSE_DATA_H
#define POSE_DATA_H

namespace analysis{

class PoseData{
  public:
    PoseData();
    PoseData(double _x,double _y,double _yaw);
    ~PoseData();
    double x;
    double y;
    double yaw;
};

analysis::PoseData::PoseData(){}

analysis::PoseData::PoseData(double _x,double _y,double _yaw)
: x(_x),y(_y),yaw(_yaw){}

analysis::PoseData::~PoseData(){}

}

#endif