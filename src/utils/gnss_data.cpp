#include "utils/gnss_data.hpp"

#include "glog/logging.h"

//静态成员变量必须在类外初始化
bool Odometry::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian Odometry::GNSSData::geo_converter;

namespace Odometry {

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}
}