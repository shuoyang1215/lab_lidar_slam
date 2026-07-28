
#include "sensor_data/gnss_data.hpp"
#include "global_definition/global_definition.h"
#include "glog/logging.h"

//静态成员变量必须在类外初始化
double eva_april::GNSSData::origin_longitude = 0.0;
double eva_april::GNSSData::origin_latitude = 0.0;
double eva_april::GNSSData::origin_altitude = 0.0;
int eva_april::GNSSData::origin_service = 0;
bool eva_april::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian eva_april::GNSSData::geo_converter;

namespace eva_april {
    void GNSSData::InitOriginPosition() {
        geo_converter.Reset(latitude, longitude, altitude);

        origin_longitude = longitude;
        origin_latitude = latitude;
        origin_altitude = altitude;

        origin_position_inited = true;

        LOG_IF(INFO, Config::use_glog) << "Init GNSS: " << GNSSData::origin_longitude << " " << GNSSData::origin_latitude << " " << GNSSData::origin_altitude;
    }

    void GNSSData::InitOriginPosition(const Eigen::Vector3d &origin) {

        longitude = origin(0);
        latitude = origin(1);
        altitude = origin(2);

        geo_converter.Reset(latitude, longitude, altitude);

        origin_longitude = longitude;
        origin_latitude = latitude;
        origin_altitude = altitude;

        origin_position_inited = true;

        LOG_IF(INFO, Config::use_glog) << "Init GNSS with provided origin: " << GNSSData::origin_longitude << " " << GNSSData::origin_latitude << " " << GNSSData::origin_altitude;
    }

    void GNSSData::ForceInitOriginPosition(double longitudeFromMap, double latitudeFromMap, double altitudeFromMap) {

        origin_longitude = longitudeFromMap;
        origin_latitude = latitudeFromMap;
        origin_altitude = altitudeFromMap;

        geo_converter.Reset(origin_latitude, origin_longitude, origin_altitude);

        origin_position_inited = true;

        LOG_IF(INFO, Config::use_glog) << "Force to init GNSS: " << GNSSData::origin_longitude << " " << GNSSData::origin_latitude << " " << GNSSData::origin_altitude;
    }

    void GNSSData::UpdateService() {
        origin_service = service;
    }


    void GNSSData::UpdateXYZ() {

        if (!origin_position_inited) {
            LOG_IF(WARNING, Config::use_glog) << "GeoConverter has not set origin position";
            return;
        }
        geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);

        LOG_IF(INFO, Config::use_glog) << "GNSS Origin LLA: " << origin_longitude << " " << origin_latitude << " " << origin_altitude;
        LOG_IF(INFO, Config::use_glog) << "GNSS ENU: " << local_E << " " << local_N << " " << local_U;
        LOG_IF(INFO, Config::use_glog) << "GNSS LLA: " << longitude << " " << latitude << " " << altitude;
    }

    bool GNSSData::SyncData(std::deque<GNSSData> &UnsyncedData, std::deque<GNSSData> &SyncedData, double sync_time) {

        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
        while (UnsyncedData.size() >= 2) {
            if (UnsyncedData.front().time > sync_time)
                return false;
//            允许两帧需要同步的数据都在雷达之前
            if (UnsyncedData.at(1).time < sync_time && UnsyncedData.size()>2) {
                UnsyncedData.pop_front();
                continue;
            }
            if (sync_time - UnsyncedData.front().time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            if (UnsyncedData.at(1).time - sync_time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            break;
        }
        if (UnsyncedData.size() < 2)
            return false;

        GNSSData front_data = UnsyncedData.at(0);
        GNSSData back_data = UnsyncedData.at(1);
        GNSSData synced_data;

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;
        synced_data.status = back_data.status;
        synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
        synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
        synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
        synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
        synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
        synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

        SyncedData.push_back(synced_data);

        return true;
    }
}