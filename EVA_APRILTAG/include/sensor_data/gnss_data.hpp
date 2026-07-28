
#ifndef GLOBAL_FUSION_SENSOR_DATA_GNSS_DATA_HPP_
#define GLOBAL_FUSION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

#include "Geocentric/LocalCartesian.hpp"
# include <Eigen/Core>
#include <boost/array.hpp>
#include <boost/typeof/typeof.hpp>
#include "config.h"

namespace eva_april {
    class GNSSData {
    public:
        double time = 0.0;
        double longitude = 0.0;
        double latitude = 0.0;
        double altitude = 0.0;
        double local_E = 0.0;
        double local_N = 0.0;
        double local_U = 0.0;
        boost::array<double, 9> pos_cov = {0.0};
        Eigen::Matrix4d gnss_pose;

        int status = 0;
        int service = 0;

        static double origin_longitude;
        static double origin_latitude;
        static double origin_altitude;

        static int origin_service;
        static bool origin_position_inited;

    private:
        static GeographicLib::LocalCartesian geo_converter;

    public:
        void InitOriginPosition();

        void InitOriginPosition(const Eigen::Vector3d &origin);

        void UpdateXYZ();

        void UpdateService();

        static void ForceInitOriginPosition(double longitudeFromMap, double latitudeFromMap, double altitudeFromMap);

        static bool SyncData(std::deque<GNSSData> &UnsyncedData, std::deque<GNSSData> &SyncedData, double sync_time);

        static bool IsInit(){
            return origin_position_inited;
        }
    };
}
#endif