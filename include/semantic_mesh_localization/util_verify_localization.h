#ifndef __UTIL_VERIFY_LOCALIZATION_H
#define __UTIL_VERIFY_LOCALIZATION_H

#include"ros/ros.h"

namespace semlocali{
    struct csv_data{

        double time;
        
        double x;
        double y;
        double z;

        double qx;
        double qy;
        double qz;
        double qw;
    };

    struct diff_place{
        double pose;
        double angle;
    };

    struct diff_stat{
        int scantimes=0;

        //XYZ
        int time_XYZ_u05=0;
        int time_XYZ_u10=0;
        int time_XYZ_u15=0;
        int time_XYZ_u20=0;
        int time_XYZ_u25=0;
        int time_XYZ_u30=0;
        int time_XYZ_u35=0;
        int time_XYZ_u40=0;
        int time_XYZ_o40=0;

        //RPY
        int time_RPY_u01=0;
        int time_RPY_u05=0;
        int time_RPY_u10=0;
        int time_RPY_u15=0;
        int time_RPY_u20=0;
        int time_RPY_u25=0;
        int time_RPY_u30=0;
        int time_RPY_o30=0;

        //XYZ
        double ratio_XYZ_u05=0.0;
        double ratio_XYZ_u10=0.0;
        double ratio_XYZ_u15=0.0;
        double ratio_XYZ_u20=0.0;
        double ratio_XYZ_u25=0.0;
        double ratio_XYZ_u30=0.0;
        double ratio_XYZ_u35=0.0;
        double ratio_XYZ_u40=0.0;
        double ratio_XYZ_o40=0.0;

        //RPY
        double ratio_RPY_u01=0.0;
        double ratio_RPY_u05=0.0;
        double ratio_RPY_u10=0.0;
        double ratio_RPY_u15=0.0;
        double ratio_RPY_u20=0.0;
        double ratio_RPY_u25=0.0;
        double ratio_RPY_u30=0.0;
        double ratio_RPY_o30=0.0;

    };


}

#endif
