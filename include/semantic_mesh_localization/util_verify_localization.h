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

}

#endif
