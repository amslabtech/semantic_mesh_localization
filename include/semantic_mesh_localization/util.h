#ifndef __UTIL_H
#define __UTIL_H
#include"ros/ros.h"
#include<time.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include <chrono>
#include<list>
#include<math.h>
#include<complex>
#include<sstream>

namespace semlocali
{ 
  using Time = ros::Time;
 
  struct color_data{
	  int r;
	  int g;
	  int b;
  };

  struct pos_trans{
      double dt;

	  double dx;
	  double dy;
	  double dz;

	  double dqx;
	  double dqy;
	  double dqz;
	  double dqw;

	  double droll;
	  double dpitch;
	  double dyaw;

	  double vx;
	  double vy;
	  double vz;

	  double vqx;
	  double vqy;
	  double vqz;
  };

}

#endif
