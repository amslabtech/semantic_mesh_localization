#ifndef __POINT_LOCALIZATION_H
#define __POINT_LOCALIZATION_H

#include "semantic_mesh_localization/mesh_localization.h"

namespace semlocali{

    class PointLocalization : public MeshLocalization{

        public:
            PointLocalization( ros::NodeHandle& node, ros::NodeHandle& privateNode);

            virtual ~PointLocalization();
            
            bool setup_point_localization( ros::NodeHandle& node, ros::NodeHandle& privateNode);

            bool build_pc_map();

            void add_semantic_pc(pcl::visualization::PCLVisualizer& viewer, std::string semantic_name, const pcl::PointCloud<pcl::PointXYZRGB> semantic_cloud, int blue, int green, int red);

            void spin_point_localization();

            void particle_filter_for_point_cloud_map();

            void update_likelihood_for_pc_map();

            double get_likelihood_for_pc_map(geometry_msgs::Pose pose);

            double compare_map_image();

        private:
            
            double point_size = 1.0;

            bool build_pc_map_checker = true;

    };
}

#endif
