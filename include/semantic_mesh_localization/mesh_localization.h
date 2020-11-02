#ifndef __MESH_LOCALIZATION_H
#define __MESH_LOCALIZATION_H

#define PI 3.141592

#include"semantic_mesh_localization/pointcloud_to_mesh.h"




namespace semloam{

    class MeshLocalization : public PcToMesh{

        public:

            MeshLocalization(ros::NodeHandle& node, ros::NodeHandle& privateNode);

            virtual ~MeshLocalization();

            void segmented_image_callback(const sensor_msgs::ImageConstPtr& msg);

            bool setup_mesh_localization(ros::NodeHandle& node, ros::NodeHandle& privateNode);

            void spin_mesh_localization();

        private:
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub;

    };

}

#endif
