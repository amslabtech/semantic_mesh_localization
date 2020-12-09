#include "semantic_mesh_localization/point_localization.h"

namespace semlocali{

    PointLocalization::PointLocalization( ros::NodeHandle& node, ros::NodeHandle& privateNode)
        : MeshLocalization(node,privateNode){

            /*
            image_sub = it_.subscribe
                ("/bonnet/color_mask" , 1, &MeshLocalization::segmented_image_callback, this);
            
            sub_odometry = node.subscribe<nav_msgs::Odometry>
                ("/odom_pose", 1, &MeshLocalization::odometry_callback, this);

            pub_pose = node.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
            pub_particle = node.advertise<geometry_msgs::PoseArray>("/particle", 1);

            estimated_pose.header.frame_id = "map";
            particle.header.frame_id = "map";

            car_state.header.frame_id = "map";
            car_state.child_frame_id = "estimated_pose";
            */
        }

    PointLocalization::~PointLocalization(){}

    bool PointLocalization::setup_point_localization(ros::NodeHandle& node, ros::NodeHandle& privateNode){
        if( !setup_mesh_localization( node, privateNode)){
            ROS_ERROR("Point Map Setup has problem");
            return false;
        }

        int i_param;
        double dparam;
        bool bparam;
        std::string sparam;

        if( privateNode.getParam("PointSize" , dparam) ){
            if(dparam < 0.0){
                ROS_ERROR("Invalid PointSize parameter");
                return false;
            }
            else{
                point_size = dparam;
            }
        }

        if( privateNode.getParam("BuildPointCloudMap", bparam) ){
            if(bparam==true || bparam==false){
                build_pc_map_checker = bparam;
            }
            else{
                ROS_ERROR("Invalid BuildPointCloudMap Parameter");
                return false;
            }
        }

        std::cout << "Point Cloud Map's setup has done" << std::endl;
        
        if(build_pc_map_checker==false && build_polygon_checker==false && read_polygon_checker==false){
            ROS_ERROR("Map option is not defined");
            ROS_ERROR("Killing process");
            return false;
        }

        if(build_pc_map_checker==true){
            if( build_pc_map()==false ){
                ROS_ERROR("Error in building point cloud map");
                return false;
            }
        }

        /*
        while(1){
            viewer.spin();
        }
        */

        return true;
    }

    bool PointLocalization::build_pc_map(){
        load_PCD();

        init_config_viewer_parameter(viewer);
        
        if(image_height > 0 && image_width > 0){
            viewer.setSize( image_width, image_height);
        }

        add_semantic_pc( viewer,     "road",         road, 128,  64, 128);
        add_semantic_pc( viewer, "sidewalk",     sidewalk, 232,  35, 244);
        add_semantic_pc( viewer, "building",     building,  70,  70,  70);
        //add_semantic_pc( viewer,     "wall",    wall, 156, 102, 102);
        add_semantic_pc( viewer,    "fence",        fence, 153, 153, 190);
        //add_semantic_pc( viewer,     "pole",    pole, 153, 153, 153);
        add_semantic_pc( viewer,"trafficsign",trafficsign,   0, 220, 220);
        add_semantic_pc( viewer, "vegetation", vegetation,  35, 142, 107);
        add_semantic_pc( viewer,  "terrain",      terrain, 152, 251, 152);
        add_semantic_pc( viewer,      "car",          car, 142,   0,   0);
        add_semantic_pc( viewer,    "truck",        truck,  70,   0,   0);
        add_semantic_pc( viewer,      "bus",          bus, 100,  60,   0);
        add_semantic_pc( viewer,"motorcycle",  motorcycle, 230,   0,   0);
        add_semantic_pc( viewer,  "bicycle",      bicycle,  32,  11, 119);

        config_tmp_viewer_parameter(viewer);

        return true;
    }

    void PointLocalization::add_semantic_pc(pcl::visualization::PCLVisualizer& viewer, std::string semantic_name, const pcl::PointCloud<pcl::PointXYZRGB> semantic_cloud, int blue, int green, int red){

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZRGB>(semantic_cloud));

        double color_r = double(red)/255.0;
        double color_g = double(green)/255.0;
        double color_b = double(blue)/255.0;

        viewer.addPointCloud( cloudin, semantic_name);

        viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_COLOR, color_r, color_g, color_b, semantic_name);
        viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, semantic_name);
        viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, semantic_name);

        std::cout << "Setting " << semantic_name << "'s point cloud has done" << std::endl;
    }

    void PointLocalization::spin_point_localization(){
        /*
        while( ros::ok() ){
            viewer.spin();
        }
        */

        wait_for_bag_data();
        particle_filter_for_point_cloud_map();

    }

    void PointLocalization::particle_filter_for_point_cloud_map(){
        std::cout << "Do Particle fileter" << std::endl;
        ros::Rate loop_rate(1.0);

        std::ofstream groundtruth_csv(groundtruth_path);
        std::ofstream odometry_csv(odometry_path);
        std::ofstream estimated_csv(estimated_path);
        std::ofstream biased_odom_csv(biased_odom_path);

        while( ros::ok() ){

            Time start_time_pf = ros::Time::now();
            Time end_time_pf;

            std::cout << "Catch ROS data" << std::endl;
            ros::spinOnce();//catch odometry and image data
            //end_time_pf = ros::Time::now();
            ////std::cout << "spinOnce  "<< end_time_pf.toSec() - start_time_pf.toSec() << std::endl;


            std::cout << "Motion update" << std::endl;
            motion_update();//Create prior distribution
            //end_time_pf = ros::Time::now();
            ////std::cout << "Motion update  "<< end_time_pf.toSec() - start_time_pf.toSec() << std::  endl;


            std::cout << "Update likelihood" << std::endl;
            update_likelihood_for_pc_map();//Observe and create posterior distribution
            //end_time_pf = ros::Time::now();
            ////std::cout << "Update likelihood  "<< end_time_pf.toSec() - start_time_pf.toSec() << s  td::endl;


            std::cout << "Estimate current pose" << std::endl;
            estimate_current_pose();//estimate current pose
            //end_time_pf = ros::Time::now();
            ////std::cout << "Estimate pose  "<< end_time_pf.toSec() - start_time_pf.toSec() << std::  endl;


            std::cout << "Publish result" << std::endl;
            publish_result();//publish as ros data EXCEPT POINTCLOUD MAP
            //end_time_pf = ros::Time::now();
            ////std::cout << "Pub result  "<< end_time_pf.toSec() - start_time_pf.toSec() << std::end  l;

            if(publish_csv_checker == true){
                start_csv = ros::Time::now();
                std::cout << "Publish as CSV file" << std::endl;
                publish_as_csv(groundtruth_csv, odometry_csv, estimated_csv, biased_odom_csv);

                end_csv = ros::Time::now();

                //std::cout << "Pub CSV Time :" << end_csv.toSec() - start_csv.toSec() <<"[s]"<<std  ::endl;
            }
            
            if(save_image_checker == true){
            
                save_image();
                std::cout << "Save segmented image as png file" << std::endl;
            
            }

            std::cout << "Resampling particle" << std::endl;
            resampling_particle();//resampling
            end_time_pf = ros::Time::now();
            double duration_pf = end_time_pf.toSec() - start_time_pf.toSec();
  
            std::cout << "Total time " << duration_pf << " [s]" << std::endl;
            std::cout << std::endl;
            std::cout << std::endl;
            loop_rate.sleep();
        }
        
        groundtruth_csv.close();
        odometry_csv.close();
        estimated_csv.close();
    }

    void PointLocalization::update_likelihood_for_pc_map(){
        double total_likelihood = 0.0;

        Time start_li = ros::Time::now();
        Time end_li;
        
        for(size_t i=0; i<likelihood.size(); i++){
            likelihood[i] = get_likelihood_for_pc_map( particle.poses[i] );
            total_likelihood += likelihood[i];
        }

        if(total_likelihood < 0.01){
            ROS_ERROR("Invalid total likelihood");
            for(size_t i=0; i<likelihood.size(); i++){
                likelihood[i] = ( (double)rand() / ((double)RAND_MAX + 1) );
            }
        }

        //Normalize likelihood
        for(size_t i=0; i<likelihood.size(); i++){
            likelihood[i] = likelihood[i] / total_likelihood;
        }

        
        std::cout << "Change camera Time :" << camera_time << "[s]" << std::endl;
        
        std::cout << "VTK Image Time :" << get_image_vtk_time<< "[s]" << std::endl;

        std::cout << "Image Compare Time :" << compare_time<< "[s]" << std::endl;
        

        camera_time = 0.0;
        get_image_vtk_time = 0.0;
        compare_time = 0.0;
    }

    double PointLocalization::get_likelihood_for_pc_map(geometry_msgs::Pose pose){
        double score = 0.0;
        if(     pose.orientation.x==0.0 &&
                pose.orientation.y==0.0 &&
                pose.orientation.z==0.0 &&
                pose.orientation.w==0.0){

            score = 0.0;

        }
        else{
            change_camera_position_for_particle( pose );

            get_image_from_pcl_visualizer();

            //score = compare_map_image();
            score = compare_image_pixelwise();
        }

        return score;
    }

    double PointLocalization::compare_map_image(){
        
        return 1.0;

    }




}
