#include "semantic_mesh_localization/mesh_localization.h"

namespace semloam{

    MeshLocalization::MeshLocalization(ros::NodeHandle& node, ros::NodeHandle& privateNode)
        : it_(node){

            image_sub = it_.subscribe
                ("/image_view/output" , 1, &MeshLocalization::segmented_image_callback, this);

            sub_odometry = node.subscribe<nav_msgs::Odometry>
                ("/odom_pose", 1, &MeshLocalization::odometry_callback, this);

            pub_pose = node.advertise<geometry_msgs::Pose>("/estimated_pose", 1);
            pub_particle = node.advertise<geometry_msgs::PoseArray>("/particle", 1);

            estimated_pose.header.frame_id = "map";
            particle.header.frame_id = "map";

            car_state.header.frame_id = "map";
            car_state.child_frame_id = "estimated_pose";

        }

    MeshLocalization::~MeshLocalization(){}

    void MeshLocalization::segmented_image_callback(const sensor_msgs::ImageConstPtr& msg){
        
        segimage = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8);
        std::cout << "catch odom data" << std::endl;
    }

    pos_trans MeshLocalization::get_relative_trans(nav_msgs::Odometry odom, nav_msgs::Odometry last_odom){
        pos_trans trans;

        trans.dx = odom.pose.pose.position.x - last_odom.pose.pose.position.x;
        trans.dy = odom.pose.pose.position.y - last_odom.pose.pose.position.y;
        trans.dz = odom.pose.pose.position.z - last_odom.pose.pose.position.z;

        trans.dt = odom.header.stamp.toSec() - last_odom.header.stamp.toSec();

        tf::Quaternion now_quat, last_quat;
        quaternionMsgToTF(     odom.pose.pose.orientation, now_quat);
        quaternionMsgToTF(last_odom.pose.pose.orientation,last_quat);

        double nroll, npitch, nyaw;
        double lroll, lpitch, lyaw;

        tf::Matrix3x3( now_quat ).getRPY( nroll, npitch, nyaw);
        tf::Matrix3x3(last_quat ).getRPY( lroll, lpitch, lyaw);

        trans.droll = nroll - lroll;
        trans.dpitch = npitch - lpitch;
        trans.dyaw = nyaw - lyaw;

        return trans;
    }

    void MeshLocalization::odometry_callback(const nav_msgs::OdometryConstPtr& msg){

        if(        msg -> pose.pose.position.x == 0.0
                && msg -> pose.pose.position.y == 0.0
                && msg -> pose.pose.position.z == 0.0
                && msg -> pose.pose.orientation.x == 0.0
                && msg -> pose.pose.orientation.y == 0.0
                && msg -> pose.pose.orientation.z == 0.0
                && msg -> pose.pose.orientation.w == 0.0
                && first_odom_checker == false
          ){//If odometry data is not broadcasted, all data of odometry will be 0.0
            //Do nothing
        }
        else{
            last_odom_data = odom_data;
            odom_data = *msg;
            first_odom_checker = true;

            odom_trans = get_relative_trans( odom_data, last_odom_data);
        }

    }

    void MeshLocalization::build_mesh_map(){

        load_PCD();
        init_config_viewer_parameter(viewer);
        generate_mesh(viewer);
        config_tmp_viewer_parameter(viewer);

        std::cout << "Mesh map build is done" << std::endl;

    }

    bool MeshLocalization::setup_mesh_localization(ros::NodeHandle& node, ros::NodeHandle& privateNode){

        if(!setup( node, privateNode)){//pc_to_mesh.h's setup function. It's used to build mesh map
            ROS_ERROR("Mapping Setup has problem");
            return false;
        }

        int iparam;
        double dparam;

        if( privateNode.getParam("particlenumber", iparam )){
                if(iparam < 1){
                    ROS_ERROR("Invalid particle number, program abort");
                    return false;
                }
                else if(iparam > 0 && iparam <15){
                    ROS_ERROR("Number of particle is valid, but too littele for localization");
                    particlenumber = iparam;
                }
                else{
                    ROS_INFO("Set number of particle: %d", iparam);
                    particlenumber = iparam;
                }
        }

        if( privateNode.getParam("xdev", dparam) ){
            if(dparam <= 0.0 ){
                ROS_ERROR("Invalid delta x diviation");
                return false;
            }
            else{
                dx_dev = dparam;
            }
        }

        if( privateNode.getParam("ydev" , dparam ) ){
            if(dparam <= 0.0){
                ROS_ERROR("Invalid delta y diviation");
                return false;
            }
            else{
                dy_dev = dparam;
            }
        }

        if( privateNode.getParam("zdev" , dparam) ){
            if(dparam <= 0.0 ){
                ROS_ERROR("Invalid delta z diviation");
                return false;
            }
            else{
                dz_dev = dparam;
            }
        }

        if( privateNode.getParam("rolldev" , dparam )){
            if(dparam <= 0.0 ){
                ROS_ERROR("Invalid delta roll diviation");
                return false;
            }
            else{
                droll_dev = dparam;
            }
        }

        if( privateNode.getParam("pitchdev", dparam) ){
            if(dparam <= 0.0){
                ROS_ERROR("Invalid delta pitch diviation");
                return false;
            }
            else{
                dpitch_dev = dparam;
            }
        }

        if( privateNode.getParam("yawdev", dparam) ){
            if(dparam <= 0.0){
                ROS_ERROR("Invalid delta yaw diviation");
                return false;
            }
            else{
                dyaw_dev = dparam;
            }
        }

        std::cout << "Mesh map setup is done..." << std::endl;
        std::cout << "Building mesh map....." << std::endl;

        build_mesh_map();

        /*
        ros::Rate rate(10);

        while( ros::ok() ){

            viewer.spinOnce();

            rate.sleep();
        }
        */

        return true;
    }

    void MeshLocalization::init_odometry_process(){

        likelihood.reserve(particlenumber); //Match number of particle and vector size

        estimated_pose.header.frame_id = odom_data.header.frame_id;
        estimated_pose.header.stamp = odom_data.header.stamp;
        estimated_pose.pose = odom_data.pose.pose;

        particle.header.frame_id = odom_data.header.frame_id;
        particle.header.stamp = odom_data.header.stamp;

        for(int i=0; i<particlenumber; i++){
            particle.poses.push_back( odom_data.pose.pose );
        }

        std::cout << "Init odometry process has done" << std::endl;
    }

    void MeshLocalization::wait_for_bag_data(){

        ros::Rate message_rate(1.0);
        while(1){
            ros::spinOnce();
            if(first_odom_checker == true){
                //最初の処理をここで行う

                init_odometry_process();

                break;
            }
            else{
                std::cout << "PLEASE PLAY BAG FILE" << std::endl;
            }
            message_rate.sleep();
        }
    }

    void MeshLocalization::spin_mesh_localization(){

        wait_for_bag_data();

        particle_filter();

    }

    void MeshLocalization::particle_filter(){

        ros::Rate loop_rate(1.0);
        while( ros::ok() ){

            ros::spinOnce();//catch odometry and image data

            motion_update();

            update_likelihood();

            estimate_current_pose();

            resampling_particle();

            publish_result();

            loop_rate.sleep();
        }
    }

    double MeshLocalization::rand_delta(double ave, double dev){
        double number = 0.0;

        std::random_device seed_gen;
        std::default_random_engine engine( seed_gen() );

        std::normal_distribution<> dist(ave,dev);

        number = dist(engine);

        return number;
    }

    void MeshLocalization::motion_update(){

        for(int i=0; i < particle.poses.size(); i++){
            //Get XYZ
            particle.poses[i].position.x = 
                particle.poses[i].position.x + rand_delta(odom_trans.dx, dx_dev);
            particle.poses[i].position.y = 
                particle.poses[i].position.y + rand_delta(odom_trans.dy, dy_dev);
            particle.poses[i].position.z = 
                particle.poses[i].position.z + rand_delta(odom_trans.dz, dz_dev);

            //get Quaternion from RPY trans
            tf::Quaternion quat;
            quaternionMsgToTF( particle.poses[i].orientation , quat );

            double roll, pitch, yaw;
            tf::Matrix3x3( quat ).getRPY( roll , pitch , yaw );

            roll = roll + rand_delta( odom_trans.droll , droll_dev );
            pitch = pitch + rand_delta( odom_trans.dpitch , dpitch_dev );
            yaw = yaw + rand_delta( odom_trans.dyaw , dyaw_dev );

            tf::Quaternion new_quat = tf::createQuaternionFromRPY( roll , pitch , yaw );
            geometry_msgs::Quaternion geo_quat;
            quaternionTFToMsg( new_quat , geo_quat );
            particle.poses[i].orientation = geo_quat;

        }
    }

    void MeshLocalization::update_likelihood(){
        //
    }

    void MeshLocalization::estimate_current_pose(){
        //
    }

    void MeshLocalization::resampling_particle(){
        //
    }

    void MeshLocalization::publish_result(){
        
        estimated_pose.header.stamp = odom_data.header.stamp;
        particle.header.stamp = odom_data.header.stamp;

        pub_pose.publish(estimated_pose);
        pub_particle.publish(particle);

        car_state.header.stamp = odom_data.header.stamp;
        car_state.transform.translation.x = estimated_pose.pose.position.x;
        car_state.transform.translation.y = estimated_pose.pose.position.y;
        car_state.transform.translation.z = estimated_pose.pose.position.z;
        car_state.transform.rotation = estimated_pose.pose.orientation;

        br.sendTransform( car_state );


    }

}



