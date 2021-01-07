#include "semantic_mesh_localization/mesh_localization.h"

namespace semlocali{

    MeshLocalization::MeshLocalization(ros::NodeHandle& node, ros::NodeHandle& privateNode)
        : it_(node){

            image_sub = it_.subscribe
                ("/bonnet/color_mask" , 1, &MeshLocalization::segmented_image_callback, this);
/*
            sub_odometry = node.subscribe<nav_msgs::Odometry>
                ("/odom_pose", 1, &MeshLocalization::odometry_callback, this);
*/
            sub_odometry = node.subscribe<nav_msgs::Odometry>
                ("/odom_pose2", 1, &MeshLocalization::odometry_callback, this);

            pub_pose = node.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
            pub_particle = node.advertise<geometry_msgs::PoseArray>("/particle", 1);
            pub_biased_odometry = node.advertise<nav_msgs::Odometry>("biased_odom", 1);
            ground_truth_pub = node.advertise<nav_msgs::Odometry>("/ground_truth_odom",1);

            estimated_pose.header.frame_id = "map";
            particle.header.frame_id = "map";

            car_state.header.frame_id = "map";
            car_state.child_frame_id = "estimated_pose";

        }

    MeshLocalization::~MeshLocalization(){}

    void MeshLocalization::segmented_image_callback(const sensor_msgs::ImageConstPtr& msg){
        
        Time start_im = ros::Time::now();

        cv_bridge::CvImagePtr seg_ptr;

        try{

            //cv_bridge::CvImagePtr seg_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
            seg_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
            cv::cvtColor( seg_ptr->image , segimage , cv::COLOR_RGB2BGRA );

            if(image_down_height != 1.0 && image_down_width != 1.0){
                cv::resize(segimage, segimage , cv::Size() , image_down_width, image_down_height);
            }
        }
        catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        Time end_im = ros::Time::now();

        //std::cout << "Image convert and resize time :" << end_im.toSec() - start_im.toSec() <<"[s]" << std::endl;

        std::cout << "catch odom data" << std::endl;
    }

    double MeshLocalization::add_bias_XY(double dt, int random_value){
        
        double dev_bias = dt * bias_XY;
        
        std::random_device seed_gen;
        std::default_random_engine engine( seed_gen() );

        std::normal_distribution<double> dist(dt, dev_bias);

        double number = dist(engine);

        bool plumin;
        if( (number - dt) > 0 ){
            plumin = true;
        }
        else{
            plumin = false;
        }

        if( step_pebbles_checker == true ){

            std::cout << "Step on pebbles" << std::endl;

            if( (random_value % pebbles_counter) == 0 ){
                if(plumin == true){
                    number = number + pebbles_bias_XYZ;
                }
                else if(plumin == false){
                    number = number - pebbles_bias_XYZ;
                }
            }
        }

        return number;
    }

    double MeshLocalization::add_bias_Z(double dt, int random_value){
        
        double dev_bias = dt * bias_Z;
        
        std::random_device seed_gen;
        std::default_random_engine engine( seed_gen() );

        std::normal_distribution<double> dist(dt, dev_bias);

        double number = dist(engine);

        bool plumin;
        if( (number - dt) > 0 ){
            plumin = true;
        }
        else{
            plumin = false;
        }

        if( step_pebbles_checker == true ){

            std::cout << "Step on pebbles" << std::endl;

            if( (random_value % pebbles_counter) == 0 ){
                if(plumin == true){
                    number = number + pebbles_bias_XYZ;
                }
                else if(plumin == false){
                    number = number - pebbles_bias_XYZ;
                }
            }
        }

        return number;
    }



    double MeshLocalization::add_bias_XYZ(double dt, int random_value){
        
        double dev_bias = dt * bias_XYZ;
        
        std::random_device seed_gen;
        std::default_random_engine engine( seed_gen() );

        std::normal_distribution<double> dist(dt, dev_bias);

        double number = dist(engine);

        bool plumin;
        if( (number - dt) > 0 ){
            plumin = true;
        }
        else{
            plumin = false;
        }

        if( step_pebbles_checker == true ){

            std::cout << "Step on pebbles" << std::endl;

            if( (random_value % pebbles_counter) == 0 ){
                if(plumin == true){
                    number = number + pebbles_bias_XYZ;
                }
                else if(plumin == false){
                    number = number - pebbles_bias_XYZ;
                }
            }
        }

        return number;
    }

    double MeshLocalization::add_bias_RPY(double dt, int random_value){
        
        double dev_bias = dt * bias_RPY;

        std::random_device seed_gen;
        std::default_random_engine engine( seed_gen() );

        std::normal_distribution<double> dist(dt, dev_bias);

        double number = dist(engine);

        bool plumin;
        if( (number - dt) > 0 ){
            plumin = true;
        }
        else{
            plumin = false;
        }

        if( step_pebbles_checker == true ){
            if( (random_value % pebbles_counter) == 0 ){
                if(plumin == true){
                    number = number + pebbles_bias_RPY;
                }
                else if(plumin == false){
                    number = number - pebbles_bias_RPY;
                }
            }
        }

        return number;
    }

    void MeshLocalization::add_bias_to_odometry(pos_trans& odom_trans){

        int random_value = rand();
        odom_trans.dx = add_bias_XY(odom_trans.dx, random_value);
        
        random_value = rand();
        odom_trans.dy = add_bias_XY(odom_trans.dy, random_value);

        random_value = rand();
        odom_trans.dz = add_bias_Z(odom_trans.dz, random_value);

        random_value = rand();
        odom_trans.droll  = add_bias_RPY(odom_trans.droll , random_value);

        random_value = rand();
        odom_trans.dpitch = add_bias_RPY(odom_trans.dpitch, random_value);

        random_value = rand();
        odom_trans.dyaw   = add_bias_RPY(odom_trans.dyaw  , random_value);

        //Biased Odometry
        biased_odom.pose.pose.position.x = biased_odom.pose.pose.position.x + odom_trans.dx;
        biased_odom.pose.pose.position.y = biased_odom.pose.pose.position.y + odom_trans.dy;
        biased_odom.pose.pose.position.z = biased_odom.pose.pose.position.z + odom_trans.dz;

        double tmp_r,tmp_p,tmp_y;
        tf::Quaternion biased_quat;
        quaternionMsgToTF( biased_odom.pose.pose.orientation, biased_quat);
        tf::Matrix3x3( biased_quat ).getRPY( tmp_r, tmp_p, tmp_y);
        tmp_r += odom_trans.droll;
        tmp_p += odom_trans.dpitch;
        tmp_y += odom_trans.dyaw;

        tf::Quaternion quat_li = tf::createQuaternionFromRPY( tmp_r, tmp_p, tmp_y);
        geometry_msgs::Quaternion qwe_quat;
        quaternionTFToMsg( quat_li, qwe_quat);

        biased_odom.pose.pose.orientation = qwe_quat;

    }

    pos_trans MeshLocalization::get_relative_trans(nav_msgs::Odometry odom, nav_msgs::Odometry last_odom){

        if(
                   last_odom.pose.pose.orientation.x == 0.0
                && last_odom.pose.pose.orientation.y == 0.0
                && last_odom.pose.pose.orientation.z == 0.0
                && last_odom.pose.pose.orientation.w == 0.0
          ){
            last_odom.pose.pose.orientation.w = 1.0;
        }

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



        /*
        std::cout << "odom" << std::endl << odom << std::endl << "last_odom" << std::endl << last_odom << std::endl;

        std::cout << "trans dx dy dz droll dpitch dyaw" << std::endl;
        std::cout << trans.dx << std::endl;
        std::cout << trans.dy << std::endl;
        std::cout << trans.dz << std::endl;
        std::cout << trans.droll << std::endl;
        std::cout << trans.dpitch << std::endl;
        std::cout << trans.dyaw << std::endl;
        */

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
          ){//If odometry data is not broadcasted, all data of odometry will be 0.0
            //Do nothing
        }
        else{
            last_odom_data = odom_data;
            odom_data = *msg;
            first_odom_checker = true;

            biased_odom.header.stamp = odom_data.header.stamp;
            biased_odom.header.frame_id = odom_data.header.frame_id;

            odom_trans = get_relative_trans( odom_data, last_odom_data);
            if(add_bias_checker == true){
                add_bias_to_odometry(odom_trans);
            }
        }

    }

    void MeshLocalization::load_semantic_polygon(pcl::visualization::PCLVisualizer& viewer, std::string semantic_name, int blue, int green, int red){
        
        pcl::PolygonMesh semantic_mesh;

        std::string polygon_file_string = polygon_data_path + polygon_file_name_base + semantic_name + ".ply";

        int load_ply_success_checker = pcl::io::loadPLYFile( polygon_file_string, semantic_mesh);

        if( load_ply_success_checker != 0 ){
            std::cout << "loading " << polygon_file_string << " was failed" << std::endl;
        }
        else{
            std::cout << "loading " << polygon_file_string << " was successfull" << std::endl;

            viewer.addPolygonMesh( semantic_mesh, semantic_name);

            double color_r = double(red)/255.0;
            double color_g = double(green)/255.0;
            double color_b = double(blue)/255.0;

            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_r, color_g, color_b, semantic_name);
            
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, semantic_name);

            std::cout << semantic_name << "'s polygon mesh is added to PCL Visualizer" << std::endl;
        }


    }

    void MeshLocalization::load_PLY(pcl::visualization::PCLVisualizer& viewer){
        
        load_semantic_polygon( viewer,     "road", 128,  64, 128);
        //load_semantic_polygon( viewer, "sidewalk", 232,  35, 244);
        load_semantic_polygon( viewer, "building",  70,  70,  70);
        //load_semantic_polygon( viewer,     "wall", 156, 102, 102);
        load_semantic_polygon( viewer,    "fence", 153, 153, 190);
        //load_semantic_polygon( viewer,     "pole", 153, 153, 153);
        //load_semantic_polygon( viewer,"trafficsign", 0, 220, 220);
        load_semantic_polygon( viewer, "vegetation",35, 142, 107);
        load_semantic_polygon( viewer,  "terrain", 152, 251, 152);
        load_semantic_polygon( viewer,      "car", 142,   0,   0);
        //load_semantic_polygon( viewer,    "truck",  70,   0,   0);
        //load_semantic_polygon( viewer,      "bus", 100,  60,   0);
        //load_semantic_polygon( viewer,"motorcycle",230,   0,   0);
        //load_semantic_polygon( viewer,  "bicycle",  32,  11, 119);

        std::cout << "Loading semantic mesh polygon is done" << std::endl;
    }

    void MeshLocalization::read_mesh_map(){

        load_PLY(viewer);
        init_config_viewer_parameter(viewer);
 
        if(image_height > 0 && image_width > 0){
            viewer.setSize( image_width, image_height);
        }

        config_tmp_viewer_parameter(viewer);

        std::cout << "Mesh map loading is done" << std::endl;

    }  

    void MeshLocalization::build_mesh_map(){

        load_PCD();
        init_config_viewer_parameter(viewer);

        if(image_height > 0 && image_width > 0){
            viewer.setSize( image_width, image_height);
        }

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
        bool bparam;
        std::string sparam;

        if( privateNode.getParam("SavePlaceChecker", bparam)){
            if( bparam==true || bparam==false ){
                save_place_checker = bparam;
            }
            else{
                ROS_ERROR("Invalid SavePlaceChecker Parameter");
                return false;
            }
        }

        if( privateNode.getParam("PlaceCSVPath" , sparam)){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid PlaceCSVPath");
                return false;
            }
            else{
                place_csv_path = sparam;
            }
        }

        if( privateNode.getParam("SavePlaceCounter", iparam)){
            if( iparam < 2 ){
                ROS_ERROR("Invalid SavePlaceCounter value");
                return false;
            }
            else{
                save_place_counter = iparam;
            }
        }

        if( privateNode.getParam("AddBiasChecker", bparam) ){
            if( bparam==true || bparam==false ){
                add_bias_checker = bparam;
            }
            else{
                ROS_ERROR("Invalid AddBiasChecker Parameter");
                return false;
            }
        }

        if( privateNode.getParam("StepPebblesChecker", bparam) ){
            if(bparam==true || bparam==false){
                step_pebbles_checker = bparam;
            }
            else{
                ROS_ERROR("Invalid SteppebblesChecker parameter");
                return false;
            }
        }

        if( privateNode.getParam("StackMuddyChecker", bparam) ){
            if( bparam==true || bparam==false ){
                stack_muddy_checker = bparam;
            }
            else{
                ROS_ERROR("Invalid StackMuddyChecker Parameter");
                return false;
            }
        }

        if( privateNode.getParam("BiasXYZ", dparam) ){
            if(dparam < 0.0 || dparam > 0.5){
                ROS_ERROR("Invalid BiasXYZ parameter, BiasXYZ must be 0< BiasXYZ < 0.5");
                return false;
            }
            else{
                bias_XYZ = dparam;
            }
        }

        if( privateNode.getParam("BiasXY", dparam) ){
            if(dparam < 0.0 || dparam > 0.5){
                ROS_ERROR("Invalid BiasXY parameter, BiasXYZ must be 0< BiasXYZ < 0.5");
                return false;
            }
            else{
                bias_XY = dparam;
            }
        }

        if( privateNode.getParam("BiasZ", dparam) ){
            if(dparam < 0.0 || dparam > 0.5){
                ROS_ERROR("Invalid BiasZ parameter, BiasXYZ must be 0< BiasXYZ < 0.5");
                return false;
            }
            else{
                bias_Z = dparam;
            }
        }

        if( privateNode.getParam("BiasRPY", dparam) ){
            if(dparam < 0.0 || dparam > 0.1){
                ROS_ERROR("BiasRPY parameter is under 0 or over 0.1");
                return false;
            }
            else{
                bias_RPY = dparam;
            }
        }

        if( privateNode.getParam("BiasPebblesXYZ", dparam) ){
            if(dparam < 0.0 || dparam > 0.1){
                ROS_ERROR("Invalid Pebbles XYZ parameter");
                return false;
            }
            else{
                pebbles_bias_XYZ = dparam;
            }
        }

        if( privateNode.getParam("BiasPebblesRPY", dparam) ){
            if(dparam < 0.0 || dparam > 0.1){
                ROS_ERROR("Invalid Pebbles RPY parameter");
                return false;
            }
            else{
                pebbles_bias_RPY = dparam;
            }
        }

        if( privateNode.getParam("PebblesCounter", iparam) ){
            if(iparam < 2){
                ROS_ERROR("Invalid PebblesCounter parameter");
                return false;
            }
            else{
                pebbles_counter = iparam;
            }
        }

        if( privateNode.getParam("ReadPolygonChecker", bparam) ){
            if(bparam==true || bparam==false){
                read_polygon_checker = bparam;
            }
            else{
                ROS_ERROR("Invalid ReadPolygonChecker parameter");
                return false;
            }
        }

        if( privateNode.getParam("BuildPolygonChecker", bparam) ){
            if(bparam==true || bparam==false){
                build_polygon_checker = bparam;
            }
            else{
                ROS_ERROR("Invalid BuildPolygonChecker");
                return false;
            }
        }

        if( privateNode.getParam("PolygonDataPath", sparam) ){
            if( sparam.length() < 1){
                ROS_ERROR("Invalid PolygonDataPath");
                return false;
            }
            else{
                polygon_data_path = sparam;
            }
        }

        if( privateNode.getParam("GroundtruthPath", sparam)){
            if(sparam.length() < 1){
                ROS_ERROR("Invalid Groundtruth file path");
                return false;
            }
            else{
                groundtruth_path = sparam;
            }
        }
        
        if( privateNode.getParam("OdometryPath", sparam)){
            if(sparam.length() < 1){
                ROS_ERROR("Invalid Odometry file path");
                return false;
            }
            else{
                odometry_path = sparam;
            }
        }

        if( privateNode.getParam("EstimatedPath", sparam)){
            if(sparam.length() < 1){
                ROS_ERROR("Invalid Estimated file path");
                return false;
            }
            else{
                estimated_path = sparam;
            }
        }

        if( privateNode.getParam("BiasedOdometryPath", sparam)){
            if(sparam.length() < 1){
                ROS_ERROR("Invalid Biased Odometry file path");
                return false;
            }
            else{
                biased_odom_path = sparam;
            }
        }

        if( privateNode.getParam("PublishCSVChecker", bparam) ){
            if( bparam == true || bparam == false ){
                publish_csv_checker = bparam;
            }
            else{
                ROS_ERROR("Invalid PublishCSVChecker");
                return false;
            }
        }

        if( privateNode.getParam("SaveImageChecker" , bparam) ){
            if( bparam==true || bparam==false ){
                save_image_checker = bparam;
            }
            else{
                ROS_ERROR("Invalid Publish Image Checker");
                return false;
            }
        }

        if(privateNode.getParam("SegImagePath", sparam)){
            if(sparam.length() < 1){
                ROS_ERROR("Invalid SegImagePath");
                return false;
            }
            else{
                seg_image_path = sparam;
            }
        }

        if(privateNode.getParam("MapImagePath", sparam)){
            if(sparam.length() < 1){
                ROS_ERROR("Invalid MapImagePath");
                return false;
            }
            else{
                map_image_path = sparam;
            }
        }

        if( privateNode.getParam("imageheight", iparam) ){
            if(iparam < 1){
                ROS_ERROR("Invalid Image height parameter");
                return false;
            }
            else{
                image_height = iparam;
                imheight_checker = true;
            }
        }

        if( privateNode.getParam("imagewidth", iparam ) ){
            if(iparam < 1){
                ROS_ERROR("Invalid Image width parameter");
                return false;
            }
            else{
                image_width = iparam;
                imwidth_checker = true;
            }
        }

        if( privateNode.getParam("imagedownwidth", dparam) ){
            if( dparam > 1.0 || dparam < 0.0){
                ROS_ERROR("Invalid downsize width ratio");
                return false;
            }
            else{
                image_down_width = dparam;
            }
        }

        if( privateNode.getParam("imagedownheight", dparam) ){
            if(dparam > 1.0 || dparam < 0.0){
                ROS_ERROR("Invalid downsize height ratio");
                return false;
            }
            else{
                image_down_height = dparam;
            }
        }

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

        if( privateNode.getParam("AverageNumber", dparam) ){
            if(dparam <=1.0){
                ROS_ERROR("Invalid AverageNumber parameter");
                return false;
            }
            else{
                average_number = dparam;
            }
        }

        std::cout << "Mesh map setup is done..." << std::endl;
        std::cout << "Building mesh map....." << std::endl;

        if(read_polygon_checker==false && build_polygon_checker==true){
            build_mesh_map();
        }
        else if(read_polygon_checker==true && build_polygon_checker==false){
            read_mesh_map();
        }
        else{
            ROS_INFO("Building Point Cloud Map");
        }

        
        ros::Rate rate(10);

        /*
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

        biased_odom = odom_data;

        double tmp_zero = 0.00;
        for(int i=0; i<particlenumber; i++){
            particle.poses.push_back( odom_data.pose.pose );
            likelihood.push_back(tmp_zero);
        }

        std::cout << "Init odometry process has done" << std::endl;
    }

    void MeshLocalization::wait_for_bag_data(){

        ros::Duration(50.0).sleep(); //Wiat for 1 minute due to loading CNN parameter

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
            //ros::spinOnce();
            message_rate.sleep();
        }
    }

    void MeshLocalization::spin_mesh_localization(){
 
        /*
        if(imheight_checker == true && imwidth_checker == true){
            viewer.setSize( image_width, image_height);
        }
        */

        wait_for_bag_data();

        particle_filter();

    }

    void MeshLocalization::particle_filter(){
        std::cout << "Do Particle fileter" << std::endl;
        ros::Rate loop_rate(10.0);

        std::ofstream groundtruth_csv(groundtruth_path);
        std::ofstream odometry_csv(odometry_path);
        std::ofstream estimated_csv(estimated_path);
        std::ofstream biased_odom_csv(biased_odom_path);

//        std::ofstream place_data_csv(place_csv_path);

        while( ros::ok() ){

            Time start_time_pf = ros::Time::now();
            Time end_time_pf;

            std::cout << "Catch ROS data" << std::endl;
            ros::spinOnce();//catch odometry and image data
            //end_time_pf = ros::Time::now();
            //std::cout << "spinOnce  "<< end_time_pf.toSec() - start_time_pf.toSec() << std::endl;

            std::cout << "Motion update" << std::endl;
            motion_update();//Create prior distribution
            //end_time_pf = ros::Time::now();
            //std::cout << "Motion update  "<< end_time_pf.toSec() - start_time_pf.toSec() << std::endl;

 
            std::cout << "Update likelihood" << std::endl;
            update_likelihood();//Observe and create posterior distribution
            //end_time_pf = ros::Time::now();
            //std::cout << "Update likelihood  "<< end_time_pf.toSec() - start_time_pf.toSec() << std::endl;

 
            std::cout << "Estimate current pose" << std::endl;
            estimate_current_pose();//estimate current pose
            //end_time_pf = ros::Time::now();
            //std::cout << "Estimate pose  "<< end_time_pf.toSec() - start_time_pf.toSec() << std::endl;

 
            std::cout << "Publish result" << std::endl;
            publish_result();//publish as ros data EXCEPT POINTCLOUD MAP
            //end_time_pf = ros::Time::now();
            //std::cout << "Pub result  "<< end_time_pf.toSec() - start_time_pf.toSec() << std::endl;

 
            if(publish_csv_checker == true){
                start_csv = ros::Time::now();

                std::cout << "Publish as CSV file" << std::endl;
                publish_as_csv(groundtruth_csv, odometry_csv, estimated_csv, biased_odom_csv);

                end_csv = ros::Time::now();

                //std::cout << "Pub CSV Time :" << end_csv.toSec() - start_csv.toSec() <<"[s]"<<std::endl;
            }

            if(save_image_checker == true /*&& ( loop_counter%save_place_counter==0 ) */ ){
                save_image(/*place_data_csv*/);
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
            loop_counter += 1;
        }

        groundtruth_csv.close();
        odometry_csv.close();
        estimated_csv.close();
        biased_odom_csv.close();
        //place_data_csv.close();

    }

    double MeshLocalization::rand_delta_RPY(double ave, double dev){

        double dev_motion = dev;

        std::random_device seed_gen;
        std::default_random_engine engine( seed_gen() );

        std::normal_distribution<double> dist(ave,dev_motion);

        double number = dist(engine);

        /*
        std::cout << "AVE" << std::endl;
        std::cout << ave << std::endl;

        std::cout << "DEV" << std::endl;
        std::cout << dev << std::endl;

        std::cout << "Number" << std::endl;
        std::cout << number << std::endl;
        */

        if(ave < 0.1) number = ave;

        return number;
    }

    double MeshLocalization::rand_delta_XYZ(double ave, double dev){

        double dev_motion = dev;

        std::random_device seed_gen;
        std::default_random_engine engine( seed_gen() );

        std::normal_distribution<double> dist(ave,dev_motion);

        double number = dist(engine);

        /*
        std::cout << "AVE" << std::endl;
        std::cout << ave << std::endl;

        std::cout << "DEV" << std::endl;
        std::cout << dev << std::endl;

        std::cout << "Number" << std::endl;
        std::cout << number << std::endl;
        */

        return number;
    }

    void MeshLocalization::motion_update(){

        start_mot = ros::Time::now();

        for(size_t i=0; i < particle.poses.size(); i++){

            //Get XYZ
            particle.poses[i].position.x = 
                particle.poses[i].position.x + rand_delta_XYZ(odom_trans.dx, dx_dev);
            particle.poses[i].position.y = 
                particle.poses[i].position.y + rand_delta_XYZ(odom_trans.dy, dy_dev);
            particle.poses[i].position.z = 
                particle.poses[i].position.z + rand_delta_XYZ(odom_trans.dz, dz_dev);

            /*
            std::cout << "poses" << std::endl;
            std::cout << particle.poses[i] << std::endl;
            */

            //get Quaternion from RPY trans
            tf::Quaternion quat;
            quaternionMsgToTF( particle.poses[i].orientation , quat );
            quat.normalize();

            //std::cout << "Before TF" << std::endl;
            //std::cout << quat << std::endl;

            double roll, pitch, yaw;
            tf::Matrix3x3( quat ).getRPY( roll , pitch , yaw );

            //std::cout << "RPY before" << std::endl;
            //std::cout << roll << std::endl;
            //std::cout << pitch << std::endl;
            //std::cout << yaw << std::endl;

            roll = roll + rand_delta_RPY( odom_trans.droll , droll_dev );
            pitch = pitch + rand_delta_RPY( odom_trans.dpitch , dpitch_dev );
            yaw = yaw + rand_delta_RPY( odom_trans.dyaw , dyaw_dev );

            /*
            std::cout << "RPY after" << std::endl;
            std::cout << roll << std::endl;
            std::cout << pitch << std::endl;
            std::cout << yaw << std::endl;
            */

            tf::Quaternion new_quat = tf::createQuaternionFromRPY( roll , pitch , yaw );
            new_quat.normalize();
/*
            std::cout << "TF quaternion" << std::endl;
            std::cout << new_quat << std::endl;
*/
            geometry_msgs::Quaternion geo_quat;
            quaternionTFToMsg( new_quat , geo_quat );

            /*
            std::cout << "Geometry_msgs quaternion" << std::endl;
            std::cout << geo_quat << std::endl;
            */

            particle.poses[i].orientation = geo_quat;

        }

        end_mot = ros::Time::now();

        //std::cout << "Motion update time :" << end_mot.toSec() - start_mot.toSec() << "[s]" << std::endl;
    }

    void MeshLocalization::change_camera_position_for_particle(geometry_msgs::Pose pose){

        start_camera = ros::Time::now();

        tf::Quaternion quat_tf;
        quaternionMsgToTF( pose.orientation , quat_tf );

        tf::Matrix3x3 rota;
        rota.setRotation( quat_tf );
        tf::Vector3 place;
        place.setValue( pose.position.x , pose.position.y , pose.position.z );

        Eigen::Matrix4f particle_pos;
        particle_pos << rota[0][0], rota[0][1] , rota[0][2], place[0],
                        rota[1][0], rota[1][1] , rota[1][2], place[1],
                        rota[2][0], rota[2][1] , rota[2][2], place[2],
                               0.0,        0.0 ,        0.0,      1.0;
        Eigen::Vector4f slide(0.2 , 0.0, 0.0, 1.0);
        Eigen::Vector4f new_pos = particle_pos * slide;
        viewer.setCameraPosition(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                new_pos[0],
                new_pos[1],
                new_pos[2],
                0.0,
                0.0,
                1.0);

        end_camera = ros::Time::now();
        camera_time += end_camera.toSec() - start_camera.toSec();
    }

    void MeshLocalization::get_image_from_pcl_visualizer(){

        start_vtk = ros::Time::now();

        render = viewer.getRenderWindow();//この処理は早い

        std::unique_ptr<uchar> pixels( render->GetRGBACharPixelData( 0, 0, render->GetSize()[0]-1, render->GetSize()[1]-1, 1) );//諸悪の根源

        //end_vtk = ros::Time::now();

        tmpimage = cv::Mat(render->GetSize()[1], render->GetSize()[0], CV_8UC4, &pixels.get()[0] );
        cv::cvtColor( tmpimage , mapimage , cv::COLOR_RGBA2BGRA);

        //Kaiten
        /*
        cv::flip(mapimage, mapimage, -1);
        cv::flip(mapimage, mapimage,  1);
        */

        cv::flip(mapimage,mapimage,0);

        //cv::imwrite("/home/amsl/image.jpg",mapimage);

        end_vtk = ros::Time::now();
        get_image_vtk_time += end_vtk.toSec() - start_vtk.toSec();

    }

    double MeshLocalization::compare_image_pixelwise(){

        start_cmp = ros::Time::now();

        double score = 0.0;

        int map_channel = mapimage.channels();
        uint8_t* mapimptr = (uint8_t*)mapimage.data;
        int map_r, map_g, map_b;

        int seg_channel = segimage.channels();
        uint8_t* segimptr = (uint8_t*)segimage.data;
        int seg_r, seg_g, seg_b;

        double diff_r, diff_g, diff_b;

        double tmp_score = 0.0;

        if(     mapimage.rows == segimage.rows &&
                mapimage.cols == segimage.cols    ){


            for(int i=0; i<mapimage.rows; i++){

                cv::Vec4b* map_ptr = mapimage.ptr<cv::Vec4b>( i );
                cv::Vec4b* seg_ptr = segimage.ptr<cv::Vec4b>( i );

                for(int j=0; j<mapimage.cols; j++){

                    cv::Vec4b map_bgr = map_ptr[ j ];
                    cv::Vec4b seg_bgr = seg_ptr[ j ];

                    tmp_score = 0.0;

                    map_b = map_bgr[0];
                    map_g = map_bgr[1];
                    map_r = map_bgr[2];

                    seg_b = seg_bgr[0];
                    seg_g = seg_bgr[1];
                    seg_r = seg_bgr[2];
                    
                    double corres_checker = cmp_pixel(seg_b,seg_g,seg_r,map_b,map_g,map_r);

                    if(map_b==0 && map_g==0 && map_r==0){
                        corres_checker = 0.0;
                    }

                    if(corres_checker > 0.0){
                        tmp_score = corres_checker;
                    }

                    score += tmp_score;
                }
            }
        }
        else{
            ROS_ERROR("Image size between Segimage and Mapimage is different");

            std::cout << "Segimage" << std::endl;
            std::cout << "Row: " << segimage.rows << std::endl;
            std::cout << "Col: " << segimage.cols << std::endl;

            std::cout << "Mapimage" << std::endl;
            std::cout << "Row: " << mapimage.rows << std::endl;
            std::cout << "Col: " << mapimage.cols << std::endl;
        }

        end_cmp = ros::Time::now();

        compare_time += end_cmp.toSec() - start_cmp.toSec();

        return score;
    }

    double MeshLocalization::cmp_pixel(double seg_b,double seg_g,double seg_r,double map_b,double map_g,double map_r){
        double diff_r = std::abs( seg_r - map_r );
        double diff_g = std::abs( seg_g - map_g );
        double diff_b = std::abs( seg_b - map_b );

        if( diff_r < 10 && diff_g < 10 && diff_b < 10 ){
            return 1.0;
        }
        
        if( (std::abs(seg_b - 128.0) < 5.0) && (std::abs(seg_g - 64.0) < 5.0) &&  (std::abs(seg_r -   128.0) < 5.0) ){//road
            if( std::abs(map_r - map_b) < 10.0  && map_r/(map_g+1) < 4.0 && map_b/(map_g+1) < 4.0) return 0.7;
        }

        if( (std::abs(seg_b - 232.0) < 5.0) && (std::abs(seg_g - 35.0) < 5.0) &&  (std::abs(seg_r -   244.0) < 5.0) ){//sidewalk
            if( std::abs(map_r - map_b) < 15.0  && map_r/map_g > 4.0 && map_b/map_g > 4.0) return 1.5;
        }

        if( (std::abs(seg_b - 142.0) < 5.0) && (std::abs(seg_g - 0.0) < 5.0) &&  (std::abs(seg_r -   0.0) < 5.0) ){//car
            if( map_g < 10.0 && map_r < 10.0 && map_b > 10.0) return 4.0;
        }

        if( (std::abs(seg_b - 70.0) < 5.0) && (std::abs(seg_g - 70.0) < 5.0) &&  (std::abs(seg_r -   70.0) < 5.0) ){//building
            if( std::abs(map_r - map_g) < 5.0 && std::abs(map_g - map_b) < 5.0 && std::abs(map_b -   map_r) < 5.0){
                return 0.7;
            }
        }

        if( (std::abs(seg_b - 152.0) < 10.0) && (std::abs(seg_g - 251.0) < 10.0) &&  (std::abs(seg_r - 152.0) < 10.0) ){//terrain (shibahu)

            if( std::abs(map_r-map_b) < 5.0 && map_g>map_b && map_g>map_r ) return 2.0;
        }
        return 0.0;
    }


    double MeshLocalization::get_likelihood(geometry_msgs::Pose pose){

        double score = 0.0;
        //Time start_i;
        //Time end_i;

        if(     pose.orientation.x == 0.0 &&
                pose.orientation.y == 0.0 &&
                pose.orientation.z == 0.0 &&
                pose.orientation.w == 0.0   ){

            score =  0.0;
        
        }
        else{

            //start_i = ros::Time::now();
            change_camera_position_for_particle( pose );
            //end_i = ros::Time::now();
            //std::cout << "Change Camera Position :" << end_i.toSec()-start_i.toSec()<<std::endl;

            //start_i = ros::Time::now();
            get_image_from_pcl_visualizer();
            //end_i = ros::Time::now();
            //std::cout << "Change Camera Position :" << end_i.toSec()-start_i.toSec()<<std::endl;

            //start_i = ros::Time::now();
            score = compare_image_pixelwise();
            //end_i = ros::Time::now();
            //std::cout << "Change Camera Position :" << end_i.toSec()-start_i.toSec()<<std::endl;

            //std::cout << std::endl;
        }

        
        //score = ( (double)rand() / ((double)RAND_MAX + 1) ); //for test
        return score;
    }

    void MeshLocalization::update_likelihood(){
        
        double total_likelihood = 0.0;

        Time start_li = ros::Time::now();
        Time end_li;

        for(size_t i=0; i<likelihood.size(); i++){
            likelihood[i] = get_likelihood( particle.poses[i] );
            total_likelihood += likelihood[i];
            //std::cout << likelihood[i] << std::endl;
        }

        if(minimum_likelihood_checker == true){
            double minimum_likelihood = 100000000.0;

            for(size_t i=0; i<likelihood.size(); i++){
                if( likelihood[i] < minimum_likelihood ) minimum_likelihood = likelihood[i];
            }

            minimum_likelihood = minimum_likelihood - 0.00001;

            double sub_total_likelihood = 0.0;
            for( size_t i=0; i < likelihood.size(); i++){
                likelihood[i] = likelihood[i] - minimum_likelihood;
                sub_total_likelihood += likelihood[i];
            }

            total_likelihood = sub_total_likelihood;
        }

        if(total_likelihood < 0.01){
            ROS_INFO("Invalid total likelihood");
            for(size_t i=0; i<likelihood.size(); i++){
                likelihood[i] = ( (double)rand() / ((double)RAND_MAX + 1) );
            }
        }

        //Normalize likelihood
        for(size_t i=0; i<likelihood.size(); i++){
            likelihood[i] = likelihood[i] / total_likelihood;
            //std::cout << likelihood[i] << std::endl;
        }
        
        std::cout << "Change camera Time :" << camera_time << "[s]" << std::endl;

        std::cout << "VTK Image Time :" << get_image_vtk_time<< "[s]" << std::endl;

        std::cout << "Image Compare Time :" << compare_time<< "[s]" << std::endl;
        

        camera_time = 0.0;
        get_image_vtk_time = 0.0;
        compare_time = 0.0;

    }

    geometry_msgs::PoseStamped MeshLocalization::max_likelihood_approach(){
        geometry_msgs::PoseStamped _pose;

        _pose.header.frame_id = "map";
        _pose.header.stamp = odom_data.header.stamp;

        double tmp_likelihood = -0.1;
        for(size_t i=0; i<particle.poses.size(); i++){

            if( likelihood[i] > tmp_likelihood ){
                tmp_likelihood = likelihood[i];

                _pose.pose.position = particle.poses[i].position;
                _pose.pose.orientation = particle.poses[i].orientation;
            }
        }

        /*
        std::cout << "estimated pose" << std::endl;
        std::cout << estimated_pose << std::endl;
        */

        return _pose;
    }

    geometry_msgs::PoseStamped MeshLocalization::average_likelihood_approach(){
        geometry_msgs::PoseStamped _pose;
        _pose.header.frame_id = "map";
        _pose.header.stamp = odom_data.header.stamp;

        geometry_msgs::PoseArray sorted_pose = particle;
        std::vector<double> sorted_likelihood = likelihood;

        double tmp_sorted_likelihood;
        geometry_msgs::Pose tmp_sorted_pose;

        bool sorted_checker = false;

        while(!sorted_checker){
            int sorting_count = 0;
            
            for(size_t i=0; i<sorted_likelihood.size()-1; i++){
                if(sorted_likelihood[i] < sorted_likelihood[i+1]){
                    sorting_count += 1;

                    tmp_sorted_likelihood = sorted_likelihood[i];
                    tmp_sorted_pose = sorted_pose.poses[i];

                    sorted_likelihood[i] = sorted_likelihood[i+1];
                    sorted_pose.poses[i] = sorted_pose.poses[i+1];

                    sorted_likelihood[i+1] = tmp_sorted_likelihood;
                    sorted_pose.poses[i+1] = tmp_sorted_pose;
                }
            }

            if(sorting_count == 0) sorted_checker = true;
        }

        /*
        std::cout << "Sorted likelihood" << std::endl;
        for(size_t i=0; i<sorted_likelihood.size(); i++){
            std::cout << sorted_likelihood[i] << std::endl;
        }
        */

        _pose.pose.orientation = sorted_pose.poses[0].orientation;

        double ave_x = 0.0;
        double ave_y = 0.0;
        double ave_z = 0.0;

        int sorted_index = int(average_number);

        for(int i=0; i<sorted_index; i++){
            ave_x += sorted_pose.poses[i].position.x;
            ave_y += sorted_pose.poses[i].position.y;
            ave_z += sorted_pose.poses[i].position.z;
        }

        ave_x = ave_x/average_number;
        ave_y = ave_y/average_number;
        ave_z = ave_z/average_number;

        _pose.pose.position.x = ave_x;
        _pose.pose.position.y = ave_y;
        _pose.pose.position.z = ave_z;


        return _pose;
    }

    void MeshLocalization::estimate_current_pose(){
        //最も尤度の高いパーティクルを現在位置として推定する
        //estimated_pose = max_likelihood_approach();
        estimated_pose = average_likelihood_approach();

        //TF broadcast
        car_state.header.stamp = odom_data.header.stamp;
        car_state.transform.translation.x = estimated_pose.pose.position.x;
        car_state.transform.translation.y = estimated_pose.pose.position.y;
        car_state.transform.translation.z = estimated_pose.pose.position.z;
        car_state.transform.rotation = estimated_pose.pose.orientation;

        /*
        std::cout << "Car state" << std::endl;
        std::cout << car_state << std::endl;
        */
        br.sendTransform( car_state );
    }

    void MeshLocalization::resampling_particle(){

        start_re = ros::Time::now();
        
        //std::cout << "Resampling first" << std::endl;

        std::vector<double> acc_weight;
        double acc=0.0;
        double step_width=0.0;
        double choose_init=0.0;

        for(size_t i=0; i<particle.poses.size(); i++){
            acc += likelihood[i];
            acc_weight.push_back(acc);
        }

        if(acc < 0.00001){
            for(size_t i=0; i<particle.poses.size(); i++){
                acc += 0.01;
                acc_weight[i] = acc;
            }
        }

        //std::cout << "Resaampling second" << std::endl;

        step_width = acc / double( particle.poses.size() );
        
        //一様分布の中から値を抽出する
        //choose_init = ( (double)rand() / ((double)RAND_MAX + 1) ) * acc;

        std::random_device uni_seed_gen;
        std::default_random_engine uni_engine(uni_seed_gen());
        std::uniform_real_distribution<double> uni_dist(0.0,step_width);

        choose_init = uni_dist(uni_engine);

        int index = 0;
        double t = choose_init;

        std::vector<double> new_likelihood;

        geometry_msgs::PoseArray new_particle;
        //new_particle.header.frame_id = "map";
        //new_particle.header.stamp = odom_data.header.stamp;
        /*
        std::cout << "choose_init" << std::endl;
        std::cout << t << std::endl;

        std::cout << "Step_width" << std::endl;
        std::cout << step_width << std::endl;
        */

        //std::cout << "Resampling third" << std::endl;

        while( new_likelihood.size() < likelihood.size() ){
            if(t < acc_weight[index]){
                new_particle.poses.push_back( particle.poses[index] );
                new_likelihood.push_back( likelihood[index] );

                t += step_width;
                if( t > acc ){
                    t = t - acc;
                }
            }
            else{
                index += 1;
                if(index >= int( acc_weight.size() )  ){
                    index = 0;
                }
                //std::cout << index << std::endl;
            }
        }

        //std::cout << "Particle resampling done" << std::endl;
        
        particle = new_particle;
        likelihood = new_likelihood;

        end_re = ros::Time::now();
        //std::cout << "Resampling Time: "<< end_re.toSec() - start_re.toSec() << "[s]" << std::endl;


    }

    void MeshLocalization::publish_result(){
        
        biased_odom.header.frame_id = "map";
        biased_odom.header.stamp = odom_data.header.stamp;

        estimated_pose.header.frame_id = "map";
        particle.header.frame_id = "map";

        estimated_pose.header.stamp = odom_data.header.stamp;
        particle.header.stamp = odom_data.header.stamp;

        pub_pose.publish(estimated_pose);
        pub_particle.publish(particle);
        pub_biased_odometry.publish(biased_odom);
        ground_truth_pub.publish(odom_data);
    }

    void MeshLocalization::publish_as_csv(std::ofstream& groundtruth_csv, std::ofstream& odometry_csv, std::ofstream& estimated_csv, std::ofstream& biased_odom_csv){

        //catch groundtruth data from bag file
        while(true){
            try{
                listener.waitForTransform("map" , "ground_truth", last_odom_data.header.stamp, ros::Duration(1.0));
                listener.lookupTransform("map" , "ground_truth" , last_odom_data.header.stamp, map_to_groundtruth);
                break;
            }
            catch(tf::TransformException ex){
                ROS_ERROR("%s", ex.what() );
                ros::Duration(0.1).sleep();
                break;
            }
        }

        double x_gt = map_to_groundtruth.getOrigin().x();
        double y_gt = map_to_groundtruth.getOrigin().y();
        double z_gt = map_to_groundtruth.getOrigin().z();

        groundtruth_csv << odom_data.header.stamp << "," 
                        << x_gt << "," 
                        << y_gt << "," 
                        << z_gt << "," << std::endl;

        odometry_csv << odom_data.header.stamp << ","
                     << odom_data.pose.pose.position.x << ","
                     << odom_data.pose.pose.position.y << ","
                     << odom_data.pose.pose.position.z << "," 
                     << odom_data.pose.pose.orientation.x << ","
                     << odom_data.pose.pose.orientation.y << ","
                     << odom_data.pose.pose.orientation.z << ","
                     << odom_data.pose.pose.orientation.w << std::endl;

        estimated_csv << odom_data.header.stamp << "," 
                      << estimated_pose.pose.position.x << ","
                      << estimated_pose.pose.position.y << ","
                      << estimated_pose.pose.position.z << "," 
                      << estimated_pose.pose.orientation.x << ","
                      << estimated_pose.pose.orientation.y << ","
                      << estimated_pose.pose.orientation.z << ","
                      << estimated_pose.pose.orientation.w << ","
                      << std::endl;

        biased_odom_csv << biased_odom.header.stamp << ","
                        << biased_odom.pose.pose.position.x << ","
                        << biased_odom.pose.pose.position.y << ","
                        << biased_odom.pose.pose.position.z << "," 
                        << biased_odom.pose.pose.orientation.x << ","
                        << biased_odom.pose.pose.orientation.y << ","
                        << biased_odom.pose.pose.orientation.z << ","
                        << biased_odom.pose.pose.orientation.w << ","
                        << std::endl;

    }

    void MeshLocalization::save_image( /*std::ofstream& place_data_csv*/ ){

        std::string file_number = std::to_string(image_counter);
        std::string file_path = seg_image_path + "image" + file_number + ".jpg";

/*
        place_data_csv << file_number << ","
            << odom_data.header.stamp << ","
            << odom_data.pose.pose.position.x << ","
            << odom_data.pose.pose.position.y << ","
            << odom_data.pose.pose.position.z << "," 
            << odom_data.pose.pose.orientation.x << ","
            << odom_data.pose.pose.orientation.y << ","
            << odom_data.pose.pose.orientation.z << ","
            << odom_data.pose.pose.orientation.w << std::endl;
*/

        cv::imwrite( file_path , segimage );

        image_counter += 1;
    }


}



