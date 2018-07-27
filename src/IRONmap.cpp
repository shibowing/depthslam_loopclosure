//
// Created by edward on 17-12-20.
//

#include"IRONmap.h"
#include <mutex>

namespace DEPTH_MAP {
    IRONmap::IRONmap(ros::NodeHandle &n, ros::NodeHandle &nh)
            : n(n), nh(nh), first_run(true),
              createGlobalMap(true), first_localmap(true), first_slidewindow(true),
              Trans_Odom2Map(Matrix4f::Identity()), loopclosure(false) {
        pose = Eigen::Matrix4f::Identity();
        GlobalKeyframeID = 0;
        LocalKeyframeID = 0;

        //设置下采样，采样点的数目
        nh.param<int>("DownSizeNum", downSizeNum, 150000);
        nh.param<string>("sourceFrame", sourceFrame, "/world");
        nh.param<string>("targetFrame", targetFrame, "/camera_rgb_optical_frame");
        nh.param<bool>("first_run", first_run, true);
        nh.param<bool>("GICP_first_run", GICP_first_run, true);
        nh.param<bool>("createGlobalMap", createGlobalMap, true);
        nh.param<bool>("loadbagfile", loadbag, false);
        nh.param<bool>("loopclosure", loopclosure, true);
        nh.param<bool>("storeLastlocalframe", storeLastlocalframe, false);
        nh.param<double>("mapVoxelLeafSize", mapVoxelLeafSize, 0.05);
        nh.param<int>("localpointmapNum", localpointNum, 20000);
        nh.param<int>("selectedPtNum", selectedPtNum, 8000);

        nh.param<int>("SkipframeNum", SkipframeNum, 4);
        nh.param<int>("SidewindowSize", SidewindowSize, 11);

        nh.param<double>("cellSize", cellSize, 0.075);
        nh.param<double>("clippingDistance", clippingDistance, 4.5);
        nh.param<double>("neighborSearchRadius", neighborSearchRadius, 0.6);
        nh.param<double>("entropyThreshold", entropyThreshold, 0.7);
        nh.param<double>("matchingTolerance", matchingTolerance, 0.035);
        nh.param<double>("subsamplingFactor", subsamplingFactor, 0.2);
        TransformPtr = new TransformCalculation(*cloud_new, *cloud_old, Trans_Odom2Map,
                                                cellSize, subsamplingFactor, clippingDistance, matchingTolerance,
                                                entropyThreshold, neighborSearchRadius);
        GlobalKeyframePtr = new KeyframeDatabase(GlobalKeyframeID, LocalKeyframeID, SidewindowSize, KeyframeTimestamp,
                                                 Trans_Keyframe2MAP, CurrndtMap);


        pointcloud_sub = n.subscribe("/camera/depth/points2", 1, &IRONmap::depthFilter, this);
        localmapPub = n.advertise<sensor_msgs::PointCloud2>("/local_point_map", 2, true);
        processlocalmapPub = n.advertise<sensor_msgs::PointCloud2>("/process_local_point_map", 2, true);
        depthFilter_pub = n.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_filter", 1);
        GICPcloudInput_pub = n.advertise<sensor_msgs::PointCloud2>("/camera/depth/GICPinput", 1);
        passThrough_cloud = n.advertise<sensor_msgs::PointCloud2>("/passThrough/cloud", 2, true);
        Removed_cloud = n.advertise<sensor_msgs::PointCloud2>("/removed/cloud", 2, true);
        Normed_cloud = n.advertise<sensor_msgs::PointCloud2>("/Normed/cloud", 2, true);
        globalmapPub = n.advertise<sensor_msgs::PointCloud2>("/GlobalMap", 2, true);
        afterTranmapPub = n.advertise<sensor_msgs::PointCloud2>("/aftertran_point_map", 2, true);

    }

    IRONmap::~ IRONmap() {

        delete TransformPtr;

    };

    void IRONmap::depthFilter(const sensor_msgs::PointCloud2ConstPtr &msg) {

        //  ROS_INFO("start filtering");
        ros::WallTime startTime = ros::WallTime::now();
        pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2 msg_filtered;
        pcl_conversions::toPCL(*msg, *cloud);

        pcl::RandomSample<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloudPtr);
        sor.setSample(downSizeNum);
        sor.setSeed(rand());
        sor.filter(msg_filtered);
        //时间差，计算预处理的时间
        sensor_msgs::PointCloud2 Filterpoint;
        pcl_conversions::fromPCL(msg_filtered, Filterpoint);
        Filterpoint.header.frame_id = msg->header.frame_id;
        depthFilter_pub.publish(Filterpoint);

        //对于GICP进行下采样
        ros::WallTime startTime2 = ros::WallTime::now();
        pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloud2Ptr(cloud2);
        pcl::PCLPointCloud2 msg_filtered2;
        pcl_conversions::toPCL(*msg, *cloud2);

        pcl::RandomSample<pcl::PCLPointCloud2> sor2;
        sor2.setInputCloud(cloud2Ptr);
        sor2.setSample(selectedPtNum);
        sor2.setSeed(rand());
        sor2.filter(msg_filtered2);


        pcl::PCLPointCloud2 *passCloud = new pcl::PCLPointCloud2;
        pcl::copyPointCloud(msg_filtered2, *passCloud);
        pcl::PCLPointCloud2ConstPtr PasscloudPtr(passCloud);
        pcl::PCLPointCloud2 pass_filtered;
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(PasscloudPtr);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 4.5);
        pass.filter(pass_filtered);
        RemoveNaNPoints(pass_filtered);

        sensor_msgs::PointCloud2 Filterpoint2;
        pcl_conversions::fromPCL(pass_filtered, Filterpoint2);
        Filterpoint2.header.frame_id = "world";
        GICPcloudInput_pub.publish(Filterpoint2);
        ROS_INFO_STREAM ("pass point cloud size: " << Filterpoint2.width * Filterpoint2.height);

        if (SkipLocalframe == SkipframeNum) {
            buildLocalmap(Filterpoint);
            GICPodometry2(*RemoveNaNcloud, Filterpoint2.header.stamp);

            if (GlobalLoopPtr->Find_Loop == false) {

                pcl::PointCloud<pcl::PointNormal>::Ptr CurrentFrame(new pcl::PointCloud<pcl::PointNormal>);
                cout << "Find_Loop:" << boolalpha << GlobalLoopPtr->Find_Loop << endl;
                pcl::transformPointCloud(CurrKeyframePoint, *CurrentFrame, pose);

                *GlobalMapCloud += *CurrentFrame;


                sensor_msgs::PointCloud2 GlobalMapCloudMsg;
                pcl::toROSMsg(*GlobalMapCloud, GlobalMapCloudMsg);
                GlobalMapCloudMsg.header.frame_id = "world";


                ROS_INFO_STREAM ("GlobalMapCloudMsg size: " << GlobalMapCloudMsg.width * GlobalMapCloudMsg.height);

                // Publish map point cloud
                if (globalmapPub.getNumSubscribers()) {

                    globalmapPub.publish(GlobalMapCloudMsg);
                }
                CurrentFrame->clear();
            }

            // mutex.unlock();
            SkipLocalframe = 0;
        }
        SkipLocalframe++;

    }


    void IRONmap::RemoveNaNPoints(pcl::PCLPointCloud2 &msg_filtered2) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(msg_filtered2, *temp_cloud);

        std::vector<int> temp;
        pcl::removeNaNFromPointCloud(*temp_cloud, *temp_cloud, temp);
        pcl::removeNaNFromPointCloud(*temp_cloud, *temp_cloud, temp);

        // Fill in the CloudIn data
        RemoveNaNcloud->width = temp_cloud->width;
        RemoveNaNcloud->height = temp_cloud->height;
        RemoveNaNcloud->is_dense = false;
        RemoveNaNcloud->points.resize(RemoveNaNcloud->width * RemoveNaNcloud->height);

        for (size_t i = 0; i < RemoveNaNcloud->points.size(); ++i) {
            RemoveNaNcloud->points[i].x = temp_cloud->points[i].x;
            RemoveNaNcloud->points[i].y = temp_cloud->points[i].y;
            RemoveNaNcloud->points[i].z = temp_cloud->points[i].z;
        }

    }


    void IRONmap::buildLocalmap(sensor_msgs::PointCloud2 &cloud_filtered) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr CurrlocalmapCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointNormal>::Ptr ICPmapCloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr AfterICPmapCloud(new pcl::PointCloud<pcl::PointNormal>);
        if (first_run) {

            if ((cloud_filtered.height * cloud_filtered.width) > 0) {
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(cloud_filtered, cloud);

                std::vector<int> temp;
                pcl::removeNaNFromPointCloud(cloud, cloud, temp);

                // Fill in the CloudIn data
                cloud_new->width = cloud.width;
                cloud_new->height = cloud.height;
                cloud_new->is_dense = false;
                cloud_new->points.resize(cloud_new->width * cloud_new->height);
                for (size_t i = 0; i < cloud_new->points.size(); ++i) {
                    cloud_new->points[i].x = cloud.points[i].x;
                    cloud_new->points[i].y = cloud.points[i].y;
                    cloud_new->points[i].z = cloud.points[i].z;
                }

                *cloud_old = *cloud_new;

                first_run = false;
            }
        }

        if (first_run == false) {
            ros::WallTime start = ros::WallTime::now();

            if ((cloud_filtered.height * cloud_filtered.width) > 0) {
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(cloud_filtered, cloud);

                std::vector<int> temp;
                pcl::removeNaNFromPointCloud(cloud, cloud, temp);

                // Fill in the CloudIn data
                cloud_new->width = cloud.width;
                cloud_new->height = cloud.height;
                cloud_new->is_dense = false;
                cloud_new->points.resize(cloud_new->width * cloud_new->height);
                ros::WallTime cloudcopytime = ros::WallTime::now();
                for (size_t i = 0; i < cloud_new->points.size(); ++i) {
                    cloud_new->points[i].x = cloud.points[i].x;
                    cloud_new->points[i].y = cloud.points[i].y;
                    cloud_new->points[i].z = cloud.points[i].z;
                }

            }

            ros::WallTime startTime = ros::WallTime::now();
            Eigen::Matrix4f CurrIntialGuess;
            CurrIntialGuess = Matrix4f::Identity();
            ROS_INFO ("The size of cloud_new %d", cloud_new->width * cloud_new->height);
            ROS_INFO ("The size of cloud_old %d", cloud_old->width * cloud_old->height);
            CurrndtMap = TransformPtr->ExtractNdtmap(cloud_new, CurrIntialGuess);

            pcl::copyPointCloud(*RemoveNaNcloud, *ICPmapCloud);
            //  pcl::transformPointCloud(*ICPmapCloud, *AfterICPmapCloud, pose);

            CurrKeyframePoint = *ICPmapCloud;
            ROS_INFO ("The size of CurrKeyframePoint %d", CurrKeyframePoint.size());
            //   std::cerr << " CurrndtMap..: " << CurrndtMap.cells.size() << " cells\n";
            std::cout << "Total NDTMAP BUILDING took: " << (ros::WallTime::now() - startTime).toSec() << std::endl;


            if (loopclosure) {
                // cout << " GlobalKeyframeID:" << GlobalKeyframeID << endl;
                //cout << "  LocalKeyframeID:" << LocalKeyframeID << endl;
                ros::WallTime graphTime = ros::WallTime::now();
                GlobalKeyframeID = GlobalKeyframeID + 1;
                LocalKeyframeID = LocalKeyframeID + 1;
                KeyframeTimestamp = (ros::WallTime::now()).toSec();
                Trans_Keyframe2MAP = pose;


                std::cerr << " CurrndtMap..: " << CurrndtMap.cells.size() << " cells\n";
                std::thread Posegraph(&IRONmap::process_pose_graph, this);
                // Posegraph.detach();
                Posegraph.join();
                std::cout << "Total Posegraph time took: " << (ros::WallTime::now() - graphTime).toSec()
                          << std::endl;

            }

            *cloud_old = *cloud_new;


        }

        //delete TransformPtr;
    }


    void IRONmap::process_pose_graph() {

        ros::WallTime graphTime = ros::WallTime::now();
        GlobalKeyframePtr->AddGlobalKeyframe(GlobalKeyframeID, LocalKeyframeID, KeyframeTimestamp,
                                             Trans_Keyframe2MAP, CurrndtMap, CurrKeyframePoint);
        std::cout << "Add frame time took: " << (ros::WallTime::now() - graphTime).toSec()
                  << std::endl;

        if (LocalKeyframeID == SidewindowSize) {
            LocalKeyframeID = 0;
        }


        if (GlobalKeyframePtr->KeyFrameDatabase2.size() >= SidewindowSize) {

            ros::WallTime graphTime2 = ros::WallTime::now();
            GlobalLoopPtr = new LoopClosure(first_slidewindow, GlobalKeyframePtr, TransformPtr);

            OptimizedPose = GlobalLoopPtr->IntialGlobalPoseGraph(GlobalKeyframePtr->KeyFrameDatabase2);
            KeyFrameDatabase = GlobalKeyframePtr->KeyFrameDatabase2;


            std::cout << "global posegraph took: " << (ros::WallTime::now() - graphTime2).toSec()
                      << std::endl;


            first_slidewindow = false;

            delete GlobalLoopPtr;

        } else {
            ROS_INFO ("The size of LocalKeyframeDatabase [%d] ", LocalKeyframeID);
        }


    }


    void IRONmap::GICPodometry2(pcl::PointCloud<pcl::PointXYZ> cloud_filtered, const ros::Time timestamp) {

        if (GICP_first_run) {
            if ((cloud_filtered.height * cloud_filtered.width) > 0) {
                *GICP_cloud_new = cloud_filtered;
                *GICP_cloud_old = *GICP_cloud_new;
                GICP_first_run = false;
            }
        }

        if (GICP_first_run == false) {
            ros::WallTime start = ros::WallTime::now();
            pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);

            *GICP_cloud_new = cloud_filtered;

            pcl::copyPointCloud(*GICP_cloud_new, *src);
            pcl::copyPointCloud(*GICP_cloud_old, *tgt);

            pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
            norm_est.setSearchMethod(
                    pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
            norm_est.setKSearch(5);
            norm_est.setInputCloud(tgt);
            norm_est.compute(*tgt);

            //GICP
            pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> gicp;
            gicp.setInputSource(src);
            gicp.setInputTarget(tgt);


            pcl::PointCloud<pcl::PointNormal> Final1;
            gicp.align(Final1);
            std::cout << "GICP has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore()
                      << std::endl;
            pose = pose * gicp.getFinalTransformation();

            Eigen::Matrix4f Ticp;
            Ticp << pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3),
                    pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
                    pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3),
                    0, 0, 0, 1;
            // buildGlobalmap(Final1, Ticp);


            std::cout << "x = " << pose(0, 3) << "  y = " << pose(1, 3) << "   z= " << pose(2, 3) << std::endl;

            static tf::TransformBroadcaster br;
            tf::Matrix3x3 rot_mat(
                    pose(0, 0), pose(0, 1), pose(0, 2),
                    pose(1, 0), pose(1, 1), pose(1, 2),
                    pose(2, 0), pose(2, 1), pose(2, 2));
            tf::Vector3 t(pose(0, 3), pose(1, 3), pose(2, 3));
            tf::Transform transform(rot_mat, t);

            br.sendTransform(
                    tf::StampedTransform(transform, timestamp, "/world",
                                         "/camera_rgb_optical_frame"));

            Eigen::Matrix3d R;
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    R(row, col) = pose(row, col);
                }
            }
            std::vector<float> euler(3);
            euler[0] = atan(R(1, 2) / R(2, 2));
            euler[1] = asin(-R(0, 2));
            euler[2] = atan(R(0, 1) / R(0, 0));
            Eigen::Quaterniond quat(R);
            std::cout << "Total GICP estimation took: " << (ros::WallTime::now() - start).toSec() << endl << endl;
            *GICP_cloud_old = *GICP_cloud_new;

        }
    }


    void IRONmap::posetransMatrix(const nav_msgs::Odometry::ConstPtr &msg) {
        Eigen::Matrix4f FinalT = Matrix4f::Identity();
        double rx, ry, rz, rw, x, y, z;
        rx = msg->pose.pose.orientation.x;
        ry = msg->pose.pose.orientation.y;
        rz = msg->pose.pose.orientation.z;
        rw = msg->pose.pose.orientation.w;
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        z = msg->pose.pose.position.z;

        Matrixtransform rotationMatrix;
        Eigen::Matrix3d R = rotationMatrix.Quaternion2RotationMatrix(rx, ry, rz, rw);

        //Eigen::Vector3d t
        Eigen::Matrix<float, 3, 1> t(x, y, z);
        FinalT << R(0, 0), R(0, 1), R(0, 2), t(0, 0),
                R(1, 0), R(1, 1), R(1, 2), t(1, 0),
                R(2, 0), R(2, 1), R(2, 2), t(2, 0),
                0, 0, 0, 1;
        Trans_Odom2Map = FinalT;
    }


    void IRONmap::PublishTrajectory(vector<Matrix4f> OptimizedPose) {

        OptimizedPath.poses.clear();
        for (int i = 0; i < OptimizedPose.size(); ++i) {
            float tx, ty, tz;
            tx = OptimizedPose[i](0, 3);
            ty = OptimizedPose[i](1, 3);
            tz = OptimizedPose[i](2, 3);

            geometry_msgs::PoseStamped pose;

            pose.pose.position.x = tx;
            pose.pose.position.y = ty;
            pose.pose.position.z = tz;
            OptimizedPath.poses.push_back(pose);
        }

        OptimizedPath.header.frame_id = "world";
        OptimizedPathPub.publish(OptimizedPath);

    }


}


void BuildGlobalmap(vector<KeyFrame2> &KeyFrameDatabase2) {

    cout << "saving the point cloud map..." << endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr PassCloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr OptCloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr VoxelCloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr TransCloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr VoxelGlobalCloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::VoxelGrid<pcl::PointNormal> voxel; // 网格滤波器，调整地图分辨率
    pcl::PassThrough<pcl::PointNormal> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 4.5); //4m以上就不要了
    voxel.setLeafSize(0.03, 0.03, 0.03);
    ros::WallTime startTime3 = ros::WallTime::now();
    cout << "GlobalKeyframePtr->KeyFrameDatabase2.size():" << KeyFrameDatabase2.size() << endl;

    for (size_t i = 0; i < KeyFrameDatabase2.size(); i = i + 3) {

        pcl::copyPointCloud(KeyFrameDatabase2[i].KeyframePoint, *OptCloud);
        /**开始滤波**/
        pass.setInputCloud(OptCloud);
        pass.filter(*PassCloud);
        // 把点云变换后加入全局地图中
        voxel.setInputCloud(PassCloud);
        voxel.filter(*VoxelCloud);
        pcl::transformPointCloud(*VoxelCloud, *TransCloud, OptimizedPose[i + 3]);
        *OptimizedGlobalCloud = *OptimizedGlobalCloud + *TransCloud;
    }

    std::cout << " map building loop took: " << (ros::WallTime::now() - startTime3).toSec() << std::endl;
    voxel.setInputCloud(OptimizedGlobalCloud);
    voxel.filter(*VoxelGlobalCloud);
    ros::WallTime startTime4 = ros::WallTime::now();
    pcl::io::savePCDFileBinary("/home/machozhao/catkin_ws/src/IRON_loopclosure/doc/Optresult.pcd",
                               *VoxelGlobalCloud);
    TransCloud->clear();
    OptimizedGlobalCloud->clear();
    cout << "optimized map is saved." << endl;
    std::cout << "Saving PCD time: " << (ros::WallTime::now() - startTime4).toSec() << std::endl;

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "IRONmap");

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    DEPTH_MAP::IRONmap IRONmap_node(n, nh);

    while (ros::ok()) {

        ros::spinOnce();
    }
    usleep(2000);
    std::cout << "OVER!" << std::endl;

    //  BuildGlobalmap(KeyFrameDatabase);

    return 0;
}
