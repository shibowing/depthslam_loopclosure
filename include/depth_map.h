//
// Created by edward on 17-10-12.
//

#ifndef DEPTH_MAP_DEPTH_MAP_H
#define DEPTH_MAP_DEPTH_MAP_H

#include<ros/ros.h>
#include<iostream>
#include <stdio.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/normal_space.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/gicp.h>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>


#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string>
#include<cstdlib>
#include<queue>
#include<thread>

using namespace Eigen;
using namespace std;


#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


#include "KeyframeDatabase.h"
#include"LoopClosure.h"
#include "TransformCalculation.h"

namespace DEPTH_MAP {

    class KeyframeDatabase;

    class LoopClosure;

    class depthmap {
        typedef pcl::PointCloud<pcl::PointNormal> DP;
        // static const std::string pointstopic = "/camera1/depth/points";
        ros::NodeHandle &nh_;
        ros::NodeHandle &n_;
        ros::Publisher depthFilter_pub;
        ros::Publisher pass_cloud_pub;
        ros::Publisher localmapPub;
        ros::Publisher processlocalmapPub;
        ros::Publisher globalmapPub;
        ros::Publisher afterTranmapPub;
        ros::Publisher passThrough_cloud;
        ros::Publisher Removed_cloud;
        ros::Publisher Normed_cloud;

        ros::Publisher newPathPub;
        ros::Subscriber pointcloud_sub;
        ros::Subscriber pointcloudfrombag_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber realsense_pose_sub;


        //  DP *tempCloud;

        boost::thread publishThread;
        boost::mutex publishLock;
        ros::Time publishStamp;

        string odomFrame;
        string mapFrame;
        string localFrame;
        double mapVoxelLeafSize;
        //Define some transformation matrix
        Eigen::Matrix4f Trans_Odom2Map;

        Eigen::Matrix4f aftIcp_pose;

        int downSizeNum;

    public:

        depthmap(ros::NodeHandle &nh, ros::NodeHandle &n);

        ~depthmap();

        void depthFilter(const sensor_msgs::PointCloud2ConstPtr &msg);

        void loadepthPointFrombag(const sensor_msgs::PointCloud2ConstPtr &msg);

        void buildGlobalmap(DP *newPointCloud, Eigen::Matrix4f Ticp);

        void posetransMatrix(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void posetransMatrixfromrealsense(const nav_msgs::Odometry::ConstPtr &msg);

        void process_pose_graph();

        Eigen::Matrix4f
        CalculateTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud,
                           Eigen::Matrix4f pose);

        Eigen::Matrix4f gicp_CalculateTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud, Eigen::Matrix4f pose);

        Eigen::Matrix4f CalculateTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, Eigen::Matrix4f pose);

        Eigen::Matrix3d Quaternion2RotationMatrix(double x, double y, double z, double w);

        void buildLocalmap(DP *newPointCloud, Eigen::Matrix4f Ticp);

        void AddkeyFrameinDataBase(int FrameID, double KeyFrametimeStamp, Matrix4f Trans_Keyframe2Map,
                                   pcl::PointCloud<pcl::PointNormal> MapPoint);

        bool first_run;
        bool first_localmap;
        bool createGlobalMap;
        bool fisrtLocalMap;
        bool loadbag;
        bool first_slidewindow;
        bool loopclosure;
        int Localmapsize;
        int localmapNum;
        int localpointNum;
        int selectedPtNum;

        nav_msgs::Path newPath;

        //关于keyframe的变量
        int SkipKeyframe;
        KeyframeDatabase *GlobalKeyframePtr;
        int GlobalKeyframeID;
        int SidewindowSize;
        double KeyframeTimestamp;
        Eigen::Matrix4f Trans_Keyframe2MAP;
        pcl::PointCloud<pcl::PointNormal> KeyframePoint;
        vector<KeyFrame2> LocalKeyframeDatabase;

        double max_corresdist;
        int max_iterations;
        double max_trans_epsilon;
        int min_number_correspondences;
        double euclidean_fitness_epsilon;
        double rej_max_corresdist;


        //关于LoopClosure的变量
        LoopClosure *LoopPtr;
        LoopClosure *GlobalLoopPtr;

        //关于Transform计算的变量
        TransformCalculation *TransformPtr;


    };
}

#endif //DEPTH_MAP_DEPTH_MAP_H




