//
// Created by edward on 17-12-20.
//

#ifndef IRONMAP_IRONMAP_H
#define IRONMAP_IRONMAP_H


//
// Created by edward on 17-12-20.
//



#include<ros/ros.h>
#include<iostream>
#include <stdio.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
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

#include "TransformCalculation.h"
#include "KeyframeDatabase.h"
#include"LoopClosure.h"

void BuildGlobalmap(vector<KeyFrame2> &KeyFrameDatabase2);

int SkipLocalframe;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_old(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr GICP_cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr GICP_cloud_old(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr RemoveNaNcloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointNormal>::Ptr OptimizedGlobalCloud(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr GlobalMapCloud(new pcl::PointCloud<pcl::PointNormal>);
vector<Matrix4f> OptimizedPose;
vector<KeyFrame2> KeyFrameDatabase;
Eigen::Matrix4f pose(4, 4);

using namespace IRON;
namespace DEPTH_MAP {

    class Matrixtransform;

    class IRONmap {
        typedef pcl::PointCloud<pcl::PointNormal> DP;
        // static const std::string pointstopic = "/camera1/depth/points";

        ros::Publisher depthFilter_pub;
        ros::Publisher GICPcloudInput_pub;
        ros::Publisher pass_cloud_pub;
        ros::Publisher localmapPub;
        ros::Publisher processlocalmapPub;
        ros::Publisher globalmapPub;
        ros::Publisher afterTranmapPub;
        ros::Publisher passThrough_cloud;
        ros::Publisher Removed_cloud;
        ros::Publisher Normed_cloud;

        ros::Publisher OptimizedPathPub;
        ros::Subscriber pointcloud_sub;
        ros::Subscriber pointcloudfrombag_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber realsense_pose_sub;
        nav_msgs::Path OptimizedPath;

        int downSizeNum;
        double mapVoxelLeafSize;
        //Define some transformation matrix
        Eigen::Matrix4f Trans_Odom2Map;

        Eigen::Matrix4f aftIRON;


    public:
        ros::NodeHandle &nh;
        ros::NodeHandle &n;

        IRONmap(ros::NodeHandle &nh, ros::NodeHandle &n);

        ~IRONmap();

        void depthFilter(const sensor_msgs::PointCloud2ConstPtr &msg);

        void RemoveNaNPoints(pcl::PCLPointCloud2 &msg_filtered2);

        void loadepthPointFrombag(const sensor_msgs::PointCloud2ConstPtr &msg);

        void IRONodometry(sensor_msgs::PointCloud2 cloud_filtered);

        void GICPodometry(sensor_msgs::PointCloud2 cloud_filtered);

        void GICPodometry2(pcl::PointCloud<pcl::PointXYZ> cloud_filtered, const ros::Time timestamp);

        void posetransMatrix(const nav_msgs::Odometry::ConstPtr &msg);

        void PublishTrajectory(vector<Matrix4f> OptimizedPose);
        //void posetransMatrixfromrealsense(const nav_msgs::Odometry::ConstPtr &msg);



        Eigen::Matrix3d Quaternion2RotationMatrix(double x, double y, double z, double w);

        void buildLocalmap(sensor_msgs::PointCloud2 &cloud_filtered);

        bool first_run;
        bool GICP_first_run;
        bool first_localmap;
        bool createGlobalMap;
        bool fisrtLocalMap;
        bool loadbag;
        bool first_slidewindow;
        bool loopclosure;
        bool storeLastlocalframe;


        int SkipframeNum;
        int localpointNum;
        int selectedPtNum;
        string sourceFrame;
        string targetFrame;

        //关于IRONTransform计算的变量
        TransformCalculation *TransformPtr;
        double subsamplingFactor;
        double cellSize;
        double clippingDistance;
        double neighborSearchRadius;
        double entropyThreshold;
        double matchingTolerance;

        void process_pose_graph();

        //store the NDTMAP keyframe
        NDTMapLitef LastndtMap, CurrndtMap;

        //关于keyframe的变量
        KeyframeDatabase *GlobalKeyframePtr;
        int GlobalKeyframeID;
        int LocalKeyframeID;
        int SidewindowSize;
        double KeyframeTimestamp;
        pcl::PointCloud<pcl::PointNormal> CurrKeyframePoint;
        Eigen::Matrix4f Trans_Keyframe2MAP;


        KeyFrame2 LastLocalKeyframe;

        vector<Pose> localPoseafterG2o;

        //关于LoopClosure的变量
        LoopClosure *LoopPtr;
        LoopClosure *GlobalLoopPtr;


    };
}

#endif //IRONMAP_IRONMAP_H





