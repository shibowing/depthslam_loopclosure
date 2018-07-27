//
// Created by edward on 17-11-10.
//
#ifndef DEPTH_MAP_KEYFRAMEDATABASE_H
#define DEPTH_MAP_KEYFRAMEDATABASE_H

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
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <vector>
#include <math.h>
#include <string>
#include<cstdlib>
#include<queue>

#include "TransformCalculation.h"

using namespace Eigen;
using namespace std;
struct KeyFrame2 {
    int FrameID;
    int LocalFrameID;
    double KeyFrametimeStamp;
    Eigen::Matrix4f Trans_Keyframe2Map;
    NDTMapLitef NDTmap;
    pcl::PointCloud<pcl::PointNormal> KeyframePoint;
};

struct Pose {
    double x;
    double y;
    double z;
};


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT> {
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation() {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray(const PointNormalT &p, float *out) const {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};


namespace DEPTH_MAP {

    class KeyframeDatabase {
    public:
        vector<KeyFrame2> KeyFrameDatabase2;
        vector<KeyFrame2> LocalKeyframeDatabase;

        KeyframeDatabase();

        KeyframeDatabase(vector<KeyFrame2> &KeyF);

        KeyframeDatabase(int FrameID, int LocalFrameID, int SidewindowSize, double KeyFrametimeStamp,
                         Eigen::Matrix4f Trans_Keyframe2Map,
                         NDTMapLitef NDTmap);

        ~KeyframeDatabase();

        struct KeyFrame {
            int FrameID;
            int LocalFrameID;
            double KeyFrametimeStamp;
            Eigen::Matrix4f Trans_Keyframe2Map;
            pcl::PointCloud<pcl::PointNormal> MapPoint;

        };
        int FrameID;
        int LocalFrameID;
        int SidewindowSize;
        double KeyFrametimeStamp;
        double gicp_FitnessScore;
        Eigen::Matrix4f Trans_Keyframe2Map;
        NDTMapLitef NDTmap;
        pcl::PointCloud<pcl::PointNormal> MapPoint;

        typedef std::shared_ptr<KeyFrame2> Ptr;
        vector<KeyFrame> KeyFrameDatabase;

        vector<KeyFrame2>
        AddGlobalKeyframe(int FrameID, int LocalFrameID, double KeyFrametimeStamp, Eigen::Matrix4f Trans_Keyframe2Map,
                          NDTMapLitef &NDTmap, pcl::PointCloud<pcl::PointNormal> &KeyframePoint);

        vector<KeyFrame2>
        AddLocalKeyframe(int FrameID, int LocalFrameID, double KeyFrametimeStamp, Eigen::Matrix4f Trans_Keyframe2Map,
                         NDTMapLitef NDTmap, pcl::PointCloud<pcl::PointNormal> &KeyframePoint);

        vector<KeyFrame2> SelectLocalKeyframe(int LocalKeyframeID);

        Eigen::Matrix3d euler2RotationMatrix(const double yaw, const double pitch, const double roll);

        Eigen::Matrix4f gicp_CalculateTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud,
                                                Eigen::Matrix4f pose);

        Eigen::Matrix4f
        GICP_CalculateTransform(pcl::PointCloud<pcl::PointNormal> newcloud, pcl::PointCloud<pcl::PointNormal> oldcloud,
                                Eigen::Matrix4f pose);

        Eigen::Matrix4f
        pairAlign(pcl::PointCloud<pcl::PointNormal> &cloud_src, pcl::PointCloud<pcl::PointNormal> &cloud_tgt,
                  Eigen::Matrix4f InitialGuess);

    };


}
#endif //DEPTH_MAP_KEYFRAMEDATABASE_H
