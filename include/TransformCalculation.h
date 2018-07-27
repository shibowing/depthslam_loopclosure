//
// Created by edward on 17-12-11.
//

#ifndef DEPTH_MAP_TRANSFORMCALCULATION_H
#define DEPTH_MAP_TRANSFORMCALCULATION_H

#include <fstream>
#include "IRON.h"
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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include"Matrixtransform.h"

using namespace IRON;

namespace DEPTH_MAP {


    class Matrixtransform;

    class TransformCalculation {
    public:
        TransformCalculation();

        TransformCalculation(pcl::PointCloud<pcl::PointXYZ> NewPoint, pcl::PointCloud<pcl::PointXYZ> OldPoint,
                             Eigen::Matrix4f InitialGuess, double cellSize, double subsamplingFactor,
                             double clippingDistance, double matchingTolerance, double entropyThreshold,
                             double neighborSearchRadius);

        ~TransformCalculation();
        ///////////////////////////////
// POINTS AND ACCESSOR:
///////////////////////////////
//
// This is an example data structure which describes a simple 3D point.
// You can, however, use any point type you like, as long as an accessor
// is provided (see below)
//
// In many cases this will prevent expensive conversion between point cloud types

        struct Pt3D {
            Pt3D() {}

            Pt3D(float x, float y, float z) {
                data[0] = x;
                data[1] = y;
                data[2] = z;
            }

            float data[3];

            float &x() {
                return data[0];
            }

            float &y() {
                return data[1];
            }

            float &z() {
                return data[2];
            }
        };


// Whatever point type is provided, this short accessor is needed
// to return const references to x, y, z; so if you cannot modify a
// given point type, you can still use it for NDT-map creation
//
        struct Pt3DAccessor {
            static const float &x(const Pt3D &s) {
                return s.data[0];
            }

            static const float &y(const Pt3D &s) {
                return s.data[1];
            }

            static const float &z(const Pt3D &s) {
                return s.data[2];
            }
        };

        Eigen::Affine3f
        fillCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputcloud, std::vector<Pt3D> &cloud, Eigen::Matrix4f pose);

        void storeMeans(const std::string &file, const NDTMapLitef &map,
                        const Eigen::Affine3f &transform = Eigen::Affine3f::Identity());

        void storeMeans(const std::string &file, const IRONDescriptorVectorf &vec);

        ///////////////////////////////
        // store cov maps
        ///////////////////////////////
        void storeCovs(const char *file, const NDTMapLitef &map, uint samples);

        void storeCovs(const char *file, const IRONDescriptorVectorf &vec, uint samples);

        void storeMatches(const char *file, const IRONMatchVectorf &matches);

        Eigen::Matrix4f CalculateTransform_Process(pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud,
                                                   Eigen::Matrix4f Currpose, double subsamplingFactor,
                                                   double cellSize, double clippingDistance,
                                                   double neighborSearchRadius,
                                                   double entropyThreshold, double matchingTolerance);

        NDTMapLitef ExtractNdtmap(pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud, Eigen::Matrix4f Currpose);

        Eigen::Matrix4f NDT_CalculateTransform(NDTMapLitef ndtMap1, NDTMapLitef ndtMap2, int frameID1, int frameID2);
        //  simple check

        // container for point clouds,
        // sensor poses, ndt maps, descriptors and matches
        std::vector<Pt3D> cloud1, cloud2;

        Eigen::Affine3f sensorPose, sensorPose1,
                sensorPose2;
        NDTMapLitef ndtMap1,
                ndtMap2, ndtMap3;
        IRONDescriptorVectorf descriptors1,
                descriptors2;
        IRONMatchVectorf matches,
                inlierset;
        IRONTransformResultf result;


        //inputcloud
        pcl::PointCloud<pcl::PointXYZ> NewPoint;
        pcl::PointCloud<pcl::PointXYZ> OldPoint;
        Eigen::Matrix4f InitialGuess;

        Matrixtransform *matrixPtr;

        double subsamplingFactor;
        double cellSize;
        double clippingDistance;
        double neighborSearchRadius;
        double entropyThreshold;
        double matchingTolerance;
        NDTLiteConfig ndtcfg;
        IRONConfig ironcfg;

    };
}


#endif //DEPTH_MAP_TRANSFORMCALCULATION_H





