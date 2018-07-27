//
// Created by edward on 17-12-11.
//

#include "TransformCalculation.h"
#include <iostream>

using namespace std;
namespace DEPTH_MAP {
    TransformCalculation::TransformCalculation() {}

    TransformCalculation::TransformCalculation(pcl::PointCloud<pcl::PointXYZ> NewPoint,
                                               pcl::PointCloud<pcl::PointXYZ> OldPoint, Eigen::Matrix4f InitialGuess,
                                               double cellSize,
                                               double subsamplingFactor, double clippingDistance,
                                               double matchingTolerance, double entropyThreshold,
                                               double neighborSearchRadius) :
            NewPoint(NewPoint), OldPoint(OldPoint), InitialGuess(InitialGuess), cellSize(cellSize),
            subsamplingFactor(subsamplingFactor),
            clippingDistance(clippingDistance), matchingTolerance(matchingTolerance),
            entropyThreshold(entropyThreshold), neighborSearchRadius(neighborSearchRadius) {

        // prepare NDT map creator (take a look into NDTLite.h for an explanation of parameters)

        // some important variables
        ndtcfg.cellSize = (float) cellSize;          // default: 0.1m x 0.1m x 0.1m cells
        ndtcfg.subsamplingFactor = (float) subsamplingFactor; // default: take only 20% of points (drawn randomly)
        ndtcfg.clippingDistance = (float) clippingDistance;  // limit memory consumption of NDTLiteCreator by
        // throwing away points that are far away from the sensor


        // prepare IRON engine (IRON.h for a summary of available options)
        // important variables
        ironcfg.matchingTolerance = (float) matchingTolerance;   // RANSAC inlier threshold: higher values will make registration more robust
        // but the transform computation more inaccurate; default: half of NDT-cell size
        ironcfg.entropyThreshold = (float) entropyThreshold;    // the lower, the more keypoints will be found and the slower registration
        ironcfg.neighborSearchRadius = (float) neighborSearchRadius; // radius around each NDT-cell for searching neighboring cells

    }

    TransformCalculation::~TransformCalculation() {}

    Eigen::Affine3f
    TransformCalculation::fillCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputcloud, std::vector<Pt3D> &cloud,
                                    Eigen::Matrix4f pose) {

        //fill in new cloud
        ROS_DEBUG("The size of[%d] database", inputcloud->height * inputcloud->width);
        if ((inputcloud->height * inputcloud->width) > 0) {
            pcl::PointCloud<pcl::PointXYZ> Currcloud;

            Currcloud = *inputcloud;

            std::vector<int> temp_new;
            pcl::removeNaNFromPointCloud(Currcloud, Currcloud, temp_new);
            Currcloud.resize(Currcloud.width * Currcloud.height);
            ros::WallTime cloudcopytime2 = ros::WallTime::now();
            for (size_t i = 0; i < Currcloud.size(); ++i) {

                //Pt3D pts(Currcloud.points[i].x,Currcloud.points[i].y,Currcloud.points[i].z);
                // cout <<"CurrentPT3DData:"<< pts.data<< endl;
                Pt3D pts;
                pts.data[0] = Currcloud.points[i].x;
                pts.data[1] = Currcloud.points[i].y;
                pts.data[2] = Currcloud.points[i].z;
//                cout<<"pts.data[0]:"<<pts.data[0]<<endl;
//                cout<<"pts.data[1]:"<<pts.data[1]<<endl;
//                cout<<"pts.data[2]:"<<pts.data[2]<<endl;
                cloud.push_back(pts);

            }

            ROS_DEBUG("The size of[%d] clouddatabase", cloud.size());
            Eigen::Matrix<float, 3, 3> rotation;
            rotation = Eigen::Matrix<float, 3, 3>::Zero();

            rotation = pose.block<3, 3>(0, 0);
            //         cout<<" rotation:"<< rotation<<endl;
            // Eigen::Matrix<double, 3, 1> translation;
            Eigen::Vector3f translation;
            translation = Eigen::Matrix<float, 3, 1>::Zero();
            translation = pose.block<3, 1>(0, 3);
            Eigen::Quaternionf q;
            q = matrixPtr->rotationMatrix2Quaternionf(rotation);

            // cout<<"translation:"<<translation<<endl;
            sensorPose = Eigen::Translation3f(translation[0], translation[1], translation[2]) *
                         Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());


        }

        return sensorPose;
    }


///////////////////////////////
// store ndt mean points
///////////////////////////////
    void TransformCalculation::storeMeans(const std::string &file, const NDTMapLitef &map,
                                          const Eigen::Affine3f &transform) {
        std::ofstream of;
        of.open(file, std::ios::out);
        for (uint i = 0; i < map.cells.size(); ++i) {
            const Eigen::Vector3f res = transform * map.cells[i].mu;
            of << res(0) << " " << res(1) << " " << res(2) << std::endl;
        }
        of.close();
    }

    void TransformCalculation::storeMeans(const std::string &file, const IRONDescriptorVectorf &vec) {
        std::ofstream of;
        of.open(file, std::ios::out);
        for (uint i = 0; i < vec.size(); ++i) {
            of << vec[i].mu()(0) << " " << vec[i].mu()(1) << " " << vec[i].mu()(2) << std::endl;
        }
        of.close();
    }


///////////////////////////////
// store cov maps
///////////////////////////////
    void TransformCalculation::storeCovs(const char *file, const NDTMapLitef &map, uint samples) {
        std::ofstream of;
        of.open(file, std::ios::out);
        XorShift rnd;
        for (uint i = 0; i < map.cells.size(); ++i) {
            const Eigen::Matrix<float, 3, 3> &cov = map.cells[i].cov;
            const Eigen::Matrix<float, 3, 3> L = cov.llt().matrixL();
            for (uint k = 0; k < samples; ++k) {
                float r1 = (rnd.rand() % 100000) / 100000.0,
                        r2 = (rnd.rand() % 100000) / 100000.0;
                float th = 2.0 * M_PI * r1,
                        ph = std::acos(2.0 * r2 - 1.0);
                float x = std::cos(th) * std::sin(ph),
                        y = std::sin(th) * std::sin(ph),
                        z = std::cos(ph);
                Eigen::Vector3f pt(x, y, z), res;
                res = 1.0 * L * pt;
                of << map.cells[i].mu(0) + res(0) << " "
                   << map.cells[i].mu(1) + res(1) << " "
                   << map.cells[i].mu(2) + res(2) << "\n";
            }
        }
        of.close();
    }

    void TransformCalculation::storeCovs(const char *file, const IRONDescriptorVectorf &vec, uint samples) {
        std::ofstream of;
        of.open(file, std::ios::out);
        XorShift rnd;
        for (uint i = 0; i < vec.size(); ++i) {
            const Eigen::Matrix<float, 3, 3> &cov = vec[i].cov();
            const Eigen::Matrix<float, 3, 3> L = cov.llt().matrixL();
            for (uint k = 0; k < samples; ++k) {
                float r1 = (rnd.rand() % 100000) / 100000.0,
                        r2 = (rnd.rand() % 100000) / 100000.0;
                float th = 2.0 * M_PI * r1,
                        ph = std::acos(2.0 * r2 - 1.0);
                float x = std::cos(th) * std::sin(ph),
                        y = std::sin(th) * std::sin(ph),
                        z = std::cos(ph);
                Eigen::Vector3f pt(x, y, z), res;
                res = 1.0 * L * pt;
                of << vec[i].mu()(0) + res(0) << " "
                   << vec[i].mu()(1) + res(1) << " "
                   << vec[i].mu()(2) + res(2) << "\n";
            }
        }
        of.close();
    }


///////////////////////////////
// store matches
///////////////////////////////
    void TransformCalculation::storeMatches(const char *file, const IRONMatchVectorf &matches) {
        std::ofstream of;
        of.open(file, std::ios::out);
        for (uint i = 0; i < matches.size(); ++i) {
            of << matches[i].from->mu()(0) << " " << matches[i].from->mu()(1) << " " << matches[i].from->mu()(2)
               << std::endl;
            of << matches[i].to->mu()(0) << " " << matches[i].to->mu()(1) << " " << matches[i].to->mu()(2) << std::endl;
            of << "\n\n";
        }
        of.close();
    }

    Eigen::Matrix4f TransformCalculation::CalculateTransform_Process(pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud,
                                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &oldcloud,
                                                                     Eigen::Matrix4f Currpose, double subsamplingFactor,
                                                                     double cellSize, double clippingDistance,
                                                                     double neighborSearchRadius,
                                                                     double entropyThreshold,
                                                                     double matchingTolerance) {


        ROS_INFO("begin_ndt_matching");
        ros::WallTime cloudcopytime2 = ros::WallTime::now();
        sensorPose1 = fillCloud(newcloud, cloud1, Currpose);
        sensorPose2 = fillCloud(oldcloud, cloud2, Currpose);
        std::cout << "Total cloudcopytime2 took: " << (ros::WallTime::now() - cloudcopytime2).toSec() << std::endl;

        // now shift the second map a bit in x direction (this will shift the new NDT-map as well)
        //sensorPose2.translation().x() += 4.0f;

        // prepare NDT map creator (take a look into NDTLite.h for an explanation of parameters)
//        NDTLiteConfig ndtcfg;
//        // some important variables
//        ndtcfg.cellSize = (float)cellSize;          // default: 0.1m x 0.1m x 0.1m cells
//        ndtcfg.subsamplingFactor = (float)subsamplingFactor; // default: take only 20% of points (drawn randomly)
//        ndtcfg.clippingDistance = (float)clippingDistance;  // limit memory consumption of NDTLiteCreator by
//        // throwing away points that are far away from the sensor
        NDTMapLiteCreator<float, std::vector<Pt3D>, Pt3DAccessor> creator(ndtcfg);
//
//        // prepare IRON engine (IRON.h for a summary of available options)
//        IRONConfig ironcfg;
//        // important variables
//        ironcfg.matchingTolerance =(float)matchingTolerance;   // RANSAC inlier threshold: higher values will make registration more robust
//        // but the transform computation more inaccurate; default: half of NDT-cell size
//        ironcfg.entropyThreshold = (float)entropyThreshold;    // the lower, the more keypoints will be found and the slower registration
//        ironcfg.neighborSearchRadius =(float)neighborSearchRadius; // radius around each NDT-cell for searching neighboring cells
        IRONEnginef engine(ironcfg);


        ROS_INFO("check the paramerter [%f], [%f],[%f],[%f] [%f],[%f]", ndtcfg.cellSize, ndtcfg.subsamplingFactor,
                 ndtcfg.clippingDistance, ironcfg.matchingTolerance, ironcfg.entropyThreshold,
                 ironcfg.neighborSearchRadius);
        // PLEASE NOTE: NDTMapLiteCreator and IRONEngine should exist only once, as
        // they do some additional computation during construction which would lead to
        // unnecessary overhead if created in a loop over and over again

        // build NDT maps
        //   ROS_DEBUG("The size of[%d] cloud1", cloud1.size());
        // ROS_DEBUG("The size of[%d] cloud2", cloud2.size());
        ros::WallTime cloudcopytime3 = ros::WallTime::now();

        creator.createMapFromPointValues(&ndtMap1, cloud1, sensorPose1);
        std::cout << "Total 4" << std::endl;
        creator.createMapFromPointValues(&ndtMap2, cloud2, sensorPose2);
        std::cerr << "NDT MAP 1..: " << ndtMap1.cells.size() << " cells\n"
                  << "NDT MAP 2..: " << ndtMap2.cells.size() << " cells\n";
        std::cout << "Total NDT MAP  took: " << (ros::WallTime::now() - cloudcopytime3).toSec() << std::endl;
        // compute IRON keypoints and descriptors
        ros::WallTime cloudcopytime4 = ros::WallTime::now();

        engine.computeDescriptors(&descriptors1, ndtMap1);
        engine.computeDescriptors(&descriptors2, ndtMap2);
        std::cerr << "KEYPOINTS 1: " << descriptors1.size() << " cells\n"
                  << "KEYPOINTS 2: " << descriptors2.size() << " cells\n";
        std::cout << "Total descriptor  took: " << (ros::WallTime::now() - cloudcopytime4).toSec() << std::endl;
        // match keypoint descriptors
        ros::WallTime cloudcopytime5 = ros::WallTime::now();
        engine.computeMatches(&matches, descriptors1, descriptors2);
        std::cerr << "MATCHES....: " << matches.size() << std::endl;
        std::cout << "Total MATCHES  took: " << (ros::WallTime::now() - cloudcopytime5).toSec() << std::endl;
        // reject outliers
        ros::WallTime cloudcopytime6 = ros::WallTime::now();
        engine.detectOutliers(&inlierset, matches);
        std::cerr << "INLIERS....: " << inlierset.size() << std::endl;
        std::cout << "Total detectOutliers  took: " << (ros::WallTime::now() - cloudcopytime6).toSec() << std::endl;
        // compute transform from inlierset
        ros::WallTime cloudcopytime7 = ros::WallTime::now();
        result = engine.computeTransform(inlierset);
        std::cout << "computeTransform  took: " << (ros::WallTime::now() - cloudcopytime7).toSec() << std::endl;
        if (!result.success) {
            std::cerr << "REGISTRATION FAILED\n";
            engine.releaseDescriptors(&descriptors1);
            engine.releaseDescriptors(&descriptors2);
        }
        engine.releaseDescriptors(&descriptors1);
        engine.releaseDescriptors(&descriptors2);
        cloud1.clear();
        cloud2.clear();
        return result.tf.matrix();

    }

    Eigen::Matrix4f
    TransformCalculation::NDT_CalculateTransform(NDTMapLitef ndtMap1, NDTMapLitef ndtMap2, int frameID1, int frameID2) {
        IRONEnginef engine(ironcfg);
        ros::WallTime cloudcopytime4 = ros::WallTime::now();

        engine.computeDescriptors(&descriptors1, ndtMap1);
        engine.computeDescriptors(&descriptors2, ndtMap2);
        std::cerr << "KEYPOINTS 1: " << descriptors1.size() << " cells\n"
                  << "KEYPOINTS 2: " << descriptors2.size() << " cells\n";
        std::cout << "Total descriptor  took: " << (ros::WallTime::now() - cloudcopytime4).toSec() << std::endl;
        // match keypoint descriptors
        ros::WallTime cloudcopytime5 = ros::WallTime::now();
        engine.computeMatches(&matches, descriptors1, descriptors2);
        std::cerr << "MATCHES....: " << matches.size() << std::endl;
        std::cout << "Total MATCHES  took: " << (ros::WallTime::now() - cloudcopytime5).toSec() << std::endl;
        // reject outliers
        ros::WallTime cloudcopytime6 = ros::WallTime::now();
        engine.detectOutliers(&inlierset, matches);
        std::cerr << "INLIERS....: " << inlierset.size() << std::endl;
        std::cout << "Total detectOutliers  took: " << (ros::WallTime::now() - cloudcopytime6).toSec() << std::endl;
        // compute transform from inlierset
        ros::WallTime cloudcopytime7 = ros::WallTime::now();
        result = engine.computeTransform(inlierset);
        std::cout << "computeTransform  took: " << (ros::WallTime::now() - cloudcopytime7).toSec() << std::endl;
        if (!result.success) {
            std::cerr << "REGISTRATION FAILED\n";
            engine.releaseDescriptors(&descriptors1);
            engine.releaseDescriptors(&descriptors2);
        }

//        ostringstream file1, file2;
//        file1 << frameID1 << ".txt";
//        file2 << frameID2 << ".txt";
        //  pcl::io::savePCDFileASCII("/home/edward/catkin_ws/src/IRON_loopclosure/doc/" + file1.str(), f1.KeyframePoint);
//        storeMeans("/home/edward/catkin_ws/src/IRON_loopclosure/doc/" + file1.str(), ndtMap1);
//        storeMeans("/home/edward/catkin_ws/src/IRON_loopclosure/doc/" + file2.str(), ndtMap2);
//        storeMeans("/home/edward/catkin_ws/src/IRON_loopclosure/doc/means_transformed1"+ file1.str(), ndtMap1, result.tf);


        engine.releaseDescriptors(&descriptors1);
        engine.releaseDescriptors(&descriptors2);
        cout << "loopclosure matrix=\n" << result.tf.matrix() << endl;

        return result.tf.matrix();

    }

    NDTMapLitef
    TransformCalculation::ExtractNdtmap(pcl::PointCloud<pcl::PointXYZ>::Ptr &newcloud, Eigen::Matrix4f Currpose) {
        ros::WallTime cloudcopytime2 = ros::WallTime::now();
        sensorPose1 = fillCloud(newcloud, cloud1, Currpose);
        //   std::cout << "Total cloudcopytime2 took: " << (ros::WallTime::now() - cloudcopytime2).toSec() << std::endl;

        ros::WallTime cloudcopytime3 = ros::WallTime::now();
        NDTMapLiteCreator<float, std::vector<Pt3D>, Pt3DAccessor> creator(ndtcfg);
        //  IRONEnginef engine(ironcfg);
        ROS_INFO("check the paramerter [%f], [%f],[%f],[%f] [%f],[%f]", ndtcfg.cellSize, ndtcfg.subsamplingFactor,
                 ndtcfg.clippingDistance, ironcfg.matchingTolerance, ironcfg.entropyThreshold,
                 ironcfg.neighborSearchRadius);
        creator.createMapFromPointValues(&ndtMap1, cloud1, sensorPose1);
        std::cerr << "NDT MAP 1..: " << ndtMap1.cells.size() << " cells\n";
        //  std::cout << "Total cloudcopytime3 took: " << (ros::WallTime::now() - cloudcopytime3).toSec() << std::endl;
        //std::vector<NDTCellLite<NumericalType> > cells;
        cloud1.clear();
        return ndtMap1;
    }


}