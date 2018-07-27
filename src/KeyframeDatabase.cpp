//
// Created by edward on 17-11-10.
//

#include "KeyframeDatabase.h"
#include <pcl/visualization/pcl_visualizer.h>

namespace DEPTH_MAP {

    KeyframeDatabase::KeyframeDatabase() {}

    KeyframeDatabase::KeyframeDatabase(vector<KeyFrame2> &KeyF) {}

    KeyframeDatabase::KeyframeDatabase(int FrameID, int LocalFrameID, int SidewindowSize, double KeyFrametimeStamp,
                                       Eigen::Matrix4f Trans_Keyframe2Map,
                                       NDTMapLitef NDTmap) : FrameID(FrameID), LocalFrameID(LocalFrameID),
                                                             SidewindowSize(SidewindowSize),
                                                             KeyFrametimeStamp(KeyFrametimeStamp),
                                                             Trans_Keyframe2Map(Trans_Keyframe2Map), NDTmap(NDTmap) {
    }

    KeyframeDatabase::~KeyframeDatabase() {}

    vector<KeyFrame2> KeyframeDatabase::AddGlobalKeyframe(int FrameID, int LocalFrameID, double KeyFrametimeStamp,
                                                          Eigen::Matrix4f Trans_Keyframe2Map,
                                                          NDTMapLitef &NDTmap,
                                                          pcl::PointCloud<pcl::PointNormal> &KeyframePoint) {
        ROS_DEBUG("Add the new Frame in the GlobalDatabase");
        KeyFrame2 CurrFrame;
        CurrFrame.FrameID = FrameID;
        CurrFrame.LocalFrameID = LocalFrameID;
        CurrFrame.KeyFrametimeStamp = KeyFrametimeStamp;
        CurrFrame.Trans_Keyframe2Map = Trans_Keyframe2Map;
        CurrFrame.NDTmap = NDTmap;
        CurrFrame.KeyframePoint = KeyframePoint;
        KeyFrameDatabase2.push_back(CurrFrame);
        ROS_DEBUG("add the [%d] FRAME ,LocalFrameID:[%d] ", FrameID, CurrFrame.LocalFrameID);
        ROS_DEBUG("The size of[%d] globalKeyFrameDatabase", KeyFrameDatabase2.size());
        return KeyFrameDatabase2;
    }

    vector<KeyFrame2> KeyframeDatabase::AddLocalKeyframe(int FrameID, int LocalFrameID, double KeyFrametimeStamp,
                                                         Eigen::Matrix4f Trans_Keyframe2Map, NDTMapLitef NDTmap,
                                                         pcl::PointCloud<pcl::PointNormal> &KeyframePoint) {

        ROS_DEBUG("Add the new Frame in the LocalDatabase");
        KeyFrame2 CurrFrame;
        CurrFrame.FrameID = FrameID;

        CurrFrame.LocalFrameID = LocalKeyframeDatabase.size() + 1;
        CurrFrame.KeyFrametimeStamp = KeyFrametimeStamp;
        CurrFrame.Trans_Keyframe2Map = Trans_Keyframe2Map;
        CurrFrame.NDTmap = NDTmap;
        CurrFrame.KeyframePoint = KeyframePoint;
        LocalKeyframeDatabase.push_back(CurrFrame);
        ROS_DEBUG("add the [%d] FRAME ,LocalFrameID:[%d] ", FrameID, CurrFrame.LocalFrameID);
        ROS_DEBUG("The size of[%d] LocalKeyframeDatabase", LocalKeyframeDatabase.size());
        return LocalKeyframeDatabase;

    }


    vector<KeyFrame2> KeyframeDatabase::SelectLocalKeyframe(int LocalKeyframeID) {
        ROS_DEBUG("The size of[%d] database", KeyFrameDatabase2.size());
        ROS_DEBUG("LocalKeyframeID [%d] ", LocalKeyframeID);
        //每次只选最后10作为slideWindow
        for (vector<KeyFrame2>::iterator it = KeyFrameDatabase2.end() - SidewindowSize;
             it != KeyFrameDatabase2.end(); it++) {
            KeyFrame2 frame;
            frame.FrameID = it->FrameID;
            //保证LocalFrameID标号都在1-10之间，目的是为了g2o顶点哈奥进行优化
            //第一次 //1,2,3,4,5,6,7,8,9,10,11,12
            //第二次     1 2 3 4 5 6 7 8 9 10 （重新标号）
            //第三次       1 2 3 4 5 6 7 8 9 10
            if (it < KeyFrameDatabase2.end() - LocalKeyframeID) {

                ROS_DEBUG("it->LocalFrameID :[%d] ", it->LocalFrameID);
                frame.LocalFrameID = it->LocalFrameID - LocalKeyframeID;
            } else {

                ROS_DEBUG("it->LocalFrameID :[%d],SidewindowSize:[%d],LocalKeyframeID:[%d] ", it->LocalFrameID,
                          SidewindowSize, LocalKeyframeID);

                frame.LocalFrameID = it->LocalFrameID + (SidewindowSize - LocalKeyframeID);
            }
            frame.KeyFrametimeStamp = it->KeyFrametimeStamp;
            frame.Trans_Keyframe2Map = it->Trans_Keyframe2Map;
            frame.NDTmap = it->NDTmap;
            frame.KeyframePoint = it->KeyframePoint;
            LocalKeyframeDatabase.push_back(frame);
            ROS_DEBUG("The frame.LocalFrameID [%d]:", frame.LocalFrameID);
        }

        //    ROS_DEBUG("The size of[%d] LocalKeyframeDatabase",LocalKeyframeDatabase.size());
        return LocalKeyframeDatabase;
    }

    Eigen::Matrix4f KeyframeDatabase::GICP_CalculateTransform(pcl::PointCloud<pcl::PointNormal> newcloud,
                                                              pcl::PointCloud<pcl::PointNormal> oldcloud,
                                                              Eigen::Matrix4f pose) {


        pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);

        pcl::copyPointCloud(newcloud, *src);
        pcl::copyPointCloud(oldcloud, *tgt);

        pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
        norm_est.setSearchMethod(
                pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
        norm_est.setKSearch(5);
        norm_est.setInputCloud(tgt);
        norm_est.compute(*tgt);

        //GICP
        pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> gicp;
        gicp.setTransformationEpsilon(1e-6);
        gicp.setMaxCorrespondenceDistance(0.01);
        gicp.setMaximumIterations(50);
        gicp.setInputSource(src);
        gicp.setInputTarget(tgt);

        pcl::PointCloud<pcl::PointNormal> Final1;
        gicp.align(Final1);
        std::cout << "GICP has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore()
                  << std::endl;
        pose = gicp.getFinalTransformation() * pose;

        return pose;
    }

    Eigen::Matrix4f KeyframeDatabase::pairAlign(pcl::PointCloud<pcl::PointNormal> &cloud_src,
                                                pcl::PointCloud<pcl::PointNormal> &cloud_tgt,
                                                Eigen::Matrix4f InitialGuess) {

        PointCloud::Ptr src(new PointCloud);
        PointCloud::Ptr tgt(new PointCloud);
        PointCloud::Ptr output(new PointCloud);
        pcl::PointCloud<pcl::PointNormal> cloud_align;
        pcl::copyPointCloud(cloud_src, *src);
        pcl::copyPointCloud(cloud_tgt, *tgt);
        // Compute surface normals and curvature
        PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
        PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

        pcl::NormalEstimation<PointT, PointNormalT> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        norm_est.setSearchMethod(tree);
        norm_est.setKSearch(30);

        norm_est.setInputCloud(src);
        norm_est.compute(*points_with_normals_src);
        pcl::copyPointCloud(*src, *points_with_normals_src);

        norm_est.setInputCloud(tgt);
        norm_est.compute(*points_with_normals_tgt);
        pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

        //
        // Instantiate our custom point representation (defined above) ...
        MyPointRepresentation point_representation;
        // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
        float alpha[4] = {1.0, 1.0, 1.0, 1.0};
        point_representation.setRescaleValues(alpha);

        //
        // Align
        pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
        reg.setTransformationEpsilon(1e-6);
        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance(0.05);
        reg.setMaximumIterations(50);
        // Set the point representation
        reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

        reg.setInputSource(points_with_normals_src);
        reg.setInputTarget(points_with_normals_tgt);
        //
        // Run the same optimization in a loop and visualize the results
        Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
        Ti = InitialGuess;
        PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
        reg.setMaximumIterations(2);
        for (int i = 0; i < 50; ++i) {

            // save cloud for visualization purpose
            points_with_normals_src = reg_result;

            // Estimate
            reg.setInputSource(points_with_normals_src);
            reg.align(*reg_result);

            //accumulate transformation between each Iteration
            Ti = reg.getFinalTransformation() * Ti;

            //if the difference between this transformation and the previous one
            //is smaller than the threshold, refine the process by reducing
            //the maximal correspondence distance
            if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
                reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
            prev = reg.getLastIncrementalTransformation();
        }
        targetToSource = Ti.inverse();
        std::cout << "has converged: " << reg.hasConverged() << std::endl;
        cout << "fitness_score:" << reg.getFitnessScore() << endl;
        gicp_FitnessScore = reg.getFitnessScore();

//        int  vp_2(0);
//        p = new pcl::visualization::PCLVisualizer("Pairwise Incremental Registration example");
//        p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);
//
//        // Get the transformation from target to source
//
//
//        //
//        // Transform target back in source frame
//        pcl::transformPointCloud(*tgt, *output, targetToSource);
//
//        p->removePointCloud("source");
//        p->removePointCloud("target");
//
//        PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
//        PointCloudColorHandlerCustom<PointT> cloud_src_h(src, 255, 0, 0);
//
//        p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
//        p->addPointCloud(src, cloud_src_h, "source", vp_2);
//
//
//
//            p->spinOnce();
//            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//            system("pause");
//
//        p->removePointCloud("source");
//        p->removePointCloud("target");
//
//        //add the source to the transformed target
//        *output += *src;

        return targetToSource;
    }


    Eigen::Matrix3d KeyframeDatabase::euler2RotationMatrix(const double yaw, const double pitch, const double roll) {
        Eigen::AngleAxisd rotation_vectorZ(yaw, Eigen::Vector3d(0, 0, 1));     //沿 Z 轴旋转 90 度
        cout.precision(4);
        cout << "rotation matrixZ =\n" << rotation_vectorZ.matrix() << endl;                //用matrix()转换成矩阵

        Eigen::AngleAxisd rotation_vectorY(pitch, Eigen::Vector3d(0, 1, 0));     //沿 Y 轴旋转 90 度
        cout.precision(4);
        cout << "rotation matrixY =\n" << rotation_vectorY.matrix() << endl;                //用matrix()转换成矩阵

        Eigen::AngleAxisd rotation_vectorX(roll, Eigen::Vector3d(1, 0, 0));     //沿 X 轴旋转 90 度
        cout.precision(4);
        cout << "rotation matrixX =\n" << rotation_vectorX.matrix() << endl;                //用matrix()转换成矩阵

        Eigen::Matrix3d Total_rotation_matrix;
        Total_rotation_matrix = rotation_vectorX.matrix() * rotation_vectorY.matrix() * rotation_vectorZ.matrix();
        // cout<<"Total_rotation matrixX =\n"<< Total_rotation_matrix <<endl;

        return Total_rotation_matrix;
    }


}