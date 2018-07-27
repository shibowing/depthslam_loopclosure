//
// Created by edward on 17-11-10.
//

#ifndef DEPTH_MAP_LOOPCLOSURE_H
#define DEPTH_MAP_LOOPCLOSURE_H

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <algorithm>
#include "KeyframeDatabase.h"
#include<nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

enum CHECK_RESULT {
    NOT_MATCHED = 0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME
};
enum EDGE_SELECT {
    GLOBAL, LOCAL
};
struct Frame {
    int FrameID;
    int LocalFrameID;
    double KeyFrametimeStamp;
    Eigen::Matrix4f Trans_Keyframe2Map;
    pcl::PointCloud<pcl::PointNormal> MapPoint;

};

// 函数声明
inline geometry_msgs::Point ToRosPoint(Pose v) {
    geometry_msgs::Point msg;
    msg.x = v.x;
    msg.y = v.y;
    msg.z = v.z;
    return msg;
}

namespace DEPTH_MAP {
    class KeyframeDatabase;

    class depthmap;

    class LoopClosure {
    public:
        typedef g2o::BlockSolver_6_3 SlamBlockSolver;
        // typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
        typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

        //  g2o::SparseOptimizer GlobalOptimizer;
        LoopClosure();

        LoopClosure(bool first_posegraph, KeyframeDatabase *GlbKeyframe, TransformCalculation *TransPtr);

        ~LoopClosure();


        vector<Pose> IntialLocalPoseGraph(vector<KeyFrame2> &KeyFrameDatabase2);

        void StartLocalPoseGraph(vector<KeyFrame2> &KeyFrameDatabase2);

        vector<Matrix4f> IntialGlobalPoseGraph(vector<KeyFrame2> &KeyFrameDatabase2);

        vector<Matrix4f> StartGlobalPoseGraph(vector<KeyFrame2> &KeyFrameDatabase2);


        // 检测近距离的回环
        void checkNearbyLoops(vector<KeyFrame2> &frames, KeyFrame2 &currFrame,
                              g2o::SparseOptimizer &opti);

        // 随机检测回环
        void checkRandomLoops(vector<KeyFrame2> &frames, KeyFrame2 &currFrame,
                              g2o::SparseOptimizer &opti);

        //Add the edge of  the posegraph
        void AddOdometryedge(KeyFrame2 &f1, KeyFrame2 &f2, g2o::SparseOptimizer &opti, enum EDGE_SELECT OdomEdge);

        void AddLoopedge(KeyFrame2 &f1, KeyFrame2 &f2, g2o::SparseOptimizer &opti, enum EDGE_SELECT LoopEdge);


        //math matrix conversion
        inline Eigen::Isometry3d Matrix4ftoIsometry3d(Eigen::Matrix4f T);

        inline Pose Matrix4ftoTransvec(Eigen::Matrix4f T);

        inline Eigen::Matrix4f Isometry3dtoMatrix4f(Eigen::Isometry3d T);

        inline double ComputeDistanceBetweenMatrix(Eigen::Matrix4f T1, Eigen::Matrix4f T2);

        inline Eigen::Vector3d GetTransVecFromMatrix(Eigen::Matrix4f T1);

        //  global loop detection
        void LoopDetection(KeyFrame2 &currframe);


        KeyFrame2 ReadCurrFrame(int index, vector<KeyFrame2> &CurrFrameVec, enum EDGE_SELECT LoopEdge);

        void GlobalOptimizeProcess(vector<KeyFrame2> &KeyFrameDatabase2);

        void BuildOptimizedGlobalmap(vector<KeyFrame2> &KeyFrameDatabase2);

        void LocalOptimizeProcess2(vector<KeyFrame2> &KeyFrameDatabase2);

        void LocalOptimizeProcess(vector<KeyFrame2> &KeyFrameDatabase2);

        void PublishLocalPoseGraph();

        void PublishGlobalPoseGraph();

        void PublishLoopdetectionArea(Pose CurrPose);

        typedef std::pair<Pose, Pose> EdgePair;
        vector<EdgePair> odometry_edge;
        vector<Matrix4f> OptimizedPose;
        vector<Pose> GlobalPoseafterG2o;

        // parameter for loopdetection
        double keyframe_threshold;
        double proximity_threshold;
        int skip_recent_poses;
        int currIndex;
        int startIndex;
        int EndIndex;
        int LocalposegraphSize;
        int NearbyLoops;
        int GRAPHSIZE;
        int odometryEdge_informationPos;
        int odometryEdge_informationAngle;
        int loopEdge_informationPos;
        int loopEdge_informationAngle;
        bool check_local_closure;
        bool check_global_closure;
        bool first_posegraph;
        bool add_new_loopclosure;
        bool Find_Loop;
        string fixed_frame_id_;
        nav_msgs::Path OptimizedPath;

        // Visualization publishers.
        ros::Publisher odometry_edge_pub_;
        ros::Publisher global_odometry_edge_pub_;
        ros::Publisher loop_edge_pub_;
        ros::Publisher graph_node_pub_;
        ros::Publisher global_graph_node_pub_;
        ros::Publisher keyframe_node_pub_;
        ros::Publisher closure_area_pub_;
        ros::Publisher optimized_globalmap_pub_;
        ros::Publisher OptimizedPathPub;

        // GLOBAL POSEGRAPH
        KeyframeDatabase *GlobalKeyframePtr;
        KeyframeDatabase *LocalKeyframePtr;
        TransformCalculation *NDTTransform;
        TransformCalculation *TransformPtr;


    };

}
#endif //DEPTH_MAP_LOOPCLOSURE_H
