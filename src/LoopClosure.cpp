//
// Created by edward on 17-11-10.
//

#include "LoopClosure.h"
#include "depth_map.h"
#include <mutex>
#include <visualization_msgs/Marker.h>
#include <g2o/core/robust_kernel_factory.h>

g2o::SparseOptimizer LocalOptimizer;  // 最后用的就是这个东东
g2o::SparseOptimizer GlobalOptimizer;  // 最后用的就是这个东东
vector<Pose> PoseafterG2o;

typedef std::pair<Pose, Pose> EdgePair;
vector<EdgePair> odometry_edge;
vector<EdgePair> loop_edge;
typedef std::pair<int, int> FramePair;
bool first_posegraph;

pcl::PointCloud<pcl::PointNormal>::Ptr GlobalOrimapCloud(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr GlobalOptmapCloud(new pcl::PointCloud<pcl::PointNormal>);

struct GlobalLoop {
    int frame1;
    int frame2;
    double distance;
};
vector<GlobalLoop> Global_Looppair;

//从大到小
bool GreaterSort(const GlobalLoop &a, const GlobalLoop &b) { return (a.distance > b.distance); }

//从小到大
bool LessSort(const GlobalLoop &a, const GlobalLoop &b) { return (a.distance < b.distance); }


namespace DEPTH_MAP {
    LoopClosure::LoopClosure() {
        ros::NodeHandle n;
        n.param<string>("fixed_frame_id_", fixed_frame_id_, "world");
        keyframe_threshold = 0.1;
        proximity_threshold = 0.3;
        check_local_closure = true;
        check_global_closure = true;
        startIndex = 1;
        currIndex = 0;
        LocalposegraphSize = 5;
        NearbyLoops = 3;
        first_posegraph = false;

        Find_Loop = false;
        odometry_edge_pub_ =
                n.advertise<visualization_msgs::Marker>("odometry_edges", 10, false);
        loop_edge_pub_ =
                n.advertise<visualization_msgs::Marker>("loop_edges", 10, false);
        graph_node_pub_ =
                n.advertise<visualization_msgs::Marker>("graph_nodes", 10, false);
        keyframe_node_pub_ =
                n.advertise<visualization_msgs::Marker>("keyframe_nodes", 10, false);
        closure_area_pub_ =
                n.advertise<visualization_msgs::Marker>("closure_area", 10, false);
        global_odometry_edge_pub_ =
                n.advertise<visualization_msgs::Marker>("global_odometry_edges", 10, false);
        global_graph_node_pub_ =
                n.advertise<visualization_msgs::Marker>("global_graph_nodes", 10, false);

    }

    LoopClosure::LoopClosure(bool first_posegraph, KeyframeDatabase *GlbKeyframe, TransformCalculation *TransPtr)
            : first_posegraph(first_posegraph),
              GlobalKeyframePtr(GlbKeyframe), TransformPtr(TransPtr) {

        ros::NodeHandle n;

        keyframe_threshold = 0.1;
        proximity_threshold = 0.3;
        skip_recent_poses = 300;
        check_local_closure = true;
        check_global_closure = true;
        add_new_loopclosure = false;
        startIndex = 1;
        currIndex = 0;
        LocalposegraphSize = 5;
        NearbyLoops = 3;

        Find_Loop = false;

        // first_posegraph=true;
        n.param<string>("fixed_frame_id_", fixed_frame_id_, "world");
        n.param<int>("odometryEdge_informationPos", odometryEdge_informationPos, 4000);
        n.param<int>("odometryEdge_informationAngle", odometryEdge_informationAngle, 100);
        n.param<int>("loopEdge_informationPos", loopEdge_informationPos, 4000);
        n.param<int>("loopEdge_informationAngle", loopEdge_informationAngle, 100);
        odometry_edge_pub_ =
                n.advertise<visualization_msgs::Marker>("odometry_edges", 10, false);
        loop_edge_pub_ =
                n.advertise<visualization_msgs::Marker>("loop_edges", 10, false);
        graph_node_pub_ =
                n.advertise<visualization_msgs::Marker>("graph_nodes", 10, false);
        keyframe_node_pub_ =
                n.advertise<visualization_msgs::Marker>("keyframe_nodes", 10, false);
        closure_area_pub_ =
                n.advertise<visualization_msgs::Marker>("closure_area", 10, false);
        global_odometry_edge_pub_ =
                n.advertise<visualization_msgs::Marker>("global_odometry_edges", 10, false);
        global_graph_node_pub_ =
                n.advertise<visualization_msgs::Marker>("global_graph_nodes", 10, false);

        optimized_globalmap_pub_ = n.advertise<sensor_msgs::PointCloud2>("/OptimizedGlobalMap", 2, true);
        OptimizedPathPub = n.advertise<nav_msgs::Path>("/OptimizedPath", 10);


    }

    LoopClosure::~LoopClosure() {
        // GlobalKeyframePtr;
    }


    vector<Matrix4f> LoopClosure::IntialGlobalPoseGraph(vector<KeyFrame2> &KeyFrameDatabase2) {

        //指定线性方程求解器使用Eigen的块求解器
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver =
                new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        //不输出调用信息
        GlobalOptimizer.setVerbose(false);
        // 构造线性求解器
        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
        // 使用LM算法进行非线性迭代
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e-16);
        GlobalOptimizer.setAlgorithm(solver);

        // 向globalOptimizer增加第一个顶点

        if (KeyFrameDatabase2.data()->FrameID == startIndex && first_posegraph == true) {

            g2o::VertexSE3 *v = new g2o::VertexSE3();
            v->setId(KeyFrameDatabase2.data()->FrameID);
            v->setFixed(true); //第一个顶点固定，不用优化
            Eigen::Matrix4f Trans_Cur2Last;
            Trans_Cur2Last = KeyFrameDatabase2.data()->Trans_Keyframe2Map;
            v->setEstimate(Matrix4ftoIsometry3d(Trans_Cur2Last));
            GlobalOptimizer.addVertex(v);

        } else {
            //ROS_DEBUG("Miss the First Frame!!!!!!!!!! ");
        }
        LoopClosure loop(first_posegraph, GlobalKeyframePtr, TransformPtr);
        OptimizedPose = loop.StartGlobalPoseGraph(KeyFrameDatabase2);


        return OptimizedPose;

    }


    vector<Matrix4f> LoopClosure::StartGlobalPoseGraph(vector<KeyFrame2> &KeyFrameDatabase2) {

        if (first_posegraph) {
            //建立前后两帧之间的约束（里程计边），因为刚开始所以都要建立约束
            for (currIndex = startIndex + 1; currIndex <= KeyFrameDatabase2.size(); currIndex++) {
                LoopClosure loop;
                KeyFrame2 currFrame = loop.ReadCurrFrame(currIndex, KeyFrameDatabase2, GLOBAL);
                AddOdometryedge(KeyFrameDatabase2[currIndex - 2], currFrame, GlobalOptimizer, GLOBAL);
            }
        } else {
            LoopClosure loop;
            int currIndex;
            currIndex = KeyFrameDatabase2.back().FrameID;
            // ROS_DEBUG("find the global frame ID!!![%d]",currIndex);
            // 只建立倒数两帧之间的约束，实际就是来一帧，建立一次约束
            AddOdometryedge(KeyFrameDatabase2[currIndex - 2], KeyFrameDatabase2[currIndex - 1], GlobalOptimizer,
                            GLOBAL);
            //对后面的帧，开始闭环检测
            LoopDetection(KeyFrameDatabase2[currIndex - 1]);
            GlobalOptimizeProcess(KeyFrameDatabase2);
        }

        ros::WallTime graphTime4 = ros::WallTime::now();


        return OptimizedPose;
    }

    /**
   * 实现深度图像的闭环检测，每次对当前帧寻找潜在闭环
   * @param currframe  当前帧
   */

    void LoopClosure::LoopDetection(KeyFrame2 &currframe) {
        ROS_DEBUG("begin the loop detection!!!");
        ROS_DEBUG("the size of keyframe [%d]", GlobalKeyframePtr->KeyFrameDatabase2.size());

        //默认设置是80帧以后才能开始闭环检测
        if (GlobalKeyframePtr->KeyFrameDatabase2.size() > skip_recent_poses &&
            GlobalKeyframePtr->KeyFrameDatabase2.size() > 320) {
            for (vector<KeyFrame2>::iterator it = GlobalKeyframePtr->KeyFrameDatabase2.end() - skip_recent_poses;
                 it != GlobalKeyframePtr->KeyFrameDatabase2.begin(); it--) {
                double distance;
                //距离小于某一特定阈值认为是存在潜在闭环
                distance = ComputeDistanceBetweenMatrix(currframe.Trans_Keyframe2Map,
                                                        it->Trans_Keyframe2Map);

                if (distance <= proximity_threshold) {
                    ROS_DEBUG(" find the loop detection!!!");
                    Pose CurrPose;
                    CurrPose = Matrix4ftoTransvec(currframe.Trans_Keyframe2Map);
                    PublishLoopdetectionArea(CurrPose);
                    GlobalLoop globalpair;
                    globalpair.frame1 = currframe.FrameID;
                    globalpair.frame2 = it->FrameID;
                    globalpair.distance = distance;
                    Global_Looppair.push_back(globalpair);
                    ROS_DEBUG(" The  potential loop pair[%d] AND [%d],Distance: [%lf]", currframe.FrameID, it->FrameID,
                              distance);
                    Find_Loop = true;
                }
            }

        }
    }


    //开始全局优化
    void LoopClosure::GlobalOptimizeProcess(vector<KeyFrame2> &KeyFrameDatabase2) {

        //如果闭环检测成功
        if (Find_Loop == true) {
            int currIndex, matchedIndex;
            sort(Global_Looppair.begin(), Global_Looppair.end(), LessSort);  //从小到大排列

            for (vector<GlobalLoop>::iterator it = Global_Looppair.begin(); it != Global_Looppair.end(); it++) {
                currIndex = it->frame1;
                matchedIndex = it->frame2;
                if (abs(currIndex - matchedIndex) != 1)
                    loopEdge_informationPos = 4000;
                loopEdge_informationAngle = 400;
                ROS_DEBUG("Distance: [%lf] , the number of potential pair [%d] And [%d]", it->distance, currIndex,
                          matchedIndex);
                AddLoopedge(KeyFrameDatabase2[matchedIndex - 1], KeyFrameDatabase2[currIndex - 1], GlobalOptimizer,
                            GLOBAL);
            }
            //完成整个优化

            std::vector<GlobalLoop>().swap(Global_Looppair);
            ros::WallTime startTime1 = ros::WallTime::now();
            ROS_DEBUG("optimizing the global pose graph, vertices:[%d] ", GlobalOptimizer.vertices().size());
            GlobalOptimizer.save("/home/machozhao/catkin_ws/src/IRON_loopclosure/doc/global_result_before.g2o");
            GlobalOptimizer.initializeOptimization();
            GlobalOptimizer.optimize(50); //可以指定优化步数
            GlobalOptimizer.save("/home/machozhao/catkin_ws/src/IRON_loopclosure/doc/global_result_after.g2o");
            ROS_DEBUG("Global optimize finished!!!");

            std::cout << "Graph Optimization took: " << (ros::WallTime::now() - startTime1).toSec() << std::endl;


            BuildOptimizedGlobalmap(KeyFrameDatabase2);

            add_new_loopclosure = false;
            Find_Loop = false;

        }
        //发布优化后的轨迹
        OptimizedPath.poses.clear();
        for (size_t i = 0; i < KeyFrameDatabase2.size(); i++) {
            // 从g2o里取出一帧
            Eigen::Matrix4f PoseMatrix = Eigen::Matrix4f::Identity();
            g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *>(GlobalOptimizer.vertex(
                    KeyFrameDatabase2[i].FrameID));

            Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
            PoseMatrix = Isometry3dtoMatrix4f(pose);
            OptimizedPose.push_back(PoseMatrix);

            // For the rviz marker to show the trajectory
            pose.translation();
            Eigen::Vector3d Pose_w;
            Pose_w = Eigen::Vector3d(pose.translation());
            Pose Pose_world;
            Pose_world.x = Pose_w(0);
            Pose_world.y = Pose_w(1);
            Pose_world.z = Pose_w(2);
            GlobalPoseafterG2o.push_back(Pose_world);

            //This is the real to display the trajectory

            geometry_msgs::PoseStamped Pose;
            Pose.pose.position.x = Pose_w(0);
            Pose.pose.position.y = Pose_w(1);
            Pose.pose.position.z = Pose_w(2);
            OptimizedPath.poses.push_back(Pose);
        }

        ROS_DEBUG("Publish trajectory [%d]", GlobalPoseafterG2o.size());

        OptimizedPath.header.frame_id = "world";
        OptimizedPathPub.publish(OptimizedPath);


    }

    void LoopClosure::BuildOptimizedGlobalmap(vector<KeyFrame2> &KeyFrameDatabase2) {

        /*
         * 实时建立点云地图
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr RemoveOutlier(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointNormal>::Ptr TransMapCloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr OptimizedGlobalCloud(new pcl::PointCloud<pcl::PointNormal>);

        ROS_DEBUG("The size of keyframe: [%d]", KeyFrameDatabase2.size());

        for (size_t i = 0; i < KeyFrameDatabase2.size(); i = i + 3) {

            Eigen::Matrix4f PoseMatrix = Eigen::Matrix4f::Identity();

            g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *>(GlobalOptimizer.vertex(
                    KeyFrameDatabase2[i].FrameID));

            Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
            PoseMatrix = Isometry3dtoMatrix4f(pose);


            pcl::transformPointCloud(KeyFrameDatabase2[i].KeyframePoint, *TransMapCloud, PoseMatrix);

            *OptimizedGlobalCloud += *TransMapCloud;

        }

//        pcl::copyPointCloud(*OptimizedGlobalCloud, *RemoveOutlier);
//        pcl::PointCloud<pcl::PointXYZ> afterRemove;
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> remove;
//        remove.setInputCloud(RemoveOutlier);
//        remove.setMeanK(50);
//        remove.setStddevMulThresh(1.0);
//        remove.filter(afterRemove);
//
//        pcl::PointCloud<pcl::PointNormal>::Ptr ptCloud(new pcl::PointCloud<pcl::PointNormal>);
//        pcl::copyPointCloud(afterRemove, *OptimizedGlobalCloud);


        ROS_DEBUG("Publish Optimized Globalmap!");
        sensor_msgs::PointCloud2 OptimizedGlobalMapCloudMsg;
        pcl::toROSMsg(*OptimizedGlobalCloud, OptimizedGlobalMapCloudMsg);
        OptimizedGlobalMapCloudMsg.header.frame_id = "world";

        if (optimized_globalmap_pub_.getNumSubscribers()) {

            optimized_globalmap_pub_.publish(OptimizedGlobalMapCloudMsg);
        }
        ROS_DEBUG("GlobalmapVector setfree!");

        OptimizedGlobalCloud->clear();
        TransMapCloud->clear();

    }

    //读取一帧的数据
    KeyFrame2 LoopClosure::ReadCurrFrame(int index, vector<KeyFrame2> &CurrFrameVec, EDGE_SELECT LoopEdge) {
        KeyFrame2 frame;
        for (vector<KeyFrame2>::iterator it = CurrFrameVec.begin(); it != CurrFrameVec.end(); it++) {

            if (LoopEdge == LOCAL) {
                if (it->LocalFrameID == index) {
                    frame.FrameID = it->FrameID;
                    frame.LocalFrameID = it->LocalFrameID;
                    frame.KeyFrametimeStamp = it->KeyFrametimeStamp;
                    frame.Trans_Keyframe2Map = it->Trans_Keyframe2Map;
                    frame.NDTmap = it->NDTmap;
                    frame.KeyframePoint = it->KeyframePoint;
                    cout << " find the designed frame!!!" << endl;
                    break;
                } else {
                    // ROS_DEBUG(" can not find the designed frame");
                }
            }
            if (LoopEdge == GLOBAL) {
                if (it->FrameID == index) {
                    frame.FrameID = it->FrameID;
                    frame.LocalFrameID = it->LocalFrameID;
                    frame.KeyFrametimeStamp = it->KeyFrametimeStamp;
                    frame.Trans_Keyframe2Map = it->Trans_Keyframe2Map;
                    frame.NDTmap = it->NDTmap;
                    frame.KeyframePoint = it->KeyframePoint;
                    cout << " find the designed frame!!!" << endl;
                    break;
                } else {
                    // ROS_DEBUG(" can not find the designed frame");
                }
            }


        }
        return frame;
    }

    void LoopClosure::AddLoopedge(KeyFrame2 &f1, KeyFrame2 &f2, g2o::SparseOptimizer &opti, EDGE_SELECT LoopEdge) {
        // 边部分
        // AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator("Tukey");
        int LastIndex, CurrIndex;
        if (LoopEdge == LOCAL) {
            LastIndex = f1.LocalFrameID;
            CurrIndex = f2.LocalFrameID;
            //  LastIndex = f1.FrameID;
            // CurrIndex = f2.FrameID;
        }
        if (LoopEdge == GLOBAL) {
            LastIndex = f1.FrameID;
            CurrIndex = f2.FrameID;
        }


        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->setVertex(0, opti.vertex(LastIndex));
        edge->setVertex(1, opti.vertex(CurrIndex));
        // edge->setRobustKernel(new g2o::RobustKernelHuber());
        //    edge->setRobustKernel(creator->construct());
        //  edge->robustKernel()->setDelta(1.0);


        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0, 0) = information(1, 1) = information(2, 2) = loopEdge_informationPos;
        information(3, 3) = information(4, 4) = information(5, 5) = loopEdge_informationAngle;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation(information);

        Eigen::Matrix4f Transform;
        //调用GICP算法
        if (LoopEdge == LOCAL && (f2.KeyframePoint.size() * f1.KeyframePoint.size()) > 0) {
            ros::WallTime startTime = ros::WallTime::now();
            Transform = GlobalKeyframePtr->GICP_CalculateTransform(f2.KeyframePoint, f1.KeyframePoint,
                                                                   Eigen::Matrix4f::Identity());

            std::cout << "Total GICP_Process took: " << (ros::WallTime::now() - startTime).toSec()
                      << std::endl;
        }
        if (LoopEdge == GLOBAL) {
            /*************************储存闭环帧*****************************************/
//            ostringstream file1, file2;
//            file1 << f1.FrameID << ".pcd";
//            file2 << f2.FrameID << ".pcd";
//            pcl::io::savePCDFileASCII("/home/machozhao/catkin_ws/src/IRON_loopclosure/doc/" + file1.str(), f1.KeyframePoint);
//            pcl::io::savePCDFileASCII("/home/machozhao/catkin_ws/src/IRON_loopclosure/doc/" + file2.str(), f2.KeyframePoint);
            /*************************使用NDT算法完成粗匹配*****************************************/
            Transform = TransformPtr->NDT_CalculateTransform(f1.NDTmap, f2.NDTmap, f1.FrameID, f2.FrameID);
            cout << "NDT_Matching.inlierset:" << TransformPtr->inlierset.size() << endl;
            /*************************使用NDT算法完成粗匹配*****************************************/
            if (TransformPtr->inlierset.size() > 8) {
                /*************************NDT算法的位姿估计作为ICP的初值*****************************************/
                //  Transform = GlobalKeyframePtr->GICP_CalculateTransform(f1.KeyframePoint, f2.KeyframePoint,Transform);
                // Transform = GlobalKeyframePtr->GICP_CalculateTransform(f1.KeyframePoint, f2.KeyframePoint,Transform);
//                Transform << 0.998478, 0.0532848, 0.0142382, -0.031649,
//                        -0.0539252, 0.997332, 0.0491921, -0.681233,
//                        -0.0115792, -0.0498852, 0.998688, -0.214574,
//                        0, 0, 0, 1;

                Transform = GlobalKeyframePtr->pairAlign(f1.KeyframePoint, f2.KeyframePoint, Transform);
                /*****************f1.KeyframePoint=matched point/f2.KeyframePoint=current  *************************************/
//                pcl::PointCloud< pcl::PointNormal > Alignresult;
//                pcl::transformPointCloud(f2.KeyframePoint, Alignresult, Transform);
//                Alignresult += f1.KeyframePoint;
//                ostringstream  file1,file2;
//                file1 << f1.FrameID << ".pcd";
//                file2 << f2.FrameID << "_";
//                pcl::io::savePCDFileASCII("/home/machozhao/catkin_ws/src/IRON_loopclosure/doc/" + file2.str()+file1.str(),   Alignresult);
//                Alignresult.clear();
            }

        }
        Eigen::Matrix4f Trans_Cur2Last;  //描述的是当前帧到历史的keyframeDatabase最后一帧的transform
        Eigen::Matrix4d Trans_Cur2Last2;    //只是想转换成double格式
        if (GlobalKeyframePtr->gicp_FitnessScore < 0.1) {

            cout << "add edge fist step!" << endl;
            Trans_Cur2Last = Transform;
            //  Trans_Cur2Last = (f1.Trans_Keyframe2Map).inverse() * f2.Trans_Keyframe2Map;
            edge->setMeasurement(Matrix4ftoIsometry3d(Trans_Cur2Last));

            edge->setRobustKernel(new g2o::RobustKernelCauchy());
            edge->robustKernel()->setDelta(1.0);

            if (TransformPtr->inlierset.size() > 8 && LoopEdge == GLOBAL) {
                cout << "add edge second step!" << endl;
                opti.addEdge(edge);
            }
            add_new_loopclosure = true;
        }
        if (LoopEdge == LOCAL) {
            opti.addEdge(edge);
        }

    }

    void LoopClosure::AddOdometryedge(KeyFrame2 &f1, KeyFrame2 &f2, g2o::SparseOptimizer &opti, EDGE_SELECT OdomEdge) {
        // 向g2o中增加这个顶点与上一帧联系的边
        // 顶点部分
        // 顶点只需设定id即可
        int LastIndex, CurrIndex;
        if (OdomEdge == LOCAL) {
            LastIndex = f1.LocalFrameID;
            CurrIndex = f2.LocalFrameID;
        }
        if (OdomEdge == GLOBAL) {
            LastIndex = f1.FrameID;
            CurrIndex = f2.FrameID;
        }

        ROS_DEBUG("add odomery edge [%d] and [%d]", LastIndex, CurrIndex);
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(CurrIndex);

        Eigen::Matrix4f Trans_f2;
        Trans_f2 = f2.Trans_Keyframe2Map;
        ROS_DEBUG("X:[%f],Y:[%f],Z:[%f]", Trans_f2(0, 3), Trans_f2(1, 3), Trans_f2(2, 3));
        v->setEstimate(Matrix4ftoIsometry3d(Trans_f2));
        opti.addVertex(v);

        // 边部分
        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->setVertex(0, opti.vertex(LastIndex));
        edge->setVertex(1, opti.vertex(CurrIndex));
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0, 0) = information(1, 1) = information(2, 2) = odometryEdge_informationPos;
        information(3, 3) = information(4, 4) = information(5, 5) = odometryEdge_informationAngle;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation(information);
        Eigen::Matrix4f Trans_f22f1;
        Trans_f22f1 = (f1.Trans_Keyframe2Map).inverse() * f2.Trans_Keyframe2Map;
        edge->setMeasurement(Matrix4ftoIsometry3d(Trans_f22f1));
        // edge->setMeasurement( T.inverse() );
        // 将此边加入图中
        opti.addEdge(edge);

    }


    /**
     * 计算矩阵之间的欧式距离
     * @param T1 矩阵1
     * @param T2 矩阵2
     * @return 欧式距离
     */
    double LoopClosure::ComputeDistanceBetweenMatrix(Eigen::Matrix4f T1, Eigen::Matrix4f T2) {

        Eigen::Vector3d T1_trans, T2_trans;
        T1_trans = GetTransVecFromMatrix(T1);
        T2_trans = GetTransVecFromMatrix(T2);
        Eigen::Vector3d diff = T1_trans - T2_trans;
        //norm为自己内积开根号
        double distance = diff.norm();
        return distance;
    }

    Eigen::Vector3d LoopClosure::GetTransVecFromMatrix(Eigen::Matrix4f T) {
        Eigen::Vector3d trans_vec;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                if (j > 2 && i < 3) {
                    trans_vec(i) = T(i, j);
                }
            }
        return trans_vec;
    }


    Eigen::Isometry3d LoopClosure::Matrix4ftoIsometry3d(Eigen::Matrix4f T) {
        Eigen::Matrix4d Trans_Cur2Last2;    //只是想转换成double格式
        Eigen::Vector3d trans_vec;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                Trans_Cur2Last2(i, j) = T(i, j);
                if (j > 2 && i < 3) {
                    trans_vec(i) = T(i, j);
                }
            }

        Eigen::Matrix3d rotation_matrix = Trans_Cur2Last2.block(0, 0, 3, 3);

        Eigen::Isometry3d Trans = Eigen::Isometry3d::Identity();
        Trans.rotate(rotation_matrix);
        Trans.pretranslate(trans_vec);
        return Trans;
    }

    Pose LoopClosure::Matrix4ftoTransvec(Eigen::Matrix4f T) {
        Eigen::Vector3d trans_vec;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                if (j > 2 && i < 3) {
                    trans_vec(i) = T(i, j);
                }
            }
        Pose temp, second;
        temp.x = trans_vec(0);
        temp.y = trans_vec(1);
        temp.z = trans_vec(2);
        return temp;
    }

    Eigen::Matrix4f LoopClosure::Isometry3dtoMatrix4f(Eigen::Isometry3d T) {
        Eigen::Matrix4d T1;
        Eigen::Matrix4f T2;
        T1 = T.matrix();
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                T2(i, j) = T1(i, j);
            }
        return T2;
    }


    void LoopClosure::checkNearbyLoops(vector<KeyFrame2> &frames, KeyFrame2 &currFrame,
                                       g2o::SparseOptimizer &opti) {

        int currIndex;
        currIndex = currFrame.LocalFrameID;
        currIndex = currFrame.LocalFrameID;
        if (currIndex % 11 == 0) {
            ROS_DEBUG("find   currIndex! [%d]", currIndex);
            AddLoopedge(frames[currIndex - 11], currFrame, opti, LOCAL);
        }

    }


    void LoopClosure::checkRandomLoops(vector<KeyFrame2> &frames, KeyFrame2 &currFrame, g2o::SparseOptimizer &opti) {

        srand((unsigned int) time(NULL));
        // 随机取一些帧进行检测

        if (currFrame.LocalFrameID <= 5) {
            // no enough keyframes, check everyone
            for (size_t i = 0; i < currFrame.LocalFrameID - 1; i++) {
                AddLoopedge(frames[i], currFrame, opti, LOCAL);
            }
        } else {
            // randomly check loops
            for (int i = 0; i < 5; i++) {
                int index = rand() % frames.size();
                AddLoopedge(frames[index], currFrame, opti, LOCAL);
            }
        }
    }

    void LoopClosure::PublishLoopdetectionArea(Pose CurrPose) {
        if (closure_area_pub_.getNumSubscribers() > 0) {
            visualization_msgs::Marker m;
            m.header.frame_id = fixed_frame_id_;
            m.ns = fixed_frame_id_;
            m.id = 4;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::SPHERE;
            m.color.r = 0.0;
            m.color.g = 0.4;
            m.color.b = 0.8;
            m.color.a = 0.4;
            m.scale.x = proximity_threshold * 2.0;
            m.scale.y = proximity_threshold * 2.0;
            m.scale.z = proximity_threshold * 2.0;
            m.pose.position.x = CurrPose.x;
            m.pose.position.y = CurrPose.y;
            m.pose.position.z = CurrPose.z;
            m.pose.orientation.x = 0.0;
            m.pose.orientation.y = 0.0;
            m.pose.orientation.z = 0.0;
            m.pose.orientation.w = 1.0;
            closure_area_pub_.publish(m);
        }

    }

    void LoopClosure::PublishLocalPoseGraph() {

        // Publish odometry edges.
        if (odometry_edge_pub_.getNumSubscribers() > 0) {
            visualization_msgs::Marker m;
            m.header.frame_id = fixed_frame_id_;
            m.ns = fixed_frame_id_;

            m.id = 0;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::LINE_LIST;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 0.8;
            m.scale.x = 0.02;

            for (size_t ii = 0; ii < PoseafterG2o.size(); ++ii) {
                m.points.push_back(ToRosPoint(PoseafterG2o[ii]));
            }
            odometry_edge_pub_.publish(m);
        }


        // Publish keyframe nodes in the pose graph.
        visualization_msgs::Marker m;
        m.header.frame_id = fixed_frame_id_;
        m.ns = fixed_frame_id_;
        m.header.stamp = ros::Time::now();
        m.id = 5;
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::SPHERE_LIST;
        m.color.r = 0.3;
        m.color.g = 0.0;
        m.color.b = 1.0;
        m.color.a = 0.8;
        m.scale.x = 0.1;
        m.scale.y = 0.1;
        m.scale.z = 0.1;
        for (size_t ii = 0; ii < PoseafterG2o.size(); ++ii) {
            m.points.push_back(ToRosPoint(PoseafterG2o[ii]));
        }

        graph_node_pub_.publish(m);

        // Publish loop closure edges.
        if (loop_edge_pub_.getNumSubscribers() > 0) {
            visualization_msgs::Marker m;
            m.header.frame_id = fixed_frame_id_;
            m.ns = fixed_frame_id_;
            m.id = 1;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::LINE_LIST;
            m.color.r = 1.0;
            m.color.g = 0.2;
            m.color.b = 0.0;
            m.color.a = 0.8;
            m.scale.x = 0.005;

            for (size_t ii = 0; ii < loop_edge.size(); ++ii) {

                m.points.push_back(ToRosPoint(loop_edge[ii].first));
                m.points.push_back(ToRosPoint(loop_edge[ii].second));
            }
            loop_edge_pub_.publish(m);
        }

    }

    void LoopClosure::PublishGlobalPoseGraph() {
        // Publish global odometry edges.
        if (global_odometry_edge_pub_.getNumSubscribers() > 0) {
            visualization_msgs::Marker m;
            m.header.frame_id = fixed_frame_id_;
            m.ns = fixed_frame_id_;

            m.id = 0;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::LINE_LIST;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 0.8;
            m.scale.x = 0.02;

            for (size_t ii = 0; ii < GlobalPoseafterG2o.size(); ++ii) {
                m.points.push_back(ToRosPoint(GlobalPoseafterG2o[ii]));
            }
            global_odometry_edge_pub_.publish(m);
        }

        // Publish keyframe nodes in the pose graph.
        if (global_graph_node_pub_.getNumSubscribers() > 0) {
            visualization_msgs::Marker m;
            m.header.frame_id = fixed_frame_id_;
            m.ns = fixed_frame_id_;
            m.id = 2;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::SPHERE_LIST;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 0.8;
            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;
            for (size_t ii = 0; ii < GlobalPoseafterG2o.size(); ++ii) {
                m.points.push_back(ToRosPoint(GlobalPoseafterG2o[ii]));
            }
            global_graph_node_pub_.publish(m);
        }
        // Publish loop closure edges.
        if (loop_edge_pub_.getNumSubscribers() > 0) {
            visualization_msgs::Marker m;
            m.header.frame_id = fixed_frame_id_;
            m.ns = fixed_frame_id_;
            m.id = 1;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::LINE_LIST;
            m.color.r = 1.0;
            m.color.g = 0.2;
            m.color.b = 0.0;
            m.color.a = 0.8;
            m.scale.x = 0.005;

            for (size_t ii = 0; ii < loop_edge.size(); ++ii) {

                m.points.push_back(ToRosPoint(loop_edge[ii].first));
                m.points.push_back(ToRosPoint(loop_edge[ii].second));
            }
            loop_edge_pub_.publish(m);
        }

    }


    vector<Pose> LoopClosure::IntialLocalPoseGraph(vector<KeyFrame2> &KeyFrameDatabase2) {

        //指定线性方程求解器使用Eigen的块求解器
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver =
                new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        //不输出调用信息
        LocalOptimizer.setVerbose(false);
        // 构造线性求解器
        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
        // 使用LM算法进行非线性迭代
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e-16);
        LocalOptimizer.setAlgorithm(solver);

        // 向localOptimizer增加第一个顶点
        if ((KeyFrameDatabase2[0].LocalFrameID == startIndex)) {

            g2o::VertexSE3 *v = new g2o::VertexSE3();
            v->setId(KeyFrameDatabase2[0].LocalFrameID);
            // v->setEstimate(Eigen::Isometry3d::Identity()); //估计为单位矩阵
            v->setFixed(true); //第一个顶点固定，不用优化
            Eigen::Matrix4f Trans_Cur2Last;
            Trans_Cur2Last = KeyFrameDatabase2[0].Trans_Keyframe2Map;
            v->setEstimate(Matrix4ftoIsometry3d(Trans_Cur2Last));
            LocalOptimizer.addVertex(v);

        } else {
            ROS_DEBUG("Miss the First Frame!!!!!!!!!! ");
        }

        LoopClosure loop(first_posegraph, GlobalKeyframePtr, TransformPtr);

        loop.StartLocalPoseGraph(KeyFrameDatabase2);
        PublishLocalPoseGraph();
        LocalOptimizer.clear();

        return PoseafterG2o;
    }


    void LoopClosure::StartLocalPoseGraph(vector<KeyFrame2> &KeyFrameDatabase2) {

        for (currIndex = startIndex + 1; currIndex <= KeyFrameDatabase2.size(); currIndex++) {

            LoopClosure loop;

            KeyFrame2 currFrame = loop.ReadCurrFrame(currIndex, KeyFrameDatabase2, LOCAL);

            AddOdometryedge(KeyFrameDatabase2[currIndex - 2], currFrame, LocalOptimizer, LOCAL); //建立相邻两帧之间的链接
            if (check_local_closure == true) {

                checkNearbyLoops(KeyFrameDatabase2, currFrame, LocalOptimizer);
                //checkRandomLoops( KeyFrameDatabase2, currFrame, globalOptimizer );
            }
        }
        //优化为姿并更新世界坐标系下的为姿态
        LocalOptimizeProcess(KeyFrameDatabase2);

    }

    void LoopClosure::LocalOptimizeProcess(vector<KeyFrame2> &KeyFrameDatabase2) {

        LocalOptimizer.initializeOptimization();
        LocalOptimizer.optimize(5); //可以指定优化步数


        for (size_t i = 0; i < KeyFrameDatabase2.size(); i++) {
            // 从g2o里取出一帧
            g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *>(LocalOptimizer.vertex(
                    KeyFrameDatabase2[i].LocalFrameID));

            Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
            pose.translation();

            Eigen::Vector3d Pose_w;
            Pose_w = Eigen::Vector3d(pose.translation());
            Pose Pose_world;
            Pose_world.x = Pose_w(0);
            Pose_world.y = Pose_w(1);
            Pose_world.z = Pose_w(2);

            PoseafterG2o.push_back(Pose_world);

            int globalFrameID, LocalFrameID;
            globalFrameID = KeyFrameDatabase2[i].FrameID;
            LocalFrameID = KeyFrameDatabase2[i].LocalFrameID;

            std::mutex mutex;
            mutex.lock();
            GlobalKeyframePtr->KeyFrameDatabase2[globalFrameID - 1].Trans_Keyframe2Map = Isometry3dtoMatrix4f(pose);
            GlobalKeyframePtr->LocalKeyframeDatabase[LocalFrameID - 1].Trans_Keyframe2Map = Isometry3dtoMatrix4f(pose);
            mutex.unlock();

        }

        ROS_DEBUG(" Local optimizing is finished !!!!");
    }


}

