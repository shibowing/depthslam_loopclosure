//
// Created by edward on 17-12-8.
//

#ifndef DEPTH_MAP_MATRIXTRANSFORM_H
#define DEPTH_MAP_MATRIXTRANSFORM_H

#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>

namespace DEPTH_MAP {
    class Matrixtransform {

    public:
        Matrixtransform();

        ~Matrixtransform();

        Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);

        Eigen::Vector3d Quaterniond2Euler(const double x, const double y, const double z, const double w);

        Eigen::Matrix3d Quaternion2RotationMatrix(const double x, const double y, const double z, const double w);

        Eigen::Quaternionf rotationMatrix2Quaternionf(Eigen::Matrix3f R);

        Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw);

        Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R);

    };
}

#endif //DEPTH_MAP_MATRIXTRANSFORM_H
