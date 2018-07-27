//
// Created by edward on 17-12-8.
//

#include "Matrixtransform.h"
//
// Created by edward on 17-12-8.
//


using namespace std;
using namespace Eigen;

namespace DEPTH_MAP {

    Matrixtransform::Matrixtransform() {

    };

    Matrixtransform::~Matrixtransform() {

    };

    Eigen::Quaterniond Matrixtransform::euler2Quaternion(const double roll, const double pitch, const double yaw) {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
        cout << "Euler2Quaternion result is:" << endl;
        cout << "x = " << q.x() << endl;
        cout << "y = " << q.y() << endl;
        cout << "z = " << q.z() << endl;
        cout << "w = " << q.w() << endl << endl;
        return q;
    }

    Eigen::Vector3d Matrixtransform::Quaterniond2Euler(const double x, const double y, const double z, const double w) {
        Eigen::Quaterniond q;
        q.x() = x;
        q.y() = y;
        q.z() = z;
        q.w() = w;

        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
        cout << "Quaterniond2Euler result is:" << endl;
        cout << "x = " << euler[2] << endl;
        cout << "y = " << euler[1] << endl;
        cout << "z = " << euler[0] << endl << endl;
    }

    Eigen::Matrix3d
    Matrixtransform::Quaternion2RotationMatrix(const double x, const double y, const double z, const double w) {
        Eigen::Quaterniond q;
        q.x() = x;
        q.y() = y;
        q.z() = z;
        q.w() = w;

        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        return R;
    }


    Eigen::Quaternionf Matrixtransform::rotationMatrix2Quaternionf(Eigen::Matrix3f R) {
        Eigen::Quaternionf q = Eigen::Quaternionf(R);

        q.normalize();
        return q;
    }

    Eigen::Matrix3d Matrixtransform::euler2RotationMatrix(const double roll, const double pitch, const double yaw) {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix3d R = q.matrix();
        return R;
    }

    Eigen::Vector3d Matrixtransform::RotationMatrix2euler(Eigen::Matrix3d R) {
        Eigen::Matrix3d m;
        m = R;
        Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);
        cout << "RotationMatrix2euler result is:" << endl;
        cout << "x = " << euler[2] << endl;
        cout << "y = " << euler[1] << endl;
        cout << "z = " << euler[0] << endl << endl;
        return euler;
    }

}
