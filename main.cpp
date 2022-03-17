//#define EIGEN_USE_MKL_ALL
//#define EIGEN_VECTORIZE_SSE4_2

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <time.h>
#include <chrono>


//Eigen的速度为何这么快！！！

using namespace std;
using namespace Eigen;
using namespace chrono;

int main() {
    std::cout << "Hello, +World!" << std::endl;

    MatrixXd m1 = MatrixXd::Random(100,100);
    MatrixXd m2 = MatrixXd::Random(100,100);
    MatrixXd m3 = MatrixXd::Zero(100,100);

    //  旋转矩阵直接使用Matrix3d，注意其中的3，不是X，直接设置为单位矩阵
    Matrix3d rotation_matrix;
    rotation_matrix.setIdentity();

    //  旋转向量
    AngleAxisd rotation_vector(M_PI / 4,Vector3d(0,0,1));

    //  小数位保留3位
    cout.precision(3);

    cout << "rotation vector angle: " << rotation_vector.angle() * (180/M_PI)
    << " axis: " << rotation_vector.axis().transpose() << endl;

    rotation_matrix = rotation_vector.toRotationMatrix();

    //  旋转向量直接使用方法matrix()或者方法toRotationMatrix()直接变为旋转矩阵，实现了罗德里格斯公式的目的
    cout << "rotation matrix = \n:" << rotation_vector.matrix() << "\n\n" << rotation_vector.toRotationMatrix() <<endl;

    Vector3d v1(1,0,0);
    Vector3d v1_rotated = rotation_vector * v1;
    cout << "(1,0,0) after rotation = " << v1_rotated.transpose() << endl;
    v1_rotated = rotation_matrix * v1;
    cout << "(1,0,0) after rotation = " << v1_rotated.transpose() << endl;

    // Euler，按zyx的顺序，由旋转矩阵直接转换成欧拉角
    Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);
    cout << "yaw pitch roll = " << euler_angles.transpose() * (180 / M_PI) << endl;

    // 变换矩阵 4x4
    Isometry3d T = Isometry3d :: Identity();
    // cout << T.matrix() << endl; 直接无法输出T，必须matix()才能形成4X4单位矩阵
    T.rotate(rotation_vector);
    // T.rotate(rotation_matrix);也可，相当于由旋转矩阵构造变换矩阵
    // 设置平移向量
    T.pretranslate(Vector3d(0,0,3));
    cout << "Transform Matrix = \n"<< T.matrix() << endl;

    // 用变换矩阵进行坐标变换
    Vector3d v1_transformed = T * v1;
    cout << "v1_transformed = \n" << v1_transformed.transpose() << endl;

    // 用旋转向量构造四元数
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quaternion = \n" << q.coeffs() << endl;
    // 用旋转矩阵构造四元数
    q = Quaterniond(rotation_matrix);
    cout << "quaternion = \n" << q.coeffs() << endl;
    // 用欧拉角构造四元数
    AngleAxisd rollAngle(AngleAxisd(euler_angles(2),Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(euler_angles(1),Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(euler_angles(0),Vector3d::UnitZ()));

    q = yawAngle * pitchAngle * rollAngle;
    cout << "quaternion = \n" << q.coeffs() << endl;

    v1_rotated = q * v1;
    cout << "(1,0,0) after rotation = " << v1_rotated.transpose() << endl;


    Vector3d v(1,2,3);
    Vector3d w(0,1,2);
    cout << "v cross w = \n" << v.cross(w) << endl;
    cout << "begin product" << endl;


    auto start = system_clock::now();
    m3 += m1.transpose() * m2;
    //m3 = m1 * m2;
    auto end = system_clock::now();

    auto duration = duration_cast<microseconds>(end-start);
    cout << double(duration.count()) * microseconds::period::num / microseconds::period::den << endl;

    //Vector3d v = {0.2,0.4,0.8};
    //v.transpose(); 转置=向量
    //v.norm();模长=实数
    //auto n = v.normalized();单位向量=向量
    //v.squaredNorm();元素的平方和=实数，开根号之后=模长=实数
    //v.cwiseSqrt();元素的开平方=向量
    //cout << v.norm() << endl;
    //cout << v.cwiseSqrt() << endl;


    return 0;
}
