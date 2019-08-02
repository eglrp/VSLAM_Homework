#include <Eigen/Core>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "ceres/autodiff.h"

#include "tools/rotation.h"
#include "common/projection.h"

using namespace Eigen;
using namespace Sophus;
using namespace g2o;
// 顶点， 9维的向量， 内参+pose+旋转

// 重写camera顶点的类型
class VertexCameraInput{
public:
    VertexCameraInput(){}

    VertexCameraInput(VectorXd camera){
        Vector6d se3;
        se3.head<3>() << camera[3], camera[4], camera[5];
        se3.tail<3>() << camera[0], camera[1], camera[2];
        _SE3 = SE3::exp(se3);
        _f = camera[6];
        _k1 = camera[7];
        _k2 = camera[8];
    }

public:
    SE3 _SE3;
    double _f;
    double _k1;
    double _k2;
};

bool CamModelWithDistort(VertexCameraInput cam, Vector3d Pw, double *prediction){
    Vector3d Pc = cam._SE3*Pw;
    // 归一化坐标系
    double xc = -Pc[0]/Pc[2];
    double yc = -Pc[1]/Pc[2];

    double r2 = xc*xc + yc * yc;
    double distortion = 1 + r2*(cam._k1 + cam._k2*r2);
    prediction[0] = cam._f * xc * distortion;
    prediction[1] = cam._f * yc * distortion;
    return true;
}

class VertexCameraBAL : public g2o::BaseVertex<9,VertexCameraInput>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update )
    {
//        Eigen::VectorXd::ConstMapType v ( update, VertexCameraBAL::Dimension );
//        _estimate += v;

        Vector6d update_se3;
        //Attention: head is t, tail is R
        update_se3 << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate._SE3 = SE3::exp(update_se3) * estimate()._SE3;
        _estimate._f += update[6];
        _estimate._k1 += update[7];
        _estimate._k2 += update[8];
    }

};

// point 只有x,y,z
class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update )
    {
        Eigen::Vector3d::ConstMapType v ( update );
        _estimate += v;
    }
};

class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }


    virtual void computeError() override   // The virtual function comes from the Edge base class. Must define if you use edge.
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> ( vertex ( 0 ) );
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> ( vertex ( 1 ) );
        // 利用重投影计算prediction
        double prediction[2];
        CamModelWithDistort(cam->estimate(), point->estimate(), prediction);
        _error[0] = double(measurement()(0) - prediction[0]);
        _error[1] = double(measurement()(1) - prediction[1]);
    }

    virtual void linearizeOplus() override {
        VertexCameraBAL *cam = static_cast<VertexCameraBAL *> ( vertex(0));
        VertexPointBAL *point = static_cast<VertexPointBAL *> ( vertex(1));

        Vector3d Pc;
        Pc = cam->estimate()._SE3 * point->estimate();
        // 相机坐标系
        double xc = Pc[0];
        double yc = Pc[1];
        double zc = Pc[2];
        // 归一化坐标系
        double xc1 = -xc / zc;
        double yc1 = -yc / zc;
        double zc1 = -1;

        // Apply second and fourth order radial distortion
        const double &k1 = cam->estimate()._k1;
        const double &k2 = cam->estimate()._k2;

        double r2 = xc1 * xc1 + yc1 * yc1;
        double distort = double(1.0) + k1 * r2 + k2 * r2 * r2;

        const double &f = cam->estimate()._f;

        Matrix<double, 2, 6> J_e_kesi;
        Matrix<double, 2, 3> J_e_pc;
        Matrix<double, 3, 6> J_pc_kesi;// = new Eigen::Matrix<double,3,6>::Zero();
        Matrix<double, 2, 1> J_e_f;
        Matrix<double, 2, 2> J_e_k;
        Matrix3d pc_hat;

        double zc_2 = zc * zc;
        double zc_3 = zc_2 * zc;
        double d2 = k1 + 2 * k2 * r2;

        J_e_pc(0, 0) = f / zc * distort + 2 * f * xc * xc / zc_3 * d2;
        J_e_pc(0, 1) = 2 * f * xc * yc / zc_3 * d2;
        J_e_pc(0, 2) = -f * xc / zc_2 * distort - 2 * f * xc * r2 / zc_2 * d2;
        J_e_pc(1, 0) = 2 * f * xc * yc / zc_3 * d2;
        J_e_pc(1, 1) = f / zc * distort + 2 * f * yc * yc / zc_3 * d2;
        J_e_pc(1, 2) = -f * yc / zc_2 * distort - 2 * f * yc * r2 / zc_2 * d2;

        pc_hat << 0, zc, -yc,
                 -zc, 0, xc,
                 yc, -xc, 0;
        J_pc_kesi.block(0,0,3,3) = Matrix3d::Identity();
        J_pc_kesi.block(0,3,3,3) = pc_hat;

        J_e_kesi = J_e_pc * J_pc_kesi;

        J_e_f(0, 0) = xc / zc * distort;
        J_e_f(1, 0) = yc / zc * distort;

        J_e_k(0, 0) = f * xc * r2 / zc;
        J_e_k(0, 1) = f * xc * r2 * r2 / zc;

        J_e_k(1, 0) = f * yc * r2 / zc;
        J_e_k(1, 1) = f * yc * r2 * r2 / zc;

        _jacobianOplusXi.block(0, 0, 2, 6) = J_e_kesi;
        _jacobianOplusXi.block(0, 6, 2, 1) = J_e_f;
        _jacobianOplusXi.block(0, 7, 2, 2) = J_e_k;

        Matrix<double, 2, 3> J_e_pw;
        J_e_pw = J_e_pc * cam->estimate()._SE3.rotation_matrix();
        _jacobianOplusXj = J_e_pw;
    }
};
