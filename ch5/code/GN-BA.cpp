//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream p3d_f, p2d_f;
    p3d_f.open(p3d_file.c_str());
    if (!p3d_f.is_open()){
        cout<< "this file is empty!" << endl;
        return -1;
    }
    p2d_f.open(p2d_file.c_str());
    if (!p2d_f.is_open()){
        cout<< "this file is empty!" << endl;
        return -1;
    }

    string sline;
    while (getline(p3d_f,sline) && !sline.empty()){
        istringstream iss(sline);
        Vector3d vector3D{0,0,0};
        iss  >> vector3D(0) >> vector3D(1) >> vector3D(2);
        p3d.push_back(vector3D);
    }

    while (getline(p2d_f,sline) && !sline.empty()){
        istringstream iss(sline);
        Vector2d vector2D{0,0};
        iss  >> vector2D(0) >> vector2D(1);
        p2d.push_back(vector2D);
    }

    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    // cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti(Matrix3d::Identity(), Vector3d::Zero()); // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        Vector2d e; // 第i个数据点的计算误差
        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE

            Vector3d nonhomo_vec = T_esti*p3d[i];
            Vector3d homo_vector = K * nonhomo_vec;
            Vector2d pixel_coor = {homo_vector(0)/homo_vector(2), homo_vector(1)/homo_vector(2)};
            e = p2d[i] - pixel_coor;
	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            double x = nonhomo_vec[0];
            double y = nonhomo_vec[1];
            double z = nonhomo_vec[2];
            double z_2 = z * z;
            // se3 默认平移在前，旋转在后！
            J <<-(fx/z), 0, fx*x/z_2,  fx*x*y/z_2, -(fx+fx*x*x/z_2), fx*y/z,
                    0, -fy/z, fy*y/z_2,fy+fy*y*y/z_2, -fy*x*y/z_2, -fy*x/z;
	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;

            cost += e.transpose() * e;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE
        // 这里在求矩阵H的时候，因为是稀疏矩阵，有利用shur消元加速求解的方法，在第十章有详细介绍
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE
        // T更新
        T_esti = Sophus::SE3::exp(dx)*T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(10) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
