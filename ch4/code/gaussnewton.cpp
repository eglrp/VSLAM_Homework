//
// Created by xuzhi.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
    double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值  这里等于是给了个初始值
    int N = 100;                                 // 数据点
    double w_sigma = 1.0;                        // 噪声Sigma值
    cv::RNG rng;                                 // OpenCV随机数产生器

    vector<double> x_data, y_data;      // 数据
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma));
    }

    // 开始Gauss-Newton迭代
    int iterations = 100;    // 迭代次数
    double cost = 0, lastCost = 0;  // 本次迭代的cost和上一次迭代的cost

    for (int iter = 0; iter < iterations; iter++) {

        Matrix3d H = Matrix3d::Zero();             // Hessian = J^T J in Gauss-Newton
        Vector3d b = Vector3d::Zero();             // bias
        cost = 0;

        // 这个计算量真的大，每一次迭代都要求H和b
        for (int i = 0; i < N; i++) {
            double xi = x_data[i], yi = y_data[i];  // 第i个数据点

            // start your code here
            double error = 0;   // 第i个数据点的计算误差
            error = yi - exp(ae*xi*xi+be*xi+ce); // 填写计算error的表达式
            Vector3d J; // 雅可比矩阵
            double temp = -exp(ae*xi*xi+be*xi+ce);
            J[0] = temp*xi*xi;  // de/da
            J[1] = temp*xi;  // de/db
            J[2] = temp;  // de/dc

            H += J * J.transpose(); // GN近似的H
            b += -error * J;    // 这里的error是个标量
            // end your code here

            cost += error * error; // 保证不被正负抵消
        }

        // todo 求解线性方程 Hx=b，建议用ldlt
 	// start your code here
        Vector3d dx;
        // 这里的dx面向的是abc，是abc的步进
        dx = H.ldlt().solve(b);
        // cout << dx[0] << "  " << dx[1] << "  " << dx[2] << endl;
	// end your code here

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost > lastCost) {
            // 误差增长了，说明近似的不够好
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // 更新abc估计值
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;

        cout << "total cost: " << cost << endl;
    }

    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
    return 0;
}