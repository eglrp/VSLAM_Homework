//
// Created by xuzhi on 19-7-15.
//

#include <sophus/so3.h>
#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>


// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "../compare.txt";

void Icp(const vector<Eigen::Vector3d>& position1,
         const vector<Eigen::Vector3d>& position2,
         Eigen::Matrix3d &R,
         Eigen::Vector3d &t){

        Eigen::Vector3d p1, p2;     // center of mass质心
        int N = position1.size();
        for ( int i=0; i<N; i++ )
        {
            p1 += position1[i];
            p2 += position2[i];
        }
        p1 = p1/N;
        p2 = p2/N;

        vector<Eigen::Vector3d>  q1 ( N ), q2 ( N ); // remove the center 去质心
        for ( int i=0; i<N; i++ )
        {
            q1[i] = position1[i] - p1;
            q2[i] = position2[i] - p2;
        }

        // compute q1*q2^T
        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        for ( int i=0; i<N; i++ )
        {
            W += q1[i] * q2[i].transpose();
        }
        cout<<"W="<<W<<endl;

        // SVD on W
        Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        // 确保行列式之积大于零  但是这里的原因我不明白
        if (U.determinant() * V.determinant() < 0)
        {
            for (int x = 0; x < 3; ++x)
            {
                U(x, 2) *= -1;
            }
        }

        cout<<"U="<<U<<endl;
        cout<<"V="<<V<<endl;

        R = U* ( V.transpose() );
        t = p1 - R * p2;
}

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2;

    vector<Eigen::Vector3d> position1;
    vector<Eigen::Vector3d> position2;



    Eigen::Quaterniond q1, q2;
    Eigen::Vector3d t1, t2;
    Sophus::SE3 T1, T2;
    ifstream trajectory;
    double time_stamp1, time_stemp2;

    trajectory.open(trajectory_file.c_str());
    if (!trajectory.is_open()){
        cout << "the file is empty!!" << endl;
        return -1;
    }

    string sLine;
    while(getline(trajectory, sLine) && !sLine.empty()){
        istringstream iss(sLine);
        iss >> time_stamp1 >> t1[0] >> t1[1] >> t1[2] >> q1.x() >> q1.y() >> q1.z() >> q1.w()
        >> time_stemp2 >> t2[0] >> t2[1] >> t2[2] >> q2.x() >> q2.y() >> q2.z() >> q2.w();
        T1 = Sophus::SE3(q1, t1);
        poses1.push_back(T1);
        position1.push_back(t1);
        T2 = Sophus::SE3(q2, t2);
        poses2.push_back(T2);
        position2.push_back(t2);
    }
    trajectory.close();

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    Icp(position1,position2, R, t);

    Sophus::SE3 SE3_update(R, t);           // 从R,t构造SE(3)


    for (auto item: poses1){
        poses.push_back(item);
    }


    for (auto item: poses2){
        item = SE3_update * item;
        poses.push_back(item);
    }


    DrawTrajectory(poses);
    return 0;
}




/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    cout<<"poses size : "<<poses.size()<<endl;

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);

        for (size_t i = 0; i < 612 - 1; i++) {
            glColor3f(0.0f, 100.0f,  0.0f);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 612; i < 1224 - 1; i++) {
            glColor3f(75.0f, 0.0f,  128.0f);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
