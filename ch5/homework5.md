## 一、ORB特征点

按照本题的指导,自行书写 ORB 的提取、描述子的计算以及
匹配的代码。代码框架参照 computeORB.cpp 文件,图像见 1.png 文件和 2.png。

#### 1. ORB提取

ORB即Oriented FAST简称，他实际上是FAST特征再加上一个旋转量。本题使用OpenCV自带的FAST提取算法，你要完成旋转部分的计算。

代码部分：

```cpp
void computeAngle(const cv::Mat &image, vector<cv::KeyPoint> &keypoints) {
    int half_patch_size = 8;

    for (auto &kp : keypoints) {
	// TODO: START YOUR CODE HERE (~7 lines)
        kp.angle = 0; // compute kp.angle
        int u = kp.pt.x;
        int v = kp.pt.y;
        if (u >= half_patch_size && u <= image.cols-half_patch_size
        && v >= half_patch_size && v < image.rows - half_patch_size) {

            double m_10 = 0;
            double m_01 = 0;
            for (int i = - half_patch_size; i < half_patch_size; ++i) {
                for (int j =- half_patch_size; j < half_patch_size; ++j) {
                    double intensity = (double)image.at<uchar>(v + j,u + i);
                    m_01 += j*intensity;
                    m_10 += i*intensity;
                }
            }
            kp.angle = (float)atan2(m_01, m_10) * 180 / pi;
        }

        // END YOUR CODE HERE
    }
    return;
}

```

CMakeList.txt文件：

```
cmake_minimum_required( VERSION 2.8 )
project( vo1 )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# find_package( OpenCV 3.1 REQUIRED )
find_package( OpenCV REQUIRED ) # use this if in OpenCV2

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable( computeORB computeORB.cpp  )
target_link_libraries( computeORB ${OpenCV_LIBS} )

```

最终输出的带旋转的FAST特征如下图所示：

![带旋转的FAST特征点](feat1.png)


**实现过程中的注意事项：**

1. computeAngle函数中输入的是灰度图像

2. image.at(x,y)取对应的坐标intensity值的时候，要注意这里的x代表rows, y代表cols.

3. kp.pt.x 和kp.pt.y中对应的x, y与像素坐标下的u，v对应，也就是x表示cols,y表示rows.

4. 在定义图像块的矩计算的时候，其中的x,y 表示的是16×16的图像块的u，v. 这一部分一定要搞明白，我整整调了两个小时，才弄出正确的结果！！！


#### 2. ORB描述

ORB描述即带旋转的BRIEF描述。指的是一个0-1组成的字符串，每个bit表示一次像素间的比较。

代码实现：
```cpp
bool inImage(int a, int b){
    return a>= 0 && a  < 640 && b  >= 0 && b < 480;
}

// compute the descriptor
void computeORBDesc(const cv::Mat &image, vector<cv::KeyPoint> &keypoints, vector<DescType> &desc) {
    // 对于每个特征点
    for (auto &kp: keypoints) {
        DescType d(256, false);
        int u = kp.pt.x;
        int v = kp.pt.y;
        double theta = kp.angle*pi/180;
        int u1_update, v1_update, u2_update, v2_update;

        for (int i = 0; i < 256; i++) {
            // TODO: START YOUR CODE HERE (~7 lines)
            d[i] = 0;  // if kp goes outside, set d.clear()

            u1_update =u + (int) (cos(theta)*ORB_pattern[i*4] -sin(theta)*ORB_pattern[i*4+1]);
            v1_update =v + (int)  (sin(theta)*ORB_pattern[i*4] + cos(theta)*ORB_pattern[i*4+1]);

            u2_update =u + (int) (cos(theta)*ORB_pattern[i*4 + 2] -sin(theta)*ORB_pattern[i*4 + 3]);
            v2_update =v + (int) (sin(theta)*ORB_pattern[i*4 + 2] + cos(theta)*ORB_pattern[i*4 + 3]);

            if (!inImage(u1_update,v1_update) || !inImage(u2_update,v2_update)){
                d.clear();
                break;
            }else{
                double intensity1 = image.at<uchar>(u1_update,v1_update);
                double intensity2 = image.at<uchar>(u2_update, v2_update);
                d[i] = (intensity1 > intensity2) ? 0 : 1;
            }
	    // END YOUR CODE HERE
        }
        desc.push_back(d);
    }

    int bad = 0;
    for (auto &d: desc) {
        if (d.empty()) bad++;
    }
    cout << "bad/total: " << bad << "/" << desc.size() << endl;
    return;
}

```

输出结果：
```json
keypoints: 638
bad/total: 41/638
keypoints: 595
bad/total: 6/595
```

过程分析：
1. 作业中描述子的类型如下，每个描述子都是用256个bool量组成的vector表示：
`typedef vector<bool> DescType`

2. 在keypoint附近找p,q的时候，先旋转p,q，再更新keypoint，然后判断更新后的keypoint是否越界，最后才是从image中找对应的灰度值，做0,1判断。我上述的这个过程非常重要，因为程序顺序有误的话，最后完全不正确。



#### 3. 暴力匹配



## 二、从E恢复R和t

## 三、使用G-N实现BA

## 四、用ICP实现轨迹对齐
