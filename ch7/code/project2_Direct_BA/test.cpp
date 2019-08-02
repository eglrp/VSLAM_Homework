//
// Created by xbot on 19-8-2.
//

//    virtual void computeError() override {
//        // TODO START YOUR CODE HERE
//        // compute projection error ...
//        const VertexSBAPointXYZ *vertexPw = static_cast<const VertexSBAPointXYZ * >(vertex(1));
//        const VertexSophus *vertexTcw = static_cast<const VertexSophus * >(vertex(0));
//
//        Vector3d Pc = vertexTcw->estimate() * vertexPw->estimate();
//        double u = fx*Pc[0]/Pc[2]+cx;
//        double v = fy*Pc[1]/Pc[2]+cy;
//        int count = 0;
//        if (u-2 <0 || v-2<0 || u+1 >= targetImg.cols || v+1 >= targetImg.rows){
//            // Todo setLevel含义
//            this->setLevel(1);
//            for (int i = 0; i < 16; ++i) {
//                _error[i] = 0;
//            }
//        }else{
//            for (int i = -2; i < 2; ++i) {
//                for (int j = -2; j < 2; ++j) {
//                    int num = 4*j+10+i;  // color存储顺序
//                    _error[count++] = origColor[num] - GetPixelValue(targetImg, u + i, v + j);
//                }
//            }
//        }
//
//        // END YOUR CODE HERE
//    }
//
//    // Let g2o compute jacobian for you
//    virtual void linearizeOplus() override {
//        if (level() == 1) {
//            _jacobianOplusXi = Matrix<double, 16, 3>::Zero();
//            _jacobianOplusXj = Matrix<double, 16, 6>::Zero();
//            return;
//        }
//        const VertexSBAPointXYZ *vertexPw = static_cast<const VertexSBAPointXYZ * >(vertex(1));
//        const VertexSophus *vertexTcw = static_cast<const VertexSophus * >(vertex(0));
//        Vector3d Pc = vertexTcw->estimate() * vertexPw->estimate();
//        float x = Pc[0];
//        float y = Pc[1];
//        float z = Pc[2];
//        float inv_z = 1.0 / z;  // 逆深度
//        float inv_z2 = inv_z * inv_z;
//        float u = x * inv_z * fx + cx;
//        float v = y * inv_z * fy + cy;
//
//        Matrix<double, 2, 3> J_Puv_Pc;
//        J_Puv_Pc(0, 0) = fx * inv_z;
//        J_Puv_Pc(0, 1) = 0;
//        J_Puv_Pc(0, 2) = -fx * x * inv_z2;
//        J_Puv_Pc(1, 0) = 0;
//        J_Puv_Pc(1, 1) = fy * inv_z;
//        J_Puv_Pc(1, 2) = -fy * y * inv_z2;
//
//        Matrix<double, 3, 6> J_Pc_kesi = Matrix<double, 3, 6>::Zero();
//        J_Pc_kesi(0, 0) = 1;
//        J_Pc_kesi(0, 4) = z;
//        J_Pc_kesi(0, 5) = -y;
//        J_Pc_kesi(1, 1) = 1;
//        J_Pc_kesi(1, 3) = -z;
//        J_Pc_kesi(1, 5) = x;
//        J_Pc_kesi(2, 2) = 1;
//        J_Pc_kesi(2, 3) = y;
//        J_Pc_kesi(2, 4) = -x;
//
//        Matrix<double, 1, 2> J_I_Puv;
//        for (int i = -2; i < 2; i++)
//            for (int j = -2; j < 2; j++) {
//                int num = 4 * j + i + 10;
//                J_I_Puv(0, 0) =
//                        (GetPixelValue(targetImg, u + i + 1, v + j) - GetPixelValue(targetImg, u + i - 1, v + j)) / 2;
//                J_I_Puv(0, 1) =
//                        (GetPixelValue(targetImg, u + i, v + j + 1) - GetPixelValue(targetImg, u + i, v + j - 1)) / 2;
//                _jacobianOplusXi.block<1, 3>(num, 0) = -J_I_Puv * J_Puv_Pc * vertexTcw->estimate().rotation_matrix();
//                _jacobianOplusXj.block<1, 6>(num, 0) = -J_I_Puv * J_Puv_Pc * J_Pc_kesi;
//            }
//    }




// TODO add vertices, edges into the graph optimizer
// START YOUR CODE HERE
// Add poses Vertex
//    for (int j = 0; j < poses.size(); ++j) {
//        VertexSophus* pCamera = new VertexSophus();
//        pCamera->setEstimate(poses[j]);
//        pCamera->setId(j);
//        optimizer.addVertex(pCamera);
//    }
//    // Add points Vertex
//    for (int k = 0; k < points.size(); ++k) {
//        g2o::VertexSBAPointXYZ* pPoint = new g2o::VertexSBAPointXYZ();
//        pPoint->setId(k);
//        pPoint->setEstimate(points[k]);
//        pPoint->setMarginalized(true);
//        optimizer.addVertex(pPoint);
//    }

// Set edges for graphs
//    int index = 1;
//    for (int p = 0; p < images.size(); ++p) {
//        for (int l = 0; l < points.size(); ++l) {
//            EdgeDirectProjection* directEdge = new EdgeDirectProjection(color[l], images[p]);
//            directEdge->setVertex(0, dynamic_cast<VertexSophus*>(optimizer.vertex(p)));
//            directEdge->setVertex(1, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(l)));
//            // Todo 维度要注意
//            directEdge->setInformation(Eigen::Matrix<double,16,16>::Identity());
//            directEdge->setId(index++);
//            if(ROBUST)
//            {
//                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                rk->setDelta(1.0); // huber 参数
//                directEdge->setRobustKernel(rk);
//            }
//
//            optimizer.addEdge(directEdge);
//        }
//    }