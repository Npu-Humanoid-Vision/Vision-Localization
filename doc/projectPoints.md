# function learning ***cv::projectPoints***

Projects 3D points to an image plane.

## main code

```cpp
// Ｘ　Ｙ　Ｚ　为空间点
double X = M[i].x, Y = M[i].y, Z = M[i].z;
// x y z 为Pc, Pc =  ＲPw+ｔ
double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];

double r2, r4, r6, a1, a2, a3, cdist, icdist2;
double xd, yd, xd0, yd0, invProj;
Vec3d vecTilt;
Vec3d dVecTilt;
Matx22d dMatTilt;
Vec2d dXdYd;

//　ｚ为零则略去不管, 否则对ｚ归一化处理
z = z ? 1./z : 1;
x *= z; y *= z;

// 畸变处理
r2 = x*x + y*y;
r4 = r2*r2;
r6 = r4*r2;
a1 = 2*x*y;
a2 = r2 + 2*x*x;
a3 = r2 + 2*y*y;
cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
xd0 = x*cdist*icdist2 + k[2]*a1 + k[3]*a2 + k[8]*r2+k[9]*r4;
yd0 = y*cdist*icdist2 + k[2]*a3 + k[3]*a1 + k[10]*r2+k[11]*r4;

// additional distortion by projecting onto a tilt plane
vecTilt = matTilt*Vec3d(xd0, yd0, 1);
invProj = vecTilt(2) ? 1./vecTilt(2) : 1;
xd = invProj * vecTilt(0);
yd = invProj * vecTilt(1);

// 针孔模型相机成像
m[i].x = xd*fx + cx;
m[i].y = yd*fy + cy;
```

自己实现一下（除了畸变处理），发现工作正常，说明成像原理理解基本正确c

```cpp

cv::Mat R_3x3, Rt_4x4(4, 4, CV_32F, cv::Scalar(0));
cv::Rodrigues(R, R_3x3);
R_3x3.convertTo(R_3x3, CV_32F);
t.convertTo(t, CV_32F);
camera_matrix.convertTo(camera_matrix, CV_32F);

for (int i=0; i<3; i++) {
    Rt_4x4.at<float>(i, 0) = R_3x3.at<float>(i, 0);
    Rt_4x4.at<float>(i, 1) = R_3x3.at<float>(i, 1);
    Rt_4x4.at<float>(i, 2) = R_3x3.at<float>(i, 2);
    Rt_4x4.at<float>(i, 3) = t.at<float>(i, 0);
}
Rt_4x4.at<float>(3, 3) = 1.0;

cv::Mat M_matrix(3, 4, CV_32F, cv::Scalar(0));
for (int i=0; i<3; i++) {
    M_matrix.at<float>(i, i) = 1;
}

M_matrix = camera_matrix*M_matrix*Rt_4x4;

// 自己投个影试试   
for (int i=0; i<ori_pnt.size(); i++) {
    cv::Mat t_pnt(4, 1, CV_32F, cv::Scalar(1));    
    t_pnt.at<float>(0, 0) = ori_pnt[i].x;
    t_pnt.at<float>(1, 0) = ori_pnt[i].y;
    t_pnt.at<float>(2, 0) = ori_pnt[i].z;

    cv::Mat t_res =  M_matrix * t_pnt;
    // 这里没对0值处理
    t_res /= t_res.at<float>(2, 0);
    cout<<t_res<<endl;
    cv::Point2f t_pres(t_res.at<float>(0, 0), t_res.at<float>(1, 0));
    reproj_point.push_back(t_pres);
}
```
