/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "min_box.h"

namespace apollo {
namespace perception {

using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using namespace std;
using namespace Eigen;

const float EPSILON = 1e-6;

bool MinBoxObjectBuilder::Build(const ObjectBuilderOptions& options,
                                std::vector<std::shared_ptr<Object>>* objects) {
  if (objects == nullptr) {
    return false;
  }

  for (size_t i = 0; i < objects->size(); ++i) {
    if ((*objects)[i]) {
      (*objects)[i]->id = i;
      BuildObject(options, (*objects)[i]);
    }
  }

  return true;
}

double MinBoxObjectBuilder::ComputeAreaAlongOneEdge(
    std::shared_ptr<Object> obj, size_t first_in_point, Eigen::Vector3d* center,
    double* lenth, double* width, Eigen::Vector3d* dir) {
  std::vector<Eigen::Vector3d> ns;
  Eigen::Vector3d v(0.0, 0.0, 0.0);
  Eigen::Vector3d vn(0.0, 0.0, 0.0);
  Eigen::Vector3d n(0.0, 0.0, 0.0);
  double len = 0;
  double wid = 0;
  size_t index = (first_in_point + 1) % obj->polygon.points.size();
  for (size_t i = 0; i < obj->polygon.points.size(); ++i) {
    if (i != first_in_point && i != index) {
      // compute v
      Eigen::Vector3d o(0.0, 0.0, 0.0);
      Eigen::Vector3d a(0.0, 0.0, 0.0);
      Eigen::Vector3d b(0.0, 0.0, 0.0);
      o[0] = obj->polygon.points[i].x;
      o[1] = obj->polygon.points[i].y;
      o[2] = 0;
      b[0] = obj->polygon.points[first_in_point].x;
      b[1] = obj->polygon.points[first_in_point].y;
      b[2] = 0;
      a[0] = obj->polygon.points[index].x;
      a[1] = obj->polygon.points[index].y;
      a[2] = 0;
      double k =
          ((a[0] - o[0]) * (b[0] - a[0]) + (a[1] - o[1]) * (b[1] - a[1]));
      k = k / ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
      k = k * -1;
      // n is pedal of src
      n[0] = (b[0] - a[0]) * k + a[0];
      n[1] = (b[1] - a[1]) * k + a[1];
      n[2] = 0;
      // compute height from src to line
      Eigen::Vector3d edge1 = o - b;
      Eigen::Vector3d edge2 = a - b;
      // cross product
      double height = fabs(edge1[0] * edge2[1] - edge2[0] * edge1[1]);
      height = height / sqrt(edge2[0] * edge2[0] + edge2[1] * edge2[1]);
      if (height > wid) {
        wid = height;
        v = o;
        vn = n;
      }
    } else {
      n[0] = obj->polygon.points[i].x;
      n[1] = obj->polygon.points[i].y;
      n[2] = 0;
    }
    ns.push_back(n);
  }
  size_t point_num1 = 0;
  size_t point_num2 = 0;
  for (size_t i = 0; i < ns.size() - 1; ++i) {
    Eigen::Vector3d p1 = ns[i];
    for (size_t j = i + 1; j < ns.size(); ++j) {
      Eigen::Vector3d p2 = ns[j];
      double dist = sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                         (p1[1] - p2[1]) * (p1[1] - p2[1]));
      if (dist > len) {
        len = dist;
        point_num1 = i;
        point_num2 = j;
      }
    }
  }
  Eigen::Vector3d vp1 = v + ns[point_num1] - vn;
  Eigen::Vector3d vp2 = v + ns[point_num2] - vn;
  (*center) = (vp1 + vp2 + ns[point_num1] + ns[point_num2]) / 4;
  (*center)[2] = obj->polygon.points[0].z;
  obj->vertex1 = vp1;
  obj->vertex2 = vp2;
  obj->vertex3 = ns[point_num1];
  obj->vertex4 = ns[point_num2];
  if (len > wid) {
    *dir = ns[point_num2] - ns[point_num1];
  } else {
    *dir = vp1 - ns[point_num1];
  }
  *lenth = len > wid ? len : wid;
  *width = len > wid ? wid : len;
  return (*lenth) * (*width);
}

void MinBoxObjectBuilder::ReconstructPolygon(const Eigen::Vector3d& ref_ct,
                                             std::shared_ptr<Object> obj) {
  if (obj->polygon.points.size() <= 0) {
    return;
  }
  size_t max_point_index = 0;
  size_t min_point_index = 0;
  Eigen::Vector3d p;
  p[0] = obj->polygon.points[0].x;
  p[1] = obj->polygon.points[0].y;
  p[2] = obj->polygon.points[0].z;
  Eigen::Vector3d max_point = p - ref_ct;//ref_ct初始化为０，也就是坐标原点，即lidar中心位置
  Eigen::Vector3d min_point = p - ref_ct;
  for (size_t i = 1; i < obj->polygon.points.size(); ++i) {
    Eigen::Vector3d p;
    p[0] = obj->polygon.points[i].x;
    p[1] = obj->polygon.points[i].y;
    p[2] = obj->polygon.points[i].z;
    Eigen::Vector3d ray = p - ref_ct;
    // clock direction           顺时针
    if (max_point[0] * ray[1] - ray[0] * max_point[1] < EPSILON) {//原点与序号x的点向量依次与序号x+1到原点的向量做数量积
      max_point = ray;                                            //找到最右（下）边的点
      max_point_index = i;
    }
    // unclock direction
    if (min_point[0] * ray[1] - ray[0] * min_point[1] > EPSILON) {
      min_point = ray;
      min_point_index = i;                                 //找到最左（上）边的点
    }
  }
  //以下代码为筛选有效边长,如果相邻的两个点在line的后面,则因为遮挡原因,视这条边为无效边
  Eigen::Vector3d line = max_point - min_point;        //以最坐边和最右边的两个点画一条线
  double total_len = 0;
  double max_dis = 0;
  bool has_out = false;
  //每个顶点与最小点划线
  for (size_t i = min_point_index, count = 0;
       count < obj->polygon.points.size();
       i = (i + 1) % obj->polygon.points.size(), ++count) {
    Eigen::Vector3d p_x;
    p_x[0] = obj->polygon.points[i].x;
    p_x[1] = obj->polygon.points[i].y;
    p_x[2] = obj->polygon.points[i].z;
    size_t j = (i + 1) % obj->polygon.points.size();
    if (j != min_point_index && j != max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[j].x;
      p[1] = obj->polygon.points[j].y;
      p[2] = obj->polygon.points[j].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < EPSILON) {    //j点在line的靠近雷达一侧
        double dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                           (p[1] - p_x[1]) * (p[1] - p_x[1]));
        total_len += dist;
        if (dist - max_dis > EPSILON) {
          max_dis = dist;
        }
      } else {
        // outline
        has_out = true;
      }
    } else if ((i == min_point_index && j == max_point_index) ||
               (i == max_point_index && j == min_point_index)) {
      size_t k = (j + 1) % obj->polygon.points.size();
      Eigen::Vector3d p_k;
      p_k[0] = obj->polygon.points[k].x;
      p_k[1] = obj->polygon.points[k].y;
      p_k[2] = obj->polygon.points[k].z;
      Eigen::Vector3d p_j;
      p_j[0] = obj->polygon.points[j].x;
      p_j[1] = obj->polygon.points[j].y;
      p_j[2] = obj->polygon.points[j].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < 0) {
      } else {
        // outline
        has_out = true;
      }
    } else if (j == min_point_index || j == max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[j].x;
      p[1] = obj->polygon.points[j].y;
      p[2] = obj->polygon.points[j].z;
      Eigen::Vector3d ray = p_x - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < EPSILON) {
        double dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                           (p[1] - p_x[1]) * (p[1] - p_x[1]));
        total_len += dist;
        if (dist > max_dis) {
          max_dis = dist;
        }
      } else {
        // outline
        has_out = true;
      }
    }
  }
  //截止这里,有效边筛选结束
  size_t count = 0;
  double min_area = std::numeric_limits<double>::max();
  for (size_t i = min_point_index; count < obj->polygon.points.size();
       i = (i + 1) % obj->polygon.points.size(), ++count) {
    Eigen::Vector3d p_x;
    p_x[0] = obj->polygon.points[i].x;
    p_x[1] = obj->polygon.points[i].y;
    p_x[2] = obj->polygon.points[i].z;
    size_t j = (i + 1) % obj->polygon.points.size();
    Eigen::Vector3d p_j;
    p_j[0] = obj->polygon.points[j].x;
    p_j[1] = obj->polygon.points[j].y;
    p_j[2] = obj->polygon.points[j].z;
    double dist = sqrt((p_x[0] - p_j[0]) * (p_x[0] - p_j[0]) +
                       (p_x[1] - p_j[1]) * (p_x[1] - p_j[1]));
    if (dist < max_dis && (dist / total_len) < 0.5) {
      continue;
    }
    if (j != min_point_index && j != max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[j].x;
      p[1] = obj->polygon.points[j].y;
      p[2] = obj->polygon.points[j].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < 0) {
        Eigen::Vector3d center;
        double length = 0;
        double width = 0;
        Eigen::Vector3d dir;
        double area =
            ComputeAreaAlongOneEdge(obj, i, &center, &length, &width, &dir);
        if (area < min_area) {
          obj->center = center;
          obj->length = length;
          obj->width = width;
          obj->direction = dir;
          min_area = area;
        }
      } else {
        // outline
      }
    } else if ((i == min_point_index && j == max_point_index) ||
               (i == max_point_index && j == min_point_index)) {
      if (!has_out) {
        continue;
      }
      Eigen::Vector3d center;
      double length = 0;
      double width = 0;
      Eigen::Vector3d dir;
      double area =
          ComputeAreaAlongOneEdge(obj, i, &center, &length, &width, &dir);
      if (area < min_area) {
        obj->center = center;
        obj->length = length;
        obj->width = width;
        obj->direction = dir;
        min_area = area;
      }
    } else if (j == min_point_index || j == max_point_index) {
      Eigen::Vector3d p;
      p[0] = obj->polygon.points[i].x;
      p[1] = obj->polygon.points[i].y;
      p[2] = obj->polygon.points[i].z;
      Eigen::Vector3d ray = p - min_point;
      if (line[0] * ray[1] - ray[0] * line[1] < 0) {
        Eigen::Vector3d center;
        double length = 0.0;
        double width = 0.0;
        Eigen::Vector3d dir;
        double area =
            ComputeAreaAlongOneEdge(obj, i, &center, &length, &width, &dir);
        if (area < min_area) {
          obj->center = center;
          obj->length = length;
          obj->width = width;
          obj->direction = dir;
          min_area = area;
        }
      } else {
        // outline
      }
    }
  }
  obj->direction.normalize();
}

void MinBoxObjectBuilder::ComputePolygon2dxy(std::shared_ptr<Object> obj) {
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl_util::PointCloudPtr cloud = obj->cloud;//存放目标点云
  SetDefaultValue(cloud, obj, &min_pt, &max_pt);//设置默认值
  if (cloud->points.size() < 4u) {                   //点云个数小于4个时,返回
    return;
  }
  GetCloudMinMax3D<pcl_util::Point>(cloud, &min_pt, &max_pt);//为啥又get一遍?zz吧 SetDefaultValue
  obj->height = static_cast<double>(max_pt[2]) - static_cast<double>(min_pt[2]);//SetDefaultValue这个里面不是算过啦?又强制float转换成了double
  // 以下这段代码并没有什么用，不知道为什么加上
  // const double min_eps = 10 * std::numeric_limits<double>::epsilon();//10*最小非零浮点数?是在干吗?

  // const double diff_x = cloud->points[1].x - cloud->points[0].x;
  // const double diff_y = cloud->points[1].y - cloud->points[0].y;
  // size_t idx = 0;
  // for (idx = 2; idx < cloud->points.size(); ++idx) {
  //   const double tdiff_x = cloud->points[idx].x - cloud->points[0].x;
  //   const double tdiff_y = cloud->points[idx].y - cloud->points[0].y;
  //   if ((diff_x * tdiff_y - tdiff_x * diff_y) > min_eps) {    //两个向量的X乘,向量10Xx0,如果点x在10的右边,则<0结束
  //     break;
  //   }
  // }
  // if (idx >= cloud->points.size()) {
  //   cloud->points[0].x += min_eps;
  //   cloud->points[0].y += min_eps;
  //   cloud->points[1].x -= min_eps;
  // }

  obj->min_height = min_pt[2];
  obj->max_height = max_pt[2];   //hrn 19.11.20

  PointCloudPtr pcd_xy(new PointCloud);     //将点云投影到地平面上,z坐标全部换成目标障碍物的最小z坐标
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl_util::Point p = cloud->points[i];
    p.z = min_pt[2];
    pcd_xy->push_back(p);
  }


  ConvexHull2DXY<pcl_util::Point> hull;    //自定义的求凸包的类
  hull.setInputCloud(pcd_xy);
  hull.setDimension(2);
  std::vector<pcl::Vertices> poly_vt;
  PointCloudPtr plane_hull(new PointCloud);
  hull.Reconstruct2dxy (plane_hull, &poly_vt);   //凸包点云存放在plane_hull中,poly_vt中的Vertices存放一组点的索引，索引是plane_hull中的点对应的索引

  // pcl::ConvexHull<pcl_util::Point> hull;                  
  // hull.setInputCloud(pcd_xy);                   
  // hull.setDimension(2);  // 设置凸包维度
  // hull.reconstruct(*plane_hull, poly_vt);


  if (poly_vt.size() == 1u) {                           //
    std::vector<int> ind(poly_vt[0].vertices.begin(),    //将poly_vt[0].vertices数组中的数据全部复制到ind中
                         poly_vt[0].vertices.end());
    TransformPointCloud(plane_hull, ind, &obj->polygon);
  } else {
    obj->polygon.points.resize(4);                               //poly_vt.size()不为1 的情况下,将minmax3d的四个顶点作为凸包
    obj->polygon.points[0].x = static_cast<double>(min_pt[0]);
    obj->polygon.points[0].y = static_cast<double>(min_pt[1]);
    obj->polygon.points[0].z = static_cast<double>(min_pt[2]);

    obj->polygon.points[1].x = static_cast<double>(min_pt[0]);
    obj->polygon.points[1].y = static_cast<double>(max_pt[1]);
    obj->polygon.points[1].z = static_cast<double>(min_pt[2]);

    obj->polygon.points[2].x = static_cast<double>(max_pt[0]);
    obj->polygon.points[2].y = static_cast<double>(max_pt[1]);
    obj->polygon.points[2].z = static_cast<double>(min_pt[2]);

    obj->polygon.points[3].x = static_cast<double>(max_pt[0]);
    obj->polygon.points[3].y = static_cast<double>(min_pt[1]);
    obj->polygon.points[3].z = static_cast<double>(min_pt[2]);
  }
}

void MinBoxObjectBuilder::ComputeGeometricFeature(const Eigen::Vector3d& ref_ct,
                                                  std::shared_ptr<Object> obj) {
  ros::Time begin111 = ros::Time::now();
  // step 1: compute 2D xy plane's polygen
  ComputePolygon2dxy(obj);
  // PointCloudBoundary2(obj);
  // step 2: construct box
  // ReconstructPolygon(ref_ct, obj);
  Mybuild(ref_ct, obj);
  
}

void MinBoxObjectBuilder::BuildObject(ObjectBuilderOptions options,
                                      std::shared_ptr<Object> object) {
  ComputeGeometricFeature(options.ref_center, object);
}

void MinBoxObjectBuilder::TransformPointCloud(pcl_util::PointCloudPtr cloud,
                         const std::vector<int>& indices,
                         pcl_util::PointDCloud* trans_cloud) {
  if (trans_cloud->size() != indices.size()) {
    trans_cloud->resize(indices.size());
  }
  for (size_t i = 0; i < indices.size(); ++i) {
    const pcl_util::Point& p = cloud->at(indices[i]);
    Eigen::Vector3d v(p.x, p.y, p.z);
    pcl_util::PointD& tp = trans_cloud->at(i);
    tp.x = v.x();
    tp.y = v.y();
    tp.z = v.z();
    //tp.intensity = p.intensity;
  }
}

void MinBoxObjectBuilder::Mybuild(const Eigen::Vector3d& ref_ct,std::shared_ptr<Object> obj)
{
  ros::Time begin_time = ros::Time::now ();
  pcl_util::PointCloudPtr cloud = obj->cloud;//存放目标点云
  if (obj->polygon.points.size() <= 0) {
    return;
  }
  int max_num = 0;
  size_t m,n;
  Eigen::Vector3d derectl,derectw;
  double max_dis = 0;
  PointCloudBoundary2(obj);

//L型拟合
//找到距离最远的两点
// for (size_t i = 0; i < cloud->points.size(); i++)
//   {
//     Eigen::Vector3d p;
//     p[0] = cloud->points[i].x;
//     p[1] = cloud->points[i].y;
//     p[2] = 0;
//     for (size_t j = i+1; j < cloud->points.size(); j++)
//     {
//       if(j>=cloud->points.size())
//       break;
//       Eigen::Vector3d q;
//       q[0] = cloud->points[j].x;
//       q[1] = cloud->points[j].y;
//       q[2] = 0;
//       if(p[0]==q[0] && p[1]==q[1])
//       continue;
//       Eigen::Vector3d temp;
//       temp = q-p;
//       double distan = sqrt(temp[0]*temp[0]+temp[1]*temp[1]);
//       if(distan > max_dis)
//       {
//         max_dis = distan;
//         m = i;
//         n = j;
//       }
//     }
//   }

// //遍历所有点与最远两点连线距离,找到最远的即为角点
// double max_dis_line = 0;
// size_t l;
// Eigen::Vector3d p,q;
// if(max_dis >0)
// {
//   p[0] = cloud->points[m].x;
//   p[1] = cloud->points[m].y;
//   p[2] = cloud->points[m].z;
//   q[0] = cloud->points[n].x;
//   q[1] = cloud->points[n].y;
//   q[2] = cloud->points[n].z;
  
//   for (size_t k = 0; k < cloud->points.size(); k++)
//   {
//     double A = q[1]-p[1];
//     double B = p[0]-q[0];
//     double C = q[0]*p[1]-q[1]*p[0];
//     double x1 = cloud->points[k].x;
//     double y1 = cloud->points[k].y;
//     double distance = fabs(A*x1+B*y1+C)/sqrt((A*A)+(B*B));
//     std::cout<<"distance: "<< distance <<std::endl;

//     if(distance > max_dis_line)
//     {
//       max_dis_line = distance;
//       l = k;
//       // std::cout<<"max_dis_line: "<< max_dis_line <<std::endl;
//       // std::cout<<"l: "<< l <<std::endl;
//     }
      
//   }
// }
// else
// {
//   std::cout<<"  no two point!!  " <<std::endl;
// }
  
// if(max_dis_line >0)
// {
//   Eigen::Vector3d o;
//   o[0] = cloud->points[l].x;
//   o[1] = cloud->points[l].y;
//   o[2] = cloud->points[l].z;
//   std::cout<<"   m: "<< m <<std::endl;
//   std::cout<<"   n: "<< n <<std::endl;
//   std::cout<<"   l: "<< l <<std::endl;


//   derectl = sqrt((o[0]-p[0])*(o[0]-p[0])+(o[1]-p[1])*(o[1]-p[1])) > sqrt((o[0]-q[0])*(o[0]-q[0])+(o[1]-q[1])*(o[1]-q[1])) ? (o-p):(o-q);
//   obj->line1 = p;
//   obj->line2 = q;
//   obj->line_point = o;
//   derectw[0] = derectl[1];
//   derectw[1] = -derectl[0];
//   derectw[2] = derectl[2];
// }
// else
// {
//   std::cout<<"  no point to line!!  " <<std::endl;
// }
//L型拟合到此结束

// //PCA
// 计算点云质心
    // Eigen::Vector4d centroid;
    // pcl::compute3DCentroid(*cloud, centroid);

    // // 构建点云矩阵
    // MatrixXd points(cloud->size(), 3);
    // for (size_t i = 0; i < cloud->size(); ++i) {
    //     points.row(i) << cloud->points[i].x - centroid[0], cloud->points[i].y - centroid[1], cloud->points[i].z - centroid[2];
    // }

    // // 计算协方差矩阵
    // Matrix3d covariance_matrix;
    // covariance_matrix = (points.transpose() * points) / float(cloud->size());

    // // 特征值分解
    // SelfAdjointEigenSolver<Matrix3d> eigen_solver(covariance_matrix);
    // Vector3d eigen_values = eigen_solver.eigenvalues();
    // Matrix3d eigen_vectors = eigen_solver.eigenvectors();

    // // 提取最大特征值对应的特征向量
    // Vector3d pca_direction = eigen_vectors.col(2);
    // derectl = pca_direction;
    // derectw[0] = derectl[1];
    // derectw[1] = -derectl[0];
    // derectw[2] = derectl[2];
    // std::cout<<"   derectw[0]: "<< derectw[0] <<std::endl;
//     //PCA结束
      

//我的边框算法
  //遍历所有存储顶点，两两连线，找出方向
  for (size_t i = 0; i < obj->polygon.points.size(); i++)
  {
    Eigen::Vector3d p;
    p[0] = obj->polygon.points[i].x;
    p[1] = obj->polygon.points[i].y;
    p[2] = obj->polygon.points[i].z;
    for (size_t j = i+1; j < obj->polygon.points.size(); j++)
    {
      if(obj->polygon.points[i].x==obj->polygon.points[j].x && obj->polygon.points[i].y==obj->polygon.points[j].y)
      continue;
      if(j>=obj->polygon.points.size())
      break;
      int num_b = 0;
      int num_f = 0;
      Eigen::Vector3d q;
      q[0] = obj->polygon.points[j].x;
      q[1] = obj->polygon.points[j].y;
      q[2] = obj->polygon.points[j].z;
      //遍历所有点与直线距离
      for (size_t k = 0; k < obj->cloud_boundary->points.size(); k++)
      {
        double A = q[1]-p[1];
        double B = p[0]-q[0];
        double C = q[0]*p[1]-q[1]*p[0];
        double x1 = obj->cloud_boundary->points[k].x;
        double y1 = obj->cloud_boundary->points[k].y;
        double distance = (A*x1+B*y1+C)/sqrt((A*A)+(B*B));
        // std::cout<<"distance: "<< distance <<std::endl;
        //直线一侧靠近的点越多，distance越大
        if(distance<0.5 && distance>0)
        num_b++;
        if(distance>-0.5 && distance<0)
        num_f++;
      }
      // std::cout<<"  num_b: "<< num_b <<std::endl;
      // std::cout<<"  num_f: "<< num_f <<std::endl;
      // std::cout<<"  max_num: "<< max_num <<std::endl;
      if(num_b>max_num||num_f>max_num)
      {
        
        // std::cout<<"i: "<< i <<std::endl;
        // std::cout<<"j: "<< j <<std::endl;
        m=i;
        n=j;
        max_num = abs(num_b) > abs(num_f) ? num_b : num_f;
        // std::cout<<"max_num: "<< max_num <<std::endl;
        derectl = q-p;
        derectw[0] = derectl[1];
        derectw[1] = -derectl[0];
        derectw[2] = derectl[2];
      }
      
    }
  }

  //AABBbox
  // derectl[0] = 2; 
  // derectl[1] = 1; 
  // derectl[2] = 0; 
  // derectw[0] = derectl[1];
  // derectw[1] = -derectl[0];
  // derectw[2] = derectl[2];
  



  double min_len;
  double max_len;
  double min_wid;
  double max_wid;

   //遍历所有点找出最小框
  for (size_t k = 0; k < cloud->points.size(); k++)
  {
    double x = cloud->points[k].x;
    double y = cloud->points[k].y;
    //点乘算出点在该方向上的长度|a||b|cosm/|a|
    double len = (derectl[0]*x+derectl[1]*y)/sqrt(derectl[0]*derectl[0]+derectl[1]*derectl[1]);
    double wid = (derectw[0]*x+derectw[1]*y)/sqrt(derectw[0]*derectw[0]+derectw[1]*derectw[1]);
    if(k==0)
    {
      min_len = len;
      max_len = len;
      min_wid = wid;
      max_wid = wid;
      // obj->center = cloud->points[k];
    }
    if(len<min_len)
    {
      min_len = len;
    }
    else if(len>max_len)
    {
      max_len = len;
    }
    if(wid<min_wid)
    {
      min_wid = wid;
    }
    else if(wid>max_wid)
    {
      max_wid = wid;
    }
  }
  
  // if(max_num!=0)
  // if(max_dis_line > 0)
  if(1)
  {
    // std::cout<<"max_dis_line: "<< max_dis_line <<std::endl;
    //计算顶点
    Eigen::Vector3d max_max, max_min, min_max, min_min;
    max_max[0] = max_wid;
    max_max[1] = max_len;

    max_min[0] = max_wid;
    max_min[1] = min_len;

    min_max[0] = min_wid;
    min_max[1] = max_len;

    min_min[0] = min_wid;
    min_min[1] = min_len;

    max_max[2] = max_min[2] = min_max[2] = min_min[2] = 0;
    box2xyz(max_max, derectl);
    box2xyz(max_min, derectl);
    box2xyz(min_max, derectl);
    box2xyz(min_min, derectl);
    obj->vertex1 = min_min;
    obj->vertex2 = min_max;
    obj->vertex3 = max_min;
    obj->vertex4 = max_max;
    obj->polygon.points.resize(4);                               //poly_vt.size()不为1 的情况下,将minmax3d的四个顶点作为凸包
    obj->polygon.points[0].x = static_cast<double>(min_min[0]);
    obj->polygon.points[0].y = static_cast<double>(min_min[1]);
    obj->polygon.points[0].z = static_cast<double>(min_min[2]);

    obj->polygon.points[1].x = static_cast<double>(min_max[0]);
    obj->polygon.points[1].y = static_cast<double>(min_max[1]);
    obj->polygon.points[1].z = static_cast<double>(min_max[2]);

    obj->polygon.points[2].x = static_cast<double>(max_min[0]);
    obj->polygon.points[2].y = static_cast<double>(max_min[1]);
    obj->polygon.points[2].z = static_cast<double>(max_min[2]);

    obj->polygon.points[3].x = static_cast<double>(max_max[0]);
    obj->polygon.points[3].y = static_cast<double>(max_max[1]);
    obj->polygon.points[3].z = static_cast<double>(max_max[2]);
    obj->center =(min_min+min_max+max_min+max_max)/4; 
    
    obj->length = max_len-min_len;
    obj->width = max_wid-min_wid;
    obj->direction = derectl;
    obj->direction.normalize();
    ros::Time end_time = ros::Time::now ();
    double time = (end_time - begin_time).toSec();
    // std::cout<<"BOX_time: "<< time <<std::endl;
  }
}

//point为转换前坐标，derection为转换前y轴在转换后坐标轴上的方向
void MinBoxObjectBuilder::box2xyz(Eigen::Vector3d &point, Eigen::Vector3d derection)
{
  //-pi1~pi1
  double a = atan(derection[1]/derection[0]);
  double b = atan(point[1]/point[0]);
  if(derection[0]<0 && derection[1]<0)
    a = a-pi1;
  if(derection[0]<0 && derection[1]>0)
    a = a+pi1;
  if(point[0]<0 && point[1]<0)
    b = b-pi1;
  if(point[0]<0 && point[1]>0)
    b = b+pi1;
  // std::cout << " a+b " << a+b << std::endl;
  double dist = sqrt(point[1]*point[1]+point[0]*point[0]);

  point[0] = dist*cos(a+b-pi1/2);
  point[1] = dist*sin(a+b-pi1/2);

}

void MinBoxObjectBuilder::PointCloudBoundary2(std::shared_ptr<Object> obj)
{
  ros::Time begin_time = ros::Time::now ();
 
	// 1 计算法向量
  pcl_util::PointCloudPtr cloud = obj->cloud;//存放目标点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < obj->cloud->points.size(); ++i) {
    pcl::PointXYZI p;
    p.x = cloud->points[i].x;
    p.y = cloud->points[i].y;
    p.z = 0;
    cloud_in->push_back(p);
  }
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	PointCloudPtr normals(new  PointCloud);
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> normalEstimation;
	normalEstimation.setInputCloud(cloud_in);
	normalEstimation.setSearchMethod(tree);
	normalEstimation.setRadiusSearch(0.15);  // 法向量的半径
	normalEstimation.compute(*normals);
 
	/*pcl计算边界*/
	pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //声明一个boundary类指针，作为返回值
	boundaries->resize(cloud_in->size()); //初始化大小
	pcl::BoundaryEstimation<pcl::PointXYZI, pcl::PointXYZINormal, pcl::Boundary> boundary_estimation; //声明一个BoundaryEstimation类
	boundary_estimation.setInputCloud(cloud_in); //设置输入点云
	boundary_estimation.setInputNormals(normals); //设置输入法线
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZI>);
	boundary_estimation.setSearchMethod(kdtree_ptr); //设置搜寻k近邻的方式
	boundary_estimation.setKSearch(30); //设置k近邻数量
	boundary_estimation.setAngleThreshold(pi1 * 0.4); //设置角度阈值，大于阈值为边界
	boundary_estimation.compute(*boundaries); //计算点云边界，结果保存在boundaries中
	
	// cout << "边界点云的点数   :  "<< boundaries->size()<< endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZI>);
	cloud_visual->resize(cloud_in->size());
	for (size_t i = 0; i < cloud_in->size(); i++)
	{
    // cout << "  加i =  "<< int(boundaries->points[i].boundary_point)<< endl;
		cloud_visual->points[i].x = cloud_in->points[i].x;
		cloud_visual->points[i].y = cloud_in->points[i].y;
		cloud_visual->points[i].z = cloud_in->points[i].z;
		if (boundaries->points[i].boundary_point != 0)
		{
      // cout << "  加入 "<< endl;
			cloud_boundary->push_back(cloud_visual->points[i]);
		}
	}
  //提取的边界
  obj->cloud_visual = cloud_visual;
  obj->cloud_boundary = cloud_boundary;
  ros::Time end_time = ros::Time::now ();
    double time = (end_time - begin_time).toSec();
    // std::cout<<"boundary_time: "<< time <<std::endl;

}

// 计算点云的AABB包围盒
void MinBoxObjectBuilder::computeAABB(std::shared_ptr<Object> obj, Vector3d& minPoint, Vector3d& maxPoint) {
  pcl_util::PointCloudPtr cloud = obj->cloud;//存放目标点云
    if (cloud->empty()) {
        ROS_ERROR("Empty point cloud!");
        return;
    }

    minPoint = Vector3d(cloud->points[0].x, cloud->points[0].y, cloud->points[0].z);
    maxPoint = Vector3d(cloud->points[0].x, cloud->points[0].y, cloud->points[0].z);
    for (const auto& point : cloud->points) {
        minPoint = minPoint.array().min(Vector3d(point.x, point.y, point.z).array());
        maxPoint = maxPoint.array().max(Vector3d(point.x, point.y, point.z).array());
    }
}

void MinBoxObjectBuilder::computeAABB(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, Vector3d& minPoint, Vector3d& maxPoint) {
    if (cloud->empty()) {
        ROS_ERROR("Empty point cloud!");
        return;
    }

    minPoint = Vector3d(cloud->points[0].x, cloud->points[0].y, cloud->points[0].z);
    maxPoint = Vector3d(cloud->points[0].x, cloud->points[0].y, cloud->points[0].z);
    for (const auto& point : cloud->points) {
        minPoint = minPoint.array().min(Vector3d(point.x, point.y, point.z).array());
        maxPoint = maxPoint.array().max(Vector3d(point.x, point.y, point.z).array());
    }
}

// 计算点云的OBB包围盒
void MinBoxObjectBuilder::computeOBB(std::shared_ptr<Object> obj, Matrix3d& rotation, Vector3d& translation, Vector3d& extents) {
  pcl_util::PointCloudPtr cloud = obj->cloud;//存放目标点云
    if (cloud->empty()) {
        ROS_ERROR("Empty point cloud!");
        return;
    }

    // Convert pcl point cloud to Eigen point cloud
    Eigen::MatrixXd points(cloud->size(), 3);
    for (size_t i = 0; i < cloud->size(); ++i) {
        points.row(i) << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
    }

    // Compute covariance matrix
    Eigen::MatrixXd centered = points.rowwise() - points.colwise().mean();
    Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(cloud->size());

    // Perform SVD decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeFullU);
    Eigen::MatrixXd U = svd.matrixU();
    
    // Compute rotation matrix
    rotation = U;

    // Compute translation (centroid)
    translation = points.colwise().mean();

    // Compute extents (half-dimensions)
    Eigen::VectorXd singularValues = svd.singularValues();
    extents = singularValues.head(3) / 2.0;
}

// 计算点云的L型包围盒
void MinBoxObjectBuilder::computeLShape(std::shared_ptr<Object> obj, Matrix3d& rotation, Vector3d& translation, Vector3d& extents) {
  pcl_util::PointCloudPtr cloud_in = obj->cloud;//存放目标点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < obj->cloud->points.size(); ++i) {
    pcl::PointXYZI p;
    p.x = cloud_in->points[i].x;
    p.y = cloud_in->points[i].y;
    p.z = 0;
    cloud->push_back(p);
  }
    if (cloud->empty()) {
        ROS_ERROR("Empty point cloud!");
        return;
    }

    // Step 1: 计算主要方向
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();
    eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));

    // Step 2: 投影点云到主要平面
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenvectors.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.0f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudProjected(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud, *cloudProjected, projectionTransform);

    // Step 3: 计算投影点云的AABB
    Vector3d minPoint, maxPoint;
    computeAABB(cloudProjected, minPoint, maxPoint);

    // Step 4: 计算L型包围盒的旋转和平移
    rotation = eigenvectors.cast<double>();
    translation = pcaCentroid.head<3>().cast<double>();

    // Step 5: 计算L型包围盒的长度和宽度（在主平面上）
    extents[0] = maxPoint[0] - minPoint[0]; // 长度
    extents[1] = maxPoint[1] - minPoint[1]; // 宽度

    // Step 6: 计算L型包围盒的高度（点云的Z范围）
    extents[2] = maxPoint[2] - minPoint[2]; // 高度
}

// 计算8个端点和中心点
void MinBoxObjectBuilder::computePoints(const Vector3d& translation, const Matrix3d& rotation, const Vector3d& extents, std::shared_ptr<Object> obj) {
    Vector3d corners[8] = {
        translation + rotation * Vector3d(extents.x(), extents.y(), extents.z()),
        translation + rotation * Vector3d(-extents.x(), extents.y(), extents.z()),
        translation + rotation * Vector3d(-extents.x(), -extents.y(), extents.z()),
        translation + rotation * Vector3d(extents.x(), -extents.y(), extents.z()),
        translation + rotation * Vector3d(extents.x(), extents.y(), -extents.z()),
        translation + rotation * Vector3d(-extents.x(), extents.y(), -extents.z()),
        translation + rotation * Vector3d(-extents.x(), -extents.y(), -extents.z()),
        translation + rotation * Vector3d(extents.x(), -extents.y(), -extents.z())
    };
    obj->vertex1 = corners[0];
    obj->vertex2 = corners[1];
    obj->vertex3 = corners[2];
    obj->vertex4 = corners[3];
    obj->polygon.points.resize(4);                               //poly_vt.size()不为1 的情况下,将minmax3d的四个顶点作为凸包
    obj->polygon.points[0].x = static_cast<double>(obj->vertex1[0]);
    obj->polygon.points[0].y = static_cast<double>(obj->vertex1[1]);
    obj->polygon.points[0].z = static_cast<double>(obj->vertex1[2]);

    obj->polygon.points[1].x = static_cast<double>(obj->vertex2[0]);
    obj->polygon.points[1].y = static_cast<double>(obj->vertex2[1]);
    obj->polygon.points[1].z = static_cast<double>(obj->vertex2[2]);

    obj->polygon.points[2].x = static_cast<double>(obj->vertex3[0]);
    obj->polygon.points[2].y = static_cast<double>(obj->vertex3[1]);
    obj->polygon.points[2].z = static_cast<double>(obj->vertex3[2]);

    obj->polygon.points[3].x = static_cast<double>(obj->vertex4[0]);
    obj->polygon.points[3].y = static_cast<double>(obj->vertex4[1]);
    obj->polygon.points[3].z = static_cast<double>(obj->vertex4[2]);
    obj->center =(corners[1]+corners[2]+corners[3]+corners[4])/4; 
    
    obj->length = extents.x()>extents.y()?(2*extents.x()):(2*extents.y());
    obj->width = extents.x()<extents.y()?(2*extents.x()):(2*extents.y());
    obj->direction = extents.x()>extents.y()?(corners[0]-corners[1]):(corners[1]-corners[2]);
    obj->direction.normalize();

}



}  // namespace perception
}  // namespace apollo
