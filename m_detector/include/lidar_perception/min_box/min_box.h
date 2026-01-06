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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_OBJECT_BUILDER_MIN_BOX_H
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_OBJECT_BUILDER_MIN_BOX_H

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <pcl/surface/concave_hull.h>
#include "lidar_perception/common/convex_hullxy.h"
#include "lidar_perception/common/pcl_types.h"
#include "lidar_perception/object.h"
#include "lidar_perception/base_object_builder.h"

#include <ros/ros.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>


#define pi1 3.14159265358979323846

namespace apollo {
namespace perception {
using namespace std;
using namespace Eigen;

class MinBoxObjectBuilder : public BaseObjectBuilder {
 public:
  MinBoxObjectBuilder() : BaseObjectBuilder() {}
  virtual ~MinBoxObjectBuilder() {}

  bool Init() override { return true; }

  bool Build(const ObjectBuilderOptions& options,
             std::vector<std::shared_ptr<Object>>* objects) override;
  std::string name() const override { return "MinBoxObjectBuilder"; }

 protected:
  void BuildObject(ObjectBuilderOptions options,
                   std::shared_ptr<Object> object);

  void ComputePolygon2dxy(std::shared_ptr<Object> obj);

  double ComputeAreaAlongOneEdge(std::shared_ptr<Object> obj,
                                 size_t first_in_point, Eigen::Vector3d* center,
                                 double* lenth, double* width,
                                 Eigen::Vector3d* dir);

  void ReconstructPolygon(const Eigen::Vector3d& ref_ct,
                          std::shared_ptr<Object> obj);

  void ComputeGeometricFeature(const Eigen::Vector3d& ref_ct,
                               std::shared_ptr<Object> obj);
  
  void TransformPointCloud(pcl_util::PointCloudPtr cloud,
                         const std::vector<int>& indices,
                         pcl_util::PointDCloud* trans_cloud);
  void Mybuild(const Eigen::Vector3d& ref_ct,std::shared_ptr<Object> obj);
  void box2xyz(Eigen::Vector3d &point, Eigen::Vector3d derection);
  void PointCloudBoundary2(std::shared_ptr<Object> obj);

  void computeAABB(std::shared_ptr<Object> obj, Vector3d& minPoint, Vector3d& maxPoint);
  void computeAABB(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, Vector3d& minPoint, Vector3d& maxPoint);
  void computeOBB(std::shared_ptr<Object> obj, Matrix3d& rotation, Vector3d& translation, Vector3d& extents);
  void computeLShape(std::shared_ptr<Object> obj, Matrix3d& rotation, Vector3d& translation, Vector3d& extents) ;
  void computePoints(const Vector3d& translation, const Matrix3d& rotation, const Vector3d& extents, std::shared_ptr<Object> obj);

};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_OBJECT_BUILDER_MIN_BOX_H
