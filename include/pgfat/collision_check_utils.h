#pragma once

#include <vector>
#include <Eigen/Dense>
#include <iostream>

#include "pgfat/geometrics.h"
#include "pgfat/data_structure.h"

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/BV/BV.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/shape/geometric_shapes.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef fcl::OBBRSS BV;
typedef fcl::BVHModel<BV> BVHM;
typedef std::shared_ptr<BVHM> BVHMPtr;

using fcl::Box;
typedef std::shared_ptr<fcl::Box> BoxPtr;

using fcl::CollisionObject;
typedef std::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudT;
typedef std::shared_ptr<PointT> PointTPtr;


struct FCLGripper
{
  BoxPtr g[3];  // Simplified block gripper
  Eigen::Isometry3d t[3];  // placement for every block to form a gripper

  // h: height of the gripper
  // w: width of the gripper
  // l: length of the gripper
  // h_f: height of the gripper finger
  // w_f: width of the gripper finger
  // l_f: length of the gripper finger
  // d_f: distance between these two fingers along l at beginning
  double h, w, l, h_f, w_f, l_f, d_f;

  void setParams(double init_h,
    double init_w, double init_l,
    double init_h_f, double init_w_f,
    double init_l_f, double init_d_f) {
    h = init_h;
    w = init_w;
    l = init_l;
    h_f = init_h_f;
    w_f = init_w_f;
    l_f = init_l_f;
    d_f = init_d_f;

    g[0] = std::make_shared<Box>(l,h,w);
    g[1] = std::make_shared<Box>(l_f,h_f,w_f);
    g[2] = std::make_shared<Box>(l_f,h_f,w_f);

    t[0].linear().setIdentity();
    t[0].translation() << -l_f + 2.5 - l/2, 0, 0;

    t[1].linear().setIdentity();
    t[1].translation() << -l_f/2 + 2.5, 0, d_f + w_f/2;
    
    t[2].linear().setIdentity();
    t[2].translation() << -l_f/2 + 2.5, 0, -d_f - w_f/2;

  }

  void changeWidth(double new_d_f) {
    t[1].linear().setIdentity();
    t[1].translation() << -l_f/2 + 2.5, 0, new_d_f + w_f/2;
    
    t[2].linear().setIdentity();
    t[2].translation() << -l_f/2 + 2.5, 0, -new_d_f - w_f/2;
  }

  
  void drawGripper(
    pcl::visualization::PCLVisualizer & vis, 
    const Eigen::Isometry3d gripper_transform,
    const std::string &id,
    double r, double g_c, double b, double opacity,
    double dist = -1.0) {
    if (dist < 0)
      changeWidth(d_f);
    else
      changeWidth(dist);

    for(int i=0; i<3; i++) {
      auto T = gripper_transform * t[i];         
      Eigen::Vector3d position(T.translation()); 
      Eigen::Quaterniond quat(T.linear());  
      Eigen::Vector3f posf;
      Eigen::Quaternionf quatf;

      posf = position.cast <float> ();
      quatf = quat.cast <float> ();
      std::string id_total = "cube" + id + std::to_string(i);
      vis.addCube(posf, quatf,
                  g[i]->side[0],
                  g[i]->side[1],
                  g[i]->side[2],
                  id_total);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      r, g_c, b, id_total);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                      opacity, id_total, 0);

      std::string id_total_line = "cube_line" + id + std::to_string(i);
      vis.addCube(posf, 
                  quatf,g[i]->side[0],
                  g[i]->side[1],
                  g[i]->side[2],
                  id_total_line);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      0, 0, 0, id_total_line);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                      opacity, id_total_line, 0);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id_total_line, 0);
      
    }
  }

};


class CollisionCheck
{
public:
  BVHMPtr mesh_model_;
  FCLGripper gripper_model_;
   
  void loadMesh(const std::vector <TrianglePlaneData> & mesh) {
    std::vector<fcl::Vec3f > points;
    std::vector<fcl::Triangle> triangles;
    mesh_model_ = std::make_shared<BVHM> ();

    for(const auto & tri_plane: mesh){
      fcl::Triangle tri;
      for(int i=0; i<3; i++){
        tri[i] = points.size();
        points.push_back(fcl::Vec3f(tri_plane.points[i](0), 
                                    tri_plane.points[i](1), 
                                    tri_plane.points[i](2)));
      }
      triangles.push_back(tri);
    }
    mesh_model_->beginModel();
    mesh_model_->addSubModel(points, triangles);
    mesh_model_->endModel();
  }
  

  bool isCollide(Eigen::Isometry3d gripper_transform, double distance) {    
    fcl::CollisionRequest request;      
    fcl::CollisionResult result[4];  

    fcl::Transform3f init;
    init.setIdentity();
    
    gripper_model_.changeWidth(distance);

    for (int i=0; i<3 ;++i) {
      Eigen::Isometry3d cur_transform = gripper_transform * gripper_model_.t[i];
      fcl::Transform3f fcl_transform;
      convertTransform(cur_transform, fcl_transform);
 
      fcl::collide(mesh_model_.get(),
                   init,gripper_model_.g[i].get(),
                   fcl_transform, request, result[i]);
      if (result[i].isCollision() == true){
        return false;
      }
    }
    return true;
  }

  void convertTransform(
    const Eigen::Isometry3d &eigen_input,
    fcl::Transform3f &fcl_output) {
    fcl::Matrix3f rotation;
    fcl::Vec3f translation;

    auto &rot = eigen_input.linear();
    auto &trans = eigen_input.translation();

    rotation.setValue(rot(0,0), rot(0,1), rot(0,2),
                      rot(1,0), rot(1,1), rot(1,2),
                      rot(2,0), rot(2,1), rot(2,2));
    translation.setValue(trans(0), trans(1), trans(2));
    fcl_output.setTransform(rotation,translation);
  }
};











