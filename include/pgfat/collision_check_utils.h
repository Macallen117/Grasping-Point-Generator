#pragma once

#include <vector>
#include <Eigen/Dense>
#include <iostream>

#include "pgfat/geometrics.h"
#include "pgfat/data_structure.h"
#include "pgfat/data_transform.h"

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/collision_data.h>
#include <fcl/BV/BV.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/narrowphase/narrowphase.h>
#include "fcl/distance.h"
#include "fcl/distance_func_matrix.h"
#include <pcl/visualization/pcl_visualizer.h>

typedef fcl::OBBRSS BV;
typedef fcl::BVHModel<BV> BVHM;
typedef std::shared_ptr<BVHM> BVHMPtr;

using fcl::Box;
typedef std::shared_ptr<fcl::Box> BoxPtr;

using fcl::Sphere;
typedef std::shared_ptr<fcl::Sphere> SpherePtr;

using fcl::CollisionObject;
typedef std::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudT;
typedef std::shared_ptr<PointT> PointTPtr;


struct FCLGripper
{
  BoxPtr g[10];  // Simplified block gripper
  Eigen::Isometry3d t[10];  // placement for every block to form a gripper
  
  // add a small region to the contact surface
  SpherePtr small_sph[6];
  BoxPtr small_box[6];
  Eigen::Isometry3d small_t[6];

  // h: height of the gripper
  // w: width of the gripper
  // l: length of the gripper
  
  // h_f: height of the gripper finger
  // w_f: width of the gripper finger
  // l_f: length of the gripper finger
  
  // h_fp: height of the gripper finger pad
  // w_fp: width of the gripper finger pad
  // l_fp: length of the gripper finger pad
  
  // d_f: 2*d_f is the distance between the two fingers along w at beginning
  double h, w, l, h_f, w_f, l_f, h_fp, w_fp, l_fp, w_diff, d_f, box_size; 

 
  void setParams(
    double init_h, double init_w, double init_l,
    double init_h_f, double init_w_f, double init_l_f,
    double init_h_fp, double init_w_fp, double init_l_fp,
    double init_w_diff, double init_d_f, double init_box_size) {
    h = init_h;
    w = init_w;
    l = init_l;
    h_f = init_h_f;
    w_f = init_w_f;
    l_f = init_l_f;
    h_fp = init_h_fp;
    w_fp = init_w_fp;
    l_fp = init_l_fp;
    w_diff = init_w_diff;
    d_f = init_d_f;
    box_size = init_box_size;

    g[0] = std::make_shared<Box>(l, h, w);
    g[1] = std::make_shared<Box>(l_f, h_f, w_f);
    g[2] = std::make_shared<Box>(l_f, h_f, w_f);
    g[3] = std::make_shared<Box>(l_fp, h_fp, w_fp);
    g[4] = std::make_shared<Box>(l_fp, h_fp, w_fp);
    g[5] = std::make_shared<Box>(l_fp, h_fp, w_fp);
    g[6] = std::make_shared<Box>(l_fp, h_fp, w_fp);
    g[7] = std::make_shared<Box>(l_fp, h_f - 2 * h_fp, w_fp - w_diff);
    g[8] = std::make_shared<Box>(l_fp, h_f - 2 * h_fp, w_fp - w_diff);
    for (int i = 0; i < 6; i++) {
      small_box[i] = std::make_shared<Box>(l_fp, h_fp, box_size); 
    }
    

    t[0].linear().setIdentity();
    t[0].translation() << -l_f + l_fp/2 - l/2, 0, 0;

    t[1].linear().setIdentity();
    t[1].translation() << -l_f/2 + l_fp/2, 0, d_f + w_f/2 + w_fp;
    
    t[2].linear().setIdentity();
    t[2].translation() << -l_f/2 + l_fp/2, 0, -d_f - w_f/2 - w_fp;
    
    t[3].linear().setIdentity();
    t[3].translation() << 0, -h_f/2 + h_fp/2, d_f + w_fp/2;
    
    t[4].linear().setIdentity();
    t[4].translation() << 0, -h_f/2 + h_fp/2, -d_f - w_fp/2;
    
    t[5].linear().setIdentity();
    t[5].translation() << 0, h_f/2 - h_fp/2, d_f + w_fp/2;
    
    t[6].linear().setIdentity();
    t[6].translation() << 0, h_f/2 - h_fp/2, -d_f - w_fp/2;
    
    t[7].linear().setIdentity();
    t[7].translation() << 0, 0, d_f + (w_fp + w_diff)/2;
    
    t[8].linear().setIdentity();
    t[8].translation() << 0, 0, -d_f - (w_fp - w_diff)/2;
    
    small_t[0].linear().setIdentity();
    small_t[0].translation() << 0, -h_f/2 + h_fp/2, d_f;
    
    small_t[1].linear().setIdentity();
    small_t[1].translation() << 0, -h_f/2 + h_fp/2, -d_f;
    
    small_t[2].linear().setIdentity();
    small_t[2].translation() << 0, h_f/2 - h_fp/2, d_f;
    
    small_t[3].linear().setIdentity();
    small_t[3].translation() << 0, h_f/2 - h_fp/2, -d_f;
    
    small_t[4].linear().setIdentity();
    small_t[4].translation() << 0, 0, d_f + w_diff;
    
    small_t[5].linear().setIdentity();
    small_t[5].translation() << 0, 0, -d_f + w_diff;

  }

  void changeWidth(double new_d_f) {
    t[1].linear().setIdentity();
    t[1].translation() << -l_f/2 + l_fp/2, 0, new_d_f + w_f/2 + w_fp;
    
    t[2].linear().setIdentity();
    t[2].translation() << -l_f/2 + l_fp/2, 0, -new_d_f - w_f/2 - w_fp;
    
    t[3].linear().setIdentity();
    t[3].translation() << 0, -h_f/2 + h_fp/2, new_d_f + w_fp/2;
    
    t[4].linear().setIdentity();
    t[4].translation() << 0, -h_f/2 + h_fp/2, -new_d_f - w_fp/2;
    
    t[5].linear().setIdentity();
    t[5].translation() << 0, h_f/2 - h_fp/2, new_d_f + w_fp/2;
    
    t[6].linear().setIdentity();
    t[6].translation() << 0, h_f/2 - h_fp/2, -new_d_f - w_fp/2;
    
    t[7].linear().setIdentity();
    t[7].translation() << 0, 0, new_d_f + (w_fp + w_diff)/2;
    
    t[8].linear().setIdentity();
    t[8].translation() << 0, 0, -new_d_f - (w_fp + w_diff)/2;
    
    small_t[0].linear().setIdentity();
    small_t[0].translation() << 0, -h_f/2 + h_fp/2, new_d_f ;
    
    small_t[1].linear().setIdentity();
    small_t[1].translation() << 0, -h_f/2 + h_fp/2, -new_d_f ;
    
    small_t[2].linear().setIdentity();
    small_t[2].translation() << 0, h_f/2 - h_fp/2, new_d_f;
    
    small_t[3].linear().setIdentity();
    small_t[3].translation() << 0, h_f/2 - h_fp/2, -new_d_f;
    
    small_t[4].linear().setIdentity();
    small_t[4].translation() << 0, 0, new_d_f + w_diff;
    
    small_t[5].linear().setIdentity();
    small_t[5].translation() << 0, 0, -new_d_f - w_diff;
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
                                      0, 1, 0, id_total);
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
                                      1, id_total_line, 0);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id_total_line, 0);
    }
    
    for(int i=3; i<9; i++) {
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
                                      1, 0, 0, id_total);
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
    
    // small box
    for(int j=0; j<6; j++) {
      auto T = gripper_transform * small_t[j];         
      Eigen::Vector3d position(T.translation()); 
      Eigen::Quaterniond quat(T.linear());  
      Eigen::Vector3f posf;
      Eigen::Quaternionf quatf;

      posf = position.cast <float> ();
      quatf = quat.cast <float> ();
      std::string id_total = "small_cube" + id + std::to_string(j);
      vis.addCube(posf, quatf,
                  small_box[j]->side[0],
                  small_box[j]->side[1],
                  small_box[j]->side[2],
                  id_total);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      0, 0, 1, id_total);
      vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                      opacity, id_total, 0);

      std::string id_total_line = "small_cube_line" + id + std::to_string(j);
      vis.addCube(posf, quatf,
                  small_box[j]->side[0],
                  small_box[j]->side[1],
                  small_box[j]->side[2],
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
  YAMLConfig config_;
  BVHMPtr mesh_model_;
  FCLGripper gripper_model_; 
  std::vector<std::vector<fcl::Contact >> contactsVec_;
  
  void setConfig(const YAMLConfig &config) {
    config_ = config;
  }  
  void loadMesh(
    const std::vector <TrianglePlaneData> & mesh) {
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
  
  std::vector<fcl::Contact>& global_pairs()
  {
    static std::vector<fcl::Contact> static_global_pairs;
    return static_global_pairs;
  }

  double isCollide(Eigen::Isometry3d gripper_transform, int mode, double distance) {         
    fcl::CollisionRequest collisionRequest;            
    if (config_.simplified_gripper == true) {
      fcl::CollisionResult collisionResult[9];       
      fcl::Transform3f init;
      init.setIdentity();         
      gripper_model_.changeWidth(distance);
      for (int i=0; i<9 ;++i) {
        Eigen::Isometry3d cur_transform = gripper_transform * gripper_model_.t[i];
        fcl::Transform3f fcl_transform;
        eigen2Fcl(cur_transform, fcl_transform);
        fcl::collide(mesh_model_.get(), init,
                     gripper_model_.g[i].get(), fcl_transform,
                     collisionRequest, collisionResult[i]);

        if (collisionResult[i].isCollision() == true){          
          return 0;
        }       
        /*
        fcl::DistanceRequest distanceRequest;
        fcl::DistanceResult distanceResult[9];
        fcl::distance(mesh_model_.get(), init,
                     gripper_model_.g[i].get(), fcl_transform,
                     distanceRequest, distanceResult[i]);       
        if (min_distance >= distanceResult[i].min_distance) {
          min_distance = distanceResult[i].min_distance;         
          std::cout<<min_distance<<std::endl;
        }
        */      
      }    
    }
    
    fcl::CollisionRequest collisionRequest_intersect;
    collisionRequest_intersect.num_max_contacts = std::numeric_limits<int>::max(); 
    collisionRequest_intersect.enable_contact = true;
    collisionRequest_intersect.enable_cost = true;
    collisionRequest_intersect.use_approximate_cost = true;
    fcl::Transform3f init;
    init.setIdentity();
    fcl::Transform3f fcl_transform;
    fcl::CollisionResult collisionResult_intersect[6];
    
    double totalCost = 0;
    if (mode == 1) {
      for (int j = 4; j < 6; j++) {
        Eigen::Isometry3d cur_transform = gripper_transform * gripper_model_.small_t[j];            
        eigen2Fcl(cur_transform, fcl_transform);
        fcl::collide(mesh_model_.get(), init,
                     gripper_model_.small_box[j].get(), fcl_transform,
                     collisionRequest_intersect, collisionResult_intersect[j]);

        if (collisionResult_intersect[j].isCollision() != true){
          return 0;
        } else {        
          std::vector <fcl::Contact> contacts_;
          std::vector <fcl::CostSource> cost_sources_ ;              
          //collisionResult_intersect[j].getContacts(contacts_);
          collisionResult_intersect[j].getCostSources(cost_sources_);
          //contactsVec_.push_back(contacts_);
          for (auto & cost_source : cost_sources_) {
            totalCost += cost_source.total_cost;
          }
        }       
      }
      return totalCost;     
    }
    
    if (mode == 2) {
      for (int j = 0; j < 4; j++) {
        Eigen::Isometry3d cur_transform = gripper_transform * gripper_model_.small_t[j];    
        eigen2Fcl(cur_transform, fcl_transform);
        fcl::collide(mesh_model_.get(), init,
                     gripper_model_.small_box[j].get(), fcl_transform,
                     collisionRequest_intersect, collisionResult_intersect[j]);
                     
        if (collisionResult_intersect[j].isCollision() != true){
          return 0;
        } else {                  
          std::vector <fcl::Contact> contacts_;
          std::vector <fcl::CostSource> cost_sources_ ;              
          //collisionResult_intersect[j].getContacts(contacts_);
          collisionResult_intersect[j].getCostSources(cost_sources_);
          //contactsVec_.push_back(contacts_);
          for (auto & cost_source : cost_sources_) {
            totalCost += cost_source.total_cost;
          }        
        }     
      } 
      return totalCost;                                     
    } else {
      return 0;
    }
  }
};


