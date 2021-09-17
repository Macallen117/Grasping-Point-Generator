#include "pgfat/visualization.h"


void Visualizer::setConfig(const YAMLConfig &config) {
  config_ = config;
}


void Visualizer::setMesh(const std::vector <TrianglePlaneData>& triangle_mesh) {
  planes_ = triangle_mesh;
}


void Visualizer::setProperty(
  const Mesh_preprocessor &mpp,
  const GraspPointGenerator &gpg) {
  mpp_ = mpp;
  gpg_ = gpg;
}


void Visualizer::display_initial(const pcl::PolygonMesh& mesh) {
  pcl::visualization::PCLVisualizer vis1("initial mesh");
  vis1.addPolygonMesh(mesh, "meshes", 0);
  vis1.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR,
    config_.mesh_color[0],
    config_.mesh_color[1],
    config_.mesh_color[2],
    "meshes");
  vis1.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
    1,
    "meshes");
  vis1.setBackgroundColor(
    config_.background_color[0],
    config_.background_color[1],
    config_.background_color[2]);

  vis1.spin();
}


void Visualizer::display_reconstruct() {
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  vtkIdType pid[3];
  int Num = planes_.size();

  for (unsigned int i = 0; i < Num; i++) {
    vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
    triangle->GetPointIds()->SetNumberOfIds(3);
    for ( unsigned int j = 0; j < 3; j++ ) {
      pid[j] = points->InsertNextPoint(
        planes_[i].points[j][0],
        planes_[i].points[j][1],
        planes_[i].points[j][2]);
    }
    cells->InsertNextCell(3, pid);
    polydata->SetPoints(points);
    polydata->SetPolys(cells);
  }

  pcl::visualization::PCLVisualizer vis2("mesh reconstruction");
  pcl::PolygonMesh mesh;
  pcl::io::vtk2mesh(polydata, mesh);

  vis2.addPolygonMesh(mesh, "meshes", 0);
  vis2.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_COLOR,
    config_.mesh_color[0],
    config_.mesh_color[1],
    config_.mesh_color[2],
    "meshes");
  vis2.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
    1,
    "meshes");
  vis2.setBackgroundColor(
    config_.background_color[0],
    config_.background_color[1],
    config_.background_color[2]);

  vis2.spin();
}

void Visualizer::display_cluster() {
  pcl::visualization::PCLVisualizer vis2("mesh segmentation");
  pcl::PolygonMesh mesh;

  int clusters_index = 0;
  srand(time(NULL));
  for (std::map<int, std::set<int>>::iterator cluster = mpp_.clusters.begin();
    cluster != mpp_.clusters.end(); cluster++) {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkIdType pid[3];
    int Num = planes_.size();

    std::set<int>::iterator face_index = cluster->second.begin();
    while (face_index != cluster->second.end()) {
      for ( unsigned int j = 0; j < 3; j++ ) {
        pid[j] = points->InsertNextPoint(
          planes_[*face_index].points[j][0],
          planes_[*face_index].points[j][1],
          planes_[*face_index].points[j][2]);
      }
      cells->InsertNextCell(3, pid);
      polydata->SetPoints(points);
      polydata->SetPolys(cells);
      face_index++;
    }
    pcl::io::vtk2mesh(polydata, mesh);  // too many mesh added
    std::string mesh_name(std::to_string(cluster->first));
    vis2.addPolygonMesh(mesh, mesh_name, 0);
    vis2.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      rand() % (N + 1) / static_cast<float>(N + 1),
      rand() % (N + 1) / static_cast<float>(N + 1),
      rand() % (N + 1) / static_cast<float>(N + 1),
      mesh_name);
    vis2.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
      3,
      mesh_name);
  }
  vis2.setBackgroundColor(
    config_.background_color[0],
    config_.background_color[1],
    config_.background_color[2]);
  /*
  vis2.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
    config_.point_size);
  */
  if (config_.display_sample_points == true) {
    vis2.addPointCloudNormals<pcl::PointXYZRGBNormal>(
      gpg_.candid_sample_cloud_,
      1, 0.5,
      "cloud_normals");
    vis2.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      config_.point_color[0],
      config_.point_color[1],
      config_.point_color[2],
      "cloud_normals");
    vis2.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
      2,
      "cloud_normals");
  }
  if (config_.display_result_points == true) {
    vis2.addPointCloudNormals<pcl::PointXYZRGBNormal> (
      gpg_.candid_result_cloud1_,
      1, 0.5,
      "cloud_normals_1");
    vis2.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      1, 0, 0,
      "cloud_normals_1");
    vis2.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
      2,
      "cloud_normals_1");
    vis2.addPointCloudNormals<pcl::PointXYZRGBNormal> (
      gpg_.candid_result_cloud2_,
      1, 0.5,
      "cloud_normals_2");
    vis2.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      0, 1, 0,
      "cloud_normals_2");
    vis2.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
      2,
      "cloud_normals_2");
  }
  vis2.spin();
}


void Visualizer::display_grasp(const pcl::PolygonMesh& mesh) {
  pcl::visualization::PCLVisualizer vis3("grasp on mesh");
  vis3.setBackgroundColor(
      config_.background_color[0],
      config_.background_color[1],
      config_.background_color[2]);
  if (config_.display_figure == true) {
    vis3.addPolygonMesh(mesh, "meshes", 0);
    vis3.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      config_.mesh_color[0],
      config_.mesh_color[1],
      config_.mesh_color[2],
      "meshes");
    vis3.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
      1,
      "meshes");    
  }
  if (config_.display_sample_points == true) {
    vis3.addPointCloudNormals<pcl::PointXYZRGBNormal>(
      gpg_.candid_sample_cloud_,
      1, 0.5,
      "cloud_normals");
    vis3.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      1, 0, 0,
      "cloud_normals");
    vis3.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
      2,
      "cloud_normals");
  }
  if (config_.display_result_points == true) {
    
    vis3.addPointCloudNormals<pcl::PointXYZRGBNormal> (
      gpg_.candid_result_cloud1_,
      1, 0.5,
      "cloud_normals_1");
    vis3.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      0, 0, 1,
      "cloud_normals_1");
    vis3.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
      3,
      "cloud_normals_1");
    vis3.addPointCloudNormals<pcl::PointXYZRGBNormal> (
      gpg_.candid_result_cloud2_,
      1, 0.5,
      "cloud_normals_2");
    vis3.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
      1, 0, 0,
      "cloud_normals_2");
    vis3.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
      3,
      "cloud_normals_2");
  }
  if (config_.display_cluster_boundary == true) {
    int line_id = 0;
    //std::map<int, std::vector<edge>>::iterator it = gpg_.bdrys_.begin();
    //it++;
    for (std::map<int, std::vector<edge>>::iterator it = gpg_.bdrys_.begin();
      it != gpg_.bdrys_.end(); it++) {
      double r = rand() % (N + 1) / static_cast<float>(N + 1); 
      double g = rand() % (N + 1) / static_cast<float>(N + 1);
      double b = rand() % (N + 1) / static_cast<float>(N + 1);    
      for (std::vector<edge>::iterator vit = it->second.begin();
      vit != it->second.end(); vit++) {       
        pcl::PointXYZRGBNormal p1, p2;
        eigen2PCL(vit->first, p1, 255,0,0);
        eigen2PCL(vit->second, p2, 255,0,0);      
        vis3.addLine(
          p1,p2,
          r, g, b,
          std::string("line") + std::to_string(line_id));
        vis3.setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, 
          std::string("line") + std::to_string(line_id));
        line_id++;
      }
    }
  }
  if (config_.display_grasp == true) {
      int id_num = 0;
      ///*     
      for (auto & grasp : gpg_.grasp_cand_collision_free1_) {
        //std::cout<<"maximum cost: "<<grasp.getContactArea()<<std::endl;
        gpg_.CollisionCheck_.gripper_model_.drawGripper(
          vis3,
          grasp.hand_transform,
          std::to_string(id_num++),
          config_.grasp_color[0],
          config_.grasp_color[1],
          config_.grasp_color[2],
          config_.gripper_opacity,
          grasp.getDist()/2.0 - config_.gripper_params[9]);
        if (id_num == 1)  break;
      }
      //*/
      /*
      for (auto & grasp : gpg_.grasp_cand_collision_free2_) {
        //std::cout<<"maximum cost: "<<grasp.getContactArea()<<std::endl;
        gpg_.CollisionCheck_.gripper_model_.drawGripper(
          vis3,
          grasp.hand_transform,
          std::to_string(id_num++),
          config_.grasp_color[0],
          config_.grasp_color[1],
          config_.grasp_color[2],
          config_.gripper_opacity,
          grasp.getDist()/2.0);
        if (id_num == 1)  break;
      }
       */    
      
       /*   
      vtkSmartPointer<vtkPoints> points_ = vtkSmartPointer<vtkPoints>::New();
      vtkSmartPointer<vtkCellArray> cells_ = vtkSmartPointer<vtkCellArray>::New();
      vtkSmartPointer<vtkPolyData> polydata_ = vtkSmartPointer<vtkPolyData>::New();
      vtkIdType pid_[3];
      
      int k = 0;
      for (auto & cont : gpg_.CollisionCheck_.contactsVec_) {         
        for (auto & conta : cont) {         
          vtkSmartPointer<vtkTriangle> triangle_ = vtkSmartPointer<vtkTriangle>::New();
          triangle_->GetPointIds()->SetNumberOfIds(3);
          for (unsigned int j = 0; j < 3; j++ ) {
            pid_[j] = points_->InsertNextPoint(
            planes_[conta.b1].points[j][0],
            planes_[conta.b1].points[j][1],
            planes_[conta.b1].points[j][2]);
          }
          cells_->InsertNextCell(3, pid_);
          polydata_->SetPoints(points_);
          polydata_->SetPolys(cells_);                   
        }
        k++;
        if (k == 6) break;        
      }
     
      pcl::PolygonMesh mesh_;     
      pcl::io::vtk2mesh(polydata_, mesh_);     
      vis3.addPolygonMesh(mesh_, "meshesre", 0);
    
      vis3.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR,
        config_.mesh_color[0],
        config_.mesh_color[1],
        config_.mesh_color[2],
        "meshesre");
      vis3.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
        1,
        "meshesre");
      //vis3.spin();
      */

  }        
  vis3.spin();
}





