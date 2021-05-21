#include "pgfat/visualization.h"


void Visualizer::setConfig(const YAMLConfig &config)
{
  config_ = config;
}

void Visualizer::setMesh(const std::vector <TrianglePlaneData>& triangle_mesh)
{
  planes_ = triangle_mesh;
	
}

void Visualizer::display_initial(const pcl::PolygonMesh& mesh)
{ 
  pcl::visualization::PCLVisualizer vis1 ("initial mesh");
  //vis1.addPointCloud<pcl::PointXYZRGBNormal> (candid_sample_cloud_);
  vis1.addPolygonMesh(mesh, "meshes",0);
  vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, config_.mesh_color[0], config_.mesh_color[1], config_.mesh_color[2], "meshes");
  vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "meshes");
  vis1.setBackgroundColor (config_.background_color[0], config_.background_color[1], config_.background_color[2]);
  //vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, config_.point_size);  
   
  vis1.spin ();
}

void Visualizer::display_reconstruct(const std::vector <TrianglePlaneData>& triangles)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();  //jide delete
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();  
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  vtkIdType pid[3];  
  int Num = triangles.size();
  
  for (unsigned int i = 0; i < Num; i++)
  {
    vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
    triangle->GetPointIds()->SetNumberOfIds(3);
    for ( unsigned int j = 0; j < 3; j++ )
    {     
      pid[j] = points->InsertNextPoint (triangles[i].points[j][0], triangles[i].points[j][1], triangles[i].points[j][2]);        
    }
    cells->InsertNextCell (3,pid);
    polydata->SetPoints ( points ); 
    polydata->SetPolys(cells);
  }

  pcl::visualization::PCLVisualizer vis2 ("mesh segmentation");
  pcl::PolygonMesh mesh;  
  pcl::io::vtk2mesh(polydata,mesh);    
    
  vis2.addPolygonMesh(mesh, "meshes",0);
  vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, config_.mesh_color[0], config_.mesh_color[1], config_.mesh_color[2], "meshes");
  vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "meshes");
  vis2.setBackgroundColor (config_.background_color[0], config_.background_color[1], config_.background_color[2]);
       
  vis2.spin ();
}



void Visualizer::display_cluster(const std::vector <TrianglePlaneData>& triangles, const std::set<std::set<int>>& clusters)
{
  pcl::visualization::PCLVisualizer vis2 ("mesh segmentation");
  pcl::PolygonMesh mesh; 
  
  int clusters_index = 0;
  double color_range = 1.0 / clusters.size();
  for (std::set<std::set<int>>::iterator cluster = clusters.begin(); cluster != clusters.end(); cluster++)
  {
    clusters_index++;
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();  //jide delete
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();  
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkIdType pid[3];  
    int Num = triangles.size();
   
    std::set<int>::iterator face_index = (*cluster).begin();
    while (face_index != (*cluster).end())
    {
      for ( unsigned int j = 0; j < 3; j++ )
      {     
        pid[j] = points->InsertNextPoint (triangles[*face_index].points[j][0], triangles[*face_index].points[j][1], triangles[*face_index].points[j][2]);        
      }
      cells->InsertNextCell (3,pid);
      polydata->SetPoints ( points ); 
      polydata->SetPolys(cells);
      face_index++;
    }
 
    pcl::io::vtk2mesh(polydata,mesh);    
    std::string mesh_name(std::to_string(clusters_index));
    vis2.addPolygonMesh(mesh, mesh_name,0);
    vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0 - config_.mesh_color[0] - color_range * clusters_index, config_.mesh_color[1] + color_range * clusters_index, config_.mesh_color[2] + color_range * clusters_index/2, mesh_name);
    vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, mesh_name);
    vis2.setBackgroundColor (config_.background_color[0], config_.background_color[1], config_.background_color[2]);         
  }
  vis2.spin ();
}



