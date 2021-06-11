#include "pgfat/visualization.h"


void Visualizer::setConfig(const YAMLConfig &config)
{
  config_ = config;
}

void Visualizer::setMesh(const std::vector <TrianglePlaneData>& triangle_mesh)
{
  planes_ = triangle_mesh;
	
}

void Visualizer::setProperty(const Mesh_preprocessor &mpp, const GraspPointGenerator &gpg)
{
  mpp_ = mpp;
  gpg_ = gpg;
}

void Visualizer::display_initial(const pcl::PolygonMesh& mesh)
{ 
  pcl::visualization::PCLVisualizer vis1 ("initial mesh");
  vis1.addPolygonMesh(mesh, "meshes",0);
  vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, config_.mesh_color[0], config_.mesh_color[1], config_.mesh_color[2], "meshes");
  vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "meshes");
  vis1.setBackgroundColor (config_.background_color[0], config_.background_color[1], config_.background_color[2]); 
   
  vis1.spin ();
}

void Visualizer::display_reconstruct()
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();  
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();  
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  vtkIdType pid[3];  
  int Num = planes_.size();
  
  for (unsigned int i = 0; i < Num; i++)
  {
    vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
    triangle->GetPointIds()->SetNumberOfIds(3);
    for ( unsigned int j = 0; j < 3; j++ )
    {     
      pid[j] = points->InsertNextPoint (planes_[i].points[j][0], planes_[i].points[j][1], planes_[i].points[j][2]);        
    }
    cells->InsertNextCell (3,pid);
    polydata->SetPoints ( points ); 
    polydata->SetPolys(cells);
  }

  pcl::visualization::PCLVisualizer vis2 ("mesh reconstruction");
  pcl::PolygonMesh mesh;  
  pcl::io::vtk2mesh(polydata,mesh);    
    
  vis2.addPolygonMesh(mesh, "meshes",0);
  vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, config_.mesh_color[0], config_.mesh_color[1], config_.mesh_color[2], "meshes");
  vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "meshes");
  vis2.setBackgroundColor (config_.background_color[0], config_.background_color[1], config_.background_color[2]);
       
  vis2.spin ();
}

void Visualizer::display_cluster()
{
  pcl::visualization::PCLVisualizer vis2 ("mesh segmentation");
  pcl::PolygonMesh mesh;  
  
  int clusters_index = 0;
  srand(time(NULL));
  for (std::set<std::set<int>>::iterator cluster = mpp_.clusters.begin(); cluster != mpp_.clusters.end(); cluster++)
  {
    clusters_index++;
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();  
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();  
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkIdType pid[3];  
    int Num = planes_.size();
   
    std::set<int>::iterator face_index = (*cluster).begin();
    while (face_index != (*cluster).end())
    {
      for ( unsigned int j = 0; j < 3; j++ )
      {     
        pid[j] = points->InsertNextPoint (planes_[*face_index].points[j][0], planes_[*face_index].points[j][1], planes_[*face_index].points[j][2]);        
      }
      cells->InsertNextCell (3,pid);
      polydata->SetPoints ( points ); 
      polydata->SetPolys(cells);
      face_index++;
    }
 
    pcl::io::vtk2mesh(polydata,mesh);      // too many mesh added to the same window!!!!!
    std::string mesh_name(std::to_string(clusters_index));
    vis2.addPolygonMesh(mesh, mesh_name,0);
    vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rand() % (N + 1) / (float)(N + 1), rand() % (N + 1) / (float)(N + 1), rand() % (N + 1) / (float)(N + 1), mesh_name);
    vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, mesh_name);                  
  }
  vis2.setBackgroundColor (config_.background_color[0], config_.background_color[1], config_.background_color[2]);
  vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, config_.point_size); 
  //vis2.addPointCloud<pcl::PointXYZRGBNormal> (gpg_.candid_sample_cloud_);
  
  vis2.addPointCloudNormals<pcl::PointXYZRGBNormal> (gpg_.candid_sample_cloud_, 1, 0.1,"cloud_normals");
  vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, config_.point_color[0], config_.point_color[1], config_.point_color[2], "cloud_normals");
  vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cloud_normals");
  
  vis2.addPointCloudNormals<pcl::PointXYZRGBNormal> (gpg_.candid_result_cloud_, 1, 0.1,"cloud_normals_2");
  vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_normals_2");
  vis2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cloud_normals_2");
  vis2.spin ();
}



