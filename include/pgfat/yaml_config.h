#include <yaml-cpp/yaml.h>
#ifndef _YAMLConfig_
#define _YAMLConfig_
struct YAMLConfig
{
  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode = YAML::LoadFile(file_name);

    Theta1 = yamlnode["Theta1"].as<double> ();
    Theta2 = yamlnode["Theta2"].as<double> ();
    
    point_generation_method = yamlnode["point_generation_method"].as<std::string>();
    Distance1 = yamlnode["Distance1"].as<double> ();
    Distance2 = yamlnode["Distance2"].as<double> ();
    
    display_figure = yamlnode["display_figure"].as<bool>();
    display_points = yamlnode["display_points"].as<bool>();
    display_grasp = yamlnode["display_grasp"].as<bool>();
        
    attach_coordination = yamlnode["attach_coordination"].as<bool>();
    background_color = yamlnode["background_color"].as<std::vector<double> >();
    mesh_color = yamlnode["mesh_color"].as<std::vector<double> >();
    point_color = yamlnode["point_color"].as<std::vector<double> >();
    grasp_color = yamlnode["grasp_color"].as<std::vector<double> >();
    point_size = yamlnode["point_size"].as<int> ();
  }
 
  std::string point_generation_method;
  double Theta1;
  double Theta2;

  double Distance1;
  double Distance2;

  bool display_figure;
  bool display_points;
  bool display_grasp;
  bool attach_coordination;

  std::vector<double> background_color;
  std::vector<double> mesh_color;
  std::vector<double> point_color;
  std::vector<double> grasp_color;
  int point_size;

};
#endif
