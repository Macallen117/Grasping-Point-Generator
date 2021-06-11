#include <yaml-cpp/yaml.h>
#ifndef _YAMLConfig_
#define _YAMLConfig_
struct YAMLConfig
{
  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode = YAML::LoadFile(file_name);

    Theta_pln = yamlnode["Theta_pln"].as<double> ();
    Theta_fct = yamlnode["Theta_fct"].as<double> ();
    
    point_generation_method = yamlnode["point_generation_method"].as<std::string>();
    Distance_bdry = yamlnode["Distance_bdry"].as<double> ();
    Distance_rnn = yamlnode["Distance_rnn"].as<double> ();
    
    H_max = yamlnode["H_max"].as<double> ();
    Theta_parl = yamlnode["Theta_parl"].as<double> ();
    N_da = yamlnode["N_da"].as<int> ();
    
    Stroke_per_jaw = yamlnode["Stroke_per_jaw"].as<double> ();
    gripper_fingers_dist = yamlnode["gripper_fingers_dist"].as<double> ();
    
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
  double Theta_pln;
  double Theta_fct;

  double Distance_bdry;
  double Distance_rnn;
  
  double H_max;
  double Theta_parl;
  int N_da;
  
  double Stroke_per_jaw;
  double gripper_fingers_dist;

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
