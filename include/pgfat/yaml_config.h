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
    
    simplified_gripper = yamlnode["simplified_gripper"].as<bool>();
    explicit_gripper = yamlnode["explicit_gripper"].as<bool>();
    
    Approach_boundary = yamlnode["Approach_boundary"].as<bool>();
    Approach_rotation = yamlnode["Approach_rotation"].as<bool>();
    N_da = yamlnode["N_da"].as<int> ();
    
    Stroke_per_jaw = yamlnode["Stroke_per_jaw"].as<double> ();
    gripper_fingers_dist = yamlnode["gripper_fingers_dist"].as<double> ();
    
    display_figure = yamlnode["display_figure"].as<bool>();
    display_cluster = yamlnode["display_cluster"].as<bool>();
    display_sample_points = yamlnode["display_sample_points"].as<bool>();
    display_result_points = yamlnode["display_result_points"].as<bool>();
    display_grasp = yamlnode["display_grasp"].as<bool>();
    display_cluster_boundary = yamlnode["display_cluster_boundary"].as<bool>();
        
    attach_coordination = yamlnode["attach_coordination"].as<bool>();
    background_color = yamlnode["background_color"].as<std::vector<double> >();
    mesh_color = yamlnode["mesh_color"].as<std::vector<double> >();
    point_color = yamlnode["point_color"].as<std::vector<double> >();
    grasp_color = yamlnode["grasp_color"].as<std::vector<double> >();
    point_size = yamlnode["point_size"].as<int> ();
    gripper_opacity = yamlnode["gripper_opacity"].as<double> ();
    gripper_params = yamlnode["gripper_params"].as<std::vector<double> >();
    gripper_depth_epsilon = yamlnode["gripper_depth_epsilon"].as<double> ();
  }
 
  std::string point_generation_method;
  double Theta_pln;
  double Theta_fct;

  double Distance_bdry;
  double Distance_rnn;
  
  double H_max;
  double Theta_parl;
  
  bool simplified_gripper;
  bool explicit_gripper;
  
  bool Approach_boundary;
  bool Approach_rotation;
  int N_da;
  
  double Stroke_per_jaw;
  double gripper_fingers_dist;

  bool display_figure;
  bool display_cluster;
  bool display_sample_points;
  bool display_result_points;
  bool display_grasp;
  bool attach_coordination;
  bool display_cluster_boundary;

  std::vector<double> background_color;
  std::vector<double> mesh_color;
  std::vector<double> point_color;
  std::vector<double> grasp_color;
  int point_size;
  double gripper_opacity;
  
  std::vector<double> gripper_params;
  double gripper_depth_epsilon;

};
#endif
