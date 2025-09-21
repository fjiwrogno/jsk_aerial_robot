#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

class FlamingoParamManager
{
public:
  FlamingoParamManager() : nh_(""), pnh_("~")
  {
    mode_sub_ = nh_.subscribe("flamingo/robot_mode", 1, &FlamingoParamManager::modeCallback, this);

    // Get config file path
    pnh_.param("simulation", is_simulation_, false);

    // Get config file path
    std::string package_path = ros::package::getPath("flamingo");
    config_path_ = package_path + "/config/";

    // Load initial parameters (default to air mode)
    pnh_.param("default_mode", current_mode_, 0);
    ROS_INFO("Flamingo Param Manager initialized. Default mode: %s", (current_mode_ == 0) ? "Air" : "Water");
    loadParams(current_mode_);
  }

private:
  void modeCallback(const std_msgs::UInt8::ConstPtr& msg)
  {
    int new_mode = msg->data;
    if (new_mode != current_mode_ && (new_mode == 0 || new_mode == 1))
    {
      ROS_INFO("Switching robot mode from %d to %d", current_mode_, new_mode);
      current_mode_ = new_mode;
      loadParams(current_mode_);
    }
  }

  void loadParams(int mode)
  {
    std::string mode_suffix = (mode == 0) ? "air" : "water";
    ROS_INFO("Loading parameters for %s mode", mode_suffix.c_str());

    std::vector<std::pair<std::string, std::string>> files_and_namespaces;
    files_and_namespaces.push_back({ "MotorInfo_" + mode_suffix + ".yaml", "/" });
    if (is_simulation_)
    {
      files_and_namespaces.push_back({ "FlamingoControl_sim_" + mode_suffix + ".yaml", "/" });
    }
    else
    {
      files_and_namespaces.push_back({ "FlamingoControl_" + mode_suffix + ".yaml", "/" });
    }
    files_and_namespaces.push_back({ "NavigationConfig_" + mode_suffix + ".yaml", "/navigation" });

    for (const auto& file_info : files_and_namespaces)
    {
      std::string file_path = config_path_ + mode_suffix + "/" + file_info.first;
      try
      {
        YAML::Node config = YAML::LoadFile(file_path);
        setParams(config, file_info.second);
        ROS_INFO("Successfully loaded %s", file_path.c_str());
      }
      catch (const YAML::Exception& e)
      {
        ROS_ERROR("Failed to load or parse %s: %s", file_path.c_str(), e.what());
      }
    }
  }

  void setParams(const YAML::Node& node, const std::string& ns)
  {
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
      std::string key = it->first.as<std::string>();
      std::string full_key = ns == "/" ? "/" + key : ns + "/" + key;

      if (it->second.IsMap())
      {
        setParams(it->second, full_key);
      }
      else if (it->second.IsScalar())
      {
        // Try to set as double, int, bool, then string
        try
        {
          nh_.setParam(full_key, it->second.as<double>());
        }
        catch (const YAML::BadConversion&)
        {
          try
          {
            nh_.setParam(full_key, it->second.as<int>());
          }
          catch (const YAML::BadConversion&)
          {
            try
            {
              nh_.setParam(full_key, it->second.as<bool>());
            }
            catch (const YAML::BadConversion&)
            {
              nh_.setParam(full_key, it->second.as<std::string>());
            }
          }
        }
      }
      else if (it->second.IsSequence())
      {
        // Convert sequence to vector of doubles or strings
        bool is_double = true;
        std::vector<double> double_vec;
        for (const auto& v : it->second)
        {
          try
          {
            double_vec.push_back(v.as<double>());
          }
          catch (const YAML::BadConversion&)
          {
            is_double = false;
            break;
          }
        }

        if (is_double)
        {
          nh_.setParam(full_key, double_vec);
        }
        else
        {
          std::vector<std::string> string_vec;
          for (const auto& v : it->second)
          {
            string_vec.push_back(v.as<std::string>());
          }
          nh_.setParam(full_key, string_vec);
        }
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber mode_sub_;
  std::string config_path_;
  int current_mode_;  // 0 for air, 1 for water
  bool is_simulation_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flamingo_param_manager");
  FlamingoParamManager manager;
  ros::spin();
  return 0;
}
