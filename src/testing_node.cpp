#include "testing.h"


using namespace std;

bool FileExist(const char* FileName)
{
    struct stat my_stat;
    return (stat(FileName, &my_stat) == 0);
}

void check_reached_goal(XYZYaw current,XYZYaw target)
{
  double diff_xyz = sqrt((target.x - current.x) * (target.x - current.x) 
                        + (target.y - current.y) * (target.y - current.y)
                        + (target.z - current.z) * (target.z - current.z)); 
  
  double diff_yaw = math_common::angular_dist(target.yaw,current.yaw);

  if (diff_xyz < reached_thresh_xyz && diff_yaw < math_common::deg2rad(reached_yaw_degrees))
      reached_goal = true; 
}

string readTxt(string file)
{
    ifstream infile; 
    infile.open(file.data());
    assert(infile.is_open());
    string s;
    while(getline(infile,s))
    {
        return s;
    }
    infile.close();             //关闭文件输入流 
}

void write_pos_data(XYZYaw pos,string path)
{
  std::ofstream out (path, std::ios::app);

    if (out.fail())
      {
        cout<< "ERROR In Data Writting, path is:" << path << endl;
      }
      out << pos.x <<","<< pos.y <<","<< pos.z << std::endl;
      cout << "Wrote data in:" << path << endl;
      out.close();
}
 
 void init_pos_file(string path)
 {
   std::ofstream out (path);
   out << "x,y,z" << std::endl;
   out.close();

 }

void cloudCB(const sensor_msgs::PointCloud2 &input)  
{  

  pcl::PointCloud<pcl::PointXYZ> cloud;  
  pcl::fromROSMsg(input, cloud);//
  ROS_INFO("[TestingNode] cloudpoint path is: %s", map_dataset_path.c_str());
  pcl::io::savePCDFileASCII (map_dataset_path, cloud); 

}  

void airsim_odom_cb(const nav_msgs::Odometry &odom_msg)
{
  current_pos.x = odom_msg.pose.pose.position.x;
  current_pos.y = odom_msg.pose.pose.position.y;
  current_pos.z = odom_msg.pose.pose.position.z;
  current_pos.yaw = utils::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation); 
  if (has_cmd)
  {
    if (!reached_goal)
    {
      ROS_INFO_STREAM("[TestingNode] got Odom: x=" << current_pos.x << " y=" << current_pos.y << " z=" << current_pos.z << " yaw=" << current_pos.yaw );
      write_pos_data(current_pos, odom_dataset_path);
    }
  }
}

void pos_cmd_cb(const quadrotor_msgs::PositionCommand &pos_cmd_msg)
{
  has_cmd = true;
  target_pos.x = pos_cmd_msg.position.x;
  target_pos.y = pos_cmd_msg.position.y;
  target_pos.z = pos_cmd_msg.position.z;
  target_pos.yaw = pos_cmd_msg.yaw;
  if (!reached_goal)
  {
    write_pos_data(target_pos, traj_dataset_path);
    // cout << "revieved cmm!!" << target_pos << endl;
    ROS_INFO_STREAM("[TestingNode] got CMD: x=" << target_pos.x << " y=" << target_pos.y << " z=" << target_pos.z << " yaw=" << target_pos.yaw );
  }
}

main (int argc, char **argv)  
{  
  ros::init (argc, argv, "Testing_node");  
  ros::NodeHandle nh("~"); 

  nh.param<std::string>("odom_dataset_path",odom_dataset_path,"");
  nh.param<std::string>("traj_dataset_path",traj_dataset_path,"");
  nh.param<std::string>("map_dataset_path",map_dataset_path,"");
  nh.param<double>("reached_thresh_xyz",reached_thresh_xyz,0);
  nh.param<double>("reached_yaw_degrees",reached_yaw_degrees,0);
  // ROS_INFO("Got param: %s", odom_dataset_path.c_str());
  init_pos_file(odom_dataset_path);
  init_pos_file(traj_dataset_path);

//   ros::Subscriber bat_sub = nh.subscribe("/sdf_map/occupancy_inflate", 10, cloudCB);//recieve pointcloud
  ros::Subscriber bat_sub = nh.subscribe("/map", 10, cloudCB);//recieve pointcloud 
  ros::Subscriber pos_cmd_sub = nh.subscribe("/position_cmd", 10, pos_cmd_cb, ros::TransportHints().tcpNoDelay()); 
  ros::Subscriber airsim_odom_sub = nh.subscribe("/traj_coodi", 10, airsim_odom_cb, ros::TransportHints().tcpNoDelay());


  ros::spin();  
  return 0;  
}