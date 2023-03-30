#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include "TData.h"
// #include "rl_dovs/request_goal.h"
#include <std_msgs/UInt8.h>


using namespace std;

double distance (double x, double y){
	return (std::sqrt(x*x+y*y));
}

class GlobalPlannerDOVS{
private:
  ros::Subscriber goal_sub, position_sub, obstacles_sub, velocity_sub;
  ros::Publisher path_pub, goal_pub, last_goal_pub;
  int index_goal = 0, index_next_regular_path = 0;
  ros::ServiceClient make_plan_client, add_obstacle_client;
  nav_msgs::Path current_path;
  std::vector<int> idx_regular_path;
  bool finished = true, planning_success = true;
  ros::NodeHandle n;
  double x, y, theta, prev_x, prev_y;
  bool request_goal = true;
  int first_request = 0, prev_request;
  bool velocity_low = false;
  double look_ahead_distance = 0.4;
  double replanning_distance = 0.7;
  ros::Time last_time_moving;
  geometry_msgs::PoseStamped last_tried_msg;
  // Method 0 -> Full-stack S-DOVS approach
  // Method 1 -> Just final goal
  // Method 2 -> Subsampling
  // Method 3 -> Arena-Rosnav waypoint generator
  // Method 4 -> Full-stack+ (Added timer and arena-like waypoints)
  int method = 0;
public:
  GlobalPlannerDOVS(){
    this->position_sub = n.subscribe("amcl_pose", 1, &GlobalPlannerDOVS::positionChange, this);
    this->velocity_sub = n.subscribe("odom", 1, &GlobalPlannerDOVS::velChange, this);
    this->goal_sub = n.subscribe("/rviz_goal", 1, &GlobalPlannerDOVS::rvizGoalCallback, this);     
    this->path_pub = n.advertise<nav_msgs::Path>("global_planner_path", 1);      
    this->goal_pub = n.advertise<geometry_msgs::PoseStamped>("dovs_goal", 1); 
    this->last_goal_pub = n.advertise<std_msgs::UInt8>("last_goal", 1);  
    this->make_plan_client = n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan",true); 
    // ros::param::get("~request_goal", request_goal);
    ros::param::get("~global_planner_method", method);
    if (method == 3){
      this->look_ahead_distance = 1.55;
      this->replanning_distance = 2.0;
    }
    else if (method == 2){
      this->look_ahead_distance = 0.4;
    }
    else if (method == 4){
      this->look_ahead_distance = 1.55;
    }
    // if (request_goal){
    //   ros::ServiceClient client_goal_request;
    //   client_goal_request = n.serviceClient<rl_dovs::request_goal>("get_current_goal");
    //   rl_dovs::request_goal srv_goal;
    //   while (!client_goal_request.call(srv_goal))
    //   {
    //       ROS_ERROR("Failed to call request goal");
    //   }
    //   geometry_msgs::PoseStamped new_goal;
    //   new_goal.pose.position.x = srv_goal.response.x;
    //   new_goal.pose.position.y = srv_goal.response.y;
    //   rvizGoalCallback(new_goal);
    // }
  }  

  void checkLastTimeMoving(){
    if(!finished && ros::Time::now() - this->last_time_moving >= ros::Duration(3.0)){
      prev_request = first_request;
      rvizGoalCallback(this->current_path.poses[this->current_path.poses.size()-1]);
      first_request = prev_request;         
    }
  }
  
  void velChange(const nav_msgs::Odometry& msg){
    // if (method == 0 || method == 4){
      if (!finished && msg.twist.twist.linear.x < 0.1){
        if (!velocity_low){
          prev_request = first_request;
          rvizGoalCallback(this->current_path.poses[this->current_path.poses.size()-1]);
          first_request = prev_request;   
        }
        velocity_low = true;
      }
      else{
        velocity_low = false;
      }
    // }
  }

  void positionChange(const geometry_msgs::PoseWithCovarianceStamped& msg){
      this->prev_x = this->x;
      this->prev_y = this->y;
      this->x = msg.pose.pose.position.x;
      this->y = msg.pose.pose.position.y;
      double w = msg.pose.pose.orientation.w;
      double z = msg.pose.pose.orientation.z;
      tf::Quaternion q(
          msg.pose.pose.orientation.x,
          msg.pose.pose.orientation.y,
          msg.pose.pose.orientation.z,
          msg.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      this->theta = yaw;
      if (!finished && !planning_success){
        rvizGoalCallback(last_tried_msg);
      }
      else if (!finished){
        bool new_goal = false;
        if (method == 4){
            while (!finished && distance(this->x - this->current_path.poses[index_goal].pose.position.x, this->y - this->current_path.poses[index_goal].pose.position.y)<look_ahead_distance){
              index_goal++;
              while (index_next_regular_path < idx_regular_path.size()-1 && idx_regular_path[index_next_regular_path] < index_goal){
                index_next_regular_path++;
                look_ahead_distance = 1.55;
              }
              if (idx_regular_path[index_next_regular_path] == index_goal){
                look_ahead_distance = 0.6;
              }
              else if (idx_regular_path[index_next_regular_path] > index_goal){
                look_ahead_distance = 1.55;
              }              
              new_goal = true;

              if (index_goal>=this->current_path.poses.size()){
                finished = true;
                index_goal --;
                std_msgs::UInt8 msg_last_goal;
                msg_last_goal.data = 1;
                last_goal_pub.publish(msg_last_goal);
              }
            }
            if (new_goal){
              goal_pub.publish(current_path.poses[index_goal]);
            }
        }
        else{
            while (!finished && distance(this->x - this->current_path.poses[index_goal].pose.position.x, this->y - this->current_path.poses[index_goal].pose.position.y)<look_ahead_distance){
              index_goal++;
              new_goal = true;
              if (index_goal>=this->current_path.poses.size()){
                finished = true;
                index_goal --;
                std_msgs::UInt8 msg_last_goal;
                msg_last_goal.data = 1;
                last_goal_pub.publish(msg_last_goal);
              }
            }
            if (new_goal){
              goal_pub.publish(current_path.poses[index_goal]);
            }
        }
        if (this->x != this->prev_x && this->y != this->prev_y){
          this->last_time_moving = ros::Time::now();
        }
        Line segment_line = Line(Tpf(this->current_path.poses[index_goal-1].pose.position.x, this->current_path.poses[index_goal-1].pose.position.y), 
                                Tpf(this->current_path.poses[index_goal].pose.position.y, this->current_path.poses[index_goal].pose.position.y));
        if (segment_line.Distance(Tpf(this->x, this->y)) > replanning_distance){
          prev_request = first_request;
          rvizGoalCallback(this->current_path.poses[this->current_path.poses.size()-1]);
          first_request = prev_request;   
        }

        cout << "FR: " << first_request << endl;
        if (first_request >= 0 && request_goal){
          prev_request = first_request;
          rvizGoalCallback(this->current_path.poses[this->current_path.poses.size()-1]);
          first_request = prev_request;   
          first_request++;     
          if (first_request >= 2){
            first_request = -1;
          }
          ROS_INFO("FIRST REQUEST\n");
        }
      }
  }  
  bool callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
  {
      // Perform the actual path planner call
          //Execute the actual path planner
      bool plan_received = false;
        if (serviceClient.call(srv)) {
            if (!srv.response.plan.poses.empty()) {
                ROS_INFO("make_plan success!");
                plan_received = true;
            }
        }
        ros::spinOnce();
        return plan_received;
  }
  void fillPathRequest(nav_msgs::GetPlan::Request &request, float start_x, float start_y, float goal_x, float goal_y)
  {
      request.start.header.frame_id ="map";
      request.start.header.stamp = ros::Time::now();
      request.goal.header.stamp = request.start.header.stamp;
      request.start.pose.position.x = start_x;//initial position x coordinate
          request.start.pose.position.y = start_y;//initial position y coordinate
          request.start.pose.orientation.w = theta;//Orientation
      request.goal.header.frame_id = "map";
          request.goal.pose.position.x = goal_x;//End point coordinates
      request.goal.pose.position.y = goal_y;
      request.goal.pose.orientation.w = theta;
      request.tolerance = 1.0;//If the goal cannot be reached, the nearest available constraint
  }
  void rvizGoalCallback(const geometry_msgs::PoseStamped& msg){
      last_tried_msg = msg;
      nav_msgs::GetPlan srv;
      ROS_INFO("Start x: %f, start y: %f\n", this->x, this->y);
      fillPathRequest(srv.request,this->x,this->y,msg.pose.position.x,msg.pose.position.y);
      planning_success = callPlanningService(make_plan_client,srv);
      if (planning_success){
        this->current_path.poses.clear();
        this->current_path.poses.push_back(srv.response.plan.poses[0]);
        if (method == 1){
            // Only final goal
        }
        else if (method ==2){
          for (int i = 40; i<srv.response.plan.poses.size(); i+=50){
            this->current_path.poses.push_back(srv.response.plan.poses[i]);
          }
        }
        else if (method ==3){
          for (int i = 40; i<srv.response.plan.poses.size(); i++){
            this->current_path.poses.push_back(srv.response.plan.poses[i]);
          }
        }
        else if (method == 0){
          nav_msgs::Path aux_path;
          aux_path.poses.clear();
          for (int i = 0; i<srv.response.plan.poses.size(); i+=30){
            aux_path.poses.push_back(srv.response.plan.poses[i]);
          }
          for (int i = 1; i<aux_path.poses.size()-1; i++){
            double term_1 =aux_path.poses[i].pose.position.y -aux_path.poses[i-1].pose.position.y;
            double term_2 =aux_path.poses[i+1].pose.position.x -aux_path.poses[i].pose.position.x;
            double term_3 =aux_path.poses[i+1].pose.position.y -aux_path.poses[i].pose.position.y;
            double term_4 =aux_path.poses[i].pose.position.x -aux_path.poses[i-1].pose.position.x;
            double diff = abs(term_1*term_2 - term_3*term_4);
            if (diff > 0.05){
              this->current_path.poses.push_back(aux_path.poses[i]);
            }
          }
          aux_path.header.frame_id = "map";
      }
      else if (method == 4){
          this->idx_regular_path.clear();
          for (int i = 30; i<srv.response.plan.poses.size(); i++){
            current_path.poses.push_back(srv.response.plan.poses[i]);
          }
          // current_path.poses.push_back(srv.response.plan.poses.back());
          cout << "PATH SIZE: " << current_path.poses.size() << endl;
          for (int i = 30; i<static_cast<int>(current_path.poses.size())-30; i+=30){
            double term_1 =current_path.poses[i].pose.position.y -current_path.poses[i-30].pose.position.y;
            double term_2 =current_path.poses[i+30].pose.position.x -current_path.poses[i].pose.position.x;
            double term_3 =current_path.poses[i+30].pose.position.y -current_path.poses[i].pose.position.y;
            double term_4 =current_path.poses[i].pose.position.x -current_path.poses[i-30].pose.position.x;
            double diff = abs(term_1*term_2 - term_3*term_4);
            if (diff > 0.05){
              this->idx_regular_path.push_back(i);
            }
          }
          this->idx_regular_path.push_back(current_path.poses.size()-1);
          this->index_next_regular_path = 0;
      }
      this->current_path.poses.push_back(srv.response.plan.poses.back());
      this->current_path.header.frame_id = "map";
      srv.response.plan.header.frame_id = "map";
      path_pub.publish(current_path);
      this-> index_goal = 1;
      goal_pub.publish(current_path.poses[index_goal]);
      finished = false;
      first_request = 0;
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planner");

  GlobalPlannerDOVS planner;

  ros::Rate loop_rate(5);

  while (ros::ok()){

    ros::spinOnce();
    planner.checkLastTimeMoving();
    loop_rate.sleep();
  }


  return 0;
}
