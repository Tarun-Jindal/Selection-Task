#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_task1/NewtonRaphsonAction.h>
#include <iostream>
#include <cmath>

class NewtonRaphsonAction
{
public:
    
  NewtonRaphsonAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    
    as_.registerGoalCallback(boost::bind(&NewtonRaphsonAction::goal, this));
    as_.registerPreemptCallback(boost::bind(&NewtonRaphsonAction::preempt, this));

    
    sub_ = nh_.subscribe("/random_number", 1, &NewtonRaphsonAction::analysis, this);
    as_.start();
  }

  ~NewtonRaphsonAction(void)
  {
  }

  void goal()
  {
    
    data_count_ = 0;
    x = 10; //initial guess
    
    
    goal_ = as_.acceptNewGoal()->samples;
  }

  void preempt()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
    analysis();
  }

  void analysis(const std_msgs::Float32::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    while(1)
    {
    data_count_++;
    feedback_.count = data_count_;
    
    float fx = pow(x,3.0)-5*x +13.0;
    float fx1 = 3*pow(x,2.0)-5.0; 

    feedback_.x1 = x-(fx/fx1);

    
    as_.publishFeedback(feedback_);

    

    if(data_count_ > goal_ || feedback_.x1 - x < 0.000001 ) 
    {
      result_.x1 = feedback_.x1;
      


      if(result_.x1 - x > 0.000001)
      {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        //set the action state to aborted
        as_.setAborted(result_);
      }
      else 
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
        break;
      }
    } 
   x = feedback_.x1;  
  }
}

protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_task1::NewtonRaphsonAction> as_;
  std::string action_name_;
  
  actionlib_task1::NewtonRaphsonFeedback feedback_;
  actionlib_task1::NewtonRaphsonResult result_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "newtonraphson");

  int data_count_, goal_;
  float x; 

  NewtonRaphsonAction newtonraphson(ros::this_node::getName());
  ros::spin();

  return 0;
}