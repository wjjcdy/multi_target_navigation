#include <navigation_test/multi_target.h>
namespace robot_ctrl {
    NavigationMultiTarget::NavigationMultiTarget(void){

        ros::NodeHandle param_n("~");
        param_n.param<int>("pose_number", pose_number_, 2);
        goal_pose_list_.resize(pose_number_);
        param_n.param<double>("pose1_x", goal_pose_list_[0].x,0);
        param_n.param<double>("pose1_y", goal_pose_list_[0].y,0);
        param_n.param<double>("pose1_theta", goal_pose_list_[0].theta,0);

        if(pose_number_>=2){
            param_n.param<double>("pose2_x", goal_pose_list_[1].x,0);
            param_n.param<double>("pose2_y", goal_pose_list_[1].y,0);
            param_n.param<double>("pose2_theta", goal_pose_list_[1].theta,0);
        } 
        if(pose_number_>=3) {
            param_n.param<double>("pose3_x", goal_pose_list_[2].x,0);
            param_n.param<double>("pose3_y", goal_pose_list_[2].y,0);
            param_n.param<double>("pose3_theta", goal_pose_list_[2].theta,0);
        }
        if(pose_number_>=4) {
            param_n.param<double>("pose4_x", goal_pose_list_[3].x,0);
            param_n.param<double>("pose4_y", goal_pose_list_[3].y,0);
            param_n.param<double>("pose4_theta", goal_pose_list_[3].theta,0);
        }
        ac_ = new MoveBaseClient("move_base", true);
        index_current_ = 0;
    }

    NavigationMultiTarget::~NavigationMultiTarget(void) {
        delete ac_;
    }

    void NavigationMultiTarget::navigationRun(void) {
        if(navi_status_ == CANCEL_NAVI || navi_status_ == NAVI_SUCCESS || navi_status_ == NAVI_FAILED)
        {   
            geometry_msgs::Pose2D goal_pose;
            goal_pose = goal_pose_list_[index_current_];
            int status = setGoal(goal_pose);
            if(status) {
                navi_status_ = START_NAVI;
                index_current_++;
                if(index_current_ >= pose_number_) {
                    index_current_ = 0;
                }
            }
        }
    }

    // for setGoal function: 开始导航调用此函数
    void NavigationMultiTarget::activeCb(void)
    {
        // logger_.i("Goal just went active！");
        ROS_INFO("Goal just went active！");
        
        // update navi status
        navi_status_ = ON_NAVI;
    }

    // for setGoal function: 导航过程中一直调用此函数
    void NavigationMultiTarget::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
    {
        // logger_.i("FeedBack(%.2f, %.2f)", feedback->base_position.pose.position.x, 
                                        // feedback->base_position.pose.position.y);
    }

    // for setGoal function: 到达目标点调用此函数
    void NavigationMultiTarget::doneCb(const actionlib::SimpleClientGoalState& state,
                            const move_base_msgs::MoveBaseResultConstPtr& result)
    {   
        // logger_.i("Navi result: %s", state.toString().c_str());
        // update navi status
        if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // logger_.i("Navigation success!");
            ROS_INFO("Navigation success!");
            navi_status_ = NAVI_SUCCESS;
        }
        else {
            ROS_WARN("Navigation fail!");
            navi_status_ = NAVI_FAILED;
        }
    }

    bool NavigationMultiTarget::setGoal(geometry_msgs::Pose2D goal_pose)
    {    
        geometry_msgs::Quaternion pose_orientation = tf::createQuaternionMsgFromYaw(goal_pose.theta*M_1_PI/180); 

        //wait for the action server to come up 
        // while(!ac->waitForServer(ros::Duration(2.0)))
        // { 
        //     ROS_WARN("Waiting for the move_base action server to come up..."); 
        // }
        if(!ac_->waitForServer(ros::Duration(1.0))) {
            // logger_.e("Connected to move base server fail when sending goal."); 
            ROS_WARN("Connected to move base server fail when sending goal");
            return false;
        }

        move_base_msgs::MoveBaseGoal goal; 
        //we'll send a goal to the robot to move 1 meter forward 
        goal.target_pose.header.frame_id = "map"; 
        goal.target_pose.header.stamp = ros::Time::now(); 
        goal.target_pose.pose.position.x = goal_pose.x;
        goal.target_pose.pose.position.y = goal_pose.y; 
        goal.target_pose.pose.position.z = 0;  
        goal.target_pose.pose.orientation = pose_orientation;
        
        // logger_.i("goal_pose %.2f %.2f %.2f", goal_pose.x, goal_pose.y, goal_pose.theta);
        
        //
        ac_->sendGoal(goal, 
                    boost::bind(&NavigationMultiTarget::doneCb, this, _1, _2), 
                    boost::bind(&NavigationMultiTarget::activeCb, this), 
                    boost::bind(&NavigationMultiTarget::feedbackCb, this, _1)); 
        return true;
    }

    void NavigationMultiTarget::cancelGoal(void)
    {
        // out_of_charge_hub_lock_ = false;

        ac_->cancelGoal();
        // logger_.i("Canncel navi.");
        // // update navi status
        // navi_status_ = NaviStatus::CANCEL_NAVI;
        // is_cancel_hand_ = true;
    }
}

// int main(int argc, char** argv){
//   ros::init(argc, argv, "simple_navigation_goals");

//   //tell the action client that we want to spin a thread by default
//   MoveBaseClient ac("move_base", true);

//   //wait for the action server to come up
//   while(!ac.waitForServer(ros::Duration(5.0))){
//     ROS_INFO("Waiting for the move_base action server to come up");
//   }

//   move_base_msgs::MoveBaseGoal goal;

//   //we'll send a goal to the robot to move 1 meter forward
//   goal.target_pose.header.frame_id = "base_link";
//   goal.target_pose.header.stamp = ros::Time::now();

//   goal.target_pose.pose.position.x = 1.0;
//   goal.target_pose.pose.orientation.w = 1.0;

//   ROS_INFO("Sending goal");
//   ac.sendGoal(goal);

//   ac.waitForResult();

//   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     ROS_INFO("Hooray, the base moved 1 meter forward");
//   else
//     ROS_INFO("The base failed to move forward 1 meter for some reason");

//   return 0;
// }