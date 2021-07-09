#ifndef _MULTI_TARGET_HPP_
#define _MULTI_TARGET_HPP_
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h> 
#include <vector>

namespace robot_ctrl {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        enum NaviStatus
    {
        CANCEL_NAVI         = 0,    // 结束导航
        START_NAVI          = 1,
        ON_NAVI             = 2,    // 导航中
        NAVI_SUCCESS        = 3,    // 到达目标点 
        NAVI_FAILED         = 4
    };

    class NavigationMultiTarget {
        public:
        NavigationMultiTarget(void);
        ~NavigationMultiTarget(void);
        void navigationRun(void);
        bool setGoal(geometry_msgs::Pose2D goal_pose);
        void cancelGoal(void);

        private:
        void activeCb(void);
        void doneCb(const actionlib::SimpleClientGoalState& state,
                            const move_base_msgs::MoveBaseResultConstPtr& result);
        void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
        
    

        private:
        MoveBaseClient* ac_;                        // for set navi goal
        NaviStatus navi_status_ = NaviStatus::CANCEL_NAVI;  // navi status
        std::vector<geometry_msgs::Pose2D> goal_pose_list_;
        int index_current_;
        int pose_number_;

    };
}
#endif