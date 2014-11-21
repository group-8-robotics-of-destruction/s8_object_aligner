#include <ros/ros.h>

#include <s8_common_node/Node.h>
#include <s8_object_aligner/object_aligner_node.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <s8_msgs/DistPose.h>
#include <geometry_msgs/Twist.h>
#include <s8_object_aligner/ObjectAlignAction.h>
#include <s8_motor_controller/StopAction.h>

using namespace s8;
using namespace s8::object_aligner_node;

class ObjectAligner : public Node {
    ros::Subscriber object_dist_pose_subscriber;
    ros::Publisher twist_publisher;
    actionlib::SimpleActionServer<s8_object_aligner::ObjectAlignAction> object_align_action_server;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;

    bool align;
    bool preempted;
    double v;
    double w;

public:
    ObjectAligner() : align(false), preempted(false), v(0.0), w(0.0), object_align_action_server(nh, ACTION_OBJECT_ALIGN, boost::bind(&ObjectAligner::action_execute_object_align_callback, this, _1), false), stop_action(ACTION_STOP, true) {
        object_align_action_server.registerPreemptCallback(boost::bind(&ObjectAligner::object_align_action_cancel_callback, this));
        object_align_action_server.start();

        object_dist_pose_subscriber = nh.subscribe<s8_msgs::DistPose>(TOPIC_OBJECT_DIST_POSE, 1, &ObjectAligner::object_dist_pose_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1);

        ROS_INFO("%s", ACTION_STOP.c_str());
        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");
    }

private:
    void action_execute_object_align_callback(const s8_object_aligner::ObjectAlignGoalConstPtr & object_align_goal) {
        ROS_INFO("STARTED: Object align action started!");
        align = true;
        preempted = false;

        const int timeout = 30; // 30 seconds.
        const int rate_hz = 10;
        ros::Rate rate(rate_hz);
        int ticks = 0;

        while(ros::ok() && align && ticks <= timeout * rate_hz) {
            rate.sleep();
            ticks++;
        }

        if(is_aligned()) {
            s8_object_aligner::ObjectAlignResult object_align_action_result;
            object_align_action_result.aligned = true;
            object_align_action_result.reason = ObjectAlignFinishedReason::ALIGNED;
            object_align_action_server.setSucceeded(object_align_action_result);
            ROS_INFO("SUCCEEDED: Object alignment action succeeded.");
        } else {
            s8_object_aligner::ObjectAlignResult object_align_action_result;
            object_align_action_result.aligned = false;

            if(ticks >= timeout * rate_hz) {
                stop();
                object_align_action_result.reason = ObjectAlignFinishedReason::TIMEOUT;
                ROS_INFO("TIMEOUT: Object alignment action timed out.");
                object_align_action_server.setAborted(object_align_action_result);
            } else if(preempted) {
                object_align_action_result.reason = ObjectAlignFinishedReason::PREEMPTED;
                ROS_INFO("PREEMPTED: Object alignment action preempted.");
                object_align_action_server.setPreempted(object_align_action_result);
            } else {
                object_align_action_result.reason = ObjectAlignFinishedReason::FAILED;
                ROS_INFO("FAILED: Object alignment action failed.");
                object_align_action_server.setAborted(object_align_action_result);
            }
        }
    }

    void object_align_action_cancel_callback() {
        align = false;
        preempted = true;
        stop();
    }

    void object_dist_pose_callback(const s8_msgs::DistPose::ConstPtr & dist_pose) {
        if(!align) {
            return;
        }

        //TODO: set/update v and w according to the given dist_pose.

        ROS_INFO("Aligning to object...");

        if(is_aligned()) {
            stop();
            align = false;
            return;
        }

        if(is_invalid_align_state()) {
            stop();
            align = false;
            return;
        }

        publish_twist();
    }

    void publish_twist() {
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = v;
        twist_msg.angular.z = w;
        twist_publisher.publish(twist_msg);
    }

    /**
     * Tells if the robot is in an aligned position to the object which mean that the aligner should let master take over again.
     */
    bool is_aligned() {
        //TODO: implement me.
        return false;
    }

    /**
     * Tells if the robot has lost the object or somehow is in another invalid state, which means that the alignment failed and master needs to take over.
     */
    bool is_invalid_align_state() {
        //TODO: implement me.
        return false;
    }

    void stop() {
        ROS_INFO("Stopping...");
        s8_motor_controller::StopGoal goal;
        goal.stop = true;
        stop_action.sendGoal(goal);

        bool finised_before_timeout = stop_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = stop_action.getState();
            ROS_INFO("Stop action finished. %s", state.toString().c_str());
        } else {
            ROS_WARN("Stop action timed out.");
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    ObjectAligner object_aligner;
    ros::spin();
    return 0;
}
