#include <ros/ros.h>

#include <s8_common_node/Node.h>
#include <s8_object_aligner/object_aligner_node.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <s8_msgs/DistPose.h>
#include <geometry_msgs/Twist.h>
#include <s8_object_aligner/ObjectAlignAction.h>
#include <s8_motor_controller/StopAction.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <cstdlib>

using namespace s8;
using namespace s8::object_aligner_node;

class ObjectAligner : public Node {
    ros::Subscriber object_dist_pose_subscriber;
    ros::Publisher twist_publisher;
    actionlib::SimpleActionServer<s8_object_aligner::ObjectAlignAction> object_align_action_server;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;

    double min_dist;
    double max_dist;
    double min_angle;
    double max_angle;
    double dist_speed, turn_speed;
    double distance;
    double pose;

    bool align;
    bool preempted;
    double v;
    double w;

public:
    ObjectAligner() : distance(0.0), pose(0.0),dist_speed(0.0),turn_speed(0.0), align(false), preempted(false), v(0.0), w(0.0), object_align_action_server(nh, ACTION_OBJECT_ALIGN, boost::bind(&ObjectAligner::action_execute_object_align_callback, this, _1), false), stop_action(ACTION_STOP, true) {
        add_params();
        ROS_INFO("dist_speed: %lf, turn_speed: %lf", dist_speed, turn_speed);

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
        stop(); //TODO remove me?
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
        distance = (double)dist_pose->dist;
        pose     = (double)dist_pose->pose;

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

        ROS_INFO("distance: %lf, min_dist: %lf, dist_speed: %lf", distance, min_dist, dist_speed);
        if(distance < min_dist) {
            v = -dist_speed;
        }
        else if(distance > max_dist) {
            v = dist_speed;
        }
        else{
            v = 0;
            ROS_INFO("v = 0");
        }

/*
        if(pose < min_angle) {
            w = turn_speed;
        }
        else if (pose > max_angle) {
            w = -turn_speed;
        }
        else {
            w = 0;
        }
*/
        w = 0;

        ROS_INFO("Aligning! v: %lf, w: %lf, distance: %lf, angle: %lf", v, w, distance, pose);
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
        return (distance >= min_dist && distance <= max_dist);// && pose >= min_angle) && pose <= max_angle;
    }

    /**
     * Tells if the robot has lost the object or somehow is in another invalid state, which means that the alignment failed and master needs to take over.
     */
    bool is_invalid_align_state() {
        return distance < 0;
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

    void add_params() {
        const char * home = ::getenv("HOME");
        boost::property_tree::ptree pt;
        boost::property_tree::read_json(home + CONFIG_DOC, pt);
        // DISTANCE
        min_dist = pt.get<double>("min_dist");
        max_dist = pt.get<double>("max_dist");
        // ANGLE
        min_angle = pt.get<double>("min_angle");
        max_angle = pt.get<double>("max_angle");
        // TURN
        dist_speed = pt.get<double>("dist_speed");
        turn_speed = pt.get<double>("turn_speed");
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    ObjectAligner object_aligner;
    ros::spin();
    return 0;
}
