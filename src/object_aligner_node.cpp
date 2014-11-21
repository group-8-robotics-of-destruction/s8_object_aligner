#include <ros/ros.h>

#include <s8_common_node/Node.h>
#include <s8_object_aligner/object_aligner_node.h>
#include <s8_msgs/DistPose.h>
#include <geometry_msgs/Twist.h>

using namespace s8;
using namespace s8::object_aligner_node;

class ObjectAligner : public Node {
    ros::Subscriber object_dist_pose_subscriber;
    ros::Publisher twist_publisher;

    double v;
    double w;

public:
    ObjectAligner() : v(0.0), w(0.0) {
        object_dist_pose_subscriber = nh.subscribe<s8_msgs::DistPose>(TOPIC_OBJECT_DIST_POSE, 1, &ObjectAligner::object_dist_pose_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1);
    }

private:
    void object_dist_pose_callback(const s8_msgs::DistPose::ConstPtr & dist_pose) {
        //TODO: set/update v and w according to the given dist_pose.
        publish_twist();
    }

    void publish_twist() {
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = v;
        twist_msg.angular.z = w;
        twist_publisher.publish(twist_msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    ObjectAligner object_aligner;
    ros::spin();
    return 0;
}
