#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <hedge_pos_publisher/hedge_pos.h>
#include <tf/transform_broadcaster.h>

using hedge_pos_publisher::hedge_pos; // hedge position message

// tf broadcaster
const std::string parentTfName = "map";
tf::TransformBroadcaster *br; 

bool base_coord_set = false;
tf::Transform base_coord;

void hedge_pos_callback(const hedge_pos& pos) {
    tf::Transform pos_transform;
    pos_transform.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
		
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    pos_transform.setRotation(q);
    // ROS_INFO("send! \n");

    // check if base_coord is set
    if (!base_coord_set) {
        base_coord = pos_transform;
        base_coord_set = true;
        return;
    }

    br->sendTransform(tf::StampedTransform(base_coord, ros::Time::now(), parentTfName.c_str(), "beacon_base"));
    std::string tf_name = "beacon" + std::to_string(pos.address);
    br->sendTransform(tf::StampedTransform(pos_transform, //ros::Time(pos.timestamp),  
                                            ros::Time::now(),  
                                            parentTfName.c_str(), tf_name));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcaster");

    if (argc < 1 || (argc > 1 && argc != 4) ) 
    {
        printf("Usage: beacon_data_broadcaster_node rate [base.x base.y base.z]");
        return -1;
    }

    ros::NodeHandle main_node;
    br = new tf::TransformBroadcaster();

    const std::string default_hedge_topic = "/beacons";
    ros::Subscriber sub = main_node.subscribe(default_hedge_topic, 1000, &hedge_pos_callback);

    // setting up base coordinate system
    if (argc == 4) 
    {
        base_coord.setOrigin(tf::Vector3(atof(argv[1]), atof(argv[2]), atof(argv[3])));
		
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        base_coord.setRotation(q);

        base_coord_set = true;
    } 

    // double rate = 10.0; // atof(argv[0]);
    //std::cout << "Rate: " << rate << std::flush;
    //ros::Rate loop_rate(atof(argv[0]));

    // ros::Rate loop_rate(rate);
    // while (ros::ok())
    // {
    // br->sendTransform(tf::StampedTransform(base_coord, ros::Time::now(), parentTfName.c_str(), "beacon_base"));

    ros::spin();
    // loop_rate.sleep();
    // }
    return 0;
}

