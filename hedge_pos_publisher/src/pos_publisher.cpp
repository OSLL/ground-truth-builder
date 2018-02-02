#include <ros/ros.h>
#include <hedge_pos_publisher/hedge_pos.h>
extern "C" {
    #include "hedge_pos_publisher/hedgehog_proxy.h"
}

using hedge_pos_publisher::hedge_pos; // hedge position message


bool terminateProgram = false;
void CtrlHandler(int signum)
{
	terminateProgram=true;
}

hedge_pos get_hedge_pos(HedgehogProxy& hedge_proxy) {
    hedge_pos res;
	while((!terminateProgram) && (!hedge_proxy.terminationRequired())) {
		PositionValue pos = hedge_proxy.get_position();
		if (pos.ready) {
            res.address = pos.address;
            res.timestamp = pos.timestamp;
            res.x = pos.x / 1000.;
            res.y = pos.y / 1000.;
            res.z = pos.z / 1000.;			
            break;
		}
	}
	return res;
}

int main(int argc, char** argv)
{
    signal (SIGINT, CtrlHandler);
    signal (SIGQUIT, CtrlHandler);

    ros::init(argc, argv, "pos_publisher");
    ros::NodeHandle main_node;

    HedgehogProxy hedge_proxy;

    std::string default_topic = "/beacons";
    double rate = 10.0; // atof(argv[0]);
    
    ros::Rate loop_rate(rate);


    ros::Publisher pos_publisher = main_node.advertise<hedge_pos>(default_topic, 1000);
    while ((!terminateProgram) && (!hedge_proxy.terminationRequired()) && ros::ok())
    {   
        if (!hedge_proxy.haveNewValues())
            continue;

        hedge_pos tmp = get_hedge_pos(hedge_proxy);
        ROS_INFO("address: %d, timestamp: %d, [X=%.3f, Y= %.3f, Z=%.3f]", 	
            (int) tmp.address,
            (int) tmp.timestamp, 
            (float) tmp.x, (float) tmp.y, (float) tmp.z);
        pos_publisher.publish(tmp);  
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

