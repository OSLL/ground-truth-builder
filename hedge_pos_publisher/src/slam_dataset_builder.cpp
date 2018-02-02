#include <cstdio>
#include <string>
#include <vector>
#include <tf/transform_listener.h>
#include <ros/ros.h>

#define _USE_MATH_DEFINES

struct EchoListenerData
{
    std::vector<tf::Vector3> positions;
    ros::Time stamp;
};

std::ostream& operator<<(std::ostream& out, const EchoListenerData& data)
{ 
    for(auto& pos: data.positions) {  
        out << "[" <<
        pos.getX() << " " <<
        pos.getY() << " " << 
        data.stamp.toSec() << "]";
    }
    return out;
}

class EchoListener
{
public:  
    EchoListener(const std::string& sourceFrame, const std::vector<std::string>& targetFrames):
        sourceFrame_(sourceFrame),
        targetFrames_(targetFrames)
    {
        for(auto& target: targetFrames_) {
            tf_.waitForTransform(sourceFrame_, target, 
                          ros::Time(), ros::Duration(1.0));
        }
    };

    EchoListener(const std::string& sourceFrame, const std::string& targetFrame):
        EchoListener(sourceFrame, std::vector<std::string>{targetFrame})
    {};

    ~EchoListener() {};

    EchoListenerData GetPosition() 
    {
      EchoListenerData data;

      tf::StampedTransform echoTransform;

      ros::Time last_time = ros::Time();
      for (auto& target: targetFrames_) {
        tf_.lookupTransform(sourceFrame_, target, last_time, echoTransform);
        data.positions.emplace_back(echoTransform.getOrigin());
        last_time = echoTransform.stamp_;
      }
      data.stamp = last_time;
      return data;
    }

private:
    tf::TransformListener tf_;

    std::string sourceFrame_;
    std::vector<std::string> targetFrames_;
};

void printEntry(const EchoListenerData& v1, const EchoListenerData& v2)
{
    std::cout.precision(3);
    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout << v1 << " " << v2 << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_dataset_builder");
    
    ros::NodeHandle nh;
    const double rate_hz = 10.0; // todo get from argv
    ros::Rate loop_rate(rate_hz);
    
    const std::string slamSourceFrame = "/map"; // todo get from argv
    const std::string slamTargetFrame = "/base_link"; // todo get from argv
    const std::string beaconSourceFrame = "/beacon_base"; // todo get from argv
    const std::vector<std::string> beaconTargetFrames = {"/beacon2", "/beacon10"};

    EchoListener echoSlamListener(slamSourceFrame, slamTargetFrame);
    EchoListener echoBeaconListener(beaconSourceFrame, beaconTargetFrames);

    while(ros::ok())
    {
        try
        {
            EchoListenerData slamData = echoSlamListener.GetPosition();
            EchoListenerData beaconData = echoBeaconListener.GetPosition();
            printEntry(slamData, beaconData);
        }
        catch (const tf::TransformException& ex)
        {
            std::cerr << "Failure at " << ros::Time::now() << "\n";
            std::cerr << "Exception thrown:" << ex.what() << "\n" << std::flush; 
        }
        
        ros::spinOnce();
        loop_rate.sleep();
  } 

  return 0;
}
