#include "onnxruntime_cxx_api.h"
#include "ros/ros.h"
#include "two_links_msgs/Float2.h"
#include "two_links_msgs/Byte2.h"
#include "geometry_msgs/PoseStamped.h"

namespace two_links_ml_control
{
//////////////////////////////////////////////////
// this node gets position from oakd and
// servo angles from serial endpoint.
// and publishe cmd.
// use message filter or not ??

class AIInferencePoint
{
private:
    
public:
    AIInferencePoint(ros::NodeHandle& node_handle)
    { }
    ~AIInferencePoint()
    { }
};

} // ns

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "ai_inference_point"
    );

    ros::NodeHandle node_handle;
}