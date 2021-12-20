#include "onnxruntime_cxx_api.h"
#include "ros/ros.h"
#include "two_links_msgs/Float2Stamped.h"
#include "two_links_msgs/Byte2.h"
#include "geometry_msgs/PoseStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

#define AI_INFERENCE_POINT_SCALE_MODIFIER 10.0f

namespace two_links_ml_control
{
//////////////////////////////////////////////////
// this node gets position from oakd and
// servo angles from serial endpoint.
// and publishe cmd.
// not sure angle60FPS & camera30FPS => possible ??

typedef message_filters::sync_policies::ApproximateTime
<two_links_msgs::Float2Stamped, geometry_msgs::PoseStamped> InferenceSyncPolicy;

class AIInferencePoint
{
private:
    const char* model_path = "/home/marsh/two_links_ml_control/two_links_ml_control_ros/resource/LookingLight.onnx";
    const char* input_name = "obs_0";
    const char* output_name = "continuous_actions";

    Ort::Env ort_env;
    Ort::SessionOptions session_options;
    Ort::Session ort_session;
    const std::array<int64_t, 2> input_shape { 1, 5 };
    const std::array<int64_t, 2> output_shape { 1, 38 };

    ros::Publisher cmd_publisher;
    message_filters::Subscriber<two_links_msgs::Float2Stamped> angles_subscriber;
    message_filters::Subscriber<geometry_msgs::PoseStamped> position_subscriber;
    message_filters::Synchronizer<InferenceSyncPolicy> synchronizer;
    two_links_msgs::Byte2Ptr cmd_msg;

public:
    AIInferencePoint(ros::NodeHandle& node_handle, const char* instance_name) :
        ort_env(
            OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING,
            instance_name
        ),
        ort_session(
            ort_env,
            model_path,
            session_options
        ),
        cmd_publisher(
            node_handle.advertise<two_links_msgs::Byte2>(
                "cmd",
                1u
            )
        ),
        angles_subscriber(
            node_handle,
            "servo_angles",
            1u
        ),
        position_subscriber(
            node_handle,
            "/oakd_yolov4_monitor/calibrated_single_pose",
            1u
        ),
        synchronizer(
            InferenceSyncPolicy(10u),
            angles_subscriber,
            position_subscriber
        )
    { 
        synchronizer.registerCallback(
            boost::bind(
                &AIInferencePoint::onSynchronizedMessageRecieved,
                this,
                _1,
                _2
            )
        );
        cmd_msg = boost::make_shared<two_links_msgs::Byte2>();
    }
    ~AIInferencePoint()
    { }

    void runInference(
        std::array<float, 5>& input_array,
        std::array<float, 38>& output_array,
        const two_links_msgs::Byte2Ptr& cmd_msg
    );

    void onSynchronizedMessageRecieved(
        const two_links_msgs::Float2StampedConstPtr& angles_msg,
        const geometry_msgs::PoseStampedConstPtr& pose_msg
    );
};

void AIInferencePoint::runInference(
    std::array<float, 5ul>& input_array,
    std::array<float, 38ul>& output_array,
    const two_links_msgs::Byte2Ptr& cmd_msg
)
{
    const Ort::RunOptions run_options;
    const auto memory_info(
        Ort::MemoryInfo::CreateCpu(
            OrtDeviceAllocator,
            OrtMemTypeCPU
        )
    );

    Ort::Value input_tensor(
        Ort::Value::CreateTensor<float>(
            memory_info,
            input_array.data(),
            input_array.size(),
            input_shape.data(),
            input_shape.size()
        )
    );

    Ort::Value output_tensor(
        Ort::Value::CreateTensor<float>(
            memory_info,
            output_array.data(),
            output_array.size(),
            output_shape.data(),
            output_shape.size()
        )
    );

    ort_session.Run(
        run_options,
        &input_name,
        &input_tensor,
        1lu,
        &output_name,
        &output_tensor,
        1lu
    );

    float max_so_far(FLT_MIN);
    uint8_t act_idx(9); // stopping value 90
    for (uint8_t i = 0; i < 19; i++)
    {
        if(output_array[i] > max_so_far)
        {
            max_so_far = output_array[i];
            act_idx = i;
        }
    }
    cmd_msg->x = act_idx * 10;

    max_so_far = FLT_MIN;
    act_idx = 9;
    for (uint8_t i = 0; i < 19; i++)
    {
        if(output_array[i + 19] > max_so_far)
        {
            max_so_far = output_array[i + 19];
            act_idx = i;
        }
    }
    cmd_msg->y = act_idx * 10;
}

void AIInferencePoint::onSynchronizedMessageRecieved
(
    const two_links_msgs::Float2StampedConstPtr& angles_msg,
    const geometry_msgs::PoseStampedConstPtr& pose_msg
)
{
    std::array<float, 5ul> input_array;
    std::array<float, 38ul> output_array { 0.0f };
    input_array[0] = angles_msg->value.x;
    input_array[1] = angles_msg->value.y;
    input_array[2] = pose_msg->pose.position.x * AI_INFERENCE_POINT_SCALE_MODIFIER;
    input_array[3] = pose_msg->pose.position.y * AI_INFERENCE_POINT_SCALE_MODIFIER;
    input_array[4] = pose_msg->pose.position.z * AI_INFERENCE_POINT_SCALE_MODIFIER;

    runInference(
        input_array,
        output_array,
        cmd_msg
    );
    cmd_publisher.publish(cmd_msg);
    ROS_INFO("cmd x %d y %d", cmd_msg->x, cmd_msg->y);
}

} // ns

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "ai_inference_point"
    );

    ros::NodeHandle node_handle;
    two_links_ml_control::AIInferencePoint inference_point(
        node_handle,
        "two_links_inference"
    );

    ros::spin();

    return 0;
}