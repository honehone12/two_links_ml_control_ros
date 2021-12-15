#include "two_links_ml_control/serial_port_buffer.hpp"
#include "ros/ros.h"
#include "two_links_msgs/Float2.h"
#include "two_links_msgs/Byte2.h"

#define SERIAL_END_POINT_BAUDRATE 115200u

namespace two_links_ml_cotrol
{
/////////////////////////////////
// this node gets servo angle 
// in 60 FPS!
// and send cmd 
// on message recieved

class SerialEndPoint
{
private:
    ros::Publisher angle_publisher;
    ros::Subscriber cmd_subscriber;
    serial_communication::SerialPortBuffer serial_port_buffer;
    serial_communication::SerialDataDescription float_data_description
    {
        8, 4, 2, SERIAL_COMMUNICATION_FLOAT_TYPE
    };
    serial_communication::SerialDataDescription byte_data_description
    {
        2, 1, 2, SERIAL_COMMUNICATION_BYTE_TYPE
    };
    std::vector<uint8_t> feedback_buffer;
    std::vector<uint8_t> cmd_buffer;
    two_links_msgs::Float2 angle_msg_cache;

public:
    SerialEndPoint(
        std::string& port_name,
        ros::NodeHandle& node_handle
    ) :
        angle_publisher(
            node_handle.advertise<two_links_msgs::Float2>(
                "servo_angles",
                1,
                false
            )
        ),
        cmd_subscriber(
            node_handle.subscribe(
                "cmd",
                1,
                &SerialEndPoint::onMessageRecieved,
                this
            )
        ),
        serial_port_buffer(
            port_name,
            SERIAL_END_POINT_BAUDRATE
        )
    { 
        feedback_buffer.resize(8, 0);
        cmd_buffer.resize(2, 0);
        ////////////////////////////////
        // init value should be 
        // reconfiged zero angles. not 0.0f. 
        angle_msg_cache.x = 0.0f;
        angle_msg_cache.y = 0.0f;
    }
    ~SerialEndPoint()
    { }

    void onMessageRecieved(const two_links_msgs::Byte2ConstPtr& msg);
    bool readFeedback();
    void publishAngleMsg()
    {
        angle_publisher.publish(angle_msg_cache);
    }
};

void SerialEndPoint::onMessageRecieved(const two_links_msgs::Byte2ConstPtr& msg)
{
    cmd_buffer[0] = msg->x;
    cmd_buffer[1] = msg->y;

    /////////////////////////////////////////
    // write directly here ??
    // how to test ??
    if(
        serial_port_buffer.write(
            cmd_buffer,
            byte_data_description
        )
    )
    {
        ROS_ERROR("failed to write cmd.");
    }
}

bool SerialEndPoint::readFeedback()
{
    if(serial_port_buffer.update(float_data_description.byte_array_length))
    {
        uint8_t byte_container(0);
        bool is_exoected_data_description(true);
        for (size_t i = 0; i < SERIAL_COMMUNICATION_DATA_DESCRIPTION_LEN; i++)
        {
            if(serial_port_buffer.readByte(&byte_container))
            {
                if(byte_container != float_data_description.bin[i])
                {
                    is_exoected_data_description = false;
                    ROS_WARN("unexpected byte %x", byte_container);
                }
            }
            else
            {
                ROS_ERROR("failed to read.");
                return false;
            }
        }
        
        if(
            is_exoected_data_description &&
            serial_port_buffer.read(
                feedback_buffer,
                float_data_description.byte_array_length
            )
        )
        {
            serial_communication::FloatTypeExchanger feedback_x; // lower
            serial_communication::FloatTypeExchanger feedback_y; // uper

            for (size_t i = 0; i < 4; i++)
            {
                feedback_x.bin[i] = feedback_buffer[i];
                feedback_y.bin[i] = feedback_buffer[i + 4];
            }
            
            if(
                isfinite(feedback_x.float_data) && 
                isfinite(feedback_y.float_data)
            )
            {
                angle_msg_cache.x = feedback_x.float_data;
                angle_msg_cache.y = feedback_y.float_data;
                ROS_INFO("feedback x %f", feedback_x.float_data);
                ROS_INFO("feedback y %f", feedback_y.float_data);
                return true;
            }
            else
            {
                ROS_WARN("recieved broken values.");
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        ROS_ERROR("failed to read.");
        return false;
    }
}

} //ns

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "serial_end_point"
    );

    std::string port_name("/dev/ttyACM0");
    if(argc == 2)
    {
        port_name = argv[1];
    }
    else
    {
        ROS_WARN("port name was not given. remaining default. (/dev/ttyACM0)");
    }

    ros::NodeHandle node_handle;
    ros::Rate loop_rate(60.0);
    two_links_ml_cotrol::SerialEndPoint ser_end_point(
        port_name,
        node_handle
    );

    while (node_handle.ok())
    {
        if(ser_end_point.readFeedback())
        {
            ser_end_point.publishAngleMsg();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}