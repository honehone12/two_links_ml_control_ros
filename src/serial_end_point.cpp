#include "two_links_ml_control/serial_port_buffer.hpp"
#include "ros/ros.h"
#include "two_links_msgs/Float2Stamped.h"
#include "two_links_msgs/Byte2.h"

#define SERIAL_END_POINT_BAUDRATE 115200u
#define SERVO_CONTROLLER_ZERO_ANGLE_RECONFIG_X 147.112137f
#define SERVO_CONTROLLER_ZERO_ANGLE_RECONFIG_Y 312.354309f

namespace two_links_ml_cotrol
{
/////////////////////////////////
// this node gets servo angle 
// in 60 FPS
// and send cmd 
// on message recieved

class ServoController
{
    struct Limitations
    {
        float x_min;
        float x_max;
        float y_min;
        float y_max;
    };

private:
    const Limitations limits { -90.0f, 90.0f, -90.0f, 90.0f };
    float signedAngleX;
    float signedAngleY;
public:
    ServoController() : signedAngleX(0.0f), signedAngleY(0.0f)
    { }
    ~ServoController()
    { }
    
    ///////////////////////////////
    // this does not work well.
    bool OutOfLimit(const two_links_msgs::Byte2ConstPtr& msg)
    {
        bool out(false);
        if(msg->x > 90)
        {
            out = signedAngleX <= limits.x_min;
        }
        else if(msg->x < 90)
        {
            out = signedAngleX >= limits.x_max;
        }
        if(msg->y > 90)
        {
            out = signedAngleY <= limits.y_min;
        }
        else if(msg->y < 90)
        {
            out = signedAngleY >= limits.y_max;
        }
        ROS_INFO("signed angle X %f Y %f : out %d", signedAngleX, signedAngleY, out);

        return out;
    }

    void initAngleMessage(const two_links_msgs::Float2StampedPtr& msg)
    {
        msg->header.frame_id = "";
        msg->header.stamp = ros::Time::now();
        msg->value.x = SERVO_CONTROLLER_ZERO_ANGLE_RECONFIG_X;
        msg->value.y = SERVO_CONTROLLER_ZERO_ANGLE_RECONFIG_Y;
    }

    void reinterpretAngleMessage(const two_links_msgs::Float2StampedPtr& msg)
    {
        msg->header.stamp = ros::Time::now();
        float raw(msg->value.x);
        msg->value.x = fmod(
            raw - SERVO_CONTROLLER_ZERO_ANGLE_RECONFIG_X + 360.0f,
            360.0f
        );
        raw = msg->value.y;
        msg->value.y = fmod(
            raw - SERVO_CONTROLLER_ZERO_ANGLE_RECONFIG_Y + 360.0f,
            360.0f
        );

        signedAngleX = msg->value.x > 180.0f ? msg->value.x - 360.0f : msg->value.x;
        signedAngleY = msg->value.y > 180.0f ? msg->value.y - 360.0f : msg->value.y;

        //ROS_INFO("angle x %f y %f", msg->value.x, msg->value.y);
    }
};

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
    two_links_msgs::Float2StampedPtr angle_msg_cache;
    ServoController servo_controller;

public:
    SerialEndPoint(
        std::string& port_name,
        ros::NodeHandle& node_handle
    ) :
        angle_publisher(
            node_handle.advertise<two_links_msgs::Float2Stamped>(
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
        angle_msg_cache = boost::make_shared<two_links_msgs::Float2Stamped>();
        servo_controller.initAngleMessage(angle_msg_cache);
    }
    ~SerialEndPoint()
    { }

    void onMessageRecieved(const two_links_msgs::Byte2ConstPtr& msg);
    bool readFeedback();
    void publishAngleMsg()
    {
        servo_controller.reinterpretAngleMessage(angle_msg_cache);
        angle_publisher.publish(angle_msg_cache);
    }

    void close()
    {
        cmd_buffer[0] = 90;
        cmd_buffer[1] = 90;
        serial_port_buffer.write(
            cmd_buffer,
            byte_data_description
        );
        serial_port_buffer.close();
    }
};

void SerialEndPoint::onMessageRecieved(const two_links_msgs::Byte2ConstPtr& msg)
{
    if(servo_controller.OutOfLimit(msg))
    {
        cmd_buffer[0] = 90;
        cmd_buffer[1] = 90;
    }
    else
    {
        cmd_buffer[0] = msg->x;
        cmd_buffer[1] = msg->y;
    }

    if(
        !serial_port_buffer.write(
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
        bool is_expected_data_description(true);
        for (size_t i = 0; i < SERIAL_COMMUNICATION_DATA_DESCRIPTION_LEN; i++)
        {
            if(serial_port_buffer.readByte(&byte_container))
            {
                if(byte_container != float_data_description.bin[i])
                {
                    is_expected_data_description = false;
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
            is_expected_data_description &&
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
                angle_msg_cache->value.x = feedback_x.float_data;
                angle_msg_cache->value.y = feedback_y.float_data;
                //ROS_INFO("feedback raw x %f", angle_msg_cache->value.x);
                //ROS_INFO("feedback raw y %f", angle_msg_cache->value.y);
                return true;
            }
            else
            {
                ROS_WARN("recieved broken values.");
                // publish cached value.
                return true;
            }
        }
    }
    
    ROS_ERROR("failed to read.");
    return false;
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
        // or publish cached value anyway ??
        if(ser_end_point.readFeedback())
        {
            ser_end_point.publishAngleMsg();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ser_end_point.close();

    return 0;
}