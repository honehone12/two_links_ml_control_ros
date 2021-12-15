#ifndef SERIAL_PORT_BUFFER_HPP_
#define SERIAL_PORT_BUFFER_HPP_

#include "boost/asio.hpp"

#include <vector>
#include <deque>
#include <cmath>

#define SERIAL_COMMUNICATION_MAX_FALSE_COUNT 100
#define SERIAL_COMMUNICATION_HEADER_SIZE 8
#define SERIAL_COMMUNICATION_DATA_DESCRIPTION_LEN 4

#define SERIAL_COMMUNICATION_BYTE_TYPE 0x00
#define SERIAL_COMMUNICATION_FLOAT_TYPE 0x01

namespace serial_communication
{

typedef boost::asio::serial_port_base::parity::type ParityType;
typedef boost::asio::serial_port_base::stop_bits StopBitsType;

union FloatTypeExchanger
{
    float float_data;
    uint8_t bin[sizeof(float)];
};

// can be better with this setting data.
// this serial port class can know many things in advance with this. 
//ex. update() can check data description.
union SerialDataDescription
{
    struct
    {
        uint8_t byte_array_length;
        uint8_t type_size;
        uint8_t type_length;
        uint8_t data_type;
    };
    uint8_t bin[4];
};

static const std::array<uint8_t, 8> header 
{
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff 
};

class SerialPortBuffer
{
private:
    boost::asio::io_service io_service;
    boost::asio::serial_port serial_port;
    std::deque<uint8_t> frontend_read_buffer;
    std::vector<uint8_t> backend_read_buffer;
    std::vector<uint8_t> backend_write_buffer;

public:
    SerialPortBuffer(
        const std::string& port_name,
        unsigned int baud_rate,
        size_t backend_buffer_size,
        ParityType parity
    );
    ~SerialPortBuffer()
    { }

    size_t size()
    {
        return frontend_read_buffer.size();
    }

    bool available()
    {
        return !frontend_read_buffer.empty();
    }

    bool update(size_t target_byte_arryay_length);
    bool readByte(uint8_t* container_ptr);
    bool read(
        std::vector<uint8_t>& read_buffer,
        unsigned int byte_array_length
    );
    bool write(
        const std::vector<uint8_t>& write_buffer,
        const SerialDataDescription& data_description
    );

    void close()
    {
        if(serial_port.is_open())
        {
            frontend_read_buffer.clear();
            serial_port.close();       
        }
    }

private:
    bool openCheck()
    {
        if(serial_port.is_open())
        {
            return true;
        }
        else
        {
            printf("serial port is closed.");

            return false;
        }
    }

    bool updateBuffer();
};

SerialPortBuffer::SerialPortBuffer(
    const std::string& port_name,
    unsigned int baud_rate,
    size_t backend_buffer_size = 15,
    ParityType parity = ParityType::none
) : serial_port(io_service)
{
    serial_port.open(port_name);
    serial_port.set_option(
        boost::asio::serial_port_base::baud_rate(baud_rate)
    );
    // ensure Arduino default 8N1
    serial_port.set_option(
        boost::asio::serial_port_base::character_size(8u)
    );
    serial_port.set_option(
        boost::asio::serial_port_base::parity(parity)
    );
    serial_port.set_option(
        boost::asio::serial_port_base::stop_bits(StopBitsType::one)
    );

    backend_read_buffer.resize(backend_buffer_size);
}

bool SerialPortBuffer::update(size_t target_byte_arryay_length)
{
    if(!updateBuffer())
    {
        return false;
    }

    unsigned int header_count(0u);
    unsigned int false_count(0u);
    uint8_t byte_container(0);
    while (header_count < SERIAL_COMMUNICATION_HEADER_SIZE)
    {
        if(frontend_read_buffer.size() > 0)
        {
            if(!readByte(&byte_container))
            {
                return false;
            }
            
            
            if(byte_container == 0xff)
            {
                header_count++;
                //printf("read header\n");
            }
            else
            {
                //printf("[WARNING] read unexpected value %d\n", byte_container);

                header_count = 0u;
                false_count++;

                if(false_count >= SERIAL_COMMUNICATION_MAX_FALSE_COUNT)
                {
                    printf("[ERROR] exit with time out.");
                    return false;
                }
            }
        }
        else
        {
            if(!updateBuffer())
            {
                return false;
            }
        }
    }

    while(
        frontend_read_buffer.size() < 
            SERIAL_COMMUNICATION_DATA_DESCRIPTION_LEN + target_byte_arryay_length
    )
    {
        if(!updateBuffer())
        {
            return false;
        }
    }

    return true;
}

bool SerialPortBuffer::readByte(uint8_t* out_ptr)
{
    if(openCheck())
    {
        if(frontend_read_buffer.size() > 0)
        {
            *out_ptr = frontend_read_buffer.front();
            frontend_read_buffer.pop_front();
            
            return true;
        }

        printf("serial port buffer is empty. call update() before read.\n");
    }
    else
    {
        printf("serial port is closed.\n");
    }
    *out_ptr = 0;

    return false;
}

bool SerialPortBuffer::read(
    std::vector<uint8_t>& read_buffer,
    unsigned int byte_array_length
)
{
    if(openCheck())
    {
        if(frontend_read_buffer.size() >= byte_array_length)
        {
            if(!read_buffer.empty())
            {
                read_buffer.clear();
            }

            read_buffer.insert(
                read_buffer.begin(),
                frontend_read_buffer.begin(),
                frontend_read_buffer.begin() + byte_array_length
            );

            frontend_read_buffer.erase(
                frontend_read_buffer.begin(),
                frontend_read_buffer.begin() + byte_array_length
            );

            return true;
        }
        else
        {
            printf("serial port buffer is not enough to read. call update() before read.\n");
        }
    }
    
    return false;
}

bool SerialPortBuffer::write(
    const std::vector<uint8_t>& write_buffer,
    const SerialDataDescription& data_description
)
{
    if(openCheck())
    {
        if(write_buffer.size() != data_description.byte_array_length)
        {
            printf("write buffer size is different from data description.\n");

            return false;
        }
        if(
            data_description.byte_array_length != 
                data_description.type_size * data_description.type_length
        )
        {
            printf("data description has inconsistency.\n");
            
            return false;
        }
        if(
            (data_description.data_type == SERIAL_COMMUNICATION_FLOAT_TYPE && 
                data_description.type_size != sizeof(float)) ||
            (data_description.data_type == SERIAL_COMMUNICATION_BYTE_TYPE &&
                data_description.type_size != sizeof(char))
        )
        {
            printf("data type and data size have inconsistency.\n");

            return false;
        }

        backend_write_buffer.clear();
        backend_write_buffer.insert(
            backend_write_buffer.begin(),
            header.begin(),
            header.end()
        );
        backend_write_buffer.insert(
            backend_write_buffer.end(),
            {
                data_description.byte_array_length,
                data_description.type_size,
                data_description.type_length,
                data_description.data_type
            }
        );
        backend_write_buffer.insert(
            backend_write_buffer.end(),
            write_buffer.begin(),
            write_buffer.end()
        );

        boost::system::error_code err_code;
        size_t written_size(0);
        try
        {
            written_size = boost::asio::write(
                serial_port,
                boost::asio::buffer(backend_write_buffer),
                err_code
            );    
        }
        catch(const boost::system::system_error& e)
        {
            printf("%s", e.what());
            printf("%s \n", err_code.message().c_str());

            return false;
        }
        
        // printf("write raw : start\n");
        // for (size_t i = 0; i < backend_write_buffer.size(); i++)
        // {
        //     printf("%d\n", backend_write_buffer[i]);
        // }
        // printf("write raw : end\n");

        if(written_size == 0)
        {
            printf("serial port returned zero. seems an error occured in writing.\n");
            printf("%s \n", err_code.message().c_str());

            return false;
        }
    }

    return true;
}

// is there a way to check readbuffer ??
// otherwise this can causes read read deadlock;
bool SerialPortBuffer::updateBuffer()
{
    if(openCheck())
    {
        boost::system::error_code err_code;
        size_t read_size(0);   
        try
        {
            read_size = boost::asio::read(
                serial_port,
                boost::asio::buffer(backend_read_buffer),
                err_code
            );
        }
        catch(const boost::system::system_error& e)
        {
            printf("%s", e.what());
            printf("%s \n", err_code.message().c_str());

            return false;
        }
        
        if(read_size == 0)
        {
            printf("serial port returned zero. seems an error occured in reading.\n");
            printf("%s \n", err_code.message().c_str());
        }
        else
        {
            // printf("backend read raw : start\n");
            // for (size_t i = 0; i < backend_read_buffer.size(); i++)
            // {
            //     printf("%x\n", backend_read_buffer[i]);
            // }
            // printf("backend read raw : end\n");

            frontend_read_buffer.insert(
                frontend_read_buffer.end(),
                backend_read_buffer.begin(),
                backend_read_buffer.end()
            );
            //printf("backend buffer size %ld \n", backend_buffer.size());

            return true;
        }
    }

    return false;
}

}

#endif
