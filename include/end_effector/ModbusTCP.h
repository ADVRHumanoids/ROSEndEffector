#ifndef _UTILS_MODBUSTCP_H_
#define _UTILS_MODBUSTCP_H_

#include "modbus/modbus.h"
#include <string>
#include <vector>

namespace ROSEE {

class ModbusTCP
{
    public:
        ModbusTCP(){};

        virtual ~ModbusTCP() {};

        bool connectDevice(std::string device, int port, int slave)
        {
            std::cout << "Connecting device: \"" << device.c_str()<< "\" at port: "<< port <<"\n";
            mb_ = modbus_new_tcp(device.c_str(), port);

            if (mb_ == NULL)
            {
                std::cout << "Unable to allocate libmodbus context.\n";
                return false;
            }
            //display_.debug("Context allocated.\nSetting timeout response.");
            //struct timeval response_timeout;
            //response_timeout.tv_sec = 3;
            //response_timeout.tv_usec = 0;
            //modbus_set_response_timeout(mb_, &response_timeout);
            modbus_set_response_timeout(mb_, 3, 0); //3sec, 0usec

            //display_.debug("Connecting modbus...");
            if(modbus_connect(mb_) == -1) 
            {
                std::cout << "Connection failed: "<< modbus_strerror(errno)<< ".\n";
                modbus_free(mb_);
                return false;
            }

            modbus_set_slave(mb_, slave);
            std::cout << "Connected to the device.\n";
            return true;
        }

        void closeConnection()
        {
            modbus_close(mb_);
            modbus_free(mb_);

            std::cout << "Disconnected to the device.\n";
        }

        bool write(int address, std::vector<uint16_t> message)
        {
            int nb = message.size();

            int ret = modbus_write_registers(mb_, address, nb, message.data());

            if (ret == -1 || ret != nb) 
            {
                std::cout << "Error in writing on regiters: "<< modbus_strerror(errno)<< ".\n";
                // closeConnection();
                return false;
            }
            return true;
        }
  
        bool read(int address, int register_number, std::vector<uint16_t> &message)
        {
            
            message.resize(register_number);
            
            int ret = modbus_read_registers(mb_, address, register_number, message.data()); //Modbus function code 0x03
            
            if (ret == -1 || ret != register_number) {
                std::cout << "Error in reading registers. Error " << errno << ": "<< modbus_strerror(errno)<< ".\n";
                // closeConnection();
                return false;
            }

            return true;
        }

    private:
        modbus_t *mb_;
};

} // namespace ROSEE

#endif
