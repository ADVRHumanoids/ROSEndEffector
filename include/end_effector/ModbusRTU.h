#ifndef _UTILS_MODBUS_H_
#define _UTILS_MODBUS_H_

#include "modbus/modbus.h"
#include <string>
#include <vector>

namespace ROSEE {

class ModbusRTU
{
    public:
        ModbusRTU(){};
        ~ModbusRTU(){};

        bool connectDevice(std::string device, int slave_id, int baud_rate, char parity, int data_bit, int stop_bit)
        {
            mb_ = modbus_new_rtu(device.c_str(), baud_rate, parity, data_bit, stop_bit);

            if(modbus_set_slave(mb_, slave_id) == -1) 
            {
                modbus_free(mb_);
                std::cout << "Invalid slave ID.\n";
                return false;
            }

            if(modbus_connect(mb_) == -1) 
            {
                std::cout << "Connection failed: " << modbus_strerror(errno) << ".\n";
                modbus_free(mb_);
                return false;
            }

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
                std::cout << "Error in writing on registers. " << modbus_strerror(errno) << ".\n";
                closeConnection();
                return false;
            }
            return true;
        }
  
        bool read(int address, int register_number, std::vector<uint16_t> &message)
        {
            message.resize(register_number);
            int ret = modbus_read_registers(mb_, address, register_number, message.data());

            if (ret == -1 || ret != register_number) {
                std::cout << "Error in reading regiters. " << modbus_strerror(errno) << ".\n";
                closeConnection();
                return false;
            }

            return true;
        }

    private:
        modbus_t *mb_;
};

} // namespace

#endif //_UTILS_MODBUS_H_
