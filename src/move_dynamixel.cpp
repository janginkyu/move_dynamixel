#include <iostream>

#include "move_dynamixel.hpp"

const bool jik::DynamixelCmd::_commCheck(const int commRes, const uint8_t dxl_error) const {
    if(!_is_connected) return false;
    //if(!_is_init) return false;
    if(commRes != COMM_SUCCESS) {
        std::cout << "Communication Failed : " << commRes << std::endl;
        return false;
    }
    else if(dxl_error != 0) {
        std::cout << "Packet Error : " << dxl_error << std::endl;
        return false;
    }
    else {
        return true;
    }
}

const bool jik::DynamixelCmd::connect(const char* dev, const int baud, const uint8_t id) {
    _portH = dynamixel::PortHandler::getPortHandler(dev);
    _packetH = dynamixel::PacketHandler::getPacketHandler(2.0);
    
    // open port
    if(_portH->openPort()) {
        std::cout << "Succeeded to open port " << dev << std::endl;
    }
    else {
        std::cout << "Failed to open port " << dev << std::endl;
        return false;
    }

    // set baud
    if(_portH->setBaudRate(baud)) {
        std::cout << "Successfully set baudrate to " << baud << std::endl;
    }
    else {
        std::cout << "Failed to set baudrate to " << baud << std::endl;
        return false;
    }

    _id = id;
    _is_connected = true;
    
    return true;
}

const bool jik::DynamixelCmd::init(const int mode) {
    if(!_is_connected) return false;
    int commRes;
    uint16_t readRes = 0;
    uint8_t dxl_error = 0;
    commRes = _packetH->read2ByteTxRx(_portH, _id, 0, &readRes, &dxl_error);
    if(readRes != 0x046A) {
        std::cout << "Please check Dynamixel Model (XM540-W150-R)" << std::endl;
        std::cout << "current model " << readRes << std::endl;
        return false;
    }

    // set mode
    commRes = _packetH->write1ByteTxRx(_portH, _id, 11, mode, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return false;
    else std::cout << "Set mode to " << mode << std::endl;

    // torque enable
    commRes = _packetH->write1ByteTxRx(_portH, _id, 64, 0x01, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return false;
    else std::cout << "Enabled torque" << std::endl;

    _is_init = true;
    return true;

}

const bool jik::DynamixelCmd::setMinMaxPos(const int min_pos, const int max_pos) {
    if(!_is_connected) return false;
    if(!_is_init) return false;
    int commRes;
    uint8_t dxl_error = 0;
    if(min_pos < 0 || min_pos > 4095) return false;
    if(max_pos < 0 || max_pos > 4095) return false;
    if(max_pos < min_pos) return setMinMaxPos(max_pos, min_pos);

    // set min position
    commRes = _packetH->write4ByteTxRx(_portH, _id, 52, min_pos, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return false;

    commRes = _packetH->write4ByteTxRx(_portH, _id, 48, max_pos, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return false;

    _max_pos = max_pos;
    _min_pos = min_pos;

    return true;
}

const bool jik::DynamixelCmd::setGoalPos_int(const int goalPos_int) {
    if(!_is_connected) return false;
    if(!_is_init) return false;
    if(goalPos_int < 0 || goalPos_int > 4095) return false;
    
    int commRes;
    uint8_t dxl_error = 0;
    uint32_t goalPos_uint = (uint32_t)goalPos_int;
    
    // set goal position
    commRes = _packetH->write4ByteTxRx(_portH, _id, 116, goalPos_uint, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return false;

    return true;
}

const int jik::DynamixelCmd::currPos_int() const {
    if(!_is_connected) return false;
    
    int commRes;
    uint8_t dxl_error = 0;
    uint32_t currPos_uint;
    
    commRes = _packetH->read4ByteTxRx(_portH, _id, 132, &currPos_uint, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return -1;
    else return (int)currPos_uint;
}

const bool jik::DynamixelCmd::disconnect() {
    int commRes;
    uint8_t dxl_error = 0;

    // torque disable
    commRes = _packetH->write1ByteTxRx(_portH, _id, 64, 0x00, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return false;
    else std::cout << "disabled torque" << std::endl;
    free(_portH);
    free(_packetH);
    _portH = NULL;
    _packetH = NULL;
    _is_connected = false;
    _is_init = false;
    return true;
}

const bool jik::DynamixelCmd::setGain(const int pgain, const int dgain, const int igain) {
    if(!_is_connected) return false;
    if(!_is_init) return false;

    int commRes;
    uint8_t dxl_error = 0;
    
    // set d gain
    commRes = _packetH->write2ByteTxRx(_portH, _id, 80, dgain, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return false;

    // set p gain
    commRes = _packetH->write2ByteTxRx(_portH, _id, 84, pgain, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return false;

    // set i gain
    commRes = _packetH->write2ByteTxRx(_portH, _id, 82, igain, &dxl_error);
    if(!_commCheck(commRes, dxl_error)) return false;

    return true;

}
