#ifndef __MOVE_DYNAMIXEL_HPP__
#define __MOVE_DYNAMIXEL_HPP__

#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace jik {
    class DynamixelCmd {
        private:

        dynamixel::PortHandler* _portH = NULL;
        dynamixel::PacketHandler* _packetH = NULL;
        bool _is_connected = false;
        bool _is_init = false;
        uint8_t _id;
        int _max_pos = 4095;
        int _min_pos = 0;

        const bool _commCheck(const int commRes, const uint8_t dxl_error) const;

        public:

        const bool connect(const char* dev, const int baud, const uint8_t id);
        const bool connect(const std::string dev, const int baud, const uint8_t id) { return connect(dev.data(), baud, id); }
        const bool init(const int mode = 0x05);
        const bool setMinMaxPos(const int min_pos, const int max_pos);
        const int currPos_int() const;
        const double currPos_deg() const { return (double)currPos_int() * 0.087890625; }
        const double currPos_rad() const { return (double)currPos_int() * 0.0015339807878856412; }

        const bool setGoalPos_int(const int goalPos_int);
        const bool setGoalPos_deg(const double goalPos_deg) { return setGoalPos_int((int)(goalPos_deg * 11.377777777777778)); }
        const bool setGoalPos_rad(const double goalPos_rad) { return setGoalPos_int((int)(goalPos_rad * 651.8986469044033)); }

        const bool setGain(const int pgain, const int dgain, const int igain = 0);

        const bool disconnect();
        DynamixelCmd() { }
        ~DynamixelCmd() { }
    };
}

#endif
