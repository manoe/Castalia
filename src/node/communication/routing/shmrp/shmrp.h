/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#ifndef _SHMRP_H_
#define _SHMRP_H_

#include <random>
#include <algorithm>
#include <map>
#include <fstream>
#include <unordered_set>
#include <ctime>
#include <cstring>

#include "node/communication/routing/VirtualRouting.h"
#include "node/communication/routing/shmrp/shmrp_m.h"
#include "node/application/ApplicationPacket_m.h"
#include "node/application/ForestFire/forest_fire_packet_m.h"
#include "node/mobilityManager/VirtualMobilityManager.h"

////enum hdmrpRoleDef {
////    SINK        = 1;
////    ROOT        = 2;
////    SUB_ROOT    = 3;
////    NON_ROOT    = 4;
////}
//

enum shmrpStateDef {
    UNDEF     = 0,
    WORK      = 1,
    INIT      = 2,
    LEARN     = 3,
    ESTABLISH = 4
};

enum shmrpTimerDef {
    SINK_START       = 1,
    T_L              = 2,
    T_ESTABLISH      = 3,
    NEW_ROUND        = 5,
    T_RELAY          = 6,
    ACK_HIST_PURGE   = 7,
    PACKET_TIMER_1   = 8
};

enum shmrpRingDef {
//    UNDEF    = 0,
    INTERNAL = 1,
    BORDER   = 2,
    EXTERNAL = 3
};

struct node_entry {
    string nw_address;
    int pathid;
    int hop;
};

class shmrp: public VirtualRouting {
    private:
        bool g_is_sink;
        string g_sink_addr;
        int g_hop;
        int g_round;
        double g_t_l;
        int g_ring_radius;
        shmrpStateDef g_state;
        std::map<std::string,node_entry> rinv_table;
        std::map<std::string,node_entry> rreq_table;
        std::map<std::string,node_entry> route_table; 

    protected:
        void startup();
        void fromApplicationLayer(cPacket *, const char *);
        void fromMacLayer(cPacket *, int, double, double);
        void timerFiredCallback(int);
        void finishSpecific();

        bool isSink() const;
        void setSinkAddress(const char *);
        std::string getSinkAddress() const;

        void sendRinv(int);
        void sendRinv(int,int);

        void setHop(int);
        int getHop() const;
        void calculateHop();

        void setRound(int);
        int  getRound() const;

        void setState(shmrpStateDef);
        shmrpStateDef getState() const;
        std::string stateToStr(shmrpStateDef) const;

        void clearRinvTable();
        void addToRinvTable(shmrpRinvPacket *);

        void clearRreqTable();
        void constructRreqTable(shmrpRingDef);

    public:
};

#endif // _SHMRP_H_
