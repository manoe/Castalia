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
    UNKNOWN  = 0,
    INTERNAL = 1,
    BORDER   = 2,
    EXTERNAL = 3
};

enum shmrpCostFuncDef {
    NOT_DEFINED    = 0,
    HOP            = 1,
    HOP_AND_INTERF = 2
};


//namespace shmrp {
    class rreq_table_empty : public std::runtime_error {
        public:
            explicit rreq_table_empty(const string &what_arg) : std::runtime_error(what_arg) {};
            explicit rreq_table_empty(const char   *what_arg) : std::runtime_error(what_arg) {};
    };

    class rinv_table_empty : public std::runtime_error {
        public:
            explicit rinv_table_empty(const string &what_arg) : std::runtime_error(what_arg) {};
            explicit rinv_table_empty(const char   *what_arg) : std::runtime_error(what_arg) {};
    };
    class rreq_table_non_empty : public std::runtime_error {
        public:
            explicit rreq_table_non_empty(const string &what_arg) : std::runtime_error(what_arg) {};
            explicit rreq_table_non_empty(const char   *what_arg) : std::runtime_error(what_arg) {};
    };
    class routing_table_empty : public std::runtime_error {
        public:
            explicit routing_table_empty(const string &what_arg) : std::runtime_error(what_arg) {};
            explicit routing_table_empty(const char   *what_arg) : std::runtime_error(what_arg) {};
    };

//}

struct node_entry {
    string nw_address;
    int pathid;
    int hop;
    bool rresp = false;
    int interf;
};

struct feat_par {
        int    ring_radius;
        double t_l;
        double t_est;
        bool   rresp_req;
        bool   rst_learn;
        bool   replay_rinv;
        shmrpCostFuncDef cost_func;
        double cost_func_alpha;
        double cost_func_beta;
};

class shmrp: public VirtualRouting {
    private:
        bool g_is_sink;
        string g_sink_addr;
        int g_hop;
        int g_round;
        feat_par fp;
        shmrpStateDef g_state;
        std::map<std::string,node_entry> rinv_table;
        std::map<std::string,node_entry> rreq_table;
        std::map<std::string,node_entry> routing_table; 

    protected:
        void startup();
        void fromApplicationLayer(cPacket *, const char *);
        void fromMacLayer(cPacket *, int, double, double);
        void timerFiredCallback(int);
        void finishSpecific();

        shmrpCostFuncDef strToCostFunc(string) const;

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
        bool isRreqTableEmpty() const;
        void constructRreqTable(shmrpRingDef);
        bool rreqEntryExists(const char *, int);
        void updateRreqTableWithRresp(const char *, int);
        bool rrespReceived() const;

        double routeCostFunction(node_entry) const;


        void clearRoutingTable();
        void constructRoutingTable(bool);
        int  selectPathid();

        void sendRreqs();

        void sendRresp(const char *,int, int);


        map<int,string> getPathsAndHops();
    public:
};

#endif // _SHMRP_H_
