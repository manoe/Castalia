/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2023                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#ifndef _EFMRP_H_
#define _EFMRP_H_

#include <random>
#include <algorithm>
#include <map>
#include <fstream>
#include <unordered_set>
#include <ctime>
#include <cstring>
#include <yaml-cpp/yaml.h>

#include "node/communication/routing/VirtualRouting.h"
#include "node/communication/routing/efmrp/efmrp_m.h"
#include "node/application/ApplicationPacket_m.h"
#include "node/application/ForestFire/forest_fire_packet_m.h"
#include "node/mobilityManager/VirtualMobilityManager.h"
#include "node/communication/mac/tMac/TMacPacket_m.h"

////enum hdmrpRoleDef {
////    SINK        = 1;
////    ROOT        = 2;
////    SUB_ROOT    = 3;
////    NON_ROOT    = 4;
////}
//

enum efmrpStateDef {
    UNDEF     = 0,
    WORK      = 1,
    INIT      = 2,
    LEARN     = 3,
    ESTABLISH = 4,
    MEASURE   = 5
};

enum efmrpTimerDef {
    SINK_START       = 1,
    T_L              = 2,
    T_ESTABLISH      = 3,
    T_MEASURE        = 4,
    NEW_ROUND        = 5,
    T_RELAY          = 6,
    ACK_HIST_PURGE   = 7,
    PACKET_TIMER_1   = 8,
    TTL              = 9
};

enum efmrpRingDef {
    UNKNOWN  = 0,
    CENTRAL  = 1,
    INTERNAL = 2,
    BORDER   = 3,
    EXTERNAL = 4
};

enum efmrpCostFuncDef {
    NOT_DEFINED          = 0,
    HOP                  = 1,
    HOP_AND_INTERF       = 2,
    HOP_EMERG_AND_INTERF = 3,
};

enum efmrpRinvTblAdminDef {
    UNDEF_ADMIN    = 0,
    ERASE_ON_LEARN = 1,
    ERASE_ON_ROUND = 2,
    NEVER_ERASE    = 3 
};


//namespace efmrp {
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
    class recv_table_empty : public std::runtime_error {
        public:
            explicit recv_table_empty(const string &what_arg) : std::runtime_error(what_arg) {};
            explicit recv_table_empty(const char   *what_arg) : std::runtime_error(what_arg) {};
    };

    class unknown_cost_function : public std::runtime_error {
        public:
            explicit unknown_cost_function(const string &what_arg) : std::runtime_error(what_arg) {};
            explicit unknown_cost_function(const char   *what_arg) : std::runtime_error(what_arg) {};
    };
    class no_available_entry : public std::runtime_error {
        public:
            explicit no_available_entry(const string &what_arg) : std::runtime_error(what_arg) {};
            explicit no_available_entry(const char   *what_arg) : std::runtime_error(what_arg) {};
    };

//}

struct node_entry {
    string nw_address;
    int  pathid;
    int  hop;
    bool rresp = false;
    int  interf;
    int  emerg;
    bool used = false;
    int  round = 0;
    int  pkt_count = 0;
    int  ack_count = 0;
};

struct feat_par {
    double ttl;
    
    
    
    int    ring_radius;
    double t_l;
    double t_est;
    double t_meas;
    bool   rresp_req;
    bool   rst_learn;
    bool   replay_rinv;
    efmrpCostFuncDef cost_func;
    double cost_func_alpha;
    double cost_func_beta;
    bool   cf_after_rresp;
    bool   random_t_l;
    double random_t_l_sigma;
    efmrpRinvTblAdminDef rinv_tbl_admin;
    bool   interf_ping;
    bool   round_keep_pong;
    bool   rand_ring_hop;
};

class efmrp: public VirtualRouting {
    private:
        bool g_is_sink;
        string g_sink_addr;
        int g_hop;
        int g_round;
        feat_par fp;
        efmrpStateDef g_state;

        std::map<std::string,node_entry> hello_table;

        std::map<std::string,node_entry> rinv_table;
        std::map<std::string,node_entry> rreq_table;
        std::map<std::string,node_entry> routing_table;
        std::map<std::string,node_entry> pong_table;
        std::map<std::string,node_entry> recv_table;
        YAML::Emitter y_out;

    protected:
        void startup();
        void fromApplicationLayer(cPacket *, const char *);
        void fromMacLayer(cPacket *, int, double, double);
        void timerFiredCallback(int);
        void finishSpecific();

        void sendHello();
        void sendHello(int, double);
        void updateHelloTable(efmrpHelloPacket *);
        
        efmrpRinvTblAdminDef strToRinvTblAdmin(string) const; 
        efmrpCostFuncDef strToCostFunc(string) const;

        bool isSink() const;
        void setSinkAddress(const char *);
        std::string getSinkAddress() const;
        double getTl();
        double getTmeas() const;
        double getTest() const;


        void sendPing(int);
        void sendPong(int);
        void storePong(efmrpPongPacket *);
        int getPongTableSize() const;
        void clearPongTable();
        void clearPongTable(int);

        void sendRinv(int);

        void setHop(int);
        int getHop() const;
        int calculateHop();

        void setRound(int);
        int  getRound() const;

        void setState(efmrpStateDef);
        std::string stateToStr(efmrpStateDef) const;

        void clearRinvTable();
        void addToRinvTable(efmrpRinvPacket *);
        int  getRinvTableSize() const;

        void clearRreqTable();
        bool isRreqTableEmpty() const;
        void constructRreqTable();
        bool rreqEntryExists(const char *, int);
        void updateRreqTableWithRresp(const char *, int);

        double calculateCostFunction(node_entry);


        void clearRoutingTable();
        void constructRoutingTable(bool);
        void constructRoutingTable(bool,bool);
        bool isRoutingTableEmpty() const;
        int  selectPathid();
        std::string getNextHop(int);
        std::string getNextHop(int, bool);
        void incPktCountInRoutingTable(std::string);

        void incPktCountInRecvTable(std::string);


        void sendRreqs();

        void sendRresp(const char *,int, int);

        void sendData(cPacket *, std::string, int); 
        void forwardData(efmrpDataPacket *, std::string, int);
        void forwardData(efmrpDataPacket *, std::string);
        std::string ringToStr(efmrpRingDef pos) const; 

        map<int,string> getPathsAndHops();

        void serializeRoutingTable();
        void serializeRoutingTable(std::map<std::string,node_entry>);

        void serializeRecvTable();
        void serializeRecvTable(std::map<std::string,node_entry>);
        std::string StateToString(efmrpStateDef);
 
        virtual void handleMacControlMessage(cMessage *);
    public:
        efmrpRingDef getRingStatus() const;
        efmrpStateDef getState() const;

        std::map<std::string,node_entry> getRoutingTable() {
            if(routing_table.empty()) {
                throw routing_table_empty("[error] Routing table empty at node");
            }
            return routing_table;
        };
        std::map<std::string,node_entry> getRecvTable() {
            if(recv_table.empty()) {
                throw recv_table_empty("[error] Recv table empty at node");
            }
            return recv_table;
        };
};

#endif // _EFMRP_H_
