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
    CENTRAL  = 1,
    INTERNAL = 2,
    BORDER   = 3,
    EXTERNAL = 4
};

enum shmrpCostFuncDef {
    NOT_DEFINED          = 0,
    HOP                  = 1,
    HOP_AND_INTERF       = 2,
    HOP_EMERG_AND_INTERF = 3,
};

enum shmrpRinvTblAdminDef {
    UNDEF_ADMIN    = 0,
    ERASE_ON_LEARN = 1,
    ERASE_ON_ROUND = 2,
    NEVER_ERASE    = 3 
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
    int pathid;
    int hop;
    bool rresp = false;
    int interf;
    int emerg;
    bool used = false;
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
        bool  random_t_l;
        double random_t_l_sigma;
        shmrpRinvTblAdminDef rinv_tbl_admin;
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
        
        shmrpRinvTblAdminDef strToRinvTblAdmin(string) const; 
        shmrpCostFuncDef strToCostFunc(string) const;

        bool isSink() const;
        void setSinkAddress(const char *);
        std::string getSinkAddress() const;
        double getTl();

        void sendRinv(int);
        void sendRinv(int,int);

        void setHop(int);
        int getHop() const;
        int calculateHop();

        void setRound(int);
        int  getRound() const;

        void setState(shmrpStateDef);
        shmrpStateDef getState() const;
        std::string stateToStr(shmrpStateDef) const;

        void clearRinvTable();
        void addToRinvTable(shmrpRinvPacket *);
        int  getRinvTableSize() const;

        void clearRreqTable();
        bool isRreqTableEmpty() const;
        void constructRreqTable();
        bool rreqEntryExists(const char *, int);
        void updateRreqTableWithRresp(const char *, int);
        bool rrespReceived() const;

        double calculateCostFunction(node_entry);

        void clearRoutingTable();
        void constructRoutingTable(bool);
        int  selectPathid();

        void sendRreqs();

        void sendRresp(const char *,int, int);

        std::string ringToStr(shmrpRingDef pos) const; 

        map<int,string> getPathsAndHops();
    public:
        shmrpRingDef getRingStatus() const;
};

#endif // _SHMRP_H_
