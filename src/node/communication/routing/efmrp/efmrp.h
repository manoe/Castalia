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
#include "node/application/ForestFire/forest_fire.h"

////enum hdmrpRoleDef {
////    SINK        = 1;
////    ROOT        = 2;
////    SUB_ROOT    = 3;
////    NON_ROOT    = 4;
////}
//

enum efmrpStateDef {
    UNDEF       = 0,
    WORK        = 1,
    INIT        = 2,
    LEARN       = 3,
    DISSEMINATE = 4
};

enum efmrpTimerDef {
    SINK_START       = 1,
    TTL              = 2
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
    int    hop;
    double nrg;
    double env;
};

struct feat_par {
    double ttl;
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
        std::map<std::string,node_entry> field_table;
        std::map<std::string,node_entry> routing_table;

        ForestFire* ff_app;

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

        void sendField(int, double, double);
        void updateFieldTable(efmrpFieldPacket *); 
        
        bool isSink() const;
        void setSinkAddress(const char *);
        std::string getSinkAddress() const;

        void setHop(int);
        int getHop() const;

        void setState(efmrpStateDef);
        std::string stateToStr(efmrpStateDef) const;

        virtual void handleMacControlMessage(cMessage *);
    public:
        efmrpStateDef getState() const;
};

#endif // _EFMRP_H_
