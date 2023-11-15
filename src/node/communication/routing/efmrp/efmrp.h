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
#include "node/application/ForestFire/forest_fire.h"
#include "node/mobilityManager/VirtualMobilityManager.h"

enum efmrpStateDef {
    UNDEF   = 0,
    INIT    = 1,
    LEARN   = 2,
    BUILD   = 3,
    WORK    = 4
};

enum efmrpTimerDef {
    SINK_START  = 1,
    TTL         = 2,
    FIELD       = 3,
    QUERY       = 4,
    ENV_CHK     = 5,
    BUILD_START = 6,
    RESTART     = 7
};

enum efmrpPathStatus {
    UNKNOWN     = 0,
    AVAILABLE   = 1,
    USED        = 2,
    DEAD        = 3,
    UNDER_QUERY = 4
};

struct path_entry {
    std::string     origin;
    efmrpPathStatus status;
};

struct ef_node_entry {
    std::string nw_address;
    int         hop;
    double      nrg;
    double      env;
    double      trg;
    std::vector<path_entry> pe;
};

struct routing_entry {
    std::string nw_address; // this should be origin
    std::string next_hop;
    double      target_value;
    efmrpPathStatus status;
    double      query_timestamp;
    int         prio;
};

struct efmrp_state_chng_entry {
    int node;
    double timestamp;
    efmrpStateDef state;
    double energy;
    double total_energy;
};


struct feat_par {
    // TIMER
    double ttl;
    double field;
    double query;
    double env_c;
    double d_update;
    double restart;

    // PARAMETER
    double alpha;
    double beta;
    int    pnum;
    double gamma;
    double n_lim;
    bool   periodic_restart;
};

class efmrp: public VirtualRouting {
    private:
        bool            g_is_sink;
        string          g_sink_addr;
        int             g_hop;
        feat_par        fp;
        efmrpStateDef   g_state;
        double          env_val;

        std::map<std::string,ef_node_entry> hello_table;
        std::map<std::string,ef_node_entry> field_table;
        std::vector<routing_entry>       routing_table;
        std::vector<efmrp_state_chng_entry> state_chng_log;
        
        ForestFire* ff_app;

        YAML::Emitter y_out;

        int             forw_pkt_count;

    protected:
        void startup();
        void fromApplicationLayer(cPacket *, const char *);
        void fromMacLayer(cPacket *, int, double, double);
        void timerFiredCallback(int);
        void finishSpecific();

        void sendHello();
        void sendHello(int, double, double, double);
        void updateHelloTable(efmrpHelloPacket *);
        bool checkHelloTable(std::string);
        void initHelloTable();


        void sendField(int, double, double, double);
        void updateFieldTable(efmrpFieldPacket *);
        void updateFieldTableWithQA(efmrpQueryAckPacket *);
        void updateFieldTableWithPE(std::string, std::string, efmrpPathStatus);
        void updateFieldTableEntry(std::string, double, double, double);
        bool checkFieldEntry(std::string);
        ef_node_entry getSinkFieldTableEntry();
        double calculateTargetValue();

        void constructPath(std::string, int prio);

        ef_node_entry getNthTargetValueEntry(int, std::vector<std::string>);
        ef_node_entry getNthTargetValueEntry(int, std::vector<std::string>, bool);
        ef_node_entry findSecondaryPath(std::string, std::vector<std::string>);

        int numOfAvailPaths(std::string);
        void initRouting();
        void cleanRouting(std::string);
        void logRouting();
        void logField();
        void addRoutingEntry(std::string, ef_node_entry, int);
        void addRoutingEntry(std::string, ef_node_entry, int, efmrpPathStatus, double timestamp);
        void updateRoutingEntry(std::string, ef_node_entry, int, efmrpPathStatus);
        bool checkRoutingEntry(std::string, int);
        routing_entry getRoutingEntry(std::string, int);
        void removeRoutingEntry(std::string, int, bool);
        double targetFunction(ef_node_entry);
        routing_entry getPath(std::string);
        routing_entry getPath(std::string, int);
        bool checkPath(std::string);
        bool isSinkNextHop();
        bool checkNextHop(std::string, int);

        void sendQuery(std::string);
        void sendQueryAck(std::string, std::string, bool);
        bool queryStarted(std::string);
        bool queryCompleted(std::string);

        void sendData(routing_entry, cPacket *);
        void sendData(std::string, cPacket *);

        void forwardData(efmrpDataPacket *);

        void sendRetreat(efmrpDataPacket *);

        void sendAlarm(efmrpAlarmDef, double, double, double);
        void removeEntries(std::string);
        void updateEntries(std::string, double, double, double);

        bool isSink() const;
        void setSinkAddress(const char *);
        std::string getSinkAddress() const;

        void setHop(int);

        void setState(efmrpStateDef);
        std::string stateToStr(efmrpStateDef) const;
        efmrpStateDef getState() const;

        std::string pathStatusToStr(efmrpPathStatus) const;

        void generateYaml();
        void serializeRoutingTable(std::vector<routing_entry> rt);
    public:
        efmrp() : g_is_sink(false),
                  g_hop(std::numeric_limits<int>::max()),
                  g_state(efmrpStateDef::UNDEF) {};
        
        std::vector<routing_entry> getRoutingTable() { return routing_table; };
        void writeState(int,double,efmrpStateDef,double);
        int getForwDataPkt() { return forw_pkt_count; };
        int getHop() const;

};

#endif /* _EFMRP_H_ */
