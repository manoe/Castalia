/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2023                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#ifndef _SMRP_H_
#define _SMRP_H_

#include <random>                                              
#include <algorithm>                                           
#include <map>                                                 
#include <fstream>                                             
#include <unordered_set>                                       
#include <ctime>                                               
#include <cstring>                                             
#include <yaml-cpp/yaml.h>                                     
 
#include "node/communication/routing/VirtualRouting.h"
#include "node/communication/routing/smrp/smrp_m.h"
#include "node/application/ApplicationPacket_m.h"
#include "node/application/ForestFire/forest_fire_packet_m.h"
#include "node/application/ForestFire/forest_fire.h"
#include "node/mobilityManager/VirtualMobilityManager.h"

enum smrpStateDef {
    UNDEF    = 0,
    INIT     = 1,
    LEARN    = 2,
    BUILD    = 3,
    WORK     = 4,
    MOBILITY = 5,
    RE_LEARN = 6
};

enum smrpTimerDef {
    SINK_START  = 1,
    TTL         = 2,
    FIELD       = 3,
    QUERY       = 4,
    ENV_CHK     = 5,
    BUILD_START = 6,
    RESTART     = 7
};

enum smrpPathStatus {
    UNKNOWN     = 0,
    AVAILABLE   = 1,
    USED        = 2,
    DEAD        = 3,
    UNDER_QUERY = 4
};

struct sm_path_entry {
    std::string     origin;
    smrpPathStatus  status;
};

struct sm_node_entry {
    std::string         nw_address;
    std::map<int,int>   hop;
    double              nrg;
    double              env;
    double              trg;
    std::vector<sm_path_entry> pe;
};

struct sm_routing_entry {
    std::string nw_address; // this should be origin
    std::string next_hop;
    double      target_value;
    smrpPathStatus status;
    double      query_timestamp;
    int         prio;
};

struct smrp_state_chng_entry {
    int node;
    double timestamp;
    smrpStateDef state;
    double energy;
    double total_energy;
};


struct sm_feat_par {
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
    bool   a_paths;
    bool   c_dead;
};

class smrp: public VirtualRouting {
    private:
        bool                g_is_sink;
        string              g_sink_addr;
        std::map<int,int>   g_hop;
        sm_feat_par         fp;
        smrpStateDef        g_state;
        double              env_val;

        std::map<std::string,sm_node_entry> hello_table;
        std::map<std::string,sm_node_entry> field_table;
        std::vector<sm_routing_entry>       routing_table;
        std::vector<smrp_state_chng_entry>  state_chng_log;
        
        ForestFire* ff_app;

        YAML::Emitter y_out;

        int             forw_pkt_count;

    protected:
        void startup();
        void fromApplicationLayer(cPacket *, const char *);
        void fromMacLayer(cPacket *, int, double, double);
        void handleNetworkControlCommand(cMessage *);

        void timerFiredCallback(int);
        void finishSpecific();

        void sendHello();
        void sendHello(int, int, double, double, double);
        void updateHelloTable(smrpHelloPacket *);
        void updateHelloTable(smrpFieldPacket *field_pkt);
        bool checkHelloTable(std::string);
        void removeHelloEntry(std::string);
        void initHelloTable();


        void sendField(map<int,int>, double, double, double);
        void updateFieldTable(smrpFieldPacket *);
        void updateFieldTableWithQA(smrpQueryAckPacket *);
        void updateFieldTableWithPE(std::string, std::string, smrpPathStatus);
        void updateFieldTableEntry(std::string, double, double, double);
        bool checkFieldEntry(std::string);
        sm_node_entry getSinkFieldTableEntry();
        double calculateTargetValue();
        void initFieldTable();

        void constructPath(std::string, int prio);

        sm_node_entry getNthTargetValueEntry(int, std::vector<std::string>);
        sm_node_entry getNthTargetValueEntry(int, std::vector<std::string>, bool);
        sm_node_entry findPath(std::string, std::vector<std::string>);

        int numOfAvailPaths(std::string,bool,bool);
        void initRouting();
        void initRoutingTable();
        void cleanRouting(std::string);
        void logRouting();
        void logField();
        void addRoutingEntry(std::string, sm_node_entry, int);
        void addRoutingEntry(std::string, sm_node_entry, int, smrpPathStatus, double timestamp);
        void updateRoutingEntry(std::string, sm_node_entry, int, smrpPathStatus);
        bool checkRoutingEntry(std::string, int);
        bool checkRoutingEntryWithOtherPrio(std::string, int);

        sm_routing_entry getRoutingEntry(std::string, int);
        void removeRoutingEntry(std::string, int, bool);
        double targetFunction(sm_node_entry);
        sm_routing_entry getPath(std::string);
        sm_routing_entry getPath(std::string, int);
        bool checkPath(std::string);
        bool checkNextHop(std::string ne);
        bool checkNextHop(std::string, int);
        int calcNextPriFromRtTable(std::string,bool);

        void sendQuery(std::string);
        void sendQueryAck(std::string, std::string, bool);
        bool queryStarted(std::string, int);
        bool queryCompleted(std::string, int);

        void sendData(sm_routing_entry, cPacket *);
        void sendData(std::string, cPacket *);

        void forwardData(smrpDataPacket *);

        void sendRetreat(smrpDataPacket *);

        void sendAlarm(smrpAlarmDef, double, double, double);
        void removeEntries(std::string);
        void updateEntries(std::string, double, double, double);

        bool isSink() const;
        void setSinkAddress(const char *);
        std::string getSinkAddress() const;

        void setHop(int, int);

        void setState(smrpStateDef);
        std::string stateToStr(smrpStateDef) const;
        smrpStateDef getState() const;

        std::string pathStatusToStr(smrpPathStatus) const;

        void generateYaml();
        void serializeRoutingTable(std::vector<sm_routing_entry> rt);

        std::map<int, int> getHop();
        bool updateHop(int,int);

    public:
        smrp() : g_is_sink(false),
                 g_state(smrpStateDef::UNDEF) {};
        
        std::vector<sm_routing_entry> getRoutingTable() { return routing_table; };
        void writeState(int,double,smrpStateDef,double);
        int getForwDataPkt() { return forw_pkt_count; };
        int getHop(int);
};

#endif /* _SMRP_H_ */
