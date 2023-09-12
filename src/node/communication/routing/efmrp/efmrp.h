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
    QUERY       = 4
};

enum efmrpPathStatus {
    UNKNOWN     = 0,
    USED        = 1,
    DEAD        = 2
};

struct efmrpPathEntry {
    std::string     origin;
    int             prio;
    efmrpPathStatus status;
};

struct node_entry {
    std::string nw_address;
    int         hop;
    double      nrg;
    double      env;
};

struct routing_entry {
    std::string nw_address;
    std::string next_hop;
    double      target_value;
    int         prio;
};

struct feat_par {
    // TIMER
    double ttl;
    double field;
    double query;

    // PARAMETER
    double alpha;
    double beta;
    bool   pnum;
};

class efmrp: public VirtualRouting {
    private:
        bool            g_is_sink;
        string          g_sink_addr;
        int             g_hop;
        feat_par        fp;
        efmrpStateDef   g_state;

        std::map<std::string,node_entry> hello_table;
        std::map<std::string,node_entry> field_table;
        std::vector<routing_entry>       routing_table;

        
        ForestFire* ff_app;

        YAML::Emitter y_out;

    protected:
        void startup();
        void fromApplicationLayer(cPacket *, const char *);
        void fromMacLayer(cPacket *, int, double, double);
        void timerFiredCallback(int);
//        void finishSpecific();

        void sendHello();
        void sendHello(int, double);
        void updateHelloTable(efmrpHelloPacket *);
        bool checkHelloTable(std::string);


        void sendField(int, double, double);
        void updateFieldTable(efmrpFieldPacket *);

        void constructPath(std::string, int prio);

        node_entry getNthTargetValueEntry(int);
        int numOfAvailPaths(std::string);
        void addRoutingEntry(std::string nw_address, node_entry ne, int prio);
        double targetFunction(node_entry);
        routing_entry getPath(std::string);

        void sendQuery(std::string, int);

        void sendData(routing_entry, cPacket *);

        bool isSink() const;
        void setSinkAddress(const char *);
        std::string getSinkAddress() const;

        void setHop(int);
        int getHop() const;

        void setState(efmrpStateDef);
        std::string stateToStr(efmrpStateDef) const;
        efmrpStateDef getState() const;
        
    public:
        efmrp() : g_is_sink(false),
                  g_hop(std::numeric_limits<int>::max()),
                  g_state(efmrpStateDef::UNDEF) {};
};

#endif /* _EFMRP_H_ */
