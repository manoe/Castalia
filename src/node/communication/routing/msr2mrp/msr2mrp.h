/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2024                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#ifndef _MSR2MRP_H_
#define _MSR2MRP_H_

#include <random>
#include <algorithm>
#include <map>
#include <fstream>
#include <unordered_set>
#include <ctime>
#include <cstring>
#include <yaml-cpp/yaml.h>
#include <cerrno>
#include <list>

#include "node/communication/routing/VirtualRouting.h"
#include "node/communication/routing/msr2mrp/msr2mrp_m.h"
#include "node/application/ApplicationPacket_m.h"
#include "node/application/ForestFire/forest_fire_packet_m.h"
#include "node/mobilityManager/VirtualMobilityManager.h"
#include "node/communication/mac/tMac/TMacPacket_m.h"
#include "node/communication/radio/Radio.h"
#include "node/application/ForestFire/forest_fire_message_m.h"
#include "node/application/ForestFire/forest_fire.h"
#include "node/communication/routing/msr2mrp/stimer.h"

#define SetTimer(a,b) \
    stimer->setTimer(0,a,b,getTimer(msr2mrpTimerDef::T_SERIAL) == -1?0:getTimer(msr2mrpTimerDef::T_SERIAL));
#define GetTimer(a) stimer->getTimer(0,a)
#define CancelTimer(a) stimer->cancelTimer(0,a);

#define updateTimer() \
    if(stimer->timerChange()) { \
        trace()<<"[info] Timer change"; \
        if(legacyGetTimer(msr2mrpTimerDef::T_SERIAL) != -1) { \
            trace()<<"[info] T_SERIAL was active, canceling"; \
            legacyCancelTimer(msr2mrpTimerDef::T_SERIAL); \
        } \
        legacySetTimer(msr2mrpTimerDef::T_SERIAL, stimer->getTimerValue()); \
    } \

#define legacySetTimer(a,b) setTimer(a,b)
#define legacyCancelTimer(a) cancelTimer(a)
#define legacyGetTimer(a) getTimer(a)


////enum hdmrpRoleDef {
////    SINK        = 1;
////    ROOT        = 2;
////    SUB_ROOT    = 3;
////    NON_ROOT    = 4;
////}
//

enum msr2mrpStateDef {
    UNDEF       = 0,
    WORK        = 1,
    INIT        = 2,
    LEARN       = 3,
    ESTABLISH   = 4,
    MEASURE     = 5,
    LOCAL_LEARN = 6,
    DEAD        = 7,
    S_ESTABLISH = 8
};

enum msr2mrpTimerDef {
    T_SINK_START     = 1,
    T_L              = 2,
    T_ESTABLISH      = 3,
    T_MEASURE        = 4,
    T_REPEAT         = 5,
    T_SEC_L          = 6,
    T_SEC_L_REPEAT   = 7,
    T_SEC_L_START    = 8,
    T_SEND_PKT       = 9,
    T_RESTART        = 10,
    T_SERIAL         = 11
};

enum msr2mrpRingDef {
    UNKNOWN  = 0,
    CENTRAL  = 1,
    INTERNAL = 2,
    BORDER   = 3,
    EXTERNAL = 4
};

enum msr2mrpCostFuncDef {
    NOT_DEFINED                         = 0,
    HOP                                 = 1,
    HOP_AND_INTERF                      = 2,
    HOP_EMERG_AND_INTERF                = 3,
    HOP_AND_PDR                         = 4,
    HOP_PDR_AND_INTERF                  = 5,
    HOP_EMERG_PDR_AND_INTERF            = 6,
    HOP_ENRGY_EMERG_PDR_AND_INTERF      = 7,
    HOP_ENRGY_EMERG_AND_PDR             = 8,
    HOP_ENRGY_AND_PDR                   = 9,
    XPR_INTERF                          = 10,
    XPR_HOP_AND_PDR                     = 11,
    XPR_HOP_PDR_AND_INTERF              = 12,
    SUM_HOP_ENERGY_EMERG_PDR_AND_INTERF = 14
};

enum msr2mrpRinvTblAdminDef {
    UNDEF_ADMIN    = 0,
    ERASE_ON_LEARN = 1,
    ERASE_ON_ROUND = 2,
    NEVER_ERASE    = 3 
};

enum msr2mrpSecLParDef {
    OFF            = 0,
    BROADCAST      = 1,
    UNICAST        = 2
};


//namespace msr2mrp {
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

class state_not_permitted : public std::runtime_error {
    public:
        explicit state_not_permitted(const string &what_arg) : std::runtime_error(what_arg) {};
        explicit state_not_permitted(const char   *what_arg) : std::runtime_error(what_arg) {};
};

//}

struct msr2mrp_pathid_entry {
    int pathid;
    int nmas;
    bool secl;
    bool secl_performed;
    bool used;
    double enrgy = 0;
    double emerg = 0;
    double pdr   = 0;
    int origin   = 0;
};

struct msr2mrp_node_entry {
    string nw_address;
    vector<msr2mrp_pathid_entry> pathid;
    int  hop;
    bool rresp = false;
    int  interf;
    double emerg = 0;
    bool used = false;
    int  round = 0;
    int  pkt_count = 0;
    int  ack_count = 0;
    int  fail_count = 0;
    bool fail = false;
    bool local = false;
    bool secl  = false;
    int  nmas = 0;
    int reroute_count = 0;
};

struct msr2mrp_state_chng_entry {
    int node;
    double timestamp;
    msr2mrpStateDef state;
    double energy;
    double total_energy;
};

struct feat_par {
    int    ring_radius;
    double t_l;
    double t_est;
    double t_meas;
    bool   rresp_req;
    bool   rst_learn;
    bool   replay_rinv;
    msr2mrpCostFuncDef cost_func;
    double cost_func_epsilon;
    double cost_func_iota;
    double cost_func_pi;
    double cost_func_phi;
    double cost_func_mu;
    double cost_func_eta;
    bool   cf_after_rresp;
    bool   random_t_l;
    double random_t_l_sigma;
    msr2mrpRinvTblAdminDef rinv_tbl_admin;
    bool   interf_ping;
    bool   round_keep_pong;
    bool   rand_ring_hop;
    bool   static_routing;
    bool   measure_w_rreq;
    int    meas_rreq_count;
    bool   calc_max_hop;
    double qos_pdr;
    bool   rt_recalc_warn;
    bool   reroute_pkt;
    msr2mrpSecLParDef second_learn;
    double t_sec_l;
    double t_sec_l_repeat;
    int    t_sec_l_timeout;
    double t_sec_l_start;
    double t_restart;
    bool   periodic_restart; 
    bool   detect_link_fail;
    bool   rt_fallb_wo_qos;
    bool   send_pfail_rwarn;
    int    fail_count;
    int    path_sel;
    double e2e_qos_pdr;
    double t_send_pkt;
    bool   rep_m_pdr;
    bool   drop_1st_rt_c;
    double drop_prob;
    bool   e2e_cost;
};

class msr2mrp: public VirtualRouting {
    private:
        bool g_is_sink;
        string g_sink_addr;
        int g_hop;
        int g_round;
        msr2mrp_pathid_entry g_pathid; // this is dangerous, even more dangerous
        bool g_sec_l = false;
        std::queue<int> g_sec_l_pathid;
        int g_sec_l_timeout = 0;
        bool g_is_master=false;
        feat_par fp;
        msr2mrpStateDef g_state;
        std::map<std::string,msr2mrp_node_entry> rinv_table;
        std::map<std::string,msr2mrp_node_entry> rreq_table;
        std::map<std::string,msr2mrp_node_entry> routing_table;
        std::map<std::string,msr2mrp_node_entry> pong_table;
        std::map<std::string,msr2mrp_node_entry> recv_table;
        std::map<std::string,msr2mrp_node_entry> backup_rreq_table;
        std::list<msr2mrpDataPacket *> pkt_list;
        std::map<std::string,msr2mrp_node_entry> traffic_table;

        YAML::Emitter y_out;

        std::vector<msr2mrp_state_chng_entry> state_chng_log;

        std::unordered_set<int> local_id_table;

        ForestFire *ff_app;

        int forw_pkt_count;

        SerialTimer *stimer;

        msr2mrp *nw_layer;

    protected:
        void startup();
        void parseRouting(std::string);
        void fromApplicationLayer(cPacket *, const char *);
        void fromMacLayer(cPacket *, int, double, double);
        void handleNetworkControlCommand(cMessage *);
        void timerFiredCallback(int);
        void finishSpecific();

        msr2mrpRinvTblAdminDef strToRinvTblAdmin(string) const; 
        msr2mrpCostFuncDef strToCostFunc(string) const;
        msr2mrpSecLParDef strToSecLPar(string) const; 

        bool isSink() const;
        void setSinkAddress(std::string);
        std::string getSinkAddress() const;
        std::string getSinkAddressFromRtTbl();
        double getTl();
        double getTmeas() const;
        double getTest() const;
        void   handleTSecLTimer();

        void sendPing(int);
        void sendPong(int);
        void storePong(msr2mrpPongPacket *);
        void clearPongTable();
        void clearPongTable(int);

        void sendRinv(int,std::string);
        void sendRinv(int,vector<msr2mrp_pathid_entry>);
        void sendRinvBasedOnHop();

        void setHop(int);
        int  calculateHop(bool);
        int  calculateHopFromRoutingTable();

        void setRound(int);

        bool checkLocalid(int id) { return local_id_table.end() != local_id_table.find(id); }; 
        void storeLocalid(int id) { local_id_table.insert(id); };
        void clearLocalids() { local_id_table.clear(); };

        void setState(msr2mrpStateDef);
        std::string stateToStr(msr2mrpStateDef) const;

        void clearRinvTable();
        void addToRinvTable(msr2mrpRinvPacket *);
        int  getRinvTableSize() const;
        void updateRinvTableFromRreqTable();
        void clearRinvTableLocalFlags();
        void removeRinvEntry(std::string);
        void markRinvEntryLocal(std::string);
        void markRinvEntryFail(std::string);
        bool checkRinvEntry(std::string id) { return rinv_table.find(id) != rinv_table.end();};


        void clearRreqTable();
        void saveRreqTable();
        void retrieveRreqTable();
        void retrieveAndMergeRreqTable();
        void mergePathids(std::vector<msr2mrp_pathid_entry>&, std::vector<msr2mrp_pathid_entry>&);
        bool isRreqTableEmpty() const;
        void constructRreqTable();
        void constructRreqTable(std::vector<int>);
        bool rreqEntryExists(const char *, int);
        bool rreqEntryExists(std::string);
        int  getRreqPktCount();
        void updateRreqTableWithRresp(const char *, int);
        bool rrespReceived() const;
        void updateRreqEntryWithEmergency(const char *, double, double);
        void removeRreqEntry(std::string, bool);
        double calculateCostFunction(msr2mrp_node_entry);


        void clearRoutingTable();
        void constructRoutingTable(bool);
        void constructRoutingTable(bool,bool,double,bool);
        void constructRoutingTableFromRinvTable();
        void addRoute(std::string, int);
        void removeRoute(std::string);
        bool isRoutingTableEmpty() const;
        msr2mrp_pathid_entry selectPathid();
        msr2mrp_pathid_entry selectPathid(bool);
        std::vector<int> selectAllPathid();
        std::vector<int> selectAllPathid(int);
        std::string pathidToStr(vector<msr2mrp_pathid_entry> pathid);
        std::string pathidToStr(vector<int> pathid);

        std::string getNextHop(int);
        std::string getNextHop(int, bool);
        void incPktCountInRoutingTable(std::string);
        bool checkPathid(int);
        bool checkRoute(std::string);
        int  calculateRepeat(const char *);
        int  getRoutingTableSize() { return routing_table.size();};

        void incPktCountInRecvTable(std::string, int, int);


        void sendRreqs();
        void sendRreqs(int);

        void sendRresp(const char *,int, int);

        void sendLreq(const char *,int, int);
        void sendLreqUnicast(int, int);
        void sendLreqBroadcast(int, int);

        void sendLresp(const char *,int, int);

        void sendData(cPacket *, std::string, int);
        void schedulePkt(cPacket *, std::string, int); 
        void forwardData(msr2mrpDataPacket *, std::string, int, bool);
        void forwardData(msr2mrpDataPacket *, std::string, bool);
        std::string ringToStr(msr2mrpRingDef pos) const;

        void sendRwarn();
        void sendRwarn(msr2mrpWarnDef, int);

        void handleLinkFailure(int);

        void serializeRoutingTable();
        void serializeRoutingTable(std::map<std::string,msr2mrp_node_entry>);

        void serializeRecvTable();
        void serializeRecvTable(std::map<std::string,msr2mrp_node_entry>);
        void serializeRadioStats(PktBreakdown);

        std::string StateToString(msr2mrpStateDef);

        virtual void handleMacControlMessage(cMessage *);

        void setSecL(bool flag);
        void setSecL(int pathid, bool flag);
        void pushSecLPathid(int);
        int  popSecLPathid();
        int getSecLPathid();
        bool isSecLPathidEmpty();
        bool secLPerformed(int round, int pathid);
        bool getSecL(int pathid);

        void incPktCountInTrafficTable(std::string, int, int);

    public:
        msr2mrp() : g_hop(std::numeric_limits<int>::max()),
                  g_round(0),
                  g_state(msr2mrpStateDef::UNDEF) {};

        msr2mrpRingDef getRingStatus() const;
        msr2mrpStateDef getState() const;
        void writeState(int,double,msr2mrpStateDef,double);
        int getHop() const;
        int getHop(int);
        int getForwDataPkt() { return forw_pkt_count; };
        int  getRound() const;
        bool getSecL();
        bool isMaster() const;


        int getPongTableSize() const;
        void initPongTableSize();

        std::map<std::string,msr2mrp_node_entry> getRoutingTable() {
            if(routing_table.empty()) {
                throw routing_table_empty("[error] Routing table empty at node");
            }
            return routing_table;
        };
        std::map<std::string,msr2mrp_node_entry> getRecvTable() {
            if(recv_table.empty()) {
                throw recv_table_empty("[error] Recv table empty at node");
            }
            return recv_table;
        };
        std::map<std::string,msr2mrp_node_entry> getRreqTable() {
            if(rreq_table.empty()) {
                throw rreq_table_empty("[error] Rreq table empty at node");
            }
            return rreq_table;
        };
        std::map<std::string,msr2mrp_node_entry> getRinvTable() {
            if(rinv_table.empty()) {
                throw rinv_table_empty("[error Rinv table empty at node");
            }
            return rinv_table;
        };
        std::map<std::string,msr2mrp_node_entry> getTrafficTable() {
            return traffic_table;
        };

        double getEnergyValue();
        double getEmergencyValue();


        void extToMacLayer(cMessage *msg) { toMacLayer(msg); };
        void extToApplicationLayer(cMessage *msg) {toApplicationLayer(msg); };
        void extToMacLayer(cPacket *msg, int addr) { toMacLayer(msg,addr); };
        std::ostream & extTrace() { return trace(); };
};

#endif // _MSR2MRP_H_
