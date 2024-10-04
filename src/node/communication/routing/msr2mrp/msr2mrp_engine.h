/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2024                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#ifndef _MSR2MRP_ENGINE_H_
#define _MSR2MRP_ENGINE_H_

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

class msr2mrp_engine;

#include "node/communication/routing/msr2mrp/msr2mrp.h"
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



////enum hdmrpRoleDef {
////    SINK        = 1;
////    ROOT        = 2;
////    SUB_ROOT    = 3;
////    NON_ROOT    = 4;
////}
//



class msr2mrp_engine {
    private:
        bool g_is_sink;
        string g_sink_addr;
        string selfAddress; // ugly hack
        int g_hop;
        int g_round;
        msr2mrp_pathid_entry g_pathid; // this is dangerous, even more dangerous
        bool g_sec_l = false;
        std::queue<int> g_sec_l_pathid;
        int g_sec_l_timeout = 0;
        bool g_is_master=false;
        msr2mrp_feat_par fp;
        msr2mrpStateDef g_state;
        bool g_limited_state=false;
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

        int forw_pkt_count;

        SerialTimer *stimer;

        msr2mrp *nw_layer;
        int netDataFrameOverhead;
        int currentSequenceNumber;

    protected:
        void startup();
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
        msr2mrp_pathid_entry selectPathid(bool);
        msr2mrp_pathid_entry selectPathid(bool, bool);
        std::vector<int> selectAllPathid();
        std::vector<int> selectAllPathid(int);
        std::string pathidToStr(vector<msr2mrp_pathid_entry> pathid);
        std::string pathidToStr(vector<int> pathid);
        std::map<int,int> getPathIdWithNum();


        std::string getNextHop(int);
        std::string getNextHop(int, bool);
        void incPktCountInRoutingTable(std::string);
        bool checkPathid(int);
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

        //virtual void handleMacControlMessage(cMessage *);

        void setSecL(bool flag);
        void setSecL(int pathid, bool flag);
        void pushSecLPathid(int);
        int  popSecLPathid();
        int getSecLPathid();
        bool isSecLPathidEmpty();
        bool secLPerformed(int round, int pathid);
        bool getSecL(int pathid);

        void incPktCountInTrafficTable(std::string, int, int);
        int resolveNetworkAddress(const char*);
        cPacket* decapsulatePacket(cPacket * pkt);

    public:
        msr2mrp_engine(msr2mrp *nw_layer, bool is_sink, bool is_master, std::string sink_addr, std::string self_addr, msr2mrp_feat_par fp, int netDataFrameOverhead, msr2mrpStateDef);

        void timerFiredCallback(int);
        void fromMacLayer(cPacket *, int, double, double);
        void fromApplicationLayer(cPacket *, const char *);
        void handleMacControlMessage(cMessage *);
        void handleNetworkControlCommand(cMessage *);

        msr2mrpRingDef getRingStatus() const;
        msr2mrpStateDef getState() const;
        int getHop() const;
        int getHop(int);
        int getForwDataPkt() { return forw_pkt_count; };
        int  getRound() const;
        bool getSecL();
        bool isMaster() const;

        bool checkRoute(std::string);
       
        bool isLimitedState() { return g_limited_state; };
        void setLimitedState(bool state) { g_limited_state=state; };

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


        std::ostream & extTrace();
        void SetTimer(int index, simtime_t time);
        simtime_t GetTimer(int index);
        void CancelTimer(int index);
        cRNG* getRNG(int index);
};


#endif // _MSR2MRP_ENGINE_H_
