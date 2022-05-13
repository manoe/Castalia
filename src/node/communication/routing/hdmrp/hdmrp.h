/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#ifndef _HDMRP_H_
#define _HDMRP_H_

#include <random>
#include <algorithm>

#include "node/communication/routing/VirtualRouting.h"
#include "node/communication/routing/hdmrp/hdmrp_m.h"
#include "node/application/ApplicationPacket_m.h"
#include "node/application/ForestFire/forest_fire_packet_m.h"

struct hdmrp_path {
    int path_id;
    string next_hop;
    int nmas;
    int len;
    std::vector<int> path_filter;
};

struct pkt_hist_entry {
    int orig;
    unsigned int seq;
    int l_seq;
    int rep_count;
    simtime_t ts;
};

struct neigh_entry {
    int address;
    bool confirmed;
    vector<int> paths;
    double rssi;
    double lqi;
};


class hdmrp: public VirtualRouting {
 private:
     int round;
     int minor_round;
     int t_l;
     int t_rreq;
     int t_start;
     int t_pkt_hist;
     int t_rsnd;
     int rep_limit;
     int event_ack_req_period;
     int event_pkt_counter;
     hdmrpStateDef state;
     hdmrpRoleDef role;
     bool no_role_change;
     bool master;
     double min_rreq_rssi;
     map<int, hdmrp_path> rreq_table;
     std::mt19937 gen(std::random_device());
     std::map<int, hdmrp_path> routing_table;
     std::map<unsigned int, hdmrpPacket *> wf_ack_buffer;
     std::vector<pkt_hist_entry> pkt_hist;
     std::map<int, neigh_entry> neigh_list;

     int d_pkt_seq;
     int sent_data_pkt;
     int recv_pkt;
     double s_rssi;
     double s_lqi;

 protected:
     void startup();
     void fromApplicationLayer(cPacket *, const char *);
     void fromMacLayer(cPacket *, int, double, double);
     void timerFiredCallback(int);
     void finishSpecific();

     void sendSRREQ();
     void sendRREQ();
     void sendRREQ(int, hdmrp_path);
     void sendRREQ(int, int, int, int); 
     void storeRREQ(hdmrpPacket *);
     hdmrp_path selectRREQ();
     void clearRREQ();
     void removeRREQ(hdmrp_path path);
     bool isRREQempty() const;
     float calculateCost(hdmrp_path) const;

     void addRoute(const hdmrp_path);
     void clearRoutes();
     void removeRoutes(vector<int>);

     hdmrp_path getRoute(const int);
     hdmrp_path getRoute();
     bool RouteExists() const;
     void confirmPaths();

     void sendAck(hdmrpPacket*, int);


     bool isSink() const;
     bool isRoot() const;
     bool isSubRoot() const;
     bool isNonRoot() const;
     void initRole(hdmrpRoleDef);
     void setRole(hdmrpRoleDef);

     bool isMaster() const;
     void setMaster(bool);

     bool isWorkingState() const;
     bool isLearningState() const;
     void setState(hdmrpStateDef);
     void initState(hdmrpStateDef);

     void initRound();
     void newRound();
     bool isNewRound(hdmrpPacket*) const;
     bool isSameRound(hdmrpPacket*) const;
     void setRound(int);
     int  getRound() const;

     void initMinorRound();
     void newMinorRound();
     bool isNewMinorRound(hdmrpPacket*) const;
     void setMinorRound(int);
     int  getMinorRound() const;

     std::vector<int> getPath_filter_array(hdmrpPacket *);
     void setPath_filter(hdmrpPacket *, vector<int>);
     std::vector<int> collectPath_filter();
     bool matchPathFilter(hdmrpPacket *);
     void sendMinorSRREQ(vector<int>);
     void sendMinorRREQ(int round, int minor_round, int path_id, int nmas, int len, vector<int> path_array);
     void sendMinorRREQ(int round, int minor_round, hdmrp_path path, std::vector<int> path_array);
     void sendMinorRREQ(int round, int minor_round, hdmrp_path path);
     void sendMinorRREQ(std::vector<int>);

     void bufferForAck(hdmrpPacket *);
     bool bufferedPktExists(int index);
     hdmrpPacket* getBufferedPkt(int index);

     void sendPathFailure(int);
     int getBackupDestination(int);

     void incrementSeqNum();
     bool findPktHistEntry(pkt_hist_entry);
     void addPktHistEntry(pkt_hist_entry);
     void updatePktHistEntryRepCount(pkt_hist_entry pkt_entry);

 public:
     set<int> getPaths();

};

#endif //HDMRP
