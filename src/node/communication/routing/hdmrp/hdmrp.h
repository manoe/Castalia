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

#include "node/communication/routing/VirtualRouting.h"
#include "node/communication/routing/hdmrp/hdmrp_m.h"
#include "node/application/ApplicationPacket_m.h"
#include "node/application/ForestFire/forest_fire_packet_m.h"

struct hdmrp_path {
    int path_id;
    string next_hop;
    int nmas;
    int len;
};

class hdmrp: public VirtualRouting {
 private:
     int round;
     int t_l;
     int t_rreq;
     int t_start;
     hdmrpStateDef state;
     hdmrpRoleDef role;
     bool no_role_change;
     bool master;
     double min_rreq_rssi;
     map<int, hdmrp_path> rreq_table;
     std::mt19937 gen(std::random_device());
     map<int, hdmrp_path> routing_table;
     map<unsigned int, hdmrpPacket *> wf_ack_buffer;
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
     hdmrp_path getRoute(const int);
     hdmrp_path getRoute();
     bool RouteExists() const; 

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
     void setRound(int);
     int  getRound() const;

     void bufferForAck(hdmrpPacket *);

 public:
     set<int> getPaths();

};

#endif //HDMRP
