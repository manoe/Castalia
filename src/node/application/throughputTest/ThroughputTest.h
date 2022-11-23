/****************************************************************************
 *  Copyright: National ICT Australia,  2007 - 2010                         *
 *  Developed at the ATP lab, Networked Systems research theme              *
 *  Author(s): Athanassios Boulis, Yuriy Tselishchev                        *
 *  This file is distributed under the terms in the attached LICENSE file.  *
 *  If you do not find this file, copies can be found by writing to:        *
 *                                                                          *
 *      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia             *
 *      Attention:  License Inquiry.                                        *
 *                                                                          *
 ****************************************************************************/

#ifndef _THROUGHPUTTEST_H_
#define _THROUGHPUTTEST_H_

#include "node/application/VirtualApplication.h"
#include <map>
#include <deque>
#include <yaml-cpp/yaml.h>

using namespace std;

enum ThroughputTestTimers {
	SEND_PACKET = 1,
    PERIODIC_MEAS = 2
};

struct pkt_stat {
    int sn;
    double time;
};

class ThroughputTest: public VirtualApplication {
 private:
	double packet_rate;
	double startupDelay;
	double delayLimit;
	float packet_spacing;
	int dataSN;
	int recipientId;
	string recipientAddress;
    bool periodic_measurement;
    double meas_period;
    int meas_queue_length;
	
	//variables below are used to determine the packet delivery rates.	
	int numNodes;
	map<long,int> packetsReceived;
	map<long,int> bytesReceived;
	map<long,int> packetsSent;
    map<int,set<int>> packetsSeen;
    map<int,deque<int>> measQueues;
    deque<pkt_stat> sentQueue;

    YAML::Emitter y_per_out;

 protected:
	void startup();
	void fromNetworkLayer(ApplicationPacket *, const char *, double, double);
	void handleRadioControlMessage(RadioControlMessage *);
	void timerFiredCallback(int);
	void finishSpecific();
    bool isPacketSeen(int source, int sn);
    void initMeasQueues();
    void addToMeasQueue(int source, int sn);
    void addToSentQueue(int sn, double time);
    int countPackets(deque<int> recv_queue, deque<pkt_stat> sent_queue);
    deque<pkt_stat> getSentQueueFromNode(int node);


 public:
	int getPacketsSent(int addr) { return packetsSent[addr]; }
	int getPacketsReceived(int addr) { return packetsReceived[addr]; }
	int getBytesReceived(int addr) { return bytesReceived[addr]; }
    deque<pkt_stat> getSentQueue() { return sentQueue; };
	
};

#endif				// _THROUGHPUTTEST_APPLICATIONMODULE_H_
