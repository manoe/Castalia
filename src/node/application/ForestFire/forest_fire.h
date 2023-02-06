/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#ifndef _FOREST_FIRE_H_
#define _FOREST_FIRE_H_

#include "node/application/VirtualApplication.h"
#include "node/application/ForestFire/forest_fire_packet_m.h"
#include "node/application/ForestFire/forest_fire_message_m.h"
#include <yaml-cpp/yaml.h>

#define REPORT_PACKET_NAME "Wildfire report"
#define EVENT_PACKET_NAME  "Wildfire event"
#define BROADCAST_PACKET_NAME "Wildfire broadcast"

struct version_info {
	double version;
	int seq;
	vector<int> parts;
};

struct report_info {
	int source;
	int seq;
	vector<int> parts;
};

enum ForestFireTimers {
	REQUEST_SAMPLE      = 1,
    EMERGENCY_BROADCAST = 2,
    EVENT_PERIOD        = 3,
    REPORT_PERIOD       = 4
};

class ForestFire : public VirtualApplication {
 private:
	double reportTreshold;
	double sampleInterval;
	int sampleSize;
	int maxPayload;
    double sensedValue;
	simtime_t outOfEnergy;
    double emergency_broadcast;
    double event_period;
    double report_period;
    double emergency_threshold;
    bool emergency;

	int currentVersion;
	int currentVersionPacket;
	int currSampleSN;
	int maxSampleAccumulated;
	int totalVersionPackets;
	int routingLevel;
	vector<version_info> version_info_table;
	vector<report_info> report_info_table;
    int event_sent;
    int report_sent;

	string reportDestination;
    bool report_timer_offset;

    map<int,set<int>> packetsSeen;
    map<int,int> reportRecv;
    map<int,int> eventRecv;
 protected:
	virtual void startup();
	void finishSpecific();
	void send2NetworkDataPacket(const char *destID, const char *pcktID, int data, int pckSeqNumber, int size);
//	int updateVersionTable(double version, int seq);
//	int updateReportTable(int src, int seq);
	void fromNetworkLayer(ApplicationPacket *, const char *, double, double);
	void timerFiredCallback(int);
	void handleSensorReading(SensorReadingMessage *);
    void sendEvent();
    void sendReport();
    void sendEmergencyBroadcast();
    bool isPacketSeen(int source, int sn);
    void alertRouting();
 public:
    int getReportSent();
    int getEventSent();
};

#endif				// _FOREST_FIRE_H_
