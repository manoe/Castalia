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
#include "node/resourceManager/ResourceManager.h"
#include "node/mobilityManager/MobilityManagerMessage_m.h"
#include "physicalProcess/WildFirePhysicalProcess/WildFirePhysicalProcess.h"
#include "node/communication/routing/VirtualRouting.h"
#include <yaml-cpp/yaml.h>

#define REPORT_PACKET_NAME "Wildfire report"
#define EVENT_PACKET_NAME  "Wildfire event"
#define BROADCAST_PACKET_NAME "Wildfire broadcast"
#define MOBILITY_PACKET_NAME "Wildfire mobility"

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
    REPORT_PERIOD       = 4,
    SRLZ_NRG            = 5,
    TEST_DM             = 6,
    DM_REST             = 7
};

class ForestFire : public VirtualApplication {
 private:
	double reportTreshold;
	double sampleInterval;
	int sampleSize;
	int maxPayload;
    double sensedValue;
	simtime_t outOfEnergy;
    bool test_dm;
    double test_dm_timer;
    bool dm_sr;
    bool dm_support;
    bool rnd_dm;
    double emergency_broadcast;
    double event_period;
    double report_period;
    double startup_delay;
    double emergency_threshold;
    double mobility_threshold;
    bool emergency;
    ResourceManager *rm;
    WildFirePhysicalProcess *wfphy_proc;
    double sense_and_mob_rad; 
    bool rest_dm_state;
    double rest_dm_timer;
    int dm_count;

    bool    pot_field;
    double  d_max;
    double  d_high;
    double  d_gamma;

    bool    srlz_pkt_arr;
    bool    srlz_nrg;
    double  t_srlz_nrg;

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

    map<int,set<int>> reportPacketsSeen;
    map<int,set<int>> eventPacketsSeen;
    map<int,int> reportRecv;
    map<int,int> eventRecv;
    YAML::Emitter yp_out;
    YAML::Emitter yn_out;

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
    void sendMobilityBroadcast();
    bool isPacketSeen(int source, int sn, std::string name);
    void alertRouting(MsgType);
    double getAverageSpentEnergy();
    void serializeEnergy();
    map<int,int> summarizeSentPkts(std::vector<map<int,set<int>>>);
    void handleMobility(cMessage *);
    void notifySensorManager(int);
    std::map<int,int> getRecvPktSum(bool);
 public:
    int getReportSent();
    int getEventSent();
    map<int,int> getReportRecv();
    map<int,int> getEventRecv();
    map<int,set<int>> getReportPacketsSeen();
    map<int,set<int>> getEventPacketsSeen();
    double getEnergyValue();
    double getEmergencyValue();
    bool isPacketSeen(int source, ApplicationPacket *pkt);
};

#endif				// _FOREST_FIRE_H_
