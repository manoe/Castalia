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

#define REPORT_PACKET_NAME "Wildfire report"
#define EVENT_PACKET_NAME  "Wildfire event"

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
	REQUEST_SAMPLE = 1,
};

class ForestFire :public VirtualApplication {
 private:
	double reportTreshold;
	double sampleInterval;
	int sampleSize;
	int maxPayload;

	simtime_t outOfEnergy;

	int currentVersion;
	int currentVersionPacket;
	int currSampleSN;
	int currentSampleAccumulated;
	int maxSampleAccumulated;
	int totalVersionPackets;
	int routingLevel;
	vector<version_info> version_info_table;
	vector<report_info> report_info_table;

	string reportDestination;
    bool report_timer_offset;

 protected:
	virtual void startup();
	void finishSpecific();
	void send2NetworkDataPacket(const char *destID, const char *pcktID, int data, int pckSeqNumber, int size);
//	int updateVersionTable(double version, int seq);
//	int updateReportTable(int src, int seq);
	void fromNetworkLayer(ApplicationPacket *, const char *, double, double);
	void timerFiredCallback(int);
	void handleSensorReading(SensorReadingMessage *);

};

#endif				// _FOREST_FIRE_H_
