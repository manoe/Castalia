/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/application/ForestFire/forest_fire.h"

Define_Module(ForestFire);

void ForestFire::startup()
{
	reportTreshold = par("reportTreshold");
//	sampleInterval = (double)par("sampleInterval") / 1000;
	sampleInterval = (double)par("sampleInterval");

    reportDestination = par("reportDestination").stringValue();
	sampleSize = par("sampleSize");

	currentVersionPacket = 0;
	currentVersion = 0;
	currSampleSN = 1;
	outOfEnergy = 0;
	currentSampleAccumulated = 0;

	maxPayload = par("maxPayloadPacketSize");
	div_t tmp_div = div(maxPayload, sampleSize);
	maxSampleAccumulated = tmp_div.quot * sampleSize;

	version_info_table.clear();
	report_info_table.clear();

    report_timer_offset=par("report_timer_offset");
	if (!isSink) {
        if(report_timer_offset) {
            setTimer(ForestFireTimers::REQUEST_SAMPLE, sampleInterval+std::atof(selfAddress.c_str()));
    }
        else {
            setTimer(ForestFireTimers::REQUEST_SAMPLE, sampleInterval);
        }
	}
}


void ForestFire::timerFiredCallback(int timer)
{
	switch (timer) {
		case ForestFireTimers::REQUEST_SAMPLE:{
            trace()<<"REQUEST_SAMPLE timer expired";
			setTimer(REQUEST_SAMPLE, sampleInterval);
			requestSensorReading();
			break;
		}
	}
}

	

void ForestFire::fromNetworkLayer(ApplicationPacket * rcvPacket,
			const char *source, double rssi, double lqi)
{
	string packetName(rcvPacket->getName());

	double data = rcvPacket->getData();
	int sequenceNumber = rcvPacket->getSequenceNumber();

	if (packetName.compare(REPORT_PACKET_NAME) == 0) {
		// this is report packet which contains sensor reading information
		// NOTE that data field is used to store source address instead of using char *source
		// this is done because some routing and flooding is done on the application layer
		// and source address will not always correctly represent the author of the sensed data
//		trace() << "Received report from " << (int)data;
//		if (updateReportTable((int)data, sequenceNumber)) {
//			// forward the packet only if we broadcast reports and this is a new (unseen) report
//			// updateReportTable returns 0 for duplicate packets
//			if (!isSink) {
//				trace() << "Forwarding report packet from node " << (int)data;
//				toNetworkLayer(rcvPacket->dup(), reportDestination.c_str());
//			}
//		}

	}// else if (packetName.compare(REPROGRAM_PACKET_NAME) == 0) {
		// this is version (reprogramming) packet
//		if (!isSink && updateVersionTable(data, sequenceNumber)) {
			// forward the packet only if not sink and its a new packet
			// updateVersionTable returns 0 for duplicate packets
//			toNetworkLayer(rcvPacket->dup(), BROADCAST_NETWORK_ADDRESS);
//		}

//	}
		else {
		trace() << "unknown packet received: [" << packetName << "]";
	}
}

void ForestFire::handleSensorReading(SensorReadingMessage * sensorMsg)
{
	string sensType(sensorMsg->getSensorType());
	double sensValue = sensorMsg->getSensedValue();

    trace()<<"Sensed value: "<<sensValue;

	//if (isSink) {
	//	trace() << "Sink recieved SENSOR_READING (while it shouldnt) "
	//	    << sensValue << " (int)" << (int)sensValue;
	//	return;
	//}
    //
	//if (sensValue < reportTreshold) {
	//	trace() << "Sensed value " << sensValue << " is less than the treshold ("
	//		<< reportTreshold << "), discarding";
	//	return;
	//}
    //
	//currentSampleAccumulated += sampleSize;
	//if (currentSampleAccumulated < maxSampleAccumulated) {
	//	trace() << "Accumulated " << currentSampleAccumulated << "/" << maxSampleAccumulated << " bytes of samples";
	//	return;
	//}
    //
	trace() << "Sending report packet, sequence number " << currSampleSN;

	ForestFirePacket *newPkt = new ForestFirePacket("ForestFire report packet", APPLICATION_PACKET);
//        createGenericDataPacket((double)self, currSampleSN, currentSampleAccumulated);
	newPkt->setName(REPORT_PACKET_NAME);
	newPkt->setData(sensValue);
	newPkt->setSequenceNumber(currSampleSN);
    newPkt->setForestFirePacketKind(ForestFirePacketDef::PERIODIC_REPORT_PACKET);
    newPkt->setByteLength(2);
	toNetworkLayer(newPkt, reportDestination.c_str());
	currentSampleAccumulated = 0;
	currSampleSN++;
}

void ForestFire::finishSpecific()
{
	if (isSink) {
		declareOutput("Report reception");
		for (int i = 0; i < (int)report_info_table.size(); i++) {
			collectOutput("Report reception", report_info_table[i].source,
					"Success", report_info_table[i].parts.size());
			collectOutput("Report reception", report_info_table[i].source,
					"Fail", report_info_table[i].seq - report_info_table[i].parts.size());
		}
	} else {
		declareOutput("Reprogram reception");
		for (int i = 0; i < (int)version_info_table.size(); i++) {
			collectOutput("Reprogram reception", "Success",
				      version_info_table[i].parts.size());
			collectOutput("Reprogram reception", "Fail",
				      version_info_table[i].seq - version_info_table[i].parts.size());
		}
	}
}

//int ForestFire::updateReportTable(int src, int seq)
//{
//	int pos = -1;
//	for (int i = 0; i < (int)report_info_table.size(); i++) {
//		if (report_info_table[i].source == src)
//			pos = i;
//	}
//
//	if (pos == -1) {
//		report_info newInfo;
//		newInfo.source = src;
//		newInfo.parts.clear();
//		newInfo.parts.push_back(seq);
//		newInfo.seq = seq;
//		report_info_table.push_back(newInfo);
//	} else {
//		for (int i = 0; i < (int)report_info_table[pos].parts.size(); i++) {
//			if (report_info_table[pos].parts[i] == seq)
//				return 0;
//		}
//		report_info_table[pos].parts.push_back(seq);
//		if (seq > report_info_table[pos].seq) {
//			report_info_table[pos].seq = seq;
//		}
//	}
//	return 1;
//}
//
//int ForestFire::updateVersionTable(double version, int seq)
//{
//	int pos = -1;
//	for (int i = 0; i < (int)version_info_table.size(); i++) {
//		if (version_info_table[i].version == version)
//			pos = i;
//	}
//
//	if (pos == -1) {
//		version_info newInfo;
//		newInfo.version = version;
//		newInfo.parts.clear();
//		newInfo.parts.push_back(seq);
//		newInfo.seq = seq;
//		version_info_table.push_back(newInfo);
//	} else {
//		for (int i = 0; i < (int)version_info_table[pos].parts.size(); i++) {
//			if (version_info_table[pos].parts[i] == seq)
//				return 0;
//		}
//		version_info_table[pos].parts.push_back(seq);
//		if (seq > version_info_table[pos].seq) {
//			version_info_table[pos].seq = seq;
//		}
//
//	}
//	return 1;
//}
//
