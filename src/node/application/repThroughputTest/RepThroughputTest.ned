//****************************************************************************
//*  Copyright: National ICT Australia,  2007 - 2010                         *
//*  Developed at the ATP lab, Networked Systems research theme              *
//*  Author(s): Athanassios Boulis, Yuriy Tselishchev                        *
//*  This file is distributed under the terms in the attached LICENSE file.  *
//*  If you do not find this file, copies can be found by writing to:        *
//*                                                                          *
//*      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia             *
//*      Attention:  License Inquiry.                                        *
//*                                                                          *  
//****************************************************************************/

package node.application.repThroughputTest;

// The sensor node module. Connects to the wireless channel in order to communicate 
// with other nodes. Connects to psysical processes so it can sample them.

simple RepThroughputTest like node.application.iApplication {
 parameters:
 	string applicationID = default ("repThroughputTest");
	bool collectTraceInfo = default (false);
	int priority = default (1);
    int packetRepetition = default (0);
	int packetHeaderOverhead = default (5);		// in bytes
	int constantDataPayload = default (100);	// in bytes
	double delayLimit = default (0); 			// application delay limit (0 - no limit)
											// discard packets arriving after the limit

	string nextRecipient = default ("0");	// Destination for packets received in this node. 
											// These parameters can be used to create an
											// application-level static routing. This way we can
											// have a multi-hop throughput test   

	double packet_rate = default (0);	// packets per second, by default we transmit no packets
	double startupDelay = default (0);	// delay in seconds before the app stars producing packets

	double latencyHistogramMax = default (200.0);
	int latencyHistogramBuckets = default (10);

 gates:
 	output toCommunicationModule;
	output toSensorDeviceManager;
    output toMobilityManager;
	input fromCommunicationModule;
	input fromSensorDeviceManager;
	input fromResourceManager;
    input fromMobilityManager;
}
