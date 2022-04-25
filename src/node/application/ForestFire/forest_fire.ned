//********************************************************************************
//*  Copyright: National ICT Australia,  2007 - 2010                             *
//*  Developed at the ATP lab, Networked Systems research theme                  *
//*  Author(s): Yuriy Tselishchev                                                *
//*  This file is distributed under the terms in the attached LICENSE file.      *
//*  If you do not find this file, copies can be found by writing to:            *
//*                                                                              *
//*      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia                 *
//*      Attention:  License Inquiry.                                            *
//*                                                                              *
//*******************************************************************************/

package node.application.ForestFire;

// The sensor node module. Connects to the wireless channel in order to communicate 
// with other nodes. Connects to psysical processes so it can sample them.

simple ForestFire like node.application.iApplication {
 parameters: 
 	string applicationID = default ("ForestFire");
	bool collectTraceInfo = default (false);
	int priority = default (1);
	int packetHeaderOverhead = default (8);		// in bytes
	int constantDataPayload = default (0);

    bool isSink = default(false);
    bool isMaster = default(false);
    bool noRoleChange = default(false);
    string Role = default("NonRoot"); // NonRoot, SubRoot, Root
    

//      bool broadcastReports = default(false);
	string reportDestination = default ("0");	//this can also be set to "-1" for broadcast
												// or to a specific node's address (e.g "0")
    bool report_timer_offset = default (true);
	double reportTreshold = default (10);
	double sampleInterval = default (100);	// in s
	int sampleSize = default (12);	// in bytes
	int maxPayloadPacketSize = default (128);	// in bytes

 gates:
 	output toCommunicationModule;
	output toSensorDeviceManager;
	input fromCommunicationModule;
	input fromSensorDeviceManager;
	input fromResourceManager;
}

