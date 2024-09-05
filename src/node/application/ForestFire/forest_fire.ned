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
    double startup_delay = default(0);
	double reportTreshold = default (10);
	double sampleInterval = default (100);	// in s
    double emergencyThreshold = default (0.8);
    double emergencyBroadcastPeriod = default (60); // in sec
    double reportPeriod = default (86400); // 24 hours
    double eventPeriod  = default (120);  // every two min
    bool test_dm = default(false); // Switch to test discrete mobility
    bool dm_sr = default(false); // Signal routing

	int sampleSize = default (12);	// in bytes
	int maxPayloadPacketSize = default (128);	// in bytes

    bool    pot_field = default(false); // potential field mode, i.e. 1 means ok, 0 means event
    double  d_max   = default(8.0);     // d_max is the maximum value of sensor read
    double  d_high  = default(4.0);     // d_high is the threshold, after sensor reading linearly decreases
    double  d_gamma = default(1);       // d_gamma is the gamma term that must be exceeded to trigger update
                                        // routing table


    bool    srlz_pkt_arr = default(false); // serialize packet arrival
    bool    srlz_nrg     = default(false); // serialize nrg consumption
    double  t_srlz_nrg   = default(10);    // srlz_nrg timer


 gates:
 	output toCommunicationModule;
	output toSensorDeviceManager;
    output toMobilityManager;
	input fromCommunicationModule;
	input fromSensorDeviceManager;
	input fromResourceManager;
    input fromMobilityManager;
}

