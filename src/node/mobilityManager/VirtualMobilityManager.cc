/****************************************************************************
 *  Copyright: National ICT Australia,  2007 - 2010                         *
 *  Developed at the ATP lab, Networked Systems research theme              *
 *  Author(s): Yuriy Tselishchev                                            *
 *  This file is distributed under the terms in the attached LICENSE file.  *
 *  If you do not find this file, copies can be found by writing to:        *
 *                                                                          *
 *      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia             *
 *      Attention:  License Inquiry.                                        *
 *                                                                          *  
 ****************************************************************************/

#include "node/mobilityManager/VirtualMobilityManager.h"

Define_Module(VirtualMobilityManager);

void VirtualMobilityManager::initialize()
{
	node = getParentModule();
	index = node->getIndex();
	network = node->getParentModule();
	wchannel = network->getSubmodule("wirelessChannel");
	
	if (!network)
		throw cRuntimeError("Unable to obtain SN pointer for deployment parameter");
		
	if (!wchannel)
		throw cRuntimeError("Unable to obtain wchannel pointer");
	
	parseDeployment();	
	trace() << "initial location(x:y:z) is " << nodeLocation.x << ":" << 
			nodeLocation.y << ":" << nodeLocation.z;
}

void VirtualMobilityManager::parseDeployment() {
	const char *ct;
	char *c; 
	double xlen = network->par("field_x");
	double ylen = network->par("field_y");
	double zlen = network->par("field_z");
	nodeLocation.phi = node->par("phi");
	nodeLocation.theta = node->par("theta");
    double randomize_range = network->par("randomize_range");
    double condiff_std_f   = network->par("condiff_std_f");
	
	string deployment = network->par("deployment");
	cStringTokenizer t(deployment.c_str(), ";");
	ct = t.nextToken();
	while (ct != NULL) {
		c = (char*)ct;
		int start_range, end_range;
		while (c[0] && c[0] == ' ')
			c++;
		if (c[0] && c[0] == '[') {
			c++;
			if (!c[0] || c[0] < '0' || c[0] > '9')
				throw cRuntimeError("Bad syntax of SN.deployment parameter: expecing a digit at\n%s", c);
			start_range = strtol(c, &c, 10);
			if (!c[0] || (c[0] != ']' && c[0] != '.'))
				throw cRuntimeError("Bad syntax of SN.deployment parameter: expecing a ']' or '.' at\n%s", c);
			if (c[0] == ']' && start_range != index) {
				ct = t.nextToken();
				continue;
			} else if (c[0] == '.' && c[1] && c[1] == '.') {
				c += 2;
				if (c[0] < '0' || c[0] > '9')
					throw cRuntimeError("Bad syntax of SN.deployment parameter: expecing a digit at\n%s", c);
				end_range = strtol(c, &c, 10);
				if (index > end_range || index < start_range) {
					ct = t.nextToken();
					continue;
				}
			}
			if (!c[0] || c[0] != ']')
				throw cRuntimeError("Bad syntax of SN.deployment parameter: expecing a ']' at\n%s", c);
			c++;
			if (c[0] != '-' || !c[1] || c[1] != '>')
				throw cRuntimeError("Bad syntax of SN.deployment parameter: expecing a '->' at\n%s", c);
			c += 2;
		} else {
			start_range = 0;
		}
			
		int random_flag = 0;
		if (strncmp(c, "uniform", strlen("uniform")) == 0) {
			nodeLocation.x = uniform(0, xlen);
			nodeLocation.y = uniform(0, ylen);
			nodeLocation.z = uniform(0, zlen);
			break;
		} else if (strncmp(c, "center", strlen("center")) == 0) {
			nodeLocation.x = xlen/2;
			nodeLocation.y = ylen/2;
			nodeLocation.z = zlen/2;
			break;
        } else if(strncmp(c, "condiff", strlen("condiff")) == 0) {
            nodeLocation.x = uniform(0, xlen);
			nodeLocation.y = normal(ylen/2, ylen/2*condiff_std_f);
			if (nodeLocation.y > ylen)
				nodeLocation.y = ylen;
			if (nodeLocation.y < 0)
				nodeLocation.y = 0;
            break;
        } else if (strncmp(c, "randomized_", strlen("randomized_")) == 0) {
			c += strlen("randomized_");
			random_flag = 1;
		}

		int gridx, gridy, gridz, gridi;
		gridi = index - start_range;
		if (c[0] < '0' || c[0] > '9')
			throw cRuntimeError("Bad syntax of SN.deployment parameter: expecing 'uniform', 'center', 'NxN[xN]' or 'randomized_NxN[xN]' at\n%s", c);
		gridx = strtol(c, &c, 10);
		if (c[0] != 'x' || !c[1] || c[1] < '0' || c[1] > '9')
			throw cRuntimeError("Bad syntax of SN.deployment parameter: expecing 'x' followed by a digit at\n%s", c);
		c++;
		gridy = strtol(c, &c, 10);
		if (c[0]) {
			if (c[0] != 'x' || !c[1] || c[1] < '0' || c[1] > '9') {
				throw cRuntimeError("Bad syntax of SN.deployment parameter: expecing 'x' followed by a digit at\n%s", c);
			} else {
				c++;
				gridz = strtol(c, &c, 10);
			}
		} else {
			gridz = 0;
		}
		
		// Ugly ?: hack. IF there is only one row/column of sensors, division by 0 happens
        nodeLocation.x = (gridi % gridx) * (xlen / (gridx==1?2:gridx - 1));
		nodeLocation.y = ((int)floor(gridi / gridx) % gridy) * (ylen / (gridy==1?2:gridy - 1));
		if (gridz > 0 && zlen > 0) {
			nodeLocation.z = ((int)floor(gridi / (gridx * gridy)) % gridz) * (zlen / (gridz - 1));
		} else {
			nodeLocation.z = 0;
		}
		
		if (random_flag) {
			nodeLocation.x += normal(0, (xlen / gridx) * randomize_range);
			if (nodeLocation.x > xlen)
				nodeLocation.x = xlen;
			if (nodeLocation.x < 0)
				nodeLocation.x = 0;
			nodeLocation.y += normal(0, (ylen / gridy) * randomize_range);
			if (nodeLocation.y > ylen)
				nodeLocation.y = ylen;
			if (nodeLocation.y < 0)
				nodeLocation.y = 0;
			if (gridz > 0 && zlen > 0) {
				nodeLocation.z += normal(0, (zlen / gridz) * randomize_range);
				if (nodeLocation.z > zlen)
					nodeLocation.z = zlen;
				if (nodeLocation.z < 0)
					nodeLocation.z = 0;
			}
		}
		break;
	}
    if(par("positionOverride")) {
    	nodeLocation.x = node->par("xCoor");
    	nodeLocation.y = node->par("yCoor");
        nodeLocation.z = node->par("zCoor");
    }
}

void VirtualMobilityManager::setLocation(double x, double y, double z, double phi, double theta)
{
	nodeLocation.x = x;
	nodeLocation.y = y;
	nodeLocation.z = z;
	nodeLocation.phi = phi;
	nodeLocation.theta = theta;
	notifyWirelessChannel();
}

void VirtualMobilityManager::setLocation(NodeLocation_type newLocation)
{
	nodeLocation = newLocation;
	notifyWirelessChannel();
}

void VirtualMobilityManager::notifyWirelessChannel()
{
	positionUpdateMsg =
	    new WirelessChannelNodeMoveMessage("location update message", WC_NODE_MOVEMENT);
	positionUpdateMsg->setX(nodeLocation.x);
	positionUpdateMsg->setY(nodeLocation.y);
	positionUpdateMsg->setZ(nodeLocation.z);
	positionUpdateMsg->setPhi(nodeLocation.phi);
	positionUpdateMsg->setTheta(nodeLocation.theta);
	positionUpdateMsg->setNodeID(getParentModule()->getIndex());
	sendDirect(positionUpdateMsg, wchannel, "fromMobilityModule");
}

NodeLocation_type VirtualMobilityManager::getLocation()
{
	return nodeLocation;
}

