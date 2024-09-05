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

enum MobilityManagerMessageType {
    MOBILITY_PERIODIC = 1;
    DISCRETE_MOBILITY = 70;
    DISCRETE_MOBILITY_ACK = 71;
    DISCRETE_MOBILITY_NACK = 72;

}

message MobilityManagerMessage {
}

message DiscreteMobilityManagerMessage extends MobilityManagerMessage {
    double x;
    double y;
}

message DiscreteMobilityManagerMessageAck extends MobilityManagerMessage {
}

message DiscreteMobilityManagerMessageNack extends MobilityManagerMessage {
}

