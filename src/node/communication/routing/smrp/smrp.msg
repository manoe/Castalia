//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2023                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************


cplusplus {{
#include "node/communication/routing/RoutingPacket_m.h"
}}

class RoutingPacket;

enum smrpPacketDef {
    UNDEF_PACKET        = 0;
    HELLO_PACKET        = 1;
    FIELD_PACKET        = 2;
    DATA_PACKET         = 3;
    QUERY_PACKET        = 4;
    QUERY_ACK_PACKET    = 5;
    RETREAT_PACKET      = 6;
    ALARM_PACKET        = 7; 
}

enum smrpAlarmDef {
    ENVIRONMENT_ALARM   = 0;
    ENERGY_ALARM        = 1;
    MOBILITY_ALARM      = 2;
}

struct smrpHop {
    int sink;
    int hop;
}


packet smrpPacket extends RoutingPacket {
    int smrpPacketKind enum (smrpPacketDef);
    string origin;
}

packet smrpHelloPacket extends smrpPacket {
    int     hop;
    int     sink;
    double  env;
    double  nrg;
    double  timestamp;
}

packet smrpFieldPacket extends smrpPacket {
    smrpHop hop[];
    double  env;
    double  nrg;
    double  trg;
}

packet smrpDataPacket extends smrpPacket {
    int     pri;
    int     sink;
    double  data;
}

packet smrpQueryPacket extends smrpPacket {
}

packet smrpQueryAckPacket extends smrpPacket {
    bool   used;
}

packet smrpRetreatPacket extends smrpPacket {
    int    pri;
    int    sink;
}

packet smrpAlarmPacket extends smrpPacket {
    int smrpAlarmKind enum (smrpAlarmDef);
    double env;
    double nrg;
    double trg;
}

packet smrpDepthUpdatePacket extends smrpPacket {
    int hop;
}
