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

enum efmrpPacketDef {
    UNDEF_PACKET        = 0;
    HELLO_PACKET        = 1;
    FIELD_PACKET        = 2;
    DATA_PACKET         = 3;
    QUERY_PACKET        = 4;
    QUERY_ACK_PACKET    = 5;
    ALARM_PACKET        = 6; 
}

enum efmrpAlarmDef {
    ENVIRONMENT_ALARM   = 0;
    ENERGY_ALARM        = 1;
}


packet efmrpPacket extends RoutingPacket {
    int efmrpPacketKind enum (efmrpPacketDef);
    string origin;
}

packet efmrpHelloPacket extends efmrpPacket {
    int hop;
    double timestamp;
}

packet efmrpFieldPacket extends efmrpPacket {
    int     hop;
    double  nrg;
    double  env;
}

packet efmrpDataPacket extends efmrpPacket {
    int     pri;
    double  data;
}

packet efmrpQueryPacket extends efmrpPacket {
    int    pri;
}

packet efmrpQueryAckPacket extends efmrpPacket {
    int    pri;
    bool   used;
}

packet efmrpAlarmPacket extends efmrpPacket {
    int efmrpAlarmKind enum (efmrpAlarmDef);
    double env;
}

