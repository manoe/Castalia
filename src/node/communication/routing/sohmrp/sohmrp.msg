//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2022                                        *
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

enum sohmrpPacketDef {
    UNDEF_PACKET        = 0;
    DATA_PACKET         = 1;
    RINV_PACKET         = 2;
    RUPD_PACKET         = 4;
    PING_PACKET         = 16;
    PONG_PACKET         = 32;
}


// RREQ = { R, S, P, len, nmas }
packet sohmrpPacket extends RoutingPacket {
    int sohmrpPacketKind enum (sohmrpPacketDef); // 1 byte
    string origin;
}

packet sohmrpRinvPacket extends sohmrpPacket {
    int round;      // 2 byte
    int pathid;     // 1 byte
    int hop;        // 2 byte
    int interf;     // 1 byte
    int emerg;      // 1 byte
}

packet sohmrpRupdPacket extends sohmrpPacket {
    int     round;      // 2 byte
    int     pathid;     // 1 byte
    int     hop;        // 2 byte
    double  pdr;        // 4 byte
}

packet sohmrpDataPacket extends sohmrpPacket {
    int pathid;
    int hop;
}

packet sohmrpPingPacket extends sohmrpPacket {
    int round;
}

packet sohmrpPongPacket extends sohmrpPacket {
    int round;
}
