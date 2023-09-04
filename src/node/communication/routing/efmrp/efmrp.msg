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

enum efmrpPacketDef {
    UNDEF_PACKET        = 0;
    HELLO_PACKET        = 1;
    DATA_PACKET         = 2;
    RINV_PACKET         = 3;
    RUPD_PACKET         = 4;
    PING_PACKET         = 16;
    PONG_PACKET         = 32;
}


// RREQ = { R, S, P, len, nmas }
packet efmrpPacket extends RoutingPacket {
    int efmrpPacketKind enum (efmrpPacketDef); // 1 byte
    string origin;
}

packet efmrpHelloPacket extends efmrpPacket {
    int    hop;
    double timestamp;
}

packet efmrpRinvPacket extends efmrpPacket {
    int round;      // 2 byte
    int pathid;     // 1 byte
    int hop;        // 2 byte
    int interf;     // 1 byte
    int emerg;      // 1 byte
}

packet efmrpRupdPacket extends efmrpPacket {
    int     round;      // 2 byte
    int     pathid;     // 1 byte
    int     hop;        // 2 byte
    double  pdr;        // 4 byte
}

packet efmrpDataPacket extends efmrpPacket {
    int pathid;
    int hop;
}

packet efmrpPingPacket extends efmrpPacket {
    int round;
}

packet efmrpPongPacket extends efmrpPacket {
    int round;
}
