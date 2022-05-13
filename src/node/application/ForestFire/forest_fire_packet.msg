//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2022                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************


cplusplus {{
#include "node/application/ApplicationPacket_m.h"
}}

class ApplicationPacket;

enum ForestFirePacketDef {
    PERIODIC_REPORT_PACKET     = 1;
    EVENT_REPORT_PACKET        = 2;
    EMERGENCY_BROADCAST_PACKET = 3;
}

packet ForestFirePacket extends ApplicationPacket {
    int ForestFirePacketKind enum (ForestFirePacketDef);
}

