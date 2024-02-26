//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2024                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************

#ifndef _STIMER_H_
#define _STIMER_H_

#include <deque>
#include "helpStructures/CastaliaModule.h"

struct TimerItem {
    int machine;
    int index;
    simtime_t time;
};

class SerialTimer {
    private:
        std::deque<TimerItem> timers;
        bool timer_change;
        std::ostream &out;
        simtime_t last_timestamp;
    protected:
        void dumpTimers();
    public:
        SerialTimer() = delete;
        SerialTimer(std::ostream &out, simtime_t current_time) : out(out), timer_change(false), last_timestamp(current_time) { timers.clear(); };
        void setTimer(int machine, int index, simtime_t time, simtime_t current_timer);
        simtime_t getTimer(int machine, int index, simtime_t current_time);
        void cancelTimer(int machine, int index, simtime_t current_time);
        bool timerChange();
        TimerItem nextTimer(simtime_t current_time);
        simtime_t getTimerValue();
};

#endif // _MSR2MRP_H_
