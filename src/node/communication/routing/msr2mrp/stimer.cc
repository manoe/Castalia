//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2024                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************


#include "stimer.h"

/* time, timer value, offset, the elapsed time of the common timer */
void SerialTimer::setTimer(int machine, int index, simtime_t time, simtime_t offset) {
    TimerItem item;
    item.machine = machine;
    item.index   = index;
    item.time    = time;

    /* sunny day */
    if(timers.size() == 0) {
        timers.push_front(item);
        return;
    }

    /* check for duplacte */
    for(auto &&ti : timers) {
        if(ti.machine == machine && ti.index == index) {
            throw std::out_of_range("Timer already set");
        }
    }
    
    /* update first timer */
    auto first = timers.begin();
    if(first->time < offset) {
        throw std::out_of_range("Offset greater than first timer");
    } else {
        first->time -= offset;
    }

    if(first->time > item.time) {
        timer_change = true;
    }

    simtime_t sum = 0;
    for(auto it = timers.begin() ; it != timers.end() ; ++it ) {
        if(it->time+sum > item.time) {
            item.time -= sum;
            timers.insert(it,item);
            return;
        }
        sum += it->time;
    }
    timers.push_back(item);
}


simtime_t SerialTimer::getTimer(int machine, int index) {
    simtime_t sum;
    for(auto ti : timers) {
        sum += ti.time;
        if(ti.machine == machine && ti.index == index) {
            return sum;
        }
    }
    return -1;
}

void SerialTimer::cancelTimer(int machine, int index) {
    simtime_t offset = 0;
    for(auto it = timers.begin() ; it != timers.end() ; ) {
        if(it->machine == machine && it->index == index) {
            offset += it->time;
            if(it == timers.begin()) {
                timer_change = true;
            }
            it = timers.erase(it);

        }
        else {
            it->time += offset;
            ++it;
        }
    }
}

bool SerialTimer::timerChange() {
    return timer_change;
}

TimerItem SerialTimer::nextTimer() {
    if(0 == timers.size()) {
        throw std::out_of_range("No timer available");
    }
    auto timer = timers.front();
    timers.pop_front();
    return timer;
}

simtime_t SerialTimer::getTimerValue() {
    if(0 == timers.size()) {
        throw std::out_of_range("No timer available");
    }
    timer_change = false;
    return timers.front().time;
}
