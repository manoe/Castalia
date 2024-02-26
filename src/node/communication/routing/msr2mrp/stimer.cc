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
void SerialTimer::setTimer(int machine, int index, simtime_t time, simtime_t current_time) {
    out<<"[stimer] setTimer(machine="<<machine<<", index="<<index<<", time="<<time<<", current_time="<<current_time<<")";
    simtime_t offset = current_time - last_timestamp;
    out<<"[stimer]: last_timestamp: "<<last_timestamp<<", offset="<<offset;
    last_timestamp = current_time;
    dumpTimers();
    TimerItem item;
    item.machine = machine;
    item.index   = index;
    item.time    = time;

    /* sunny day */
    if(timers.size() == 0) {
        timer_change = true;
        out<<"[stimer] No timer present, this is the first one.";
        timers.push_front(item);
        return;
    }
    out.flush();

    /* check for duplacte */
    for(auto &&ti : timers) {
        if(ti.machine == machine && ti.index == index) {
            throw std::out_of_range("Timer already set");
        }
    }
    
    /* update first timer */
    auto first = timers.begin();
    out<<"[stimer] first item - machine: "<<first->machine<<" index: "<<first->index<<" time: "<<first->time;
    if(first->time < offset) {
        throw std::out_of_range("Offset greater than first timer, non removed timer?");
    } else {
        first->time -= offset;
    }

    if(first->time > item.time) {
        out<<"[stimer] New timer expires earlier than actual. Refresh needed."; 
        timer_change = true;
        first->time -= item.time;
        timers.push_front(item);
        dumpTimers();
        return;
    }

    simtime_t sum = 0;
    for(auto it = timers.begin() ; it != timers.end() ; ++it ) {
        if(it->time+sum > item.time) {
            item.time -= sum;
            timers.insert(it,item);
            // it csokkentes?
            it->time -= item.time;
            out<<"[stimer] Timer inserted.";
            dumpTimers();
            return;
        }
        sum += it->time;
    }

    out<<"FASZOM5";

    item.time -= sum;
    timers.push_back(item);
    out<<"[stimer] Timer pushed back.";
    dumpTimers();
}


simtime_t SerialTimer::getTimer(int machine, int index, simtime_t current_time) {
    out<<"[stimer] getTimer(machine="<<machine<<", index="<<index<<", current_time="<<current_time<<")";
    dumpTimers();
    simtime_t offset = current_time - last_timestamp;
    out<<"[stimer]: last_timestamp: "<<last_timestamp<<", offset="<<offset;
    last_timestamp = current_time;

    simtime_t sum;
    for(auto ti : timers) {
        sum += ti.time;
        if(ti.machine == machine && ti.index == index) {
            return sum-offset;
        }
    }
    return -1;
}

void SerialTimer::cancelTimer(int machine, int index, simtime_t current_time) {
    out<<"[stimer] cancelTimer(machine="<<machine<<", index="<<index<<", current_time="<<current_time<<")";
    dumpTimers();
    simtime_t offset = 0;
    offset = current_time - last_timestamp;
    out<<"[stimer]: last_timestamp: "<<last_timestamp<<", offset="<<offset;
    last_timestamp = current_time;

    if(timers.begin()->time < offset) {
        throw runtime_error("[error] Timer not removed?");
    }
    timers.begin()->time -= offset;

    for(auto it = timers.begin() ; it != timers.end() ; ) {
        if(it->machine == machine && it->index == index) {
            out<<"[stimer] Timer found";
            offset += it->time;
            if(it == timers.begin() && timers.size() > 1) {
                timer_change = true;
            }
            it = timers.erase(it);

        }
        else {
            it->time += offset;
            ++it;
        }
    }
    dumpTimers();
}

bool SerialTimer::timerChange() {
    // 
    return timer_change;
}

TimerItem SerialTimer::nextTimer(simtime_t current_time) {
    out<<"[stimer] nextTimer()";
    dumpTimers();

    simtime_t offset = 0;
    offset = current_time - last_timestamp;
    out<<"[stimer]: last_timestamp: "<<last_timestamp<<", offset="<<offset;
    last_timestamp = current_time;


    if(0 == timers.size()) {
        throw std::out_of_range("No timer available");
    }
    auto timer = timers.front();
    timers.pop_front();
    if(0 != timers.size()) {
        timer_change = true;
    } else {
        timer_change = false;
    }
    return timer;
}

simtime_t SerialTimer::getTimerValue() {
    out<<"[stimer] getTimerValue()";
    if(0 == timers.size()) {
        throw std::out_of_range("No timer available");
    }
    timer_change = false;
    auto item = timers.front();
    out<<"[stimer] timer - machine: "<<item.machine<<" index: "<<item.index<<" item.time: "<<item.time; 
    return item.time;
}

void SerialTimer::dumpTimers() {
    out<<"[stimer] dumpTimers()";
    for(auto ti: timers) {
        out<<"[stimer] machine: "<<ti.machine<<" index: "<<ti.index<<" time: "<<ti.time<<std::endl;
    }
}

