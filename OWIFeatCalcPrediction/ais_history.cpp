#include "ais.h"

#define MAIN_LOGGER
#include "logger.hpp"

namespace ais {

bool c_history::event_occurred(c_event& _event) {
	for (std::vector<c_event>::iterator it = events.begin() ; it != events.end(); ++it) {
		if (it->compare_events(*it)) return true;
	}
	return false;
}

bool c_history::get_event(EVENT_TYPE event_type, double time, c_event& event) {
	for (std::vector<c_event>::iterator it = events.begin(); it != events.end(); ++it ) { 
		if (it->event_type == event_type && it->time == time) {
			event = *it;
			return true;
		}
	}
	return false;
}

bool c_history::get_first_event(EVENT_TYPE event_type, c_event& event) {
	for (std::vector<c_event>::iterator it = events.begin(); it != events.end(); ++it ) { 
		if (it->event_type == event_type) {
			event = *it;
			return true;
		}
	}
	return false;
}

bool c_history::get_next_event(EVENT_TYPE event_type, c_event& event) {
	for (size_t i = event.h_ind+1; i < events.size(); i++) {
		if (events[i].event_type == event_type) {
			event = events[i];
			return true;
		}
	}
	return false;
}

bool c_history::get_first_event(double time, c_event& event) {
	for (std::vector<c_event>::iterator it = events.begin(); it != events.end(); ++it ) { 
		if (it->time == time) {
			event = *it;
			return true;
		}
	}
	return false;
}

bool c_history::get_next_event(double time, c_event& event) {
	for (size_t i = event.h_ind+1; i < events.size(); i++) {
		if (events[i].time == time) {
			event = events[i];
			return true;
		}
	}
	return false;
}

bool c_history::get_last_events(EVENT_TYPE event_type, size_t num_events, std::vector<c_event>& last_events) {
	last_events.resize(num_events);
	size_t i = num_events-1;
	for (std::vector<c_event>::reverse_iterator rit = events.rbegin(); rit != events.rend(); ++rit ) { 
		if (rit->event_type == event_type) {
			last_events[i]=*rit;
			if (i == 0) return true;
			i--;
		}
	}
	return false;
}

bool c_history::get_prev_time(double time, double& prev_time) {
	for (std::vector<c_event>::reverse_iterator rit = events.rbegin(); rit != events.rend(); ++rit ) { 
		if (rit->time < time) {
			prev_time = rit->time;
			return true;
		}
	}
	return false;
}

void c_history::add_event(c_event _event) {
	_event.h_ind = events.size();
	events.push_back(_event);
}

void c_history::print() {
#ifdef LOGGING
	for (std::vector<c_event>::iterator it = events.begin(); it != events.end(); ++it ) { 
		if (it->param_value.size() > 0) LOG("time=", it->time , "  event_type=", it->event_type, "  param_value=", it->param_value[0]);
		else LOG("time=", it->time , "  event_type=", it->event_type);
	}
#endif
}

}
