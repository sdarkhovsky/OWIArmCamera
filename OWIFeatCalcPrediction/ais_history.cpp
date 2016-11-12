#include "ais_history.h"

namespace ais {

bool c_history::event_occurred(event& _event) {
	for (std::vector<event>::iterator it = events.begin() ; it != events.end(); ++it) {
		if (it->compare_events(*pred_it) return true;
	}
	return false;
}

bool c_history::get_event(EVENT_TYPE event_type, double time, c_event& event) {
	for (vector<event>::iterator it = events.begin(); it != events.end(); ++it ) { 
		if (it->event_type == event_type && it->time == time) {
			event = *it;
			return true;
	}
	return false;
}

bool c_history::get_first_event(EVENT_TYPE event_type, c_event& event) {
	for (vector<event>::iterator it = events.begin(); it != events.end(); ++it ) { 
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
	for (vector<event>::iterator it = events.begin(); it != events.end(); ++it ) { 
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

std::vector<c_event>& c_history::get_latest_events(EVENT_TYPE event_type, size_t num_events) {
	std::vector<c_event> latest_events;
	for (vector<c_event>::reverse_iterator rit = events.rbegin(); rit != events.rend(); ++rit ) { 
		if (rit->event_type == event_type) {
			latest_events.push_back(*rit);
			if (latest_events.size() == num_events) break;
		}
	}
	return latest_events;
}

void c_history::add_event(c_event _event) {
	_event.h_ind = events.size();
	events.push_back(_event);
}

}
