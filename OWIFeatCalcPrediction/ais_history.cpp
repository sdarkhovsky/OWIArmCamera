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

bool c_history::get_next_event(EVENT_TYPE event_type, c_event& prev_event, c_event& event) {
	for (size_t i = prev_event.h_ind+1; i < events.size(); i++) {
		if (events[i].event_type == event_type) {
			event = events[i];
			return true;
		}
	}
	return false;
}

void c_history::add_event(event _event) {
	_event.h_ind = events.size();
	events.push_back(_event);
}

}
