#include "owi_history.h"

namespace ais {

// see the notebook #12 for additional information
std::vector<event>& owi_history::get_events_by_time(double time) {
	std::vector<event> selected_events;
	for (std::vector<event>::iterator it = events.begin() ; it != events.end(); ++it) {
		if (it->time == time) selected_events.push_back(*it);
	}
	return selected_events;
}


bool owi_history::get_last_event_of_type(EVENT_TYPE _event_type, event& _event) {
	for (vector<event>::reverse_iterator rit = events.rbegin(); rit != events.rend(); ++rit ) { 
		if (rit->event_type == _event_type) {
		_event = *rit;
		return true;
	}
	return false;
}

bool owi_history::get_previous_event_of_same_type(event& _event, event& prev_event) {
	if (_event.h_ind == 0) return false;
	for (size_t i=_event.h_ind-1; i>0; i-- ) { 
		if (events[i].event_type == _event.event_type) {
			prev_event = events[i];
			return true;
		}
	}
	return false;
}

bool owi_history::get_cause_events_for_effect_event(event& effect_event, std::set<event>& cause_types, std::vector<event> cause_events) {
	if (effect_event.h_ind == 0) return false;
	cause_events = cause_types;
	size_t found_cause_events = 0;
	for (size_t ict=0; ict < cause_types.size(); ict++) {
		for (size_t i=effect_event.h_ind-1; i>0; i-- ) { 
			if (effect_event.time + cause_types[ict].time == events[i].time && cause_types[ict].event_type == events[i].event_type) {
				cause_events[ict] = events[i];
				found_cause_events++;
				break;
			}
			// limit search by time
			if (effect_event.time + cause_types[ict].time < events[i].time) break;
	}

	if (found_cause_events == cause_types.size()) return true;

	return false;
}


bool owi_history::event_occurred(event& _event) {
	for (std::vector<event>::iterator it = events.begin() ; it != events.end(); ++it) {
		if (it->compare_events(*pred_it) return true;
	}
	return false;
}

void owi_history::get_events_preceding_the_event(event& _event, std::vector<event>& preceding_events) {
	preceding_events.clear();
	double start_time = _event->time;
	bool found_first_preceding_event = false;
	for (vector<event>::reverse_iterator rit = events.rbegin(); rit != events.rend(); ++rit ) { 
		if (!found_first_preceding_event) {
			if (rit->time < _event->time) {
				found_first_preceding_event = true;
				start_time = rit->time;
				preceding_events.push_back(*rit);
			}
			continue;
		} 

		if (rit->time < start_time) break;
		preceding_events.push_back(*rit);
	}
}

void owi_history::get_events_by_time_interval(double start_time, double end_time, std::vector<event>& events) {
	events.clear();
	for (vector<event>::reverse_iterator rit = events.rbegin(); rit != events.rend(); ++rit ) { 
		if (rit->time <= end_time && rit->time >= start_time) events.push_back(*rit);
		if (rit->time < start_time) break;
	}
}

void owi_history::get_event_counts(event& effect, event& cause, int& n11, int& n01) {
	n11=0; n01=0;
	double time_difference = effect.time - cause.time;
	n11++;
	for (vector<event>::iterator it = events.begin(); it != events.end(); ++it ) { 
		if (it->time >= cause.time) break;
		if (it->event_type == cause.event_type) {
			// find effect event type within the time_difference
			bool found = false;
			for (vector<event>::iterator eff_it = it+1; eff_it != events.end(); ++eff_it ) { 
				if (eff_it->time- it->time > time_difference) break;
				if (eff_it->event_type == effect.event_type) {
					found = true;
					break;
				}
			}
			if (found) n11++;
			else n01++;
		}
	}
}

bool owi_history::get_event(EVENT_TYPE event_type, double time, c_event& event) {
	for (vector<event>::iterator it = events.begin(); it != events.end(); ++it ) { 
		if (it->event_type == event_type && it->time == time) {
			event = *it;
			return true;
	}
	return false;
}

bool owi_history::get_first_event(EVENT_TYPE event_type, c_event& event) {
	for (vector<event>::iterator it = events.begin(); it != events.end(); ++it ) { 
		if (it->event_type == event_type) {
			event = *it;
			return true;
		}
	}
	return false;
}

bool owi_history::get_next_event(EVENT_TYPE event_type, c_event& prev_event, c_event& event) {
	for (size_t i = prev_event.h_ind+1; i < events.size(); i++) {
		if (events[i].event_type == event_type) {
			event = events[i];
			return true;
		}
	}
	return false;
}

void owi_history::add_event(event _event) {
	_event.h_ind = events.size();
	events.push_back(_event);
}

}
