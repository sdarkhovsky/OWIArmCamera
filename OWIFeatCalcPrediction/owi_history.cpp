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


std::vector<event>& owi_history::get_events_by_type(event& _event) {
	std::vector<event> selected_events;
	for (std::vector<event>::iterator it = events.begin() ; it != events.end(); ++it) {
		if (it->event_type == _event->event_type) selected_events.push_back(*it);
	}
	return selected_events;
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



void owi_history::add_event(event _event) {
	events.push_back(_event);
}

}
