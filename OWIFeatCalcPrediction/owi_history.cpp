#include "owi_history.h"

// see the notebook #12 for additional information
std::vector<ais::event> owi_history::get_events_by_time(double time) {
	std::vector<ais::event> selected_events;
	for (std::vector<ais::event>::iterator it = events.begin() ; it != events.end(); ++it) {
		if (it->time == time) selected_events.push_back(*it);
	}
}


void owi_history::add_event(ais::event _event) {
	events.push_back(_event);
}

