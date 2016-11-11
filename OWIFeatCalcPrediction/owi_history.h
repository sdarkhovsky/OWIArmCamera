#ifndef OWI_HISTORY_H
#define OWI_HISTORY_H

#include "ais_event.h"

namespace ais {

class owi_history
{
public:
	std::vector<ais::event> events;

	std::vector<ais::event> get_events_by_time(double time);
	bool get_last_event_of_type(EVENT_TYPE _event_type, event& _event);
	bool get_previous_event_of_same_type(event& _event, event& prev_event);
	bool event_occurred(event& _event);
	bool get_cause_events_for_effect_event(event& effect_event, std::set<event>& cause_event_types, std::vector<event>& cause_events);

	void get_events_preceding_the_event(event& _event, std::vector<event>& preceding_events);
	void get_events_by_time_interval(double start_time, double end_time, std::vector<event>& events);
	void get_event_counts(event& effect, event& cause, int& n11, int& n01);

	bool get_event(EVENT_TYPE event_type, double time, c_event& event);
	bool get_first_event(EVENT_TYPE event_type, c_event& event);
	bool get_next_event(EVENT_TYPE event_type, c_event& prev_event, c_event& event);

	void add_event(ais::event _event);
};

}
#endif
