#ifndef OWI_HISTORY_H
#define OWI_HISTORY_H

#include "ais_event.h"

namespace ais {

class owi_history
{
public:
	std::vector<ais::event> events;

	std::vector<ais::event> get_events_by_time(double time);
	std::vector<ais::event> get_events_by_type(ais::event& _event);
	void get_events_preceding_the_event(event& _event, std::vector<event>& preceding_events) {
	void get_events_by_time_interval(double start_time, double end_time, std::vector<event>& events);
	void get_event_counts(event& effect, event& cause, int& n11, int& n01);
	void add_event(ais::event _event);
};

}
#endif
