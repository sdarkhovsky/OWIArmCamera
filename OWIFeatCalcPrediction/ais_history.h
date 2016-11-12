#ifndef AIS_HISTORY_H
#define AIS_HISTORY_H

#include "ais_event.h"
#include "ais_prediction.h"

namespace ais {

class c_history
{
public:
	std::vector<c_event> events;

	bool event_occurred(c_event& _event);

	bool get_event(EVENT_TYPE event_type, double time, c_event& event);
	bool get_first_event(EVENT_TYPE event_type, c_event& event);
	bool get_next_event(EVENT_TYPE event_type, c_event& event);

	bool get_first_event(double time, c_event& event);
	bool get_next_event(double time, c_event& event);

	std::vector<c_event>& get_latest_events(EVENT_TYPE event_type, size_t num_events);

	void add_event(c_event _event);
};

}
#endif
