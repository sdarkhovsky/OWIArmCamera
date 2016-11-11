#ifndef AIS_HISTORY_H
#define AIS_HISTORY_H

#include "ais_event.h"
#include "ais_prediction.h"

namespace ais {

class c_history
{
public:
	std::vector<ais::event> events;

	bool event_occurred(event& _event);

	bool get_event(EVENT_TYPE event_type, double time, c_event& event);
	bool get_first_event(EVENT_TYPE event_type, c_event& event);
	bool get_next_event(EVENT_TYPE event_type, c_event& prev_event, c_event& event);

	void add_event(ais::event _event);
};

}
#endif
