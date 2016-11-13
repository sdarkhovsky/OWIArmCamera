#include "ais_event.h"

namespace ais {

bool c_event::compare_events(c_event& another_event)
{
	if (event_type != another_event.event_type) return false;
	if (param_value.size() != another_event.param_value.size()) return false;
	for (size_t i = 0; i < param_value.size(); i++) {
		if (param_value[i] != another_event.param_value[i]) return false;
	} 

	return true;
}

}
