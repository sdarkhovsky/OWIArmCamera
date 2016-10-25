#ifndef OWI_HISTORY_H
#define OWI_HISTORY_H

#include "ais_event.h"

class owi_history
{
public:
	std::vector <ais::event> events;
	std::vector <ais::event> get_events_by_time(double time);
	void add_event(ais::event _event);
};

#endif
