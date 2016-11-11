#ifndef AIS_EVENT_H
#define AIS_EVENT_H

#include <vector>
#include <string>

// see the notebook #12 for more information

namespace ais {

enum EVENT_TYPE { UNDEFINED_EVENT, B_EVENT, GC_EVENT, MC_EVENT, W_EVENT, U_EVENT };

class c_event
{
public:
	c_event() {
			event_type = UNDEFINED_EVENT;
			time = 0;
			param_value = 0;
	}

	c_event(double _time, EVENT_TYPE _event_type, std::vector<double> _param_value)	{
		event_type = _event_type;
		time = _time;
		param_value = _param_value;
	}
	virtual bool compare_events(event& another_event) =0;

	double time; // event occurrence time in history, or time relative to an effect event if used in the cause-effect context
	EVENT_TYPE event_type;
	std::vector<double> param_value;
	size_t h_ind; // index in the history.events
};


}

#endif
