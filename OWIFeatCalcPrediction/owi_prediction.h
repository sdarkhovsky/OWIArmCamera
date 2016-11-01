#ifndef OWI_PREDICTION_H
#define OWI_PREDICTION_H

#include <vector>

namespace ais {
	typedef void prediction_function(double time, owi_history& history, std::vector<event>& predicted_events);

	class prediction_function {
	public:
		std::set<std::pair<EVENT_TYPE, double>> causes;
		EVENT_TYPE effect;
		std::vector<parameter> cause_parameters;
		std::vector<parameter> effect_parameters;
		computational_procedure();
		(cause, parameter), (cause, parameter), (effect, parameter) {body}
	};


	void update_prediction_functions(double cur_time, owi_history& history);
	void predict_events(double cur_time, owi_history& history, std::vector<event>& predicted_events);
	void find_events_causing_the_event(event& _event, owi_history& history, std::set<EVENT_TYPE>& causing_event_types)
}

#endif
