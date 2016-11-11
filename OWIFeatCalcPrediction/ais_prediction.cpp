#include "owi_prediction.h"
#include "owi_history.h"

namespace ais {
	void c_effect::add_effect_sample(event& effect_event) {
		double prev_mean = mean;
		mean = mean*(num_samples/(num_samples+1)+effect_event.param_value/(num_samples+1);
		if (num_samples>0) {
			variance = variance*((num_samples-1)/num_samples)+prev_mean*prev_mean+
				effect_event.param_value*effect_event.param_value/num_samples-mean*mean*((num_samples+1)/num_samples);
		}
		num_samples = num_samples+1;
	}

	bool c_prediction_map::add_cause_effect_sample(event& cause_event, event& effect_event) {
		c_cause cause(cause_event.event_type, cause_event.param_value);
		c_effect& effect = map[cause];
		if (effect.event_type != UNDEFINED_EVENT && effect.event_type != effect_event.event_type) return false;
		effect.add_effect_sample(effect_event);
		return true;
	}

	void update_binary_prediction_map(event& _event) {
		c_prediction_map prediction_map;
		event& cause_event;
		event& effect_event;
 		if (!history.get_first_event(GC_EVENT, cause_event)) return;
		while(true) {
			if (history.get_event(W_EVENT, cause_event.time, effect_event))	{
				prediction_map.add_cause_effect_sample(cause_event, effect_event);
			}
	 		if (!history.get_next_event(GC_EVENT, cause_event)) break;
		}
	}

	void update_prediction_map(double cur_time, owi_history& history, std::vector<event>& predicted_events)
	{
		// check whether the events were correctly predicted
		for (std::vector<event>::iterator pred_it = predicted_events.begin() ; pred_it != predicted_events.end(); ++pred_it) {
			if (history.event_occurred(*pred_it)) continue;

			update_binary_prediction_map(*pred_it);
		}
	}

    void predict_events(double cur_time, owi_history& history, std::vector<event>& predicted_events)
	{
		// check which prediction functions are applicable and apply them;
		for (std::vector<prediction_function>::iterator it = prediction_functions.begin() ; it != prediction_functions.end(); ++it) {		
			*it(cur_time, history, predicted_events);
		}
	}
}

