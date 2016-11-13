#include "ais.h"

namespace ais {
	extern c_ais g_ais;

	void c_effect::add_effect_sample(c_event& effect_event) {
		size_t i;
		// see notebook 12, p.38
		std::vector<double> prev_mean = mean;
		size_t param_size = effect_event.param_value.size();
		if (num_samples == 0) {
			mean.resize(param_size);
			variance.resize(param_size);
		}
		for (i = 0; i < param_size; i++) {
			mean[i] = mean[i]*(num_samples/(num_samples+1))+effect_event.param_value[i]/(num_samples+1);
		}
		if (num_samples>0) {
			for (i = 0; i < param_size; i++) {
				variance[i] = variance[i]*((num_samples-1)/num_samples)+prev_mean[i]*prev_mean[i]+
					effect_event.param_value[i]*effect_event.param_value[i]/num_samples-mean[i]*mean[i]*((num_samples+1)/num_samples);
			}
		}
		num_samples = num_samples+1;
	}

	bool c_prediction_map::add_cause_effect_sample(c_event& cause_event, c_event& effect_event) {
		c_cause cause(cause_event.event_type, cause_event.param_value);
		// this assumes that each cause has an effect with only one type of effect
        // in general any pair of events should be allowed for the cause and effect events
		c_effect& effect = map[cause];
		if (effect.event_type != UNDEFINED_EVENT && effect.event_type != effect_event.event_type) return false;
		effect.add_effect_sample(effect_event);
		return true;
	}

	bool c_prediction_map::predict_effect_event(c_event& cause_event, c_event& effect_event) {
		c_cause cause(cause_event.event_type, cause_event.param_value);
		c_effect& effect = map[cause];
		if (effect.event_type == UNDEFINED_EVENT) return false;
		effect_event = 	c_event(cause_event.time + effect.time_delay, effect.event_type, effect.mean);
		return true;
	}

	void update_binary_prediction_map(c_event& _event) {
		c_event cause_event;
		c_event effect_event;
 		if (!g_ais.history.get_first_event(GC_EVENT, cause_event)) return;
		while(true) {
			if (g_ais.history.get_event(ANGULAR_VELOCITY_EVENT, cause_event.time, effect_event))	{
				g_ais.prediction_map.add_cause_effect_sample(cause_event, effect_event);
			}
	 		if (!g_ais.history.get_next_event(GC_EVENT, cause_event)) break;
		}
	}

	void create_ANGULAR_VELOCITY_EVENT_for_the_latest_ORIENTATION_EVENT()
	{
		EVENT_TYPE event_type = ORIENTATION_EVENT;
		EVENT_TYPE derived_event_type = ANGULAR_VELOCITY_EVENT;
		size_t num_events = 2;
		std::vector<c_event> last_events;
		if (!g_ais.history.get_last_events(event_type, num_events, last_events)) return;

		std::vector<double> param_value;
		double angular_velocity = (last_events[1].param_value[0] - last_events[0].param_value[0])/(last_events[1].time - last_events[0].time);
		param_value.push_back(angular_velocity);
		g_ais.history.add_event(c_event(last_events[1].time, derived_event_type, param_value));
	}

	void update_prediction_map(double cur_time, std::list<c_event>& predicted_events)
	{
		create_ANGULAR_VELOCITY_EVENT_for_the_latest_ORIENTATION_EVENT();

		c_event cur_event;
 		if (!g_ais.history.get_first_event(cur_time, cur_event)) return;
		while(true) {
			// check whether the event was predicted
			bool event_was_predicted = false;
			for (std::list<c_event>::iterator it = predicted_events.begin() ; it != predicted_events.end(); ++it) {
				if (cur_event.compare_events(*it)) {
					predicted_events.erase(it);
					event_was_predicted = true;
					break;
				}
			}

			if (!event_was_predicted) {
11111111111111111111111 modify update_binary_prediction_map to detect causes of cur_event
				update_binary_prediction_map(cur_event);
			}

	 		if (!g_ais.history.get_next_event(cur_time, cur_event)) break;
		}

		for (std::list<c_event>::iterator it = predicted_events.begin() ; it != predicted_events.end(); ++it) {
1111111111111111111111111111111111111111
update prediction_map for teh predicted but not ocurred events
		}
	}

    void predict_events(double cur_time, std::list<c_event>& predicted_events)
	{
		// check which prediction maps are applicable and apply them;
		c_event cause_event;
		c_event effect_event;
		predicted_events.clear();
 		if (!g_ais.history.get_first_event(cur_time, cause_event)) return;
		while(true) {
			if (g_ais.prediction_map.predict_effect_event(cause_event, effect_event)) predicted_events.push_back(effect_event);
	 		if (!g_ais.history.get_next_event(cur_time, cause_event)) break;
		}
	}
}

