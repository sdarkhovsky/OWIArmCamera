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
		// this assumes that each cause has an effect with only one type of effect
        // in general any pair of events should be allowed for the cause and effect events
		c_effect& effect = map[cause];
		if (effect.event_type != UNDEFINED_EVENT && effect.event_type != effect_event.event_type) return false;
		effect.add_effect_sample(effect_event);
		return true;
	}

	bool c_prediction_map::predict_effect_event(event& cause_event, event& effect_event) {
		c_cause cause(cause_event.event_type, cause_event.param_value);
		c_effect& effect = map[cause];
		if (effect.event_type == UNDEFINED_EVENT) return false;
		std::vector<double> param_value;
		param_value.push_back(effect.mean);
		c_event _effect_event(cause_event.time + effect.time_delay, effect.event_type, param_value);
		effect_event = 	_effect_event;
		return true;
	}

	void update_binary_prediction_map(event& _event) {
		event& cause_event;
		event& effect_event;
 		if (!history.get_first_event(GC_EVENT, cause_event)) return;
		while(true) {
			if (history.get_event(ANGULAR_VELOCITY_EVENT, cause_event.time, effect_event))	{
				prediction_map.add_cause_effect_sample(cause_event, effect_event);
			}
	 		if (!history.get_next_event(GC_EVENT, cause_event)) break;
		}
	}

	void create_ANGULAR_VELOCITY_EVENT_for_the_latest_ORIENTATION_EVENT()
	{
		EVENT_TYPE event_type = ORIENTATION_EVENT;
		EVENT_TYPE derived_event_type = ANGULAR_VELOCITY_EVENT;
		size_t num_events = 2;
		std::vector<c_event>& latest_events = history.get_latest_events(event_type, num_events);
		if (events.size() != num_events) return;

		std::vector<double> param_value;
		double angular_velocity = (latest_events[1].param_value - latest_events[1].param_value)/(latest_events[1].time - latest_events[1].time);
		param_value.push_back(angular_velocity);
		history.add_event(c_event(time, ANGULAR_VELOCITY_EVENT, param_value));
	}

	void update_prediction_map(double cur_time, std::vector<event>& predicted_events)
	{
		create_ANGULAR_VELOCITY_EVENT_for_the_latest_ORIENTATION_EVENT();

		// check whether the events were correctly predicted
		for (std::vector<event>::iterator pred_it = predicted_events.begin() ; pred_it != predicted_events.end(); ++pred_it) {
			if (history.event_occurred(*pred_it)) continue;

			update_binary_prediction_map(*pred_it);
		}
	}

    void predict_events(double cur_time, std::vector<event>& predicted_events)
	{
		// check which prediction maps are applicable and apply them;
		event& cause_event;
		event& effect_event;
		predicted_events.clear();
 		if (!history.get_first_event(cur_time, cause_event)) return;
		while(true) {
			if (prediction_map.predict_effect_event(cause_event, effect_event)) predicted_events.push_back(effect_event);
	 		if (!history.get_next_event(cur_time, cause_event)) break;
		}
	}
}

