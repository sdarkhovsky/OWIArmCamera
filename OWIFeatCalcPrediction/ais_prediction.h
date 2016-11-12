#ifndef OWI_PREDICTION_H
#define OWI_PREDICTION_H

#include <vector>

namespace ais {
	class c_cause {
	public:
		c_cause(EVENT_TYPE _event_type, std::vector<double>& _param_value) {
			event_type = _event_type;
			param_value = _param_value;
		}

		EVENT_TYPE event_type;
		std::vector<double> param_value;
	}

	class c_effect {
	public:
		c_effect() {
			event_type = UNDEFINED_EVENT;
			time_delay = 0;
			mean = 0;
			variance = 0;
			num_samples = 0;
		}
		void add_effect_sample(event& effect_event);
		EVENT_TYPE event_type;
		double time_delay;
		double mean;
		double variance;
		double num_samples;
	}


	class c_prediction_map {
	public:
		std::map<c_cause, c_effect> map;
		bool add_cause_effect_sample(event& cause_event, event& effect_event);
		bool predict_effect_event(event& cause_event, event& effect_event);
	};


	void update_prediction_map(double cur_time);
	void predict_events(double cur_time, std::vector<event>& predicted_events);
}

#endif
