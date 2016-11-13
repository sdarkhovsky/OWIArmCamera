#ifndef OWI_PREDICTION_H
#define OWI_PREDICTION_H

#include <vector>
#include <list>
#include <map>

namespace ais {
	class c_cause {
	public:
		c_cause(EVENT_TYPE _event_type, std::vector<double>& _param_value) {
			event_type = _event_type;
			param_value = _param_value;
		}

		EVENT_TYPE event_type;
		std::vector<double> param_value;
	};

	class c_effect {
	public:
		c_effect() {
			event_type = UNDEFINED_EVENT;
			time_delay = 0;
			num_samples = 0;
		}
		void add_effect_sample(c_event& effect_event);
		EVENT_TYPE event_type;
		double time_delay;
		std::vector<double> mean;
		std::vector<double> variance;
		double num_samples;
	};


	struct compare_causes {
		bool operator()(const c_cause& a, const c_cause& b) const {
			if (a.event_type < b.event_type) return true;

			size_t a_size = a.param_value.size();
			size_t b_size = b.param_value.size();
			if (a_size < b_size) return true;
			for (size_t i = 0; i < a_size; i++) {
				if (a.param_value[i] < b.param_value[i]) return true;
			}
		    return false;
		}
	};


	class c_prediction_map {
	public:
		std::map<c_cause, c_effect, compare_causes> map;
		bool add_cause_effect_sample(c_event& cause_event, c_event& effect_event);
		bool predict_effect_event(c_event& cause_event, c_event& effect_event);
	};


	void update_prediction_map(double cur_time, std::list<c_event>& predicted_events);
	void predict_events(double cur_time, std::list<c_event>& predicted_events);
}

#endif
