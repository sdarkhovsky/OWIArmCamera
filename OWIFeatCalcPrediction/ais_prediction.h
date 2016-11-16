#ifndef OWI_PREDICTION_H
#define OWI_PREDICTION_H

#include <vector>
#include <list>
#include <map>

namespace ais {
	class c_cause {
	public:
		c_cause() {
			event_type = UNDEFINED_EVENT;
		}

		c_cause(c_event& cause_event) {
			event_type = cause_event.event_type;
			param_value = cause_event.param_value;
		}

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

		c_effect(EVENT_TYPE _event_type, std::vector<double>& _param_value) {
			event_type = _event_type;
			mean = _param_value;
			variance.resize(mean.size());
			time_delay = 0;
			num_samples = 1;
		}

		c_effect(c_event& effect_event) {
			event_type = effect_event.event_type;
			mean = effect_event.param_value;
			variance.resize(mean.size());
			time_delay = 0;
			num_samples = 1;
		}

		void add_effect_sample(c_event& effect_event);
		bool compare(c_event& event, double& distance);
		EVENT_TYPE event_type;
		double time_delay;
		std::vector<double> mean;
		std::vector<double> variance;
		double num_samples;
	};

	class c_cause_effect_pair {
	public:
		c_cause_effect_pair(c_cause& _cause, c_effect& _effect) {
			cause = _cause;
			effect = _effect;
			hit_counter = 1;
			miss_counter = 0;		
		}
		void print();

		c_cause cause;
		c_effect effect;
		double hit_counter;
		double miss_counter;
		std::list<c_cause_effect_pair>::iterator temp_binary_map_iterator;
	};

	class c_prediction_map {
	public:
		std::list<c_cause_effect_pair> binary_map;
		bool find_cause_effect_map(c_cause& cause, EVENT_TYPE effect_event_type, std::list<c_cause_effect_pair>::iterator& map_iterator);
		bool add_cause_effect_sample(c_event& cause_event, c_event& effect_event);
		bool compare_causes(const c_cause& a, const c_cause& b) const ;
		void get_predictions(c_event& cause_event, std::list<c_cause_effect_pair>& predictions);
	};

	void interpret_observed_events_and_update_prediction_map(double cur_time);
    void predict_events(double cur_time, std::list<c_cause_effect_pair>& predictions);
}

#endif
