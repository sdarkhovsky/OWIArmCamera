#include "ais.h"
#include <limits>       // std::numeric_limits

#include "logger.hpp"

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
#ifdef LOGGING
		if (effect_event.event_type == ANGULAR_VELOCITY_EVENT) {
			LOG("time= ", effect_event.time, " mean before=", mean[0], " param_value=", effect_event.param_value[0]);
		}
#endif
		for (i = 0; i < param_size; i++) {
			mean[i] = mean[i]*(num_samples/(num_samples+1))+effect_event.param_value[i]/(num_samples+1);
		}

#ifdef LOGGING
		if (effect_event.event_type == ANGULAR_VELOCITY_EVENT) {
			LOG(" mean after=", mean[0]);
		}
#endif

		if (num_samples>0) {
			for (i = 0; i < param_size; i++) {
				variance[i] = variance[i]*((num_samples-1)/num_samples)+prev_mean[i]*prev_mean[i]+
					effect_event.param_value[i]*effect_event.param_value[i]/num_samples-mean[i]*mean[i]*((num_samples+1)/num_samples);
			}
		}
		num_samples = num_samples+1;
	}

	bool c_effect::compare(c_event& event, double& distance)
	{
		if (event_type != event.event_type) return false;
		if (mean.size() != event.param_value.size()) return false;
		distance = 0;
		for (size_t i = 0; i < mean.size(); i++) {
			distance += (mean[i] - event.param_value[i])*(mean[i] - event.param_value[i]);
		}

		return true;
	}

	bool c_prediction_map::compare_causes(const c_cause& a, const c_cause& b) const {
		if (a.event_type != b.event_type) return false;
		size_t a_size = a.param_value.size();
		size_t b_size = b.param_value.size();
		if (a_size != b_size) return false;
		for (size_t i = 0; i < a_size; i++) {
			if (a.param_value[i] != b.param_value[i]) return false;
		}
	    return true;
	}


	bool c_prediction_map::find_cause_effect_map(c_cause& cause, EVENT_TYPE effect_event_type, std::list<c_cause_effect_pair>::iterator& map_iterator) {
		for (auto it = std::begin(binary_map); it != std::end(binary_map); ++it) {
			if (compare_causes(it->cause, cause) && it->effect.event_type == effect_event_type) {
				map_iterator = it;
				return true;
			}
		}
		return false;
	}

	bool c_prediction_map::add_cause_effect_sample(c_event& cause_event, c_event& effect_event) {
		std::list<c_cause_effect_pair>::iterator map_iterator;
		c_cause cause(cause_event);
		if (!find_cause_effect_map(cause, effect_event.event_type, map_iterator)) {
			c_effect effect(effect_event);
			effect.time_delay = effect_event.time - cause_event.time;

			c_cause_effect_pair cause_effect_pair(cause, effect);
			binary_map.push_back(cause_effect_pair);
			return true;
		}

		map_iterator->effect.add_effect_sample(effect_event);
		return true;
	}

	// append cause-effect pairs to predictions
	void c_prediction_map::get_predictions(c_event& cause_event, std::list<c_cause_effect_pair>& predictions) {
		c_cause cause(cause_event);

		for (auto it = std::begin(binary_map); it != std::end(binary_map); ++it) {
			if (compare_causes(it->cause, cause)) {
				it->temp_binary_map_iterator = it;
				predictions.push_back(*it);
			}
		}
	}

	void c_cause_effect_pair::print() {
		LOG("cause type=",cause.event_type, " param_value=",cause.param_value[0], " effect type=", effect.event_type, " mean=", effect.mean[0], " variance=", effect.variance[0], " var/mean ratio=", effect.mean[0] != 0.0 ? effect.variance[0]/effect.mean[0] : 0);
	}

	void update_effect_event_prediction(double prev_time, c_event& effect_event) {
		c_event cause_event;
		if (g_ais.history.get_event(ACTUATOR_COMMAND_EVENT, prev_time, cause_event))	{
				g_ais.prediction_map.add_cause_effect_sample(cause_event, effect_event);
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

	bool find_best_interpretation(c_event& event, std::list<c_cause_effect_pair>& predictions, std::list<c_cause_effect_pair>::iterator& best_it) {
		double min_distance = std::numeric_limits<double>::max();
		bool  found_interpretation = false;
		for (std::list<c_cause_effect_pair>::iterator it = predictions.begin() ; it != predictions.end(); ++it) {
			double distance;
			if (it->effect.compare(event, distance) && distance < min_distance) {
				min_distance = distance; 
				best_it = it;
				found_interpretation = true;
			}
		}
		return found_interpretation;
	}


	void interpret_observed_events_and_update_prediction_map(double cur_time)
	{
		std::list<c_cause_effect_pair> predictions;

		double prev_time;
		if (!g_ais.history.get_prev_time(cur_time, prev_time)) return;

		predict_events(prev_time, predictions);

#ifdef LOGGING
		for (std::list<c_cause_effect_pair>::iterator it = predictions.begin() ; it != predictions.end(); ++it) {
			it->print();
		}
#endif

		create_ANGULAR_VELOCITY_EVENT_for_the_latest_ORIENTATION_EVENT();

		c_event cur_event;
 		if (!g_ais.history.get_first_event(cur_time, cur_event)) return;
		while(true) {
			if (cur_event.predictable()) {
				std::list<c_cause_effect_pair>::iterator best_it;
				if (find_best_interpretation(cur_event, predictions, best_it)) {
					best_it->temp_binary_map_iterator->effect.add_effect_sample(cur_event);
					best_it->temp_binary_map_iterator->hit_counter++;
					predictions.erase(best_it);
				} 
				else {
					update_effect_event_prediction(prev_time, cur_event);
				}
			}

	 		if (!g_ais.history.get_next_event(cur_time, cur_event)) break;
		}

		for (std::list<c_cause_effect_pair>::iterator it = predictions.begin() ; it != predictions.end(); ++it) {
			// update prediction_map for the predicted but not ocurred events
			
			it->temp_binary_map_iterator->miss_counter++;
		}
	}

    void predict_events(double time, std::list<c_cause_effect_pair>& predictions)
	{
		predictions.clear();

		// find applicable prediction maps
		c_event cause_event;
 		if (!g_ais.history.get_first_event(time, cause_event)) return;
		while(true) {
			g_ais.prediction_map.get_predictions(cause_event, predictions);
	 		if (!g_ais.history.get_next_event(time, cause_event)) break;
		}
	}
}

