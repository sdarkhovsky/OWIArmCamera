#ifndef OWI_PREDICTION_H
#define OWI_PREDICTION_H

#include <vector>

namespace ais {
	typedef void prediction_function(double time, owi_history& history, std::vector<event>& predicted_events);

	void update_prediction_functions(double cur_time, owi_history& history);
	void predict_events(double cur_time, owi_history& history, std::vector<event>& predicted_events);
}

#endif
