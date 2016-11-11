#include "ais.h"

namespace ais {

c_prediction_map prediction_map;
c_history history;

c_history& c_ais::get_history() {
	return history;
}

c_prediction_map& c_ais::get_c_prediction_map() {
	return prediction_map
}

};

