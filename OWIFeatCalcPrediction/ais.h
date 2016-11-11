#ifndef AIS_H
#define AIS_H

#include "ais_history.h"
#include "aid_prediction.h"

// see the notebook #12 for more information

namespace ais {

class c_ais
{
public:
	c_history& get_history();
	c_prediction_map& get_c_prediction_map();
};


}

#endif
