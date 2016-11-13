#ifndef AIS_H
#define AIS_H

#include "ais_history.h"
#include "ais_prediction.h"

// see the notebook #12 for more information

namespace ais {

class c_ais
{
public:
	c_prediction_map prediction_map;
	c_history history;
};


}

#endif
