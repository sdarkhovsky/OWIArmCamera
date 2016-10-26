#include "owi_prediction.h"
#include "owi_history.h"

namespace ais {
	std::vector<prediction_function> prediction_functions;


	void find_events_causing_the_event(event& _event, owi_history& history, std::set<EVENT_TYPE>& causing_event_types)
	{
		// see the notebook 12, p.34
		double n11, n01;
		std::vector<event> preceding_events;
		
		causing_events.clear();
		history.get_events_preceding_the_event(_event, preceding_events);

		for (std::vector<event>::iterator it = preceding_events.begin() ; it != preceding_events.end(); ++it) {
			history.get_event_counts(_event, *it, n11, n01);
#define CAUSING_THRESHOLD 0.8
			if (n11/(n11+n01) > CAUSING_THRESHOLD) causing_event_types.insert(it->event_type);
		}
	}

	void update_prediction_functions(double cur_time, owi_history& history, std::vector<event>& predicted_events)
	{
		// check whether the events were correctly predicted
		// this implementation checks only the latest events, rather than any history event
		std::vector<ais::event> latest_events = history.get_events_by_time(cur_time);
		// compare the actual events and the predicted events
		for (std::vector<event>::iterator it = latest_events.begin() ; it != latest_events.end(); ++it) {
			bool found = false;
			for (std::vector<event>::iterator pred_it = predicted_events.begin() ; pred_it != predicted_events.end(); ++pred_it) {
				found = it->compare_with_predicted_event(*pred_it);
				if (found) break;
			}
			if (found) continue;

			// the event was not predicted, create a prediction function for the actual event type
			
			find_events_causing_the_event(*it, history);

			// the prediction function is created in run time
			map_the_causing_events_parameters_to_the_event_parameters();
		}
	}

    void predict_events(double cur_time, owi_history& history, std::vector<event>& predicted_events)
	{
		// check which prediction functions are applicable and apply them;
		for (std::vector<prediction_function>::iterator it = prediction_functions.begin() ; it != prediction_functions.end(); ++it) {		
			*it(cur_time, history, predicted_events);
		}
	}
}













111111111111111111111111111111111111111111


void update_prediction(b_event& be)
{
	// select preceding events
    const b_event& prev_b_event = *std::prev(predicted_event);
}

void update_predictions(owi_history& history)
{
	/* check if the latest history records can be predicted correctly
     * if not, update the incorrect prediction functions */
	predict_events(

	const b_event& actual_b_event = b_events.back();
	update_prediction(actual_b_event);
}


111111111111111111111111111111111111111111111111111
void owi_history::predict()
{
	std::vector<b_event>::iterator be_it, prev_be_it;

	// predict history events 
	prev_be_it = b_events.begin();
	be_it = prev_be_it; be_it++;
 	while ( be_it != b_events.end())
	{
		pred_b_event pred_be;
		pred_be.predict(be_it->time);
		pred_b_events.push_back(pred_be);
	}
}

void learn(double time, owi_history history)
{
	const b_event& predicted_event = b_events.back();

	// select preceding events
    const b_event& prev_b_event = *std::prev(predicted_event);

	const g_event& prev_g_event = g_events.back();
	while (prev_g_event.time >= predicted_event.time)
		prev_g_event = *std::prev(prev_g_event);

	// find parameters of the prediction fcn
	predicted_event

}

b_event& predict(double time)
{

}

