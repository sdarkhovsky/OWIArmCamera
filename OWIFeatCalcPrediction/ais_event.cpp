
namespace ais {

bool b_event::compare_events(event& another_event)
{
	if (event_type != another_event.event_type) return false;
	b_event& event2 = (b_event&)another_event;

	if (center == event2.center && orientation == event2.orientation) return true;
	return false;
}

bool g_event::compare_events(event& another_event)
{
	if (event_type != another_event.event_type) return false;
	g_event& event2 = (g_event&)another_event;

	if (joint_command == event2.joint_command) return true;

	return false;
}

