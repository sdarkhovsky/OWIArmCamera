// define LOGGING_LEVEL_1 or LOGGING_LEVEL_2 here or add them to CPPFLAGS in the makefile. For example, #define LOGGING_LEVEL_1 or CPPFLAGS=-DLOGGING_LEVEL_1
#include "logger.hpp"

int main()
{
	LOG("Starting the application..");
	for( short i = 0 ; i < 3 ; i++ )
	{
		LOG("The value of 'i' is ", i , ". " , 3 - i - 1 , " more iterations left ");
	}
	LOG_WARN("Loop over");
	LOG_ERR("All good things come to an end.. :(");
	return 0;
}
