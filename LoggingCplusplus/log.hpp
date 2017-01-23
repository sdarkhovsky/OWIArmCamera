// Logging code is taken from http://www.drdobbs.com/cpp/a-lightweight-logger-for-c/240147505#
#ifndef LOG_HPP
#define LOG_HPP

#include <string>
#include <sstream>
#include <mutex>
#include <memory>
#include <fstream>

namespace logging {

// listing 6
enum severity_type
{
	debug = 1,
	error,
	warning
};

// listing 8
template< typename log_policy >
class logger
{
	unsigned log_line_number;
	std::string get_time();
	std::string get_logline_header();
	std::stringstream log_stream;
	log_policy* policy;
	std::mutex write_mutex;
	
	//Core printing functionality
	void print_impl();
	template<typename First, typename...Rest>
	void print_impl(First parm1, Rest...parm);
public:
	logger( const std::string& name );

	template< severity_type severity , typename...Args >
	void print( Args...args );

	~logger();
};

// listing 5
template< typename log_policy >
	template< severity_type severity , typename...Args >
void logger< log_policy >::print( Args...args )
{
	write_mutex.lock();
	switch( severity )
	{
		case severity_type::debug:
			log_stream<<"<DEBUG> :";
			break;
		case severity_type::warning:
			log_stream<<"<WARNING> :";
			break;
		case severity_type::error:
			log_stream<<"<ERROR> :";
			break;
	};
	print_impl( args... );
	write_mutex.unlock();
}

// listing 3
class log_policy_interface
{
public:
	virtual void		open_ostream(const std::string& name) = 0;
	virtual void		close_ostream() = 0;
	virtual void		write(const std::string& msg) = 0;

};

/*
 * Implementation which allow to write into a file
 */

class file_log_policy : public log_policy_interface
{
	std::unique_ptr< std::ofstream > out_stream;
public:
	file_log_policy() : out_stream( new std::ofstream ) {}
	void open_ostream(const std::string& name);
	void close_ostream();
	void write(const std::string& msg);
	~file_log_policy();
};

// implementation

// listing 10
template< typename log_policy >
logger< log_policy >::logger( const std::string& name )
{
	log_line_number = 0;
	policy = new log_policy;
	if( !policy )
	{
		throw std::runtime_error("LOGGER: Unable to create the logger instance"); 
	}
	policy->open_ostream( name );
}

template< typename log_policy >
logger< log_policy >::~logger()
{
	if( policy )
	{
		policy->close_ostream();
		delete policy;
	}
}

// listing 9
template< typename log_policy >
std::string logger< log_policy >::get_time()
{
	std::string time_str;
	time_t raw_time;
	
	time( & raw_time );
#ifdef _WINDOWS
    char buffer[255];
    ctime_s(buffer, sizeof(buffer), &raw_time);
    time_str = buffer;
#else
    time_str = ctime(&raw_time);
#endif

	//without the newline character
	return time_str.substr( 0 , time_str.size() - 1 );
}

template< typename log_policy >
std::string logger< log_policy >::get_logline_header()
{
	std::stringstream header;

	header.str("");
	header.fill('0');
	header.width(7);
	header << log_line_number++ <<" < "<<get_time()<<" - ";

	header.fill('0');
	header.width(7);
	header <<clock()<<" > ~ ";

	return header.str();
}

// listing 7
template< typename log_policy >
void logger< log_policy >::print_impl()
{
	policy->write( get_logline_header() + log_stream.str() );
	log_stream.str("");
}

template< typename log_policy >
	template<typename First, typename...Rest >
void logger< log_policy >::print_impl(First parm1, Rest...parm)
{
	log_stream<<parm1;
	print_impl(parm...);	
}


}

#endif


