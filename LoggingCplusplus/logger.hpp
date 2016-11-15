// Logging code is taken from http://www.drdobbs.com/cpp/a-lightweight-logger-for-c/240147505#
// define LOGGING_LEVEL_1 or LOGGING_LEVEL_2 here or add them to CPPFLAGS in the makefile. For example, #define LOGGING_LEVEL_1 or CPPFLAGS=-DLOGGING_LEVEL_1
#ifndef LOGGER_HPP
#define LOGGER_HPP

#include "log.hpp"

logging::logger< logging::file_log_policy > log_inst( "execution.log" );

#ifdef LOGGING_LEVEL_1
#define LOGGING
#define LOG log_inst.print< logging::severity_type::debug >
#define LOG_ERR log_inst.print< logging::severity_type::error >
#define LOG_WARN log_inst.print< logging::severity_type::warning >
#else
#define LOG(...) 
#define LOG_ERR(...)
#define LOG_WARN(...)
#endif

#ifdef LOGGING_LEVEL_2
#define LOGGING
#define ELOG log_inst.print< logging::severity_type::debug >
#define ELOG_ERR log_inst.print< logging::severity_type::error >
#define ELOG_WARN log_inst.print< logging::severity_type::warning >
#else
#define ELOG(...) 
#define ELOG_ERR(...)
#define ELOG_WARN(...)
#endif

#endif
