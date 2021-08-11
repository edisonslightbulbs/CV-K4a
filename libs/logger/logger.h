#ifndef LOGGER_H
#define LOGGER_H

#if __linux__
#include <gflags/gflags.h>
#include <glog/logging.h>

static void logger(int& t_argc, char** t_argvptr)
{
    google::ParseCommandLineFlags(&t_argc, &t_argvptr, true);
    google::InitGoogleLogging(t_argvptr[0]);
    google::SetLogDestination(google::GLOG_INFO, "Log.txt");
}
#endif
#endif // LOGGER_H
