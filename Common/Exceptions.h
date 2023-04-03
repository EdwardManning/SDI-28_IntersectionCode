#pragma once
#include <exception>

struct vehicle_time_fail : public std::exception
{
    const char* what() const throw()
    {
        return "Vehicle Time Fail";
    }
};

struct impossible_value_fail : public std::exception
{
    const char* what() const throw()
    {
        return "Impossible Value Fail";
    }
};

struct impossible_state_fail : public std::exception
{
    const char* what() const throw()
    {
        return "Impossible State Fail";
    }
};

struct command_rejected_fail : public std::exception
{
    const char* what() const throw()
    {
        return "command_rejected_fail";
    }
};