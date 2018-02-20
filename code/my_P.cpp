// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_P.cpp
/// @brief	Generic P algorithm

#include <AP_Math/AP_Math.h>
#include "AC_P.h"

#define roll_P 0.15f
#define pitch_P 0.125f

const AP_Param::GroupInfo AC_P::var_info[] = {
   
    AP_GROUPINFO("P",    0, AC_P, roll_P, pitch_P , 0),
    AP_GROUPEND
};

float AC_P::get_roll(float error) const
{
    return (float)error * roll_P;
}

void AC_P::load_gains_roll()
{
    roll_P.load();
}

void AC_P::save_gains_roll()
{
    roll_P.save();
}


float AC_P::get_pitch(float error) const
{
    return (float)error * pitch_P;
}

void AC_P::load_gains_pitch()
{
    pitch_P.load();
}

void AC_P::save_gains_pitch()
{
    pitch_P.save();
}

void AC_P::get_p()
{
    return get_p();
}