// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID.h"


#define roll_P 0.015f
#define roll_I 0.1f
#define roll_D 0.004f
#define roll_IMAX 100
#define roll_FILTER 5.0f
#define roll_DT 0.01f
#define roll_INITIAL_FF 0.0f

#define pitch_P 0.15f
#define pitch_I 0.6f
#define pitch_D 0.0125f
#define pitch_IMAX 100
#define pitch_FILTER 5.0f
#define pitch_DT 0.01f
#define pitch_INITIAL_FF 0.0f

const AP_Param::GroupInfo AC_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_PID, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_PID, _ki, 0),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D",    2, AC_PID, _kd, 0),

    // 3 was for uint16 IMAX
    // 4 is used by TradHeli for FF

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 5, AC_PID, _imax, 0),

    // @Param: FILT
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FILT", 6, AC_PID, _filt_hz, AC_PID_FILT_HZ_DEFAULT),

    AP_GROUPEND
};

// Constructor
AC_PID::AC_PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt) :
    _dt(dt),
    _integrator(0.0f),
    _input(0.0f),
    _derivative(0.0f)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _imax = fabsf(initial_imax);
    filt_hz(initial_filt_hz);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));
}

// set_dt - set time step in seconds
void AC_PID::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
}

// filt_hz - set input filter hz
void AC_PID::filt_hz(float hz)
{
    _filt_hz.set(fabsf(hz));

    // sanity check _filt_hz
    _filt_hz = MAX(_filt_hz, AC_PID_FILT_HZ_MIN);
}

// set_input_filter_all - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PID::set_input_filter_all(float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _input = input;
        _derivative = 0.0f;
    }

    // update filter and calculate derivative
    float input_filt_change = get_filt_alpha() * (input - _input);
    _input = _input + input_filt_change;
    if (_dt > 0.0f) {
        _derivative = input_filt_change / _dt;
    }
}

void AC_PID::set_input_filter_d(float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _derivative = 0.0f;
    }

    // update filter and calculate derivative
    if (_dt > 0.0f) {
        float derivative = (input - _input) / _dt;
        _derivative = _derivative + get_filt_alpha() * (derivative-_derivative);
    }

    _input = input;
}

float AC_PID::get_p_roll()
{
    _pid_info.P = (roll_input * roll_kp);
    return _pid_info.P;
}

float AC_PID::get_i_roll()
{
    if(!is_zero(roll_ki) && !is_zero(roll_dt)) {
        _integrator += ((float)_input * roll_ki) * roll_dt;
        if (_integrator < -roll_imax) {
            _integrator = -roll_imax;
        } else if (_integrator > roll_imax) {
            _integrator = roll_imax;
        }
        _pid_info.I = _integrator;
        return _integrator;
    }
    return 0;
}

float AC_PID::get_d_roll()
{
    _pid_info.D = (roll_kd * _derivative);
    return _pid_info.D;
}

float AC_PID::get_p_pitch()
{
    _pid_info.P = (_input * pitch_kp);
    return _pid_info.P;
}

float AC_PID::get_i_pitch()
{
    if(!is_zero(pitch_ki) && !is_zero(pitch_dt)) {
        _integrator += ((float)_input * pitch_ki) * pitch_dt;
        if (_integrator < -pitch_imax) {
            _integrator = -pitch_imax;
        } else if (_integrator > pitch_imax) {
            _integrator = pitch_imax;
        }
        _pid_info.I = _integrator;
        return _integrator;
    }
    return 0;
}

float AC_PID::get_d_pitch()
{
    _pid_info.D = (pitch_kd * _derivative);
    return _pid_info.D;
}

float AC_PID::get_p()
{
    return get_p();
}

float AC_PID::get_pi()
{
    return get_p() + get_i();
}

float AC_PID::get_pid()
{
    return get_p() + get_i() + get_d();
}

void AC_PID::reset_I()
{
    _integrator = 0;
}

void AC_PID::load_gains_roll()
{
    roll_kp.load();
    roll_ki.load();
    roll_kd.load();
    roll_imax.load();
    roll_imax = fabsf(_imax);
    roll_filt_hz.load();
}

void AC_PID::save_gains_roll()
{
    roll_kp.save();
    roll_ki.save();
    roll_kd.save();
    roll_imax.save();
    roll_filt_hz.save();
}

void AC_PID::load_gains_pitch()
{
    pitch_kp.load();
    pitch_ki.load();
    pitch_kd.load();
    pitch_imax.load();
    pitch_imax = fabsf(_imax);
    pitch_filt_hz.load();
}

void AC_PID::save_gains_pitch()
{
    pitch_kp.save();
    pitch_ki.save();
    pitch_kd.save();
    pitch_imax.save();
    pitch_filt_hz.save();
}

void AC_PID::operator_roll() (float p, float i, float d, float imaxval, float input_filt_hz, float dt)
{
    roll_kp = p;
    roll_ki = i;
    roll_kd = d;
    roll_imax = fabsf(imaxval);
    roll_filt_hz = input_filt_hz;
    roll_dt = dt;
}

void AC_PID::operator_pitch() (float p, float i, float d, float imaxval, float input_filt_hz, float dt)
{
    pitch_kp = p;
    pitch_ki = i;
    pitch_kd = d;
    pitch_imax = fabsf(imaxval);
    pitch_filt_hz = input_filt_hz;
    pitch_dt = dt;
}

// calc_filt_alpha - recalculate the input filter alpha
float AC_PID::get_filt_alpha() const
{
    if (is_zero(_filt_hz)) {
        return 1.0f;
    }

    // calculate alpha
    float rc = 1/(M_2PI*_filt_hz);
    return _dt / (_dt + rc);
}
