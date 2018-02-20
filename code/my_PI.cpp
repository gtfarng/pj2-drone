// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_PI_2D.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PI_2D.h"

#define roll_P 0.1f
#define roll_I 0.25f
#define roll_IMAX 10

#define pitch_P 0.1f
#define pitch_I 0.23f
#define pitch_IMAX 100


const AP_Param::GroupInfo AC_PI_2D::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_PI_2D, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_PI_2D, _ki, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 2, AC_PI_2D, _imax, 0),

    // @Param: FILT_HZ
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FILT_HZ", 3, AC_PI_2D, _filt_hz, AC_PI_2D_FILT_HZ_DEFAULT),

    AP_GROUPEND
};

// Constructor
AC_PI::AC_PI(float initial_p, float initial_i, float initial_imax, float initial_filt_hz, float dt) :
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _ki = initial_i;
    _imax = fabsf(initial_imax);
    filt_hz(initial_filt_hz);

    // reset input filter to first value received
    _flags._reset_filter = true;
}

// set_dt - set time step in seconds
void AC_PI::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
    calc_filt_alpha();
}

// filt_hz - set input filter hz
void AC_PI_2D::filt_hz(float hz)
{
    _filt_hz.set(fabsf(hz));

    // sanity check _filt_hz
    _filt_hz = MAX(_filt_hz, AC_PI_2D_FILT_HZ_MIN);

    // calculate the input filter alpha
    calc_filt_alpha();
}


void AC_PI::set_input(const Vector2f &input)
{
    
    if (!isfinite(input.x) || !isfinite(input.y)) {
        return;
    }

    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _input = input;
    }

    Vector2f input_filt_change = (input - _input) * _filt_alpha;
    _input = _input + input_filt_change;
}

Vector2f AC_PI::get_roll() const
{
    return (_input * roll_kp);
}

Vector2f AC_PI::get_i_roll()
{
    if(!is_zero(roll_ki) && !is_zero(roll_dt)) {
        _integrator += (_input * roll_ki) * roll_dt;
        float integrator_length = _integrator.length();
        if ((integrator_length > roll_imax) && (integrator_length > 0)) {
            _integrator *= (roll_imax / integrator_length);
        }
        return _integrator;
    }
    return Vector2f();
}

Vector2f AC_PI::get_p_pitch() const
{
    return (_input * pitch_kp);
}

Vector2f AC_PI::get_i_pitch()
{
    if(!is_zero(pitch_ki) && !is_zero(pitch_dt)) {
        _integrator += (_input * pitch_ki) * pitch_dt;
        float integrator_length = _integrator.length();
        if ((integrator_length > pitch_imax) && (integrator_length > 0)) {
            _integrator *= (pitch_imax / integrator_length);
        }
        return _integrator;
    }
    return Vector2f();
}

// get_i_shrink - get_i but do not allow integrator to grow in length (it may shrink)
Vector2f AC_PI::get_i_shrink()
{
    if (!is_zero(_ki) && !is_zero(_dt)) {
        float integrator_length_orig = MIN(_integrator.length(),_imax);
        _integrator += (_input * _ki) * _dt;
        float integrator_length_new = _integrator.length();
        if ((integrator_length_new > integrator_length_orig) && (integrator_length_new > 0)) {
            _integrator *= (integrator_length_orig / integrator_length_new);
        }
        return _integrator;
    }
    return Vector2f();
}


Vector2f AC_PI::get_p()
{
    return get_p();
}

Vector2f AC_PI::get_pi()
{
    return get_p() + get_i();
}

void AC_PI_2D::reset_I()
{
    _integrator.zero();
}

void AC_PI_roll::load_gains()
{
    roll_kp.load();
    roll_ki.load();
    roll_imax.load();
    roll_imax = fabsf(_imax);
    roll_filt_hz.load();

    
    calc_filt_alpha();
}

void AC_PI_roll::save_gains()
{
    roll_kp.save();
    roll_ki.save();
    roll_imax.save();
    roll_filt_hz.save();
}


void AC_PI_pitch::load_gains()
{
    pitch_kp.load();
    pitch_ki.load();
    pitch_imax.load();
    pitch_imax = fabsf(_imax);
    pitch_filt_hz.load();

    calc_filt_alpha();
}


void AC_PI_pitch::save_gains()
{
    pitch_kp.save();
    pitch_ki.save();
    pitch_imax.save();
    pitch_filt_hz.save();
}


void AC_PI::operator_roll() (float p, float i, float imaxval, float input_filt_hz, float dt)
{
    roll_kp = p;
    roll_ki = i;
    roll_imax = fabsf(imaxval);
    roll_filt_hz = input_filt_hz;
    roll_dt = dt;
    
    calc_filt_alpha();
}

void AC_PI::operator_pitch() (float p, float i, float imaxval, float input_filt_hz, float dt)
{
    pitch_kp = p;
    pitch_ki = i;
    pitch_imax = fabsf(imaxval);
    pitch_filt_hz = input_filt_hz;
    pitch_dt = dt;
    
    calc_filt_alpha();
}

// calc_filt_alpha - recalculate the input filter alpha
void AC_PI_2D::calc_filt_alpha()
{
    if (is_zero(_filt_hz)) {
        _filt_alpha = 1.0f;
        return;
    }
  
    // calculate alpha
    float rc = 1/(M_2PI*_filt_hz);
    _filt_alpha = _dt / (_dt + rc);
}
