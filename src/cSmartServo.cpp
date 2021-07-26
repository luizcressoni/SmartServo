/*! \file smartservo.cpp */
#include "smartservo.h"

/** Servo kinematic parameters **************/
#define SERVO_TASKDELAY           10        //!< ms between loop evaluations
#define MIN_DISTANCE              20        //!< minimum travel distance to apply s-curve
#define ACCELERATION              0.25      //!< acceleration. Empirical 
#define VMIN                      0.2       //!< minimum speed in degrees/SERVO_TASKDELAY
#define VMAX                      10        //!< maximum speed in degrees/SERVO_TASKDELAY


/** constructor
*/
cSmartServo::cSmartServo()
{
    mu32time = millis();
}

/** SetLimits
    \brief Sets the limits for servo movement and map them to range [0-180]
    \param _min value that will be set when moving to zero
    \param _max value that will be set when move to 180
*/
void cSmartServo::SetLimits(uint8_t _min, uint8_t _max)
{
    mu8min = _min;
    mu8max = _max;
    calc_remap();
    mu8currentpos = remap_servo_pos(mu8currentpos);
}

/** GetRealPos
    \brief Should read actual position from servo, but did not work
    \return current position
*/
uint8_t cSmartServo::GetRealPos()
{
    return mu8currentpos;
}

/** init
    \brief Inits smartservo parameters
    \param _index PWM channel for ESP32 [0-15]
    \param _iopin GPIO pin where the servo control signal will output to. Must be a PWM capable pin
    \param _min See SetLimits
    \param _max See SetLimits
    \param _initpos Initial servo desired position
*/
void cSmartServo::init(uint8_t _index, uint8_t _iopin, uint8_t _min, uint8_t _max, uint8_t _initpos)
{
    mu8index = _index;
    mu8iopin = _iopin;
    SetLimits(_min, _max);
    mservo.attach(mu8iopin);
    MoveTo(_initpos);
    meServoScurvePhase = eServoScurvePhaseIddle;
}

/** MoveTo
    \brief Move servo to desired position in constant speed
    \param _pos Goal position
 */
void cSmartServo::MoveTo(uint8_t _pos)
{
    mservo.write(remap_servo_pos(_pos));
    mu8currentpos = _pos;
}

/** calc_remap
    \brief Calculate the slope for linear interpolation.
            Since always remaps from range [0-180] there
            is no neet to implement full line equation 
*/
void  cSmartServo::calc_remap()
{
    mf32remap_slope = (mu8max - mu8min) / 180.0;
}

/** remap_servo_pos
    \brief Remaps position in rage [0-180] to [min-max]
    \param _pos Goal position
    \return the remaped position
*/
uint8_t cSmartServo::remap_servo_pos(uint8_t _pos)
{
    uint8_t ret = mu8min + mf32remap_slope * _pos;
    return ret;
}

/** Scurve
    \brief Call this to move to any position using an s-curve
            If distance is less than MIN_DISTANCE, will move in constant speed
    \param _endpos goal position
    \param _vmax maximum speed in degrees/SERVO_TASKDELAYms
*/
void cSmartServo::Scurve(uint8_t _endpos, uint8_t _vmax)
{
    direction = (_endpos > mu8currentpos);
    if(!mservo.attached())
    {
        mservo.attach(mu8iopin, mu8index);
    }
    if(abs(_endpos - mu8currentpos) < MIN_DISTANCE)
    {
        MoveTo(_endpos);
        task_rest();
    }
    else
    {
        mu8maxspeed = _vmax;
        mu8goalpos = _endpos;
        mf32pos = mu8currentpos;
        mf32middlepoint = (mu8goalpos - mu8currentpos)/2.0 + mu8currentpos;
        mf32speed = VMIN;
        mu8deltapoint = 255;
        meServoScurvePhase = eServoScurvePhaseAccel;
    }
}

/** Detach
    \brief Stops sending PWM to servo. Good for noise servos
            and if mechanics can keep still without the signal
*/
void cSmartServo::Detach()
{
    mservo.detach();
}

/** task_accel
    \brief FSM task for acceleration phase
*/
void cSmartServo::task_accel()
{
    mf32speed += ACCELERATION;
    if(mf32speed >= mu8maxspeed)
    {
        mu8deltapoint = 2 * mf32middlepoint - mu8currentpos;
        meServoScurvePhase = eServoScurvePhaseCteSpeed;
    }
    if((direction && updatepos() >= mf32middlepoint) ||
        (!direction && updatepos() <= mf32middlepoint))
    {
        meServoScurvePhase = eServoScurvePhaseDeaccel;
    }
}

/** task_ctespeed
    \brief FSM task for constant speed phase
*/
void cSmartServo::task_ctespeed()
{
    if((direction && updatepos() >= mu8deltapoint) ||
        (!direction && updatepos() <= mu8deltapoint))
    {
        meServoScurvePhase = eServoScurvePhaseDeaccel;
    }
}

/** task_deaccel
    \brief FSM task for deacceleration phase
*/
void cSmartServo::task_deaccel()
{
    mf32speed -= ACCELERATION;
    if(mf32speed < VMIN)
        mf32speed = VMIN;
    updatepos();
}

/** task_rest
    \brief FSM task for resting phase
        Currently does nothing
*/
void cSmartServo::task_rest()
{
    meServoScurvePhase = eServoScurvePhaseIddle;
}

/** loop
    \brief call this as soon as possible
    \return TRUE is servo is at desired position
*/
bool cSmartServo::loop()
{
    if(mu32time >= millis() + SERVO_TASKDELAY)
    {
        mu32time = millis();   
        switch (meServoScurvePhase)
        {
        case eServoScurvePhaseIddle:
            break;
        case eServoScurvePhaseAccel:    task_accel();       break;
        case eServoScurvePhaseCteSpeed: task_ctespeed();    break;
        case eServoScurvePhaseDeaccel:  task_deaccel();     break;
        case eServoScurvePhaseRest:     task_rest();        break;
        default:
            meServoScurvePhase = eServoScurvePhaseIddle;
            break;
        }
    }
    return meServoScurvePhase == eServoScurvePhaseIddle;
}

/** update_pos
    \brief used by FSM task to set servo signal
    \return current position
*/
uint8_t cSmartServo::updatepos()
{
    uint8_t pos = mu8currentpos;
    if(meServoScurvePhase != eServoScurvePhaseIddle)
    {
        if(mf32pos <  mu8goalpos && direction)
            mf32pos += mf32speed;
        else if (mf32pos > mu8goalpos && !direction)
            mf32pos -= mf32speed;
        if(mf32pos < 0.0)
            mf32pos = 0.0;
        pos = round(mf32pos);
        if(pos == mu8goalpos)
        {
            meServoScurvePhase = eServoScurvePhaseRest;
        }
        MoveTo(pos);
    }
    return pos;
}

//eof cSmartServo.cpp