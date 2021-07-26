/*! \file smartservo.h */
#pragma once
#include <Arduino.h>
#include <Servo.h>

/** Smart Servo ******************************************
Implemens a simple S-curve to move the servos
from initial position to goal position with
constant acceleration
1 - Movement starts with VMIN speed and accelerates until
    a) reaches VMAX or
    b) reaches middle point (goal - initial)/2
2 - After this, 
    if (a), runs in cte speed until reaches a
    distance from the goal position that is equal to
    the distance from the start to the (a) point (=c)
    Then, or if(b)
3 - Deaccelerate to VMIN until goal point is reached 

VMAX and acceleration values were chosen empirically


^ Pos                ---g           <- goal
|                ---
|              --
|             c                     
|            b                      <- middle point
|           a                       <- positon at VMAX
|        --
|     ---
|i---                               <- initial
|
+---------------------------> t

*********************************************/

enum eServoScurvePhase
{
    eServoScurvePhaseIddle = 0,
    eServoScurvePhaseAccel,
    eServoScurvePhaseCteSpeed,
    eServoScurvePhaseDeaccel,
    eServoScurvePhaseRest
};

/** class cSmartServo
    \brief Implements servo control with constant acceleation s-curve
            https://github.com/luizcressoni/SmartServo
    \author Luiz Cressoni Filho
*/
class cSmartServo
{
 public:
    cSmartServo();
    void    init(uint8_t _index, uint8_t _iopin, uint8_t _min, uint8_t _max, uint8_t _initpos);
    void    SetLimits(uint8_t _min, uint8_t _max);
    void    MoveTo(uint8_t _pos);
    void    Scurve(uint8_t _endpos, uint8_t _vmax);
    uint8_t GetRealPos();
    void    Detach();
    bool    loop();
 protected:
    Servo mservo;
    float mf32remap_slope;
    uint8_t mu8iopin, mu8index, mu8min, mu8max, mu8currentpos, mu8goalpos;
    uint8_t mu8deltapoint, mu8maxspeed;
    bool direction;
    eServoScurvePhase meServoScurvePhase;
    float mf32speed, mf32middlepoint, mf32pos;

    void    calc_remap();
    uint8_t remap_servo_pos(uint8_t _pos);
    //s-curve related
    uint8_t updatepos();
    //FSM
    void    task_accel();
    void    task_ctespeed();
    void    task_deaccel();
    void    task_rest();
};

//eof smartservo.h
