/*
 * Tiny_Winch.h
 *
 * Created: 12/1/2014 11:20:06 AM
 *  Author: dmatthews
 */ 


#ifndef TINY_WINCH_H_
#define TINY_WINCH_H_

void setup(void);
void Motor_On_FWD(void);
void Motor_On_REV(void);
void Motor_Off(void);
void Start_Delay(void);

// States
void State1(void);
void State2(void);
void State3(void);
void State1A(void);
void State2A(void);
void State3A(void);

#endif /* TINY_WINCH_H_ */