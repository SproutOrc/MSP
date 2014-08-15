#ifndef _MOTION_H_
#define _MOTION_H_

typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

void initMotion();
void forntMotion(u16 leftSpeed, u16 rightSpeed);
void backMotion(u16 leftSpeed, u16 rightSpeed);
void rightMotion(u16 leftSpeed, u16 rightSpeed);
void leftMotion(u16 leftSpeed, u16 rightSpeed);
void stopMotion();

#endif

