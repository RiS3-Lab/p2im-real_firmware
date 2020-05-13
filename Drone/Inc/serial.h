/*
 * serial_new.h
 *
 *  Created on: Jun 12, 2017
 *      Author: Heethesh
 */

#ifndef SERIAL_H_
#define SERIAL_H_

void serialBegin();
int serialAvailable();
void serialWrite(unsigned char ch);
unsigned char serialRead();
void serialFlush();
void serialPrint(char* data);
void serialInt(int val);
void serialFloat(float val);

#endif /* SERIAL_H_ */
