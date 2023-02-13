/*
 * AT24C08.h
 *
 *  Created on: 2019. 12. 4.
 *      Author: Administrator
 */

#ifndef INC_AT24C08_H_
#define INC_AT24C08_H_


typedef union _Parser
{
	unsigned char byte[4];
	float f;
}Parser;

void AT24C08_Page_Write(unsigned char page, unsigned char* data, unsigned char len);
void AT24C08_Page_Read(unsigned char page, unsigned char* data, unsigned char len);
void EP_PIDGain_Write(unsigned char id, float PGain, float IGain, float DGain);
unsigned char EP_PIDGain_Read(unsigned char id, float* PGain, float* IGain, float* DGain);


#endif /* INC_AT24C08_H_ */
