/* 
 * File:   InterchipDMA.h
 * Author: Chris Hajduk
 *
 * Created on February 2, 2014, 2:10 PM
 */

//TODO: Clean up this H file and the corresponding C file

#ifndef INTERCHIPDMA_H
#define	INTERCHIPDMA_H







//Function Prototypes
#if !PATH_MANAGER
void init_DMA0();
void ProcessTxData(unsigned char *buffer);
void ProcessRxData(unsigned char *buffer);
void init_DMA1();
#endif
void init_SPI();


#endif	/* INTERCHIPDMA_H */

