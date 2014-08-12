#ifndef _SPI_H_
#define _SPI_H_

char spi_readByte  (char addr, char * data);
char spi_readWord  (char addr, unsigned int * data);
char spi_readBytes (char addr, unsigned char * buffer, unsigned char len);
char spi_writeByte (char addr, char data);
char spi_writeWord (char addr, unsigned int data);
char spi_writeBytes(char addr, char * buffer, unsigned char len);

#endif
