/* ParallelFlash. Library - for filesystem-like access to SPI Serial Flash memory
 * https://github.com/PaulStoffregen/ParallelFlash.
 * Copyright (C) 2015, Paul Stoffregen, paul@pjrc.com, f.boesing, f.boesing@gmx.de
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy.
 * Please support PJRC's efforts to develop open source software by purchasing
 * Teensy or other genuine PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "ParallelFlash.h"
//#include "util/ParallelFlash_directwrite.h"
#include <arm_math.h>
#include <core_cmInstr.h>

//Don't edit - partly hardcoded !
const int flash_sck   =  20; //PTD5 FlashPin 6
const int flash_cs    =   5; //PTD7 FlashPin 1
const int flash_sio0  =   2; //PTD0 FlashPin 5
const int flash_sio1  =  14; //PTD1 FlashPin 2
const int flash_sio2  =   7; //PTD2 FlashPin 3
const int flash_sio3  =   8; //PTD3 FlashPin 7

//Todo: calc these from Pinnumbers..?
#define MASK_CS 		(1<<7) //PTD7
#define MASK_SCK		(1<<5) //PTD5
#define MASK_SIO0		(1<<0) //PTD0
#define MASK_DATA		(0x0F)

#define MASK_ALL		( MASK_CS | MASK_SCK | MASK_DATA )

#define CSASSERT()  	{GPIO_D->PCOR = MASK_CS;}
#define CSRELEASE() 	{GPIO_D->PDOR = MASK_CS | MASK_SCK;}
#define CSRELEASESPI()	{GPIO_D->PDOR = MASK_CS;}

struct sPortConfig {
  uint32_t PCR0;uint32_t PCR1;uint32_t PCR2;uint32_t PCR3;
  uint32_t PCR4;uint32_t PCR5;uint32_t PCR6;uint32_t PCR7;
  uint32_t PCR8;uint32_t PCR9;uint32_t PCR10;uint32_t PCR11;
  uint32_t PCR12;uint32_t PCR13;uint32_t PCR14;uint32_t PCR15;
  uint32_t PCR16;uint32_t PCR17;uint32_t PCR18;uint32_t PCR19;
  uint32_t PCR20;uint32_t PCR21;uint32_t PCR22;uint32_t PCR23;
  uint32_t PCR24;uint32_t PCR25;uint32_t PCR26;uint32_t PCR27;
  uint32_t PCR28;uint32_t PCR29;uint32_t PCR30;uint32_t PCR31;
};
static volatile struct sPortConfig * const pinConfigD = (struct sPortConfig *)0x4004C000;

struct sGPIO {
  uint32_t PDOR; //Port Data Output Register
  uint32_t PSOR; //Port Set Output Register
  uint32_t PCOR; //Port Clear Output Register
  uint32_t PTOR; //Port Toggle Output Register
  uint32_t PDIR; //Port Data Input Register
  uint32_t PDDR; //Port Data Direction Register
};

//volatile struct sGPIO * const GPIO_A = (struct sGPIO *)0x400FF000;
//volatile struct sGPIO * const GPIO_B = (struct sGPIO *)0x400FF040;
//volatile struct sGPIO * const GPIO_C = (struct sGPIO *)0x400FF080;
static volatile struct sGPIO * const GPIO_D = (struct sGPIO *)0x400FF0C0;
//volatile struct sGPIO * const GPIO_E = (struct sGPIO *)0x400FF0D0;


#if (F_CPU == 144000000)
#define flash_Wait0 {  }
#define flash_Wait1 {  }
#define flash_Wait2 {  }
#define flash_Wait3 { asm volatile ("\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n"); }
#elif (F_CPU == 120000000)
#define flash_Wait0 {  }
#define flash_Wait1 {  }
#define flash_Wait2 {  }
#define flash_Wait3 { asm volatile ("\tNOP\n\tNOP\n\tNOP\n"); }
#elif (F_CPU == 96000000)
#define flash_Wait0 {  }
#define flash_Wait1 {  }
#define flash_Wait2 {  }
#define flash_Wait3 { asm volatile ("\tNOP\n\tNOP\n\tNOP\n"); }
#elif (F_CPU == 72000000)
#define flash_Wait0 { }
#define flash_Wait1 { }
#define flash_Wait2 { }
#define flash_Wait3 { asm volatile ("\tNOP\n\tNOP\n"); }
#elif (F_CPU <= 48000000)
#define flash_Wait0 { }
#define flash_Wait1 { }
#define flash_Wait2 { }
#define flash_Wait3 {  asm volatile ("\tNOP\n");}
#endif

uint16_t ParallelFlashChip::dirindex = 0;
uint8_t ParallelFlashChip::flags = 0;
uint8_t ParallelFlashChip::busy = 0;

#define FLAG_32BIT_ADDR		0x01	// larger than 16 MByte address
//#define FLAG_STATUS_CMD70	0x02	// requires special busy flag check
//#define FLAG_DIFF_SUSPEND	0x04	// uses 2 different suspend commands
//#define FLAG_MULTI_DIE		0x08	// multiple die, don't read cross 32M barrier
#define FLAG_256K_BLOCKS	0x10	// has 256K erase blocks
//#define FLAG_DIE_MASK		0xC0	// top 2 bits count during multi-die erase

	
void ParallelFlashChip::writeByte(const uint8_t val) {
  uint32_t clk0, clk1;
  GPIO_D->PDDR |= 0x0f;
  clk0 = GPIO_D->PDOR & ~MASK_ALL;  
  clk1 = clk0 | MASK_SCK; 
   
  GPIO_D->PDOR = clk0;
  flash_Wait2; 
  GPIO_D->PDOR = clk1  | (val >> 4);  
  flash_Wait2;
  GPIO_D->PDOR = clk0;
  flash_Wait2;
  GPIO_D->PDOR = clk1 | (val & 0x0f);
}

void ParallelFlashChip::writeBytes(const uint8_t * buf, const int len) {
  uint32_t clk0, clk1;
  uint8_t val;
  
  GPIO_D->PDDR |= 0x0f;
  clk0 = GPIO_D->PDOR & ~MASK_ALL;  
  clk1 = clk0 | MASK_SCK; //CS=0; CLK=1

  for (int i = 0; i< len; i++) {
	GPIO_D->PDOR = clk0;
	flash_Wait1; 
	val = buf[i];
	GPIO_D->PDOR = clk1  | (val >> 4);  
	flash_Wait2;
	GPIO_D->PDOR = clk0;
	flash_Wait2;
	GPIO_D->PDOR = clk1 | (val & 0x0f);
  }
}

void ParallelFlashChip::write16(const uint16_t val) {
	uint16_t buf = __REV16(val);
	writeBytes((uint8_t*) &buf, 2);
}

uint8_t ParallelFlashChip::readByte(void) {
  GPIO_D->PDDR &= ~0x0f;    
  
  volatile uint32_t val;
  
  GPIO_D->PCOR = MASK_CS;
  
  GPIO_D->PTOR = MASK_SCK;
  flash_Wait2;
  GPIO_D->PTOR = MASK_SCK;
  flash_Wait3;
  
  val = (GPIO_D->PDIR<< 4) & 0xf0;
  
  GPIO_D->PTOR = MASK_SCK;
  flash_Wait2;
  GPIO_D->PTOR = MASK_SCK;
  flash_Wait3;
  return val | ( GPIO_D->PDIR & 0x0f );
 
}

void ParallelFlashChip::readBytes( uint8_t * const buf, int len) {
  //if (len == 0) return;

  uint32_t val;
  uint8_t *src = buf;
  const uint8_t *target = src + len;

  
  GPIO_D->PDDR &= ~0x0f;      
  GPIO_D->PCOR = MASK_CS;
   
  while (((uintptr_t) src & 0x03) != 0 && src < target)  {
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val = (GPIO_D->PDIR<< 4) & 0xf0;
    GPIO_D->PTOR = MASK_SCK;	
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    *src = val | ( GPIO_D->PDIR & 0x0f );
	src += 1;
  }

  while (src <= target - 4)  {
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val = (GPIO_D->PDIR & 0x0f) << 4;
    GPIO_D->PTOR = MASK_SCK;	
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f);
	
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 12;
    GPIO_D->PTOR = MASK_SCK;	
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 8;

    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 20;
    GPIO_D->PTOR = MASK_SCK;	
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 16;

    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 28;
    GPIO_D->PTOR = MASK_SCK;	
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 24;
	
	*(uint32_t*) src = val;
	src += 4;
  }
  
  while (src < target)  {
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val = (GPIO_D->PDIR<< 4) & 0xf0;
    GPIO_D->PTOR = MASK_SCK;	
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    *src = val | ( GPIO_D->PDIR & 0x0f );
	src += 1;
  }

}

void ParallelFlashChip::wait(void)
{
	uint32_t status;
	//Serial.print("wait-");
	writeByte(0x05); //Read Status Byte #1
	while (1) {
	//	writeByte(0x05); //Read Status Byte #1
		status = readByte();			
		//Serial.printf("b=%02x.", status & 0xFF);
		if (!(status & 1)) break;
	}
	CSRELEASE();
	busy = 0;
	
	//Serial.println();
}

void ParallelFlashChip::read(uint32_t addr, void *buf, uint32_t len)
{
	uint8_t *p = (uint8_t *)buf;
	uint8_t b, f, status;

	//memset(p, 0, len);
	f = flags;	
	b = busy;
	if (b) {
		// read status register ... chip may no longer be busy
		writeByte(0x05); //Read Status Byte #1
		status = readByte();
		if (!(status & 1)) b = 0;
		CSRELEASE();
		if (b == 0) {
			// chip is no longer busy :-)
			busy = 0;
		} else if (b < 3) {
			writeByte(0x06); // write enable		
			CSRELEASE();	
		
			writeByte(0x75);// Suspend command
			CSRELEASE();
			
			writeByte(0x05); //Read Status Byte #1		
			do {
				status = readByte();
			} while ((status & 0x01));
			CSRELEASE();
			
		} else {
			// chip is busy with an operation that can not suspend
			wait();			// should we wait without ending
			b = 0;			// the transaction??
		}
	}


	if (f & FLAG_32BIT_ADDR) {			
		writeByte(0x0b);
		write16(addr >> 16);			
		write16(addr);		
		readByte();//dummy
	} else {
		write16(0x0b00 | ((addr >> 16) & 255));
		write16(addr);
		readByte();//dummy
	}
		
	readBytes(p, len);		
	CSRELEASE();

	if (b) {	
		writeByte(0x06); // write enable		
		CSRELEASE();
		//delayMicroseconds(1);		
		writeByte(0x7A);// Suspend command
		CSRELEASE();		
	}
}

void ParallelFlashChip::write(uint32_t addr, const void *buf, uint32_t len)
{
	const uint8_t *p = (const uint8_t *)buf;
	uint32_t max, pagelen;

	 //Serial.printf("WR: addr %08X, len %d\n", addr, len);
	do {
		if (busy) wait();		
		writeByte(0x06); // write enable		
		CSRELEASE();			
		max = 256 - (addr & 0xFF);
		pagelen = (len <= max) ? len : max;		
		 //Serial.printf("WR: addr %08X, pagelen %d\n", addr, pagelen);
		if (flags & FLAG_32BIT_ADDR) {
			writeByte(0x02); // program page command
			write16(addr >> 16);
			write16(addr);
		} else {
			write16(0x0200 | ((addr >> 16) & 255));
			write16(addr);
		}
		addr += pagelen;
		len -= pagelen;
		//delayMicroseconds(1);//Is this needed ?
		do {
			writeByte(*p++);			
		} while (--pagelen > 0);
		CSRELEASE();
		busy = 1;
	} while (len > 0);
}

void ParallelFlashChip::eraseAll()
{
	if (busy) wait();
	uint8_t id[3];
	readID(id);
	//Serial.printf("ID: %02X %02X %02X\n", id[0], id[1], id[2]);	
	// bulk erase command
	writeByte(0x06); // write enable		
	CSRELEASE();
	//delayMicroseconds(1);
	writeByte(0xC7);
	CSRELEASE();
	busy = 3;
}

void ParallelFlashChip::eraseBlock(uint32_t addr)
{
	uint8_t f = flags;
	if (busy) wait();
	writeByte(0x06); // write enable		
	CSRELEASE();
	//delayMicroseconds(1);	
	if (f & FLAG_32BIT_ADDR) {
		writeByte(0xD8); 		
		write16(addr >> 16);
		write16(addr);
	} else {
		write16(0xD800 | ((addr >> 16) & 255));
		write16(addr);
	}
	CSRELEASE();
	busy = 2;
}


bool ParallelFlashChip::ready()
{
	uint32_t status;
	if (!busy) return true;

	// all others work by simply reading the status reg
	writeByte(0x05);
	status = readByte();	
	CSRELEASE();	
	//Serial.printf("ready=%02x\n", status & 0xFF);
	if ((status & 1)) return false;
	
	busy = 0;
	if (flags & 0xC0) {
		// continue a multi-die erase
		eraseAll();
		return false;
	}
	return true;
}


#define ID0_WINBOND	0xEF

//#define FLAG_32BIT_ADDR	0x01	// larger than 16 MByte address
//#define FLAG_STATUS_CMD70	0x02	// requires special busy flag check
//#define FLAG_DIFF_SUSPEND	0x04	// uses 2 different suspend commands
//#define FLAG_256K_BLOCKS	0x10	// has 256K erase blocks

void ParallelFlashChip::enterQPI()
{
	if (busy) wait();

	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x38); //Enter QPI Mode
	CSRELEASESPI();
	GPIO_D->PDDR &= ~0x0f;
	
}

void ParallelFlashChip::exitQPI()
{
	if (busy) wait();
	
	writeByte(0xff); // Exit QPI Mode
	CSRELEASE();
	GPIO_D->PCOR = MASK_SCK;
	GPIO_D->PDDR = (GPIO_D->PDDR & ~0x0f) | MASK_SIO0;
}


bool ParallelFlashChip::begin()
{	
	uint8_t id[3];
	uint8_t f;
	uint32_t size;

	pinMode(flash_cs, OUTPUT);
	digitalWriteFast(flash_cs, 1);
	pinMode(flash_sck, OUTPUT);
	digitalWriteFast(flash_sck, 1);


	pinMode(flash_sio0, OUTPUT);
	pinMode(flash_sio1, INPUT);

	//Reset/Hold
	pinMode(flash_sio3, INPUT_PULLUP);

	//Software Reset
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x66); 
	CSRELEASESPI();	
	delayMicroseconds(1);
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x99);
	CSRELEASESPI();
	delayMicroseconds(100);

	//Configure Pins for fast switching/reading
	//todo: use somehow masks or pinnumbers here
	pinConfigD->PCR0 = 0x100;
	pinConfigD->PCR1 = 0x100;
	pinConfigD->PCR2 = 0x100;
	pinConfigD->PCR3 = 0x100;
  
	pinConfigD->PCR5 = 0x100;
	pinConfigD->PCR7 = 0x100;
	//Don't use pinMode below this point! It would change the PCR settings
	
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x06); //Write Enable
	CSRELEASESPI();	

#ifdef DEBUG
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x05); //Read Status Register 1
	uint8_t r = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	CSRELEASESPI();
	Serial.print("Status Register 1:0x");
	Serial.println(r, HEX);

	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x15); //Read Status Register 3
	r = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	Serial.print("Status Register 3:0x");
	Serial.println(r, HEX);
	CSRELEASESPI();

#endif
	//Enter QPI Mode

	//Serial.println("Enabling QPI in SR2");
	//Enable QPI in Statusregister 2:
	//digitalWriteFast(flash_cs, 0);
	CSASSERT();
	shiftOut( flash_sio0,  flash_sck , MSBFIRST, 0x31); //Write Status Register 2
	shiftOut( flash_sio0,  flash_sck , MSBFIRST,  2);
	CSRELEASESPI();
	delay(16); //needed here
#ifdef DEBUG
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x35); //Read Status Register 2
	r = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	CSRELEASESPI();
	Serial.print("Status Register 2:0x");
	Serial.println(r, HEX);
#endif

	enterQPI();

	readID(id);
	f = 0;
	size = capacity(id);
	if (size > 16777216) {
		// more than 16 Mbyte requires 32 bit addresses
		f |= FLAG_32BIT_ADDR;

		// micron & winbond & macronix use command
		writeByte(0x06); // write enable		
		CSRELEASE();
		//delayMicroseconds(1);		
		writeByte(0xB7); // enter 4 byte addr mode
		CSRELEASE();
		
	}
	flags = f;
	return true;
}

// chips tested: https://github.com/PaulStoffregen/ParallelFlash./pull/12#issuecomment-169596992
//
void ParallelFlashChip::sleep()
{
	if (busy) wait();
	writeByte(0xB9); // Deep power down command
	CSRELEASE();
}

void ParallelFlashChip::wakeup()
{
	writeByte(0xAB); // Wake up from deep power down command
	CSRELEASE();
}

void ParallelFlashChip::readID(uint8_t *buf)
{
	if (busy) wait();

	writeByte(0x9F);
	readBytes(buf, 3);  // manufacturer ID, memory type, capacity
	CSRELEASE();
//	Serial.printf("ID: %02X %02X %02X\n", buf[0], buf[1], buf[2]);
}

void ParallelFlashChip::readSerialNumber(uint8_t *buf) //needs room for 8 bytes
{

	exitQPI();
	
	GPIO_D->PCOR = (1<<5);
	CSASSERT();

	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x4B); 
	for (int i=0; i<4; i++) {
		shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	}
	for (int i=0; i<8; i++) {
		buf[i] = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	}
	CSRELEASESPI();
	
	enterQPI();

//	uint8_t id[3];
//	readID(id);
    //Serial.printf("Serial Number: %02X %02X %02X %02X %02X %02X %02X %02X\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);	
}

uint32_t ParallelFlashChip::capacity(const uint8_t *id)
{
	if (id[2] >= 16 && id[2] <= 31) {
		return 1ul << id[2];
	} else
	if (id[2] >= 32 && id[2] <= 37) {
		return 1ul << (id[2] - 6);
	} else  
	return 1048576; // unknown chips, default to 1 MByte
}

uint32_t ParallelFlashChip::blockSize()
{
	// Spansion chips >= 512 mbit use 256K sectors
	if (flags & FLAG_256K_BLOCKS) return 262144;
	// everything else seems to have 64K sectors
	return 65536;
}




/*
Chip		Uniform Sector Erase
		20/21	52	D8/DC
		-----	--	-----
W25Q64CV	4	32	64
W25Q128FV	4	32	64

*/



//			size	sector			busy	pgm/erase	chip
// Part			Mbyte	kbyte	ID bytes	cmd	suspend		erase
// ----			----	-----	--------	---	-------		-----
// Winbond W25Q64CV	8	64	EF 40 17
// Winbond W25Q128FV	16	64	EF 40 18	05	single		60 & C7
// Winbond W25Q256FV	32	64	EF 40 19	


ParallelFlashChip ParallelFlash;
