/* Low level driver for AXI/DMA
 *
 * author: arios@atc.us.es and federico.corradi@gmail.com
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdbool.h>
#include <math.h>
#include <ncurses/curses.h>

// Memory block AXI/DMA
#define MM2S_CONTROL_REGISTER 0x00
#define MM2S_STATUS_REGISTER 0x04
#define MM2S_START_ADDRESS 0x18
#define MM2S_LENGTH 0x28

#define S2MM_CONTROL_REGISTER 0x30
#define S2MM_STATUS_REGISTER 0x34
#define S2MM_DESTINATION_ADDRESS 0x48
#define S2MM_LENGTH 0x58

#define TRANSLEN 8 //bytes multiple of 4
#define BURST 10

#define device_addr_offset 0x40400000
#define dest_addr_offset 0x0F000000
#define src_addr_offset 0x0E000000

//declare functions
unsigned int dma_set(unsigned int* dma_virtual_address, int offset,
		unsigned int value);
unsigned int dma_get(unsigned int* dma_virtual_address, int offset);
int dma_mm2s_sync(unsigned int* dma_virtual_address);
int dma_s2mm_sync(unsigned int* dma_virtual_address);
void dma_s2mm_status(unsigned int* dma_virtual_address);
void dma_mm2s_status(unsigned int* dma_virtual_address);
void memdump(char* virtual_address, int byte_count);
void hexDump(char *desc, void *addr, int len);

void initAxiMemoryPointers(void);
bool waitValidAxiDataToRead(int wordsNumber);
int writeAxiCommit(int wordsNumber, unsigned int startPos);
void resetAXIDMA(void);
int dma_s2mm_sync_halted_and_notIDLE(unsigned int* dma_virtual_address);
void stopS2MM(void);

static int dh;
static unsigned int* virtual_address;
static unsigned int* virtual_destination_address;
static unsigned int* virtual_source_address;

inline void initAxiMemoryPointers() {

	dh = open("/dev/mem", O_RDWR | O_SYNC); // Open /dev/mem which represents the whole physical memory
	virtual_address = (unsigned int *) mmap(NULL, 65535, PROT_READ | PROT_WRITE,
			MAP_SHARED, dh, device_addr_offset); // Memory map AXI Lite register block
	virtual_destination_address = (unsigned int *) mmap(NULL, 65535,
			PROT_READ | PROT_WRITE, MAP_SHARED, dh, dest_addr_offset); // Memory map destination address
	virtual_source_address = (unsigned int *) mmap(NULL, 65535,
			PROT_READ | PROT_WRITE, MAP_SHARED, dh, src_addr_offset); // Memory map source address
}

inline void resetAXIDMA() {
	//Reset DMA engines
	dma_set(virtual_address, S2MM_CONTROL_REGISTER, 4);
	dma_set(virtual_address, MM2S_CONTROL_REGISTER, 4);
	dma_set(virtual_address, S2MM_CONTROL_REGISTER, 0);
	dma_set(virtual_address, MM2S_CONTROL_REGISTER, 0);
}

inline int writeAxiCommit(int wordsNumber, unsigned int startPos) {

	int numbytes = 0;

	if (wordsNumber > 0 || wordsNumber <= 64) {

		dma_s2mm_sync_halted_and_notIDLE(virtual_address);

		// Stop S2MM and MM2S
		//dma_set(virtual_address, S2MM_CONTROL_REGISTER, 0);
		//dma_set(virtual_address, MM2S_CONTROL_REGISTER, 0);

		// Set destination and source addresses
		//dma_set(virtual_address, S2MM_DESTINATION_ADDRESS, dest_addr_offset);
		dma_set(virtual_address, MM2S_START_ADDRESS,
				src_addr_offset + startPos);

		// Enable interruptions and start S2MM and MM2S
		//dma_set(virtual_address, S2MM_CONTROL_REGISTER, 0xf001);
		dma_set(virtual_address, MM2S_CONTROL_REGISTER, 0xf001);

		//dma_mm2s_status(virtual_address);
		//dma_s2mm_status(virtual_address);

		// Set tranference length for S2MM and MM2S. S2MM must be set before MM2S. In this point the tranferece starts
		//dma_set(virtual_address, S2MM_LENGTH, TRANSLEN*wordsNumber);
		dma_set(virtual_address, MM2S_LENGTH, TRANSLEN * wordsNumber);

		dma_mm2s_sync(virtual_address);

		dma_set(virtual_address, MM2S_CONTROL_REGISTER, 0); // Stop MM2S
		//dma_set(virtual_address, MM2S_STATUS_REGISTER, 2); // Clear idle
		dma_set(virtual_address, MM2S_STATUS_REGISTER, 0x1000); // Clear IOC_Irq

		numbytes = TRANSLEN * wordsNumber;
	}

	return(numbytes);
}

inline bool waitValidAxiDataToRead(int wordsNumber) {

	//dma_set(virtual_address, S2MM_CONTROL_REGISTER, 0); // Stop S2MM
	dma_set(virtual_address, S2MM_STATUS_REGISTER, 2); // Clear idle
	dma_set(virtual_address, S2MM_STATUS_REGISTER, 0x1000); // Clear IOC_Irq
	dma_set(virtual_address, S2MM_DESTINATION_ADDRESS, dest_addr_offset);
	dma_set(virtual_address, S2MM_CONTROL_REGISTER, 0xf001);
	dma_set(virtual_address, S2MM_LENGTH, TRANSLEN * wordsNumber);

	dma_s2mm_sync(virtual_address); // Wait until Idle or IOC_Irq bit is 1
	//printf("Destination memory block:\n"); 
	//memdump(virtual_destination_address, TRANSLEN*burst);
}

inline void stopS2MM() {
	dma_set(virtual_address, S2MM_CONTROL_REGISTER, 0); // Stop S2MM
	dma_set(virtual_address, S2MM_STATUS_REGISTER, 2); // Clear idle
	dma_set(virtual_address, S2MM_STATUS_REGISTER, 0x1000); // Clear IOC_Irq
}

inline unsigned int dma_set(unsigned int* dma_virtual_address, int offset,
		unsigned int value) {
	dma_virtual_address[offset >> 2] = value;
}

inline unsigned int dma_get(unsigned int* dma_virtual_address, int offset) {
	return dma_virtual_address[offset >> 2];
}

inline int dma_mm2s_sync(unsigned int* dma_virtual_address) {
	unsigned int mm2s_status = dma_get(dma_virtual_address,
			MM2S_STATUS_REGISTER);
	while (!(mm2s_status & 1 << 12) || !(mm2s_status & 1 << 1)) {
		//dma_s2mm_status(dma_virtual_address);
		//dma_mm2s_status(dma_virtual_address);

		mm2s_status = dma_get(dma_virtual_address, MM2S_STATUS_REGISTER);
	}
}

inline int dma_s2mm_sync(unsigned int* dma_virtual_address) {
	unsigned int s2mm_status = dma_get(dma_virtual_address,
			S2MM_STATUS_REGISTER);

	while (!(s2mm_status & (1 << 12)) || !(s2mm_status & (1 << 1))) {
		//dma_s2mm_status(dma_virtual_address);
		//dma_mm2s_status(dma_virtual_address);

		s2mm_status = dma_get(dma_virtual_address, S2MM_STATUS_REGISTER);
	}
}

inline int dma_s2mm_sync_halted_and_notIDLE(unsigned int* dma_virtual_address) {
	unsigned int s2mm_status = dma_get(dma_virtual_address,
			S2MM_STATUS_REGISTER);

	while ((s2mm_status & (1 << 12)) || (s2mm_status & (1 << 1)) /*|| !(s2mm_status & (1<<0))*/) {
		//dma_s2mm_status(dma_virtual_address);
		//dma_mm2s_status(dma_virtual_address);

		s2mm_status = dma_get(dma_virtual_address, S2MM_STATUS_REGISTER);
	}
}

inline void dma_s2mm_status(unsigned int* dma_virtual_address) {
	unsigned int status = dma_get(dma_virtual_address, S2MM_STATUS_REGISTER);
	printf("Stream to memory-mapped status (0x%08x@0x%02x):", status,
			S2MM_STATUS_REGISTER);
	if (status & 0x00000001)
		printf(" halted");
	else
		printf(" running");
	if (status & 0x00000002)
		printf(" idle");
	if (status & 0x00000008)
		printf(" SGIncld");
	if (status & 0x00000010)
		printf(" DMAIntErr");
	if (status & 0x00000020)
		printf(" DMASlvErr");
	if (status & 0x00000040)
		printf(" DMADecErr");
	if (status & 0x00000100)
		printf(" SGIntErr");
	if (status & 0x00000200)
		printf(" SGSlvErr");
	if (status & 0x00000400)
		printf(" SGDecErr");
	if (status & 0x00001000)
		printf(" IOC_Irq");
	if (status & 0x00002000)
		printf(" Dly_Irq");
	if (status & 0x00004000)
		printf(" Err_Irq");
	printf("\n");
}

inline void dma_mm2s_status(unsigned int* dma_virtual_address) {
	unsigned int status = dma_get(dma_virtual_address, MM2S_STATUS_REGISTER);
	printf("Memory-mapped to stream status (0x%08x@0x%02x):", status,
			MM2S_STATUS_REGISTER);
	if (status & 0x00000001)
		printf(" halted");
	else
		printf(" running");
	if (status & 0x00000002)
		printf(" idle");
	if (status & 0x00000008)
		printf(" SGIncld");
	if (status & 0x00000010)
		printf(" DMAIntErr");
	if (status & 0x00000020)
		printf(" DMASlvErr");
	if (status & 0x00000040)
		printf(" DMADecErr");
	if (status & 0x00000100)
		printf(" SGIntErr");
	if (status & 0x00000200)
		printf(" SGSlvErr");
	if (status & 0x00000400)
		printf(" SGDecErr");
	if (status & 0x00001000)
		printf(" IOC_Irq");
	if (status & 0x00002000)
		printf(" Dly_Irq");
	if (status & 0x00004000)
		printf(" Err_Irq");
	printf("\n");
}

inline void memdump(char* virtual_address, int byte_count) {
	char *p = virtual_address;
	int offset;
	unsigned int data = 0;
	unsigned int data_low = 0;

	for (offset = 0; offset < byte_count; offset++) {
		data |= (p[offset] & 0xFF) << ((offset % 4) * 8);
		if (offset % 8 == 7) {
			printf("0x%08x%08x\n", data, data_low);
			data = 0;
			data_low = 0;
		} else {
			if (offset % 4 == 3) {
				data_low = data;
				data = 0;
			}
		}
	}
}

inline void memdump_checking(char* virtual_address, int byte_count) {
	char *p = virtual_address;
	int offset;
	unsigned int data = 0;
	unsigned int data_low = 0;
	unsigned int data_low_bkp = 0;

	for (offset = 0; offset < byte_count; offset++) {
		data |= (p[offset] & 0xFF) << ((offset % 4) * 8);
		if (offset % 8 == 7) {
			printf("0x%08x%08x\n", data, data_low);
			if (data != 3 || data_low != data_low_bkp) {
				resetAXIDMA();
				printf(
						"Error in the sequence. is expected: high --> 00000003, low --> %d. Received: high --> %d, low --> %d",
						data_low_bkp, data, data_low);
			} else if (data_low == 0x3f) {
				data_low_bkp = 0;
			} else {
				data_low_bkp += 0x100;
			}
			data = 0;
			data_low = 0;
		} else {
			if (offset % 4 == 3) {
				data_low = data;
				data = 0;
			}
		}
	}
}

