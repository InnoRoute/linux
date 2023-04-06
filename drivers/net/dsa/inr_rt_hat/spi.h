/**
*@file
*@brief spi definitions
*@author M.Ulbricht 2021
*@copyright GNU Public License v3.
*
**/
#define WR_LOCAL  0x0F
#define RD_LOCAL  0x3C
#define WR_MMI  0xC3
#define RD_MMI  0xF0
#define GET_SPI_ERROR 0


void RT_SPI_init (void);
void RT_SPI_exit (void);
void RT_SPI_set_cs (uint8_t data);
void RT_SPI_set_w_data (uint32_t data);
void RT_SPI_proc_write (uint32_t addr);
void RT_SPI_proc_read (uint32_t addr);
uint32_t RT_SPI_get_r_data (void);
uint8_t RT_SPI_get_semaphor (void);
void RT_SPI_file_write (char *buffer, size_t size);
void nibbletwist (uint8_t * data, uint8_t length);
uint32_t RT_SPI_read (uint32_t addr);
void RT_SPI_write (uint32_t addr, uint32_t data);
void init_interrupts (void);
void INR_SPI_MMI_interrupt (void);
int INR_SPI_interrupt_thread (void *nix);
void INR_SPI_autocmd (uint32_t * addr, uint8_t write);
union address
{
    uint32_t addr;
    uint8_t bytes[4];
};
#define DEBUG 0
#define SPI_DEBUG 0

#define INR_SPI_MMI_read(addr)		RT_SPI_read(addr)	//RT_SPI_read((RD_MMI<<24)+(0x00ffffff&(addr)))       /*<read from MMI */
#define INR_SPI_MMI_write(content, addr)	RT_SPI_write(addr,content)	//RT_SPI_write(((WR_MMI<<24)+(0x00ffffff&(addr))),content)/*<write to MMI */

#define INR_SPI_SPI_read(addr)		RT_SPI_read(addr)	//RT_SPI_read((RD_LOCAL<<24)+(0x00ffffff&(addr)))     /*<read from SPI */
#define INR_SPI_SPI_write(content, addr)	RT_SPI_write(addr,content)	//RT_SPI_write(((WR_LOCAL<<24)+(0x00ffffff&(addr))),content)/*<write to SPI */

#define SPI_INT_REG_XADC 0
#define SPI_INT_REG_FRAME_ECCE2 1
#define SPI_INT_REG_TAMPER 2
#define SPI_INT_REG_SPI_ERROR 3
#define SPI_INT_REG_FIFO_OVERFLOW 4
#define SPI_INT_REG_FIFO_UNDERRUN 5
#define SPI_INT_REG_EXT 6
#define SPI_INT_REG_WATCHDOG 7
#define SPI_INT_REG_MMI 8
#define SPI_INT_REG_TEST 9
#define SPI_INT_REG_TxConf0 10
#define SPI_INT_REG_TxConf1 11
#define SPI_INT_REG_TxConf2 12
