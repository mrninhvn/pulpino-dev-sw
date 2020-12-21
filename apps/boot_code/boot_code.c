// Copyright 2017 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.


#include <spi.h>
#include <gpio.h>
#include <uart.h>
#include <utils.h>
#include <pulpino.h>

#ifdef FPGA_ARTY_A7_100T
#define SPANSION_S25FL
#endif

#ifdef SISLAB_PULPINO
#define SPANSION_S25FS_SINGLE_MODE
#endif

// definition for SPI Flash command
#define WREN 0x06

#ifdef SPANSION_S25FL
  #define WRR 0x01 // write reg
  #define WRR_DATA_LEN 16
  #define WRR_DATA (0x0000 << 16) // disable QPI for ARTY A7 100T
  #define FAST_READ 0x0B // no QPI read
  #define SPI_READ_MODE SPI_CMD_RD
  #define ADDR_LEN 24
#else
  #ifdef SPANSION_S25FS_SINGLE_MODE
    #define WRR 0x71 // write any reg command for SPANSION S25FS
    #define WRR_DATA 0x80000308
    #define WRR_DATA_LEN 32
    #define FAST_READ 0x0B // single read
    #define SPI_READ_MODE SPI_CMD_RD
    #define ADDR_LEN 24
  #else // pulpino default's
    #define WRR 0x71 // write any reg command for SPANSION S25FS
    #define WRR_DATA 0x80000348
    #define WRR_DATA_LEN 32
    #define FAST_READ 0xEB // QPI read
    #define SPI_READ_MODE SPI_CMD_QRD
    #define ADDR_LEN 32
  #endif
#endif

const char g_numbers[] = {
                           '0', '1', '2', '3', '4', '5', '6', '7',
                           '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
                         };

int check_spi_flash();
void load_block(unsigned int addr, unsigned int len, int* dest);
void uart_send_block_done(unsigned int i);
void jump_and_start(volatile int *ptr);

int main()
{
  /* sets direction for SPI master pins with only one CS */
  spi_setup_master(1);
#ifdef PULPINO_FPGA
  uart_set_cfg(0, 0xd);
#else
  uart_set_cfg(0, 1);
#endif

  for (int i = 0; i < 3000; i++) {
    //wait some time to have proper power up of external flash
    #ifdef __riscv__
        asm volatile ("nop");
    #else
        asm volatile ("l.nop");
    #endif
  }

  /* divide sys clock by 4 */
  *(volatile int*) (SPI_REG_CLKDIV) = 0x4;

  if (check_spi_flash()) {
    uart_send("ERROR: Spansion SPI flash not found\n", 36);
    while (1);
  }


  uart_send("Loading from SPI\n", 17);

  // sends write enable command
  spi_setup_cmd_addr(WREN, 8, 0, 0);
  spi_set_datalen(0);
  spi_start_transaction(SPI_CMD_WR, SPI_CSN0);
  while ((spi_get_status() & 0xFFFF) != 1);

  spi_setup_cmd_addr(WRR, 8, WRR_DATA, WRR_DATA_LEN);
  spi_set_datalen(0);
  spi_start_transaction(SPI_CMD_WR, SPI_CSN0);
  while ((spi_get_status() & 0xFFFF) != 1);

#ifdef SPANSION_S25FL		/* S25FL takes long time to write CR1 */
  int status = 1;
  while (status != 0){
    spi_setup_cmd_addr(0x05, 8, 0, 0);
    spi_set_datalen(16);
    spi_setup_dummy(0,0);
    spi_start_transaction(SPI_CMD_RD, SPI_CSN0);
    spi_read_fifo(&status,16);
    status &= 0x1;
  }
#endif
  //-----------------------------------------------------------
  // Read header
  //-----------------------------------------------------------

  int header_ptr[8];
#ifdef FPGA_ARTY_A7_100T
  /* for running on FPGA kit */
  int addr = 0x00400000;
  /* for RTL simulation */
  /* int addr = 0x0; */
#else
  int addr = 0;
#endif

  spi_setup_dummy(8, 0);

  // cmd fast read, needs 8 dummy cycles
  spi_setup_cmd_addr(FAST_READ, 8, ((addr << 8) & 0xFFFFFF00), ADDR_LEN);
  spi_set_datalen(8 * 32);
  spi_start_transaction(SPI_READ_MODE, SPI_CSN0);
  spi_read_fifo(header_ptr, 8 * 32);

  int instr_start = header_ptr[0];
  int *instr = (int *) header_ptr[1];
  int instr_size =  header_ptr[2];
  int instr_blocks = header_ptr[3];

  int data_start = header_ptr[4];
  int *data = (int *) header_ptr[5];
  int data_size = header_ptr[6];
  int data_blocks = header_ptr[7];


  //-----------------------------------------------------------
  // Read Instruction RAM
  //-----------------------------------------------------------

  uart_send("Copying Instructions\n", 21);

  addr = instr_start;
  spi_setup_dummy(8, 0);
  for (int i = 0; i < instr_blocks; i++) { //reads 16 4KB blocks
    // cmd fast read, needs 8 dummy cycles
    spi_setup_cmd_addr(FAST_READ, 8, ((addr << 8) & 0xFFFFFF00), ADDR_LEN);
    spi_set_datalen(32768);
    spi_start_transaction(SPI_READ_MODE, SPI_CSN0);
    spi_read_fifo(instr, 32768);

    instr += 0x400;  // new address = old address + 1024 words
    addr  += 0x1000; // new address = old address + 4KB

    uart_send_block_done(i);
  }

  while ((spi_get_status() & 0xFFFF) != 1);

  //-----------------------------------------------------------
  // Read Data RAM
  //-----------------------------------------------------------

  uart_send("Copying Data\n", 13);

  uart_wait_tx_done();
  addr = data_start;
  spi_setup_dummy(8, 0);
  for (int i = 0; i < data_blocks; i++) { //reads 16 4KB blocks
    // cmd fast read, needs 8 dummy cycles
    spi_setup_cmd_addr(FAST_READ, 8, ((addr << 8) & 0xFFFFFF00), ADDR_LEN);
    spi_set_datalen(32768);
    spi_start_transaction(SPI_READ_MODE, SPI_CSN0);
    spi_read_fifo(data, 32768);

    data += 0x400;  // new address = old address + 1024 words
    addr += 0x1000; // new address = old address + 4KB

    uart_send_block_done(i);
  }
#ifdef SPANSION_S25FL
  // sends write enable command
  spi_setup_cmd_addr(WREN, 8, 0, 0);
  spi_set_datalen(0);
  spi_start_transaction(SPI_CMD_WR, SPI_CSN0);
  while ((spi_get_status() & 0xFFFF) != 1);

  // restore QPI mode for Arty A7 100T
  spi_setup_cmd_addr(WRR, 8, (0x0002 << 16), 16);
  spi_set_datalen(0);
  spi_start_transaction(SPI_CMD_WR, SPI_CSN0);
  while ((spi_get_status() & 0xFFFF) != 1);
  status = 1;
  while (status != 0){
    spi_setup_cmd_addr(0x05, 8, 0, 0);
    spi_set_datalen(16);
    spi_setup_dummy(0,0);
    spi_start_transaction(SPI_CMD_RD, SPI_CSN0);
    spi_read_fifo(&status,16);
    status &= 0x1;
  }
#endif
  uart_send("Done, jumping to Instruction RAM.\n", 34);

  uart_wait_tx_done();

  //-----------------------------------------------------------
  // Set new boot address -> exceptions/interrupts/events rely
  // on that information
  //-----------------------------------------------------------

  BOOTREG = 0x00;

  //-----------------------------------------------------------
  // Done jump to main program
  //-----------------------------------------------------------

  //jump to program start address (instruction base address)
  jump_and_start((volatile int *)(INSTR_RAM_START_ADDR));
}

int check_spi_flash() {
  int err = 0;
  int rd_id[2];

  // reads flash ID
  spi_setup_cmd_addr(0x9F, 8, 0, 0);
  spi_set_datalen(64);
  spi_setup_dummy(0, 0);
  spi_start_transaction(SPI_CMD_RD, SPI_CSN0);
  spi_read_fifo(rd_id, 64);


  // id should be 0x0102194D
  if (((rd_id[0] >> 24) & 0xFF) != 0x01)
    err++;

  // check flash model is 128MB or 256MB 1.8V
  if ( (((rd_id[0] >> 8) & 0xFFFF) != 0x0219) &&
       (((rd_id[0] >> 8) & 0xFFFF) != 0x2018) )
    err++;

  return err;
}

void load_block(unsigned int addr, unsigned int len, int* dest) {
  // cmd 0xEB fast read, needs 8 dummy cycles
  spi_setup_cmd_addr(0xEB, 8, ((addr << 8) & 0xFFFFFF00), 32);
  spi_set_datalen(len);
  spi_start_transaction(SPI_CMD_QRD, SPI_CSN0);
  spi_read_fifo(dest, len);
}

void jump_and_start(volatile int *ptr)
{
#ifdef __riscv__
  asm("jalr x0, %0\n"
      "nop\n"
      "nop\n"
      "nop\n"
      : : "r" (ptr) );
#else
  asm("l.jr\t%0\n"
      "l.nop\n"
      "l.nop\n"
      "l.nop\n"
      : : "r" (ptr) );
#endif
}

void uart_send_block_done(unsigned int i) {
  unsigned int low  = i & 0xF;
  unsigned int high = i >>  4; // /16

  uart_send("Block ", 6);

  uart_send(&g_numbers[high], 1);
  uart_send(&g_numbers[low], 1);

  uart_send(" done\n", 6);

  uart_wait_tx_done();
}
