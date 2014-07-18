#include "application.h"

#include <cc3k.h>

#include <lcd.h>

int dfu() {

 FLASH_OTA_Update_SysFlag = 0x0000;
 Save_SystemFlags();
 BKP_WriteBackupRegister(BKP_DR10, 0x0000);
 USB_Cable_Config(DISABLE);
 NVIC_SystemReset();

 return 0;
}

extern "C" {

  uint32_t dma_int_count = 0;
  uint32_t int_count = 0;

  volatile int dma_pending = 0;

  cc3k_t driver;
  cc3k_config_t driver_config;

  void SPI_DMA_IntHandler(void)
  {
    dma_int_count++; 

    // This is probably not what we want to do, need to investigate DMA/SPI interrupts more..
    while (SPI_I2S_GetFlagStatus(CC3000_SPI, SPI_I2S_FLAG_BSY ) != RESET)
    DMA_ClearFlag(CC3000_SPI_TX_DMA_TCFLAG | CC3000_SPI_RX_DMA_TCFLAG);
  
    dma_pending = 0;

    cc3k_spi_done(&driver);
  }

  void SPI_EXTI_IntHandler(void)
  {
    digitalWrite(D1, HIGH);
    cc3k_interrupt(&driver);
    int_count++;
    digitalWrite(D1, LOW);
  }
}

void _spi(uint8_t *out, uint8_t *in, uint16_t size, int async)
{
  dma_pending = 1;
  CC3000_SPI_DMA_Channels(DISABLE);

  CC3000_DMA_Config(CC3000_DMA_RX, in, size);
  CC3000_DMA_Config(CC3000_DMA_TX, out, size);

  /* Enable DMA SPI Interrupt */
  DMA_ITConfig(CC3000_SPI_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);

  /* Enable DMA Channels */
  CC3000_SPI_DMA_Channels(ENABLE);

  if(async == 0)
    while(dma_pending == 1) {}
}

void spark_cc3k_enable(int enable)
{
  CC3000_Write_Enable_Pin(enable);
}

int spark_cc3k_read_interrupt(void)
{
  return CC3000_Read_Interrupt_Pin();
}

void spark_cc3k_enable_interrupt(int enable)
{
  if(enable)
    NVIC_EnableIRQ(CC3000_WIFI_INT_EXTI_IRQn);
  else
    NVIC_DisableIRQ(CC3000_WIFI_INT_EXTI_IRQn);
}

void spark_cc3k_assert_cs(int assert)
{
  if(assert)
  {
    digitalWrite(D0, LOW);
    CC3000_CS_LOW();
  }
  else
  {
    digitalWrite(D0, HIGH);
    CC3000_CS_HIGH();
  }
}

cc3k_status_t init_res;

lcd_t lcd;

void setup()
{
  Serial.begin(115200);

  pinMode(D7, OUTPUT);
  digitalWrite(D7, LOW);

  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);

  digitalWrite(D0, HIGH);
  digitalWrite(D3, LOW);

  ssd1306_init(&lcd, D3, D2, D6);
  ssd1306_clear(&lcd);
  ssd1306_print(&lcd, "Hello World!");
  ssd1306_cursor(&lcd, 0,1);

  CC3000_WIFI_Init();
  CC3000_Interrupt_Enable();
  // Setup the STM32 hardware, from hw_config.c
  CC3000_SPI_DMA_Init();

  // Setup the driver callbacks
  driver_config.delayMicroseconds = Delay_Microsecond;
  driver_config.enableChip = spark_cc3k_enable;
  driver_config.readInterrupt = spark_cc3k_read_interrupt;
  driver_config.enableInterrupt = spark_cc3k_enable_interrupt;
  driver_config.assertChipSelect = spark_cc3k_assert_cs;
  driver_config.spiTransaction = _spi;

}

void print_buffers()
{
  int x;
  char tmp[257];

  bzero(tmp, 257);
  for(x=0;x<32;x++)
  {
    sprintf(tmp+x*2, "%02X", driver.packet_rx_buffer[x]);
  }


  Serial.println(tmp);

  bzero(tmp, 257);
  for(x=0;x<32;x++)
  {
    sprintf(tmp+x*2, "%02X", driver.packet_tx_buffer[x]);
  }

  Serial.println(tmp);
}

void loop()
{
  cc3k_status_t status;
  int c;
  
  //char s[21];

  //snprintf(s, 21, "%u", millis());
  //ssd1306_print(&lcd, s);

  status = cc3k_loop(&driver, millis());
  if(status != CC3K_OK)
  {
    Serial.print("CC3K Loop Error: ");
    Serial.println(status);
  }

  if(Serial.available())
  {
    c = Serial.read();

    switch(c)
    {
      case 0x3:
        ssd1306_clear(&lcd);
        ssd1306_cursor(&lcd, 0,0);
        break;
      case 0x4:
        ssd1306_clear(&lcd);
        ssd1306_print(&lcd, "DFU Mode");
        dfu();
        break;
      default:
        ssd1306_write(&lcd, c);
        break;
    }

#if 0
    switch(c)
    {
      case 'D':
        // Reboot into DFU mode
        dfu();
        break;
      case 's':
        Serial.print("Initialize CC3K: ");
        init_res = cc3k_init(&driver, &driver_config);
        Serial.println(init_res);
        break;

      case 'q':
        //cc3k_status(&driver);
        break;

      case 'b':
        print_buffers();
        break;

      case 'i':
        Serial.print("INT Pin: ");
        Serial.println(spark_cc3k_read_interrupt());
        Serial.print("Buffers available: ");
        Serial.println(driver.buffers);
        Serial.print("WLAN Status: ");
        Serial.println(driver.wlan_status);
        Serial.print("CC3K State: ");
        Serial.println(driver.state);
        Serial.print("CC3K Unhandled State: ");
        Serial.println(driver.unhandled_state);
        Serial.print("Commands TX: ");
        Serial.println(driver.stats.commands);
        Serial.print("Events RX: ");
        Serial.println(driver.stats.events);
        Serial.print("Unsolicited Events RX: ");
        Serial.println(driver.stats.unsolicited);
        Serial.print("DMA Interrupts: ");
        Serial.println(dma_int_count);
        Serial.print("HW Interrupts: ");
        Serial.println(int_count);
        Serial.print("Unhandled Interrupts: ");
        Serial.println(driver.stats.unhandled_interrupts);
        break;

      default:
        Serial.print("Unknown command: ");
        Serial.println(c);
        break;
    }
#endif
  }
}
