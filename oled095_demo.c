/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header file */
#include "Board.h"

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on Board_GPIO_BUTTON0.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Clear the GPIO interrupt and toggle an LED */
    //GPIO_toggle(Board_GPIO_LED0);

    uint_fast8_t v = GPIO_read(index);
    GPIO_write(Board_GPIO_LED0, !v);
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on Board_GPIO_BUTTON1.
 *  This may not be used for all boards.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Clear the GPIO interrupt and toggle an LED */
  //  GPIO_toggle(Board_GPIO_LED1);

    uint_fast8_t v = GPIO_read(index);
    GPIO_write(Board_GPIO_LED1, !v);
}

const uint_least8_t reset_pin = MY_GPIO_EPAPER_RST;
const uint_least8_t dc_pin = MY_GPIO_EPAPER_DC;
const uint_least8_t cs_pin = MY_GPIO_SPI_CS;
const uint_least8_t busy_pin = CC1310_LAUNCHXL_GPIO_S1; // use button s1 as input

SPI_Handle      spi;

void Send_Spi(unsigned char data)
{
    GPIO_write(MY_GPIO_SPI_CS, 0);

    SPI_Transaction spiTransaction;
    uint8_t         transmitBuffer;
    uint8_t         receiverBuffer;
    bool            transferOK;

    receiverBuffer = 0;
    transmitBuffer = data;

    spiTransaction.count = 1;
    spiTransaction.txBuf = &transmitBuffer;
    spiTransaction.rxBuf = &receiverBuffer;
    transferOK = SPI_transfer(spi, &spiTransaction);

    if (!transferOK) {
        // Error in SPI or transfer already in progress.
        while(1);
    }

    GPIO_write(MY_GPIO_SPI_CS, 1);

}
void Write_Command(unsigned char Data)
{
//unsigned char i;
//
//    OLED_CS_Clr();
//    OLED_RS_Clr();
//    for (i=0; i<8; i++)
//    {
//        OLED_SCLK_Clr();
//                if(Data&0x80)
//           OLED_SDIN_Set();
//        else
//           OLED_SDIN_Clr();
//        Data = Data << 1;
////      uDelay(1);
//        OLED_SCLK_Set();
////      uDelay(1);
//    }
////  SCLK=0;
//    OLED_RS_Set();
//    OLED_CS_Set();
    GPIO_write(dc_pin, 0);
    Send_Spi(Data);
}

void Write_Data(unsigned char Data)
{
//    unsigned char i;
//
//    OLED_CS_Clr();
//    OLED_RS_Set();
//    for (i=0; i<8; i++)
//    {
//        OLED_SCLK_Clr();
//                if(Data&0x80)
//           OLED_SDIN_Set();
//        else
//           OLED_SDIN_Clr();
//        Data = Data << 1;
////      uDelay(1);
//        OLED_SCLK_Set();
////      uDelay(1);
//    }
////  SCLK=0;
//    OLED_RS_Set();
//    OLED_CS_Set();
    GPIO_write(dc_pin, 1);
    Send_Spi(Data);
}



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Instruction Setting
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Set_Column_Address(unsigned char a, unsigned char b)
{
    Write_Command(0x15);            // Set Column Address
    Write_Command(a);                   //   Default => 0x00 (Start Address)
    Write_Command(b);                   //   Default => 0x5F (End Address)
}


void Set_Row_Address(unsigned char a, unsigned char b)
{
    Write_Command(0x75);            // Set Row Address
    Write_Command(a);                   //   Default => 0x00 (Start Address)
    Write_Command(b);                   //   Default => 0x3F (End Address)
}



void Set_Remap_Format(unsigned char d)
{
    Write_Command(0xA0);            // Set Re-Map / Color Depth
    Write_Command(d);                   //   Default => 0x40
                                    //     Horizontal Address Increment
                                    //     Column Address 0 Mapped to SEG0
                                    //     Color Sequence: A => B => C
                                    //     Scan from COM0 to COM[N-1]
                                    //     Disable COM Split Odd Even
                                    //     65,536 Colors
}


void Set_Start_Line(unsigned char d)
{
    Write_Command(0xA1);            // Set Vertical Scroll by RAM
    Write_Command(d);                   //   Default => 0x00   00-63D
}


void Set_Display_Offset(unsigned char d)
{
    Write_Command(0xA2);            // Set Vertical Scroll by Row
    Write_Command(d);                   //   Default => 0x00    00-63D
}


void Set_Display_Mode(unsigned char d)
{
    Write_Command(0xA4|d);          // Set Display Mode
                                    //   Default => 0xA4
                                    //     0xA4 (0x00) => Normal Display
                                    //     0xA5 (0x01) => Entire Display On, All Pixels Turn On at GS Level 63
                                    //     0xA6 (0x02) => Entire Display Off, All Pixels Turn Off
                                    //     0xA7 (0x03) => Inverse Display
}



void Set_Display_On_Off(unsigned char d)
{
    Write_Command(0xAE|d);          // Set Display On/Off
                                    // Default => 0xAE
                                    // 0xAE (0x00) => Display Off (Sleep Mode On)
                                    // 0xAF (0x01) => Display On (Sleep Mode Off)
}

void Set_Power_Saving_Mode(unsigned char d)
{
    Write_Command(0xB0);            // Set Power Saving Mode
    Write_Command(d);                   // Default => 0x1A
}

void Set_Reset_Pre_charge_period(unsigned char d)
{
    Write_Command(0xB1);            // Set Reset (Phase1)/Pre-charge (Phase 2) period
    Write_Command(d);                   // Default => 0x31
}

void Set_Oscillator_Frequency_Clock_Divider(unsigned char d)
{
    Write_Command(0xB3);            // Set Display Clock Divider / Oscillator Frequency
    Write_Command(d);                   // Default => 0xDO
                                    // A[3:0] => Display Clock Divider
                                    // A[7:4] => Oscillator Frequency
}





void Set_Precharge_Period(unsigned char d)
{
    Write_Command(0xB6);            // Set Second Pre-Charge Period
    Write_Command(d);                   //   Default => 0x08 (8 Display Clocks)
}


void Set_Pre_charge_Level(unsigned char d)
{
    Write_Command(0xBB);            // Set Pre-charge Level
    Write_Command(d);                   //   Default => 0x3E (0.5*VCC)
}

void Set_VCOMH(unsigned char d)
{
    Write_Command(0xBE);            // Set COM Deselect Voltage Level
    Write_Command(d);                   //   Default => 0x3E (0.83*VCC)
}


void Set_Contrast_Color(unsigned char a, unsigned char b, unsigned char c)
{
    Write_Command(0x81);            // Set Contrast Current for Color A,
    Write_Command(a);                   //   Default => 0x80 (Maximum)
    Write_Command(0x82);            // Set Contrast Current for Color B,
    Write_Command(b);                   //   Default => 0x80 (Maximum)
    Write_Command(0x83);            // Set Contrast Current for Color C,
    Write_Command(c);                   //   Default => 0x80 (Maximum)
}

void Set_Second_Pre_charge_Speed_of_Color(unsigned char a, unsigned char b, unsigned char c)
{
    Write_Command(0x8A);            // Set Second Pre-charge Speed of Color A,
    Write_Command(a);               //   Default => 0x80 (Color A)
    Write_Command(0x8B);            // Set Second Pre-charge Speed of Color B,
    Write_Command(b);               //   Default => 0x80 (Color B)
    Write_Command(0x8C);            // Set Second Pre-charge Speed of Color C,
    Write_Command(c);               //   Default => 0x80 (Color C)
}

void Set_Master_Current(unsigned char d)
{
    Write_Command(0x87);            // Master Contrast Current Control
    Write_Command(d);               //   Default => 0x0F (Maximum)
}


void Set_Master_Configuration(unsigned char d)
{
    Write_Command(0xAD);            // Master Contrast Configuration
    Write_Command(d);                   //   Default => 0x8E (Maximum)
}




void Set_Multiplex_Ratio(unsigned char d)
{
    Write_Command(0xA8);            // Set Multiplex Ratio
    Write_Command(d);                   //   Default => 0x3F  N = A[5:0] from 15d to 63d(3Fh)
}


void Set_Command_Lock(unsigned char d)
{
    Write_Command(0xFD);            // Set Command Lock
    Write_Command(d);                   // Default => 0x12
                                    // 0x12 => Driver IC interface is unlocked from entering command.
                                    // 0x16 => All Commands are locked except 0xFD.

}




//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void OLED_Init()
{
unsigned char i;


//    GPIO_InitTypeDef  GPIO_InitStructure;
//
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOG, ENABLE);  //??PC,D,G????

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_6;     //PD3,PD6????
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         //????
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//??50MHz
//    GPIO_Init(GPIOD, &GPIO_InitStructure);    //???GPIOD3,6
//    GPIO_SetBits(GPIOD,GPIO_Pin_3|GPIO_Pin_6);  //PD3,PD6 ???

// #if OLED_MODE==1
//
//    GPIO_InitStructure.GPIO_Pin =0xFF; //PC0~7 OUT????
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//    GPIO_SetBits(GPIOC,0xFF); //PC0~7???
//
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;               //PG13,14,15 OUT????
//    GPIO_Init(GPIOG, &GPIO_InitStructure);
//    GPIO_SetBits(GPIOG,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);                         //PG13,14,15 OUT  ???
//
// #else
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;                 //PC0,1 OUT????
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//    GPIO_SetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1);                       //PC0,1 OUT  ???
//
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;               //PG15 OUT????   RST
//    GPIO_Init(GPIOG, &GPIO_InitStructure);
//    GPIO_SetBits(GPIOG,GPIO_Pin_15);                         //PG15 OUT  ???
//
//
// #endif
//    OLED_RST_Clr();
    GPIO_write(reset_pin, 0);
    for(i=0;i<250;i++)
    {
//        uDelay(250);
        usleep(250);
    }
//    OLED_RST_Set();
    GPIO_write(reset_pin, 1);

     Set_Display_On_Off(0x00);          // Display Off (0x00/0x01)
     Set_Remap_Format(0x72);            // Set Horizontal Address Increment
                                        //0x72(rgb)0x74(bgr)

     Set_Start_Line(0x00);                      // Set Mapping RAM Display Start Line (0x00~0x3F)
     Set_Display_Offset(0x00);                  // Shift Mapping RAM Counter (0x00~0x7F)
     Set_Display_Mode(0x00);                    // Normal Display Mode (0x00/0x01/0x02/0x03)
     Set_Multiplex_Ratio(0x3F);                 // 1/128 Duty (0x0F~0x7F)
     Set_Master_Configuration(0x8e);            // set master configuration
     Set_Power_Saving_Mode(0x0b);               // set power save
     Set_Reset_Pre_charge_period(0x31);         //phase 1 and 2 period adjustment
     Set_Oscillator_Frequency_Clock_Divider(0xf0);
                                                //display clock divider / oscillator frequency
     Set_Second_Pre_charge_Speed_of_Color(0x64, 0x78, 0x64);
                                                //Set Second Pre-change Speed For Color
     Set_Pre_charge_Level(0x3a);                //Set Pre-Change Level
     Set_VCOMH(0x3e);                           // Set Common Pins Deselect Voltage Level as 0.82*VCC
     Set_Master_Current(0x06);              // Set Scale Factor of Segment Output Current Control    Brightness
     Set_Contrast_Color(0x91,0x50,0x7d);    // Set Contrast of Color A (Red)
                                            // Set Contrast of Color B (Green)
                                            // Set Contrast of Color C (Blue)
     Set_Display_On_Off(0x01);              // Display On (0x00/0x01)
     //Fill_RAM(0x00,0x00);                 // Clear Screen
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Full Screen)
//
//    a: RRRRRGGG
//    b: GGGBBBBB
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Fill_RAM(unsigned char a, unsigned char b)
{
unsigned char i,j;

    Set_Column_Address(0x00,0x5F);
    Set_Row_Address(0x00,0x3F);
//  Set_Write_RAM();

    for(i=0;i<64;i++)
    {
        for(j=0;j<96;j++)
        {
            Write_Data(a);
            Write_Data(b);
        }
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Partial or Full Screen)
//
//    a: Column Address of Start
//    b: Column Address of End
//    c: Row Address of Start
//    d: Row Address of End
//    e: RRRRRGGG
//    f: GGGBBBBB
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Fill_Block(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f)
{
unsigned char i,j;

    Set_Column_Address(a,b);
    Set_Row_Address(c,d);
//  Set_Write_RAM();

    for(i=0;i<(d-c+1);i++)
    {
        for(j=0;j<(b-a+1);j++)
        {
            Write_Data(e);
            Write_Data(f);
        }
    }
}

#define Max_Column  0x5f            // 0-95
#define Max_Row     0x3f            // 0-63
#define Brightness  0x0F
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Color Bar (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Rainbow()
{
    // White => Column 1~16
        Fill_Block(0x00,0x0c,0x00,Max_Row,0xFF,0xFF);

    // Yellow => Column 17~32
        Fill_Block(0x0c,0x18,0x00,Max_Row,0xFF,0xE0);

    // Purple => Column 33~48
        Fill_Block(0x18,0x24,0x00,Max_Row,0xF8,0x1F);

    // Cyan => Column 49~64
        Fill_Block(0x24,0x30,0x00,Max_Row,0x07,0xFF);

    // Red => Column 65~80
        Fill_Block(0x30,0x3c,0x00,Max_Row,0xF8,0x00);

    // Green => Column 81~96
        Fill_Block(0x3c,0x48,0x00,Max_Row,0x07,0xE0);

    // Blue => Column 97~112
        Fill_Block(0x48,0x54,0x00,Max_Row,0x00,0x1F);

    // Black => Column 113~128
        Fill_Block(0x54,Max_Column,0x00,Max_Row,0x00,0x00);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Pattern (Partial or Full Screen)
//
//    a: Column Address of Start
//    b: Column Address of End
//    c: Row Address of Start
//    d: Row Address of End
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Show_64k_Pattern(unsigned char *Data_Pointer, unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{
    unsigned char *Src_Pointer;
    unsigned int i,j;
    j=(b-a+1)*(d-c+1);
        Src_Pointer=Data_Pointer;
        Set_Column_Address(a,b);
        Set_Row_Address(c,d);
    //  Set_Write_RAM();

        /*for(i=0;i<(d-c+1);i++)
        {
            for(j=0;j<(b-a+1);j++)
            {
                Write_Data(*Src_Pointer);
                Src_Pointer++;
                Write_Data(*Src_Pointer);
                Src_Pointer++;
            }
        }
    }*/
    //      Set_Column_Address(a,b);
    //  Set_Row_Address(c,d);
    for (i = 0; i<j; i++)
    {       Write_Data(*Src_Pointer);
                Src_Pointer++;
                Write_Data(*Src_Pointer);
                Src_Pointer++;
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Checkboard (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Checkerboard()
{
unsigned char i,j;

    Set_Column_Address(0x00,0x5F);
    Set_Row_Address(0x00,0x3F);
//  Set_Write_RAM();

    for(i=0;i<64;i++)
    {
        for(j=0;j<96;j++)
        {
            Write_Data(0xFF);
            Write_Data(0xFF);
            Write_Data(0x00);
            Write_Data(0x00);
        }
        for(j=0;j<96;j++)
        {
            Write_Data(0x00);
            Write_Data(0x00);
            Write_Data(0xFF);
            Write_Data(0xFF);
        }
    }
}


// 50=> 0.5 second
void Delay(unsigned int ms)
{
    usleep(ms*1000*10);
}

#include "gimage.h"

void demo()
{
    OLED_Init();
    Fill_RAM(0x00,0x00);                        // Clear Screen

    Rainbow();

    while(1)
      {
       Fill_RAM(0x00,0x00);     // Clear Screen
       Rainbow();
       Delay(50);
       //
       Fill_RAM(0x00,0x00);     // Clear Screen
        // Show Pattern - Format
        Show_64k_Pattern(gImage2,0x00,0x5f,0x00,0x3f);   //80*64
     // Show_64k_Pattern(&Format,0x00,0x4f,0x00,0x3f);   //ok 96*64
        Delay(50);
                       Fill_RAM(0x00,0x00);     // Clear Screen
        // Show Pattern - Format
        Show_64k_Pattern(gImage3,0x00,0x5f,0x00,0x3f);   //80*64
     // Show_64k_Pattern(&Format,0x00,0x4f,0x00,0x3f);   //ok 96*64
        Delay(50);
                       Fill_RAM(0x00,0x00);     // Clear Screen
        // Show Pattern - Format
        Show_64k_Pattern(gImage4,0x00,0x5f,0x00,0x3f);   //80*64
     // Show_64k_Pattern(&Format,0x00,0x4f,0x00,0x3f);   //ok 96*64
        Delay(50);
                       Fill_RAM(0x00,0x00);     // Clear Screen
        // Show Pattern - Format
        Show_64k_Pattern(gImage1,0x00,0x5f,0x00,0x3f);   //80*64
     // Show_64k_Pattern(&Format,0x00,0x4f,0x00,0x3f);   //ok 96*64
        Delay(50);

        Fill_RAM(0x00,0x00);        // Clear Screen



        Delay(1);
        Checkerboard();
        Delay(50);

       Fill_RAM(0x00,0x1f);                    //all red
       Delay(50);
       Fill_RAM(0x07,0xe0);                   //all green
       Delay(50);
       Fill_RAM(0xf8,0x00);                   //all blue
       Delay(50);
       Fill_RAM(0xff,0xff);                   //all while
       Delay(50);
       Fill_RAM(0x07,0xff);                   //all yellow
       Delay(50);
       Fill_RAM(0xff,0xe0);                   //  qin
       Delay(50);
       Fill_RAM(0xf8,0x1f);                   //  zi
       Delay(50);
      }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* install Button callback */
    GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);

    // fix default config, enable both edge interrupt
    GPIO_PinConfig config;
    GPIO_getConfig(Board_GPIO_BUTTON0, &config);
    config |= GPIO_CFG_IN_INT_BOTH_EDGES;
    GPIO_setConfig(Board_GPIO_BUTTON0, config);

    /* Enable interrupts */
    GPIO_enableInt(Board_GPIO_BUTTON0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on Board_GPIO_BUTTON1.
     */
    if (Board_GPIO_BUTTON0 != Board_GPIO_BUTTON1) {

        GPIO_PinConfig config;
        GPIO_getConfig(Board_GPIO_BUTTON1, &config);
        config |= GPIO_CFG_IN_INT_BOTH_EDGES;
        GPIO_setConfig(Board_GPIO_BUTTON1, config);

        /* Install Button callback */
        GPIO_setCallback(Board_GPIO_BUTTON1, gpioButtonFxn1);
        GPIO_enableInt(Board_GPIO_BUTTON1);
    }

    SPI_init();

    SPI_Params      spiParams;
    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spiParams.dataSize = 8;       // 8-bit data size

    spi = SPI_open(CC1310_LAUNCHXL_SPI0, &spiParams);
    if (spi == NULL) {
        while (1);  // SPI_open() failed
    }

    demo();

    return NULL;
}
