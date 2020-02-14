/*! Analog Devices DC2350A-B Demonstration Board.
* LTC6813: Multicell Battery Monitors
*
*@verbatim
*NOTES
* Setup:
*   Set the terminal baud rate to 115200 and select the newline terminator.
*   Ensure all jumpers on the demo board are installed in their default positions from the factory.
*   Refer to Demo Manual.
*
*USER INPUT DATA FORMAT:
* decimal : 1024
* hex     : 0x400
* octal   : 02000  (leading 0)
* binary  : B10000000000
* float   : 1024.0
*@endverbatim
*
* https://www.analog.com/en/products/ltc6813-1.html
* https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/dc2350a-b.html
*
********************************************************************************
* Copyright 2019(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/*! @file
    @ingroup LTC6813-1
*/

/************************************* Read me *******************************************
In this sketch book:
  -All Global Variables are in Upper casing
  -All Local Variables are in lower casing
  -The Function wakeup_sleep(TOTAL_IC) : is used to wake the LTC681x from sleep state.
   It is defined in LTC681x.cpp
  -The Function wakeup_idle(TOTAL_IC) : is used to wake the ICs connected in daisy chain
   via the LTC6820 by initiating a dummy SPI communication. It is defined in LTC681x.cpp
*******************************************************************************************/

/************************* Includes ***************************/
#include "LTC6813.h"
#include "LTC681x.h"
#include "LT_SPI.h"
#include "Linduino.h"
#include "UserInterface.h"
#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <Timer.h>
/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2

/**************** Local Function Declaration *******************/
void measurement_loop(uint8_t datalog_en);
void print_menu(void);
void print_wrconfig(void);
void print_wrconfigb(void);
void print_rxconfig(void);
void print_rxconfigb(void);
void print_cells(uint8_t datalog_en);
void print_aux(uint8_t datalog_en);
void print_stat(void);
void print_aux1(uint8_t datalog_en);
void print_sumofcells(void);
void check_mux_fail(void);
void print_selftest_errors(uint8_t adc_reg, int8_t error);
void print_overlap_results(int8_t error);
void print_digital_redundancy_errors(uint8_t adc_reg, int8_t error);
void print_open_wires(void);
void print_pec_error_count(void);
int8_t select_s_pin(void);
int8_t select_ic(void);
void print_wrpwm(void);
void print_rxpwm(void);
void print_wrsctrl(void);
void print_rxsctrl(void);
void print_wrpsb(uint8_t type);
void print_rxpsb(uint8_t type);
void print_wrcomm(void);
void print_rxcomm(void);
void check_mute_bit(void);
void print_conv_time(uint32_t conv_time);
void check_error(int error);
void serial_print_text(char data[]);
void serial_print_hex(uint8_t data);
char read_hex(void);
char get_char(void);

/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
const uint8_t TOTAL_IC = 1; //!< Number of ICs in the daisy chain

/********************************************************************
 ADC Command Configurations. See LTC681x.h for options
*********************************************************************/
const uint8_t ADC_OPT = ADC_OPT_DISABLED;         //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED;             //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;   //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;     //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;   //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL;              //!< Register Selection
const uint8_t SEL_REG_A = REG_1;                  //!< Register Selection
const uint8_t SEL_REG_B = REG_2;                  //!< Register Selection
const uint16_t MEASUREMENT_LOOP_TIME = 500;       //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED;  //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = DISABLED;  //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = DISABLED;    //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop
/************************************
  END SETUP
*************************************/

/*******************************************************
 Global Battery Variables received from 681x commands
 These variables store the results from the LTC6813
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable
//cell_asic BMS_compare[TOTAL_IC]; // Copy the BMS_IC to make the sorting
/*************************************************************************
 Set configuration register. Refer to the data sheet
**************************************************************************/
bool REFON = true;                                                                                         //!< Reference Powered Up Bit
bool ADCOPT = false;                                                                                       //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false, false, true, true, true};                                                     //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool GPIOBITS_B[4] = {false, false, false, false};                                                         //!< GPIO Pin Control // Gpio 6,7,8,9
uint16_t UV = UV_THRESHOLD;                                                                                //!< Under voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD;                                                                                //!< Over voltage Comparison Voltage
bool DCCBITS_A[12] = {false, false, false, false, false, false, false, false, false, false, false, false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7] = {false, false, false, false, false, false, false};                                     //!< Discharge cell switch //Dcc 0,13,14,15
bool DCTOBITS[4] = {true, false, true, false};                                                             //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
bool FDRF = false;               //!< Force Digital Redundancy Failure Bit
bool DTMEN = true;               //!< Enable Discharge Timer Monitor
bool PSBITS[2] = {false, false}; //!< Digital Redundancy Path Selection//ps-0,1

/*!**********************************************************************
 \brief  Initializes hardware and variables
  @return void
 ***********************************************************************/
Timer tcb;

/*Customize Global Variable*********************************************/
bool start_discharge = false;
bool start_balance = false;
bool discharge_stat[18] = {false};
int num_of_finish = 0;
double vmin[TOTAL_IC];
double vmax[TOTAL_IC];

double consvmax[TOTAL_IC] = {0};
double consvmin[TOTAL_IC];

/**************************************************************************/

void setup()
{
  Serial.begin(1000000);
  byte ssPin = 4; // or any other pin, stay away from 0,1, leave those for Serial()
  byte defaultSS = 4;
  pinMode(ssPin, OUTPUT);
  digitalWrite(ssPin, HIGH);  // typical slave select OFF state
  pinMode(defaultSS, OUTPUT); // 10 MUST be an output for device to be SPI master (328P)
  SPI.begin();

  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
  LTC6813_init_cfg(TOTAL_IC, BMS_IC);
  LTC6813_init_cfgb(TOTAL_IC, BMS_IC);
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    LTC6813_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
    LTC6813_set_cfgrb(current_ic, BMS_IC, FDRF, DTMEN, PSBITS, GPIOBITS_B, DCCBITS_B);
  }
  LTC6813_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC, BMS_IC);
  print_menu();
  for (int i = 0; i < 18; i++)
  {
    vmin[i] = 5;
    consvmax[i] = 5;
  }
  tcb.every(2000, starttowork);
}

/**********************************Start of Customize Function*******************************/
void moniV() // Every loop will Check out the Dischaerging cell is lower than the lowest cell or not
{
  uint32_t conv_time = 0;
  int8_t error = 0;
  wakeup_sleep(TOTAL_IC);
  LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6813_pollAdc();
  error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
  check_error(error);                                  //Check error to enable the function

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    //Serial.println(consvmin[current_ic]);
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
    {
      if (BMS_IC[current_ic].cells.c_codes[i] * 0.0001 < consvmin[current_ic])
      {
        //Serial.println(consvmin[current_ic]);
        wakeup_sleep(TOTAL_IC);
        LTC6813_clear_custom2_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
        LTC6813_wrcfg(TOTAL_IC, BMS_IC);
        LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
        discharge_stat[i] = false;
      }
    }
  }
}

void battery_charge_balance()
{
  uint32_t conv_time = 0;
  int8_t error = 0;
  double charge_buffer = 0;
  int light;
  wakeup_sleep(TOTAL_IC);
  LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  conv_time = LTC6813_pollAdc();
  error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
  check_error(error);                                  //Check error to enable the function

  run_command('AOV');
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    light = 0;
    charge_buffer = 0;
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
    {
      if (light > 2)
      {
        charge_buffer = 0.05;
      }
      //Serial.println(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30/18+0.05+charge_buffer,4);
      if (BMS_IC[current_ic].cells.c_codes[i] * 0.0001 > BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30 / 18 + 0.05 + charge_buffer)
      {
        light = light + 1;
        wakeup_sleep(TOTAL_IC);
        LTC6813_set_custom_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
        LTC6813_wrcfg(TOTAL_IC, BMS_IC);
        LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
      }
      else
      {
        wakeup_sleep(TOTAL_IC);
        LTC6813_clear_custom2_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
        LTC6813_wrcfg(TOTAL_IC, BMS_IC);
        LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
      }
    }
  }
}
void starttowork()
{
  wakeup_sleep(TOTAL_IC);
  LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  Serial.print(F("AOV :"));
  run_command('AOV');
  print_avgofvoltage();
  for (int j = 0; j < TOTAL_IC; j++)
  {
    vmin[j] = 5;
    vmax[j] = 0;
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
    {
      vmin[j] > BMS_IC[j].cells.c_codes[i] * 0.0001 ? vmin[j] = BMS_IC[j].cells.c_codes[i] * 0.0001 : 1;
      vmax[j] < BMS_IC[j].cells.c_codes[i] * 0.0001 ? vmax[j] = BMS_IC[j].cells.c_codes[i] * 0.0001 : 1;
    }
    Serial.print(F("min : "));
    Serial.print(vmin[j], 4);
    Serial.print(F("  max : "));
    Serial.println(vmax[j], 4);
  }

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("MAX difference Voltage_"));
    Serial.print(F("IC "));
    Serial.print(current_ic + 1);
    Serial.print(F(" : "));
    Serial.println(vmax[current_ic] - vmin[current_ic], 4);
    Serial.print(F("Highest Voltage:"));
    Serial.println(vmax[current_ic], 4);
    Serial.print(F("Lowest Voltage:"));
    Serial.println(vmin[current_ic], 4);
  }
  print_cells(DATALOG_DISABLED);
  if (start_balance)
  {
    battery_charge_balance();
  }
  if (start_discharge)
  {
    moniV();
    Serial.println(F("Discharging!!!!!!!"));
  }
}
int Maximum()
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (vmax[current_ic] > 3.73)
    {
      for (int i = 0; i < 5; i++)
      {
        Serial.println(F("Over Voltage > 3.73 !"));
      }
      return 0;
    }
  }
}
int Minimum()
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)

  {
    if (vmin[current_ic] < 2.5)
    {
      for (int i = 0; i < 5; i++)
      {
        Serial.println(F("Under Voltage < 2.5 !"));
      }
      return 0;
    }
  }
}
/*void sorting(cell_asic *BMS_IC)
{
  double temp = 0;
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    for (int i = 0; i < 18; i++)
    {
      BMS_compare[current_ic].cells.c_codes[i] = BMS_IC[current_ic].cells.c_codes[i];
    }

    for (int i = 0; i < 18; i++)
    {

      for (int j = 0; j < i; j++)
      {
        if (BMS_compare[current_ic].cells.c_codes[j + 1] > BMS_compare[current_ic].cells.c_codes[j])
        {
          temp = BMS_compare[current_ic].cells.c_codes[j + 1];
          BMS_compare[current_ic].cells.c_codes[j + 1] = BMS_compare[current_ic].cells.c_codes[j];
          BMS_compare[current_ic].cells.c_codes[j] = temp;
        }
      }
    }
  }
}*/
/***************************************End of Customize Function****************************/

/*!*********************************************************************
  \brief Main loop
   @return void
***********************************************************************/
void loop()
{

  wakeup_sleep(TOTAL_IC);
  LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  run_command(3);
  run_command(4);
  //sorting(BMS_IC);

  if (Serial.available()) // Check for user input
  {
    uint32_t user_command;
    user_command = read_int(); // Read the user command
    if (user_command == 'm')
    {
      print_menu();
    }
    else
    {
      Serial.println(user_command);
      run_command(user_command);
    }
  }
  //Maximum();
  Minimum();
  tcb.update();
  //delay(1000);
}

/*!*****************************************
  \brief executes the user command
    @return void
*******************************************/
void run_command(uint32_t cmd)
{

  uint8_t streg = 0;
  int8_t error = 0;
  uint32_t conv_time = 0;
  int8_t s_pin_read = 0;
  int8_t s_ic = 0;

  switch (cmd)
  {
  case 1: // Write and read Configuration Register
    wakeup_sleep(TOTAL_IC);
    LTC6813_wrcfg(TOTAL_IC, BMS_IC);  // Write into Configuration Register
    LTC6813_wrcfgb(TOTAL_IC, BMS_IC); // Write into Configuration Register B
    print_wrconfig();
    print_wrconfigb();

    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC, BMS_IC); // Read Configuration Register
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC, BMS_IC); // Read Configuration Register B
    check_error(error);
    print_rxconfig();
    print_rxconfigb();
    break;

  case 2: // Read Configuration Register
    wakeup_sleep(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC, BMS_IC);
    check_error(error);
    print_rxconfig();
    print_rxconfigb();
    break;

  case 3: // Start Cell ADC Measurement
    wakeup_sleep(TOTAL_IC);
    LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    conv_time = LTC6813_pollAdc();
    //print_conv_time(conv_time);
    break;

  case 4: // Read Cell Voltage Registers
    wakeup_sleep(TOTAL_IC);
    error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    //print_cells(DATALOG_DISABLED);
    break;

  case 23: // Enable a discharge transistor
    s_pin_read = select_s_pin();
    wakeup_sleep(TOTAL_IC);
    LTC6813_set_discharge(s_pin_read, TOTAL_IC, BMS_IC);
    LTC6813_wrcfg(TOTAL_IC, BMS_IC);
    LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
    print_wrconfig();
    print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC, BMS_IC);
    check_error(error);
    print_rxconfig();
    print_rxconfigb();
    break;

  case 'm': //prints menu
    print_menu();
    break;

  case 'AOV': //Start Combined Cell Voltage and Sum of cells
    wakeup_sleep(TOTAL_IC);
    LTC6813_adcvsc(ADC_CONVERSION_MODE, ADC_DCP);
    conv_time = LTC6813_pollAdc();
    //print_conv_time(conv_time);
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    //print_cells(DATALOG_DISABLED);
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdstat(SEL_REG_A, TOTAL_IC, BMS_IC); // Set to read back stat register A
    check_error(error);
    //print_avgofvoltage();
    break;

  case 86: //start discharge to the lowest voltage of all cells and ICs
    wakeup_sleep(TOTAL_IC);
    LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    conv_time = LTC6813_pollAdc();
    error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);                                  //Check error to enable the function
    for (int i = 0; i < TOTAL_IC; i++)
    {
      consvmin[i] = vmin[i];
    }
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        if (BMS_IC[current_ic].cells.c_codes[i] * 0.0001 > consvmin[current_ic])
        {
          Serial.println(consvmin[current_ic],4);
          wakeup_sleep(TOTAL_IC);
          LTC6813_set_custom_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
          LTC6813_wrcfg(TOTAL_IC, BMS_IC);
          LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
          discharge_stat[i] = true;
        }
      }
    }
    start_discharge = true;
    Serial.println(F("start discharge"));
    break;

  case 87: //charge balance
    Serial.println(F("Ha Ha !!"));
    wakeup_sleep(TOTAL_IC);
    LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    conv_time = LTC6813_pollAdc();
    error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);

    start_balance = true;
    Serial.println(F("start balance !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
    break;

  case 88:
    Serial.println(F("Stop !!"));
    wakeup_sleep(TOTAL_IC);
    LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    conv_time = LTC6813_pollAdc();
    error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);

    start_discharge = false;
    start_balance = false;
    Serial.println(F("Stop balance !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));

    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        wakeup_sleep(TOTAL_IC);
        LTC6813_clear_custom2_discharge(i + 1, current_ic, TOTAL_IC, BMS_IC);
        LTC6813_wrcfg(TOTAL_IC, BMS_IC);
        LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
      }
    }
    break;

  case 89: //clear discharge of seleted cell of all ICs
    s_pin_read = select_s_pin();
    wakeup_sleep(TOTAL_IC);
    LTC6813_clear_custom_discharge(s_pin_read, TOTAL_IC, BMS_IC);
    LTC6813_wrcfg(TOTAL_IC, BMS_IC);
    LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
    //print_wrconfig();
    //print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC, BMS_IC);
    check_error(error);
    //print_rxconfig();
    //print_rxconfigb();
    break;

  case 90: //clear discharge of seleted cell of selected ICs
    Serial.println("Clear Discharge !!");
    s_ic = select_ic() - 1;
    s_pin_read = select_s_pin();
    wakeup_sleep(TOTAL_IC);
    LTC6813_clear_custom2_discharge(s_pin_read, s_ic, TOTAL_IC, BMS_IC);
    LTC6813_wrcfg(TOTAL_IC, BMS_IC);
    LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
    //print_wrconfig();
    //print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC, BMS_IC);
    check_error(error);
    //print_rxconfig();
    //print_rxconfigb();
    break;

  case 91: //set discharge of seleted cell of selected ICs
    Serial.println(F("Set Discharge !!"));
    s_ic = select_ic() - 1;
    s_pin_read = select_s_pin();
    wakeup_sleep(TOTAL_IC);
    LTC6813_set_custom_discharge(s_pin_read, s_ic, TOTAL_IC, BMS_IC);
    LTC6813_wrcfg(TOTAL_IC, BMS_IC);
    LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
    //print_wrconfig();
    //print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC, BMS_IC);
    check_error(error);
    //print_rxconfig();
    //print_rxconfigb();
    break;

  case 92: //calculate minimal and maxium
    wakeup_sleep(TOTAL_IC);
    LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    conv_time = LTC6813_pollAdc();
    print_conv_time(conv_time);

    error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    print_cells(DATALOG_DISABLED);

    for (int j = 0; j < TOTAL_IC; j++)
    {
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        vmin[j] > BMS_IC[j].cells.c_codes[i] * 0.0001 ? vmin[j] = BMS_IC[j].cells.c_codes[i] * 0.0001 : 1;
        vmax[j] < BMS_IC[j].cells.c_codes[i] * 0.0001 ? vmax[j] = BMS_IC[j].cells.c_codes[i] * 0.0001 : 1;
      }
      Serial.print(F("min : "));
      Serial.print(vmin[j], 4);
      Serial.print(F("  max : "));
      Serial.println(vmax[j], 4);
    }

    break;

  default:
    char str_error[] = "Incorrect Option \n";
    serial_print_text(str_error);
    break;
  }
}

/*!**********************************************************************************************************************************************
 \brief For writing/reading configuration data or measuring cell voltages or reading aux register or reading status register in a continuous loop
 @return void
*************************************************************************************************************************************************/
void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  char input = 0;

  Serial.println(F("Transmit 'm' to quit"));

  while (input != 'm')
  {
    if (Serial.available() > 0)
    {
      input = read_char();
    }

    if (WRITE_CONFIG == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6813_wrcfg(TOTAL_IC, BMS_IC);
      LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
      print_wrconfig();
      print_wrconfigb();
    }

    if (READ_CONFIG == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      error = LTC6813_rdcfg(TOTAL_IC, BMS_IC);
      check_error(error);
      error = LTC6813_rdcfgb(TOTAL_IC, BMS_IC);
      check_error(error);
      print_rxconfig();
      print_rxconfigb();
    }

    if (MEASURE_CELL == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      LTC6813_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6813_rdcv(0, TOTAL_IC, BMS_IC);
      check_error(error);
      print_cells(datalog_en);
    }

    if (MEASURE_AUX == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6813_adax(ADC_CONVERSION_MODE, AUX_CH_ALL);
      LTC6813_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6813_rdaux(0, TOTAL_IC, BMS_IC); // Set to read back all aux registers
      check_error(error);
      print_aux(datalog_en);
    }

    if (MEASURE_STAT == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6813_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
      LTC6813_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6813_rdstat(0, TOTAL_IC, BMS_IC); // Set to read back all aux registers
      check_error(error);
      print_stat();
    }

    if (PRINT_PEC == ENABLED)
    {
      print_pec_error_count();
    }

    delay(MEASUREMENT_LOOP_TIME);
  }
}

/*!*********************************
  \brief Prints the main menu
  @return void
***********************************/
void print_menu(void)
{
  Serial.println(F("List of LTC6813 Command:"));
  Serial.println(F("Write and Read Configuration: 1                            |Loop measurements with data-log output : 12                            |Set Discharge: 23   "));
  Serial.println(F("Read Configuration: 2                                      |Clear Registers: 13                                                    |Clear Discharge: 24   "));
  Serial.println(F("Start Cell Voltage Conversion: 3                           |Run Mux Self Test: 14                                                  |Write and Read of PWM : 25"));
  Serial.println(F("Read Cell Voltages: 4                                      |Run ADC Self Test: 15                                                  |Write and  Read of S control : 26"));
  Serial.println(F("Start Aux Voltage Conversion: 5                            |ADC overlap Test : 16                                                  |Clear S control register : 27"));
  Serial.println(F("Read Aux Voltages: 6                                       |Run Digital Redundancy Test: 17                                        |SPI Communication  : 28"));
  Serial.println(F("Start Stat Voltage Conversion: 7                           |Open Wire Test for single cell detection: 18                           |I2C Communication Write to Slave :29"));
  Serial.println(F("Read Stat Voltages: 8                                      |Open Wire Test for multiple cell or two consecutive cells detection:19 |I2C Communication Read from Slave :30"));
  Serial.println(F("Start Combined Cell Voltage and GPIO1, GPIO2 Conversion: 9 |Open wire for Auxiliary Measurement: 20                                |Enable MUTE : 31"));
  Serial.println(F("Start  Cell Voltage and Sum of cells : 10                  |Print PEC Counter: 21                                                  |Disable MUTE : 32"));
  Serial.println(F("Loop Measurements: 11                                      |Reset PEC Counter: 22                                                  |Set or reset the gpio pins: 33 \n "));
  Serial.println(F("Print 'm' for menu"));
  Serial.println(F("Please enter command: \n "));
}

/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6813
 to the serial port.
 @return void
 ********************************************************************************/
void print_wrconfig(void)
{
  int cfg_pec;
  Serial.println(F("Written Configuration A Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6, &BMS_IC[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println("\n");
  }
}

/*!******************************************************************************
 \brief Prints the Configuration Register B data that is going to be written to
 the LTC6813 to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfigb(void)
{
  int cfg_pec;
  Serial.println(F("Written Configuration B Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGB IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].configb.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6, &BMS_IC[current_ic].configb.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println("\n");
  }
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6813 to the serial port.
 @return void
 *******************************************************************/
void print_rxconfig(void)
{
  Serial.println(F("Received Configuration A Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[7]);
    Serial.println("\n");
  }
}

/*!*****************************************************************
 \brief Prints the Configuration Register B that was read back from
 the LTC6813 to the serial port.
  @return void
 *******************************************************************/
void print_rxconfigb(void)
{
  Serial.println(F("Received Configuration B Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGB IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].configb.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].configb.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].configb.rx_data[7]);
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints cell voltage codes to the serial port
  @return void
 *************************************************************/
void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(", ");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        Serial.print(" C");
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.println();
    }
    else
    {

      Serial.print(F(" Cells, "));
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(F(","));
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      for (int i = 0; i < 5; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }

      for (int i = 6; i < 10; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
      }

      Serial.print(F(" Vref2"));
      Serial.print(":");
      Serial.print(BMS_IC[current_ic].aux.a_codes[5] * 0.0001, 4);
      Serial.println();

      Serial.print(" OV/UV Flags : 0x");
      Serial.print((uint8_t)BMS_IC[current_ic].aux.a_codes[11], HEX);
      Serial.println();
    }
    else
    {
      Serial.print(" AUX, ");

      for (int i = 0; i < 12; i++)
      {
        Serial.print((uint8_t)BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_stat(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    double itmp;

    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30, 4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    itmp = (double)((BMS_IC[current_ic].stat.stat_codes[1] * (0.0001 / 0.0076)) - 276); //Internal Die Temperature(°C) = ITMP • (100 µV / 7.6mV)°C - 276°C
    Serial.print(itmp, 4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[2] * 0.0001, 4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[3] * 0.0001, 4);
    Serial.println();
    Serial.print(F(" OV/UV Flags:"));
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[2]);
    Serial.print(F("\tMux fail flag:"));
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.mux_fail[0]);
    Serial.print(F("\tTHSD:"));
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.thsd[0]);
    Serial.println();
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes (GPIO 1 & 2)
  @return void
 *****************************************************************************/
void print_aux1(uint8_t datalog_en)
{

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      for (int i = 0; i < 2; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i + 1, DEC);
        Serial.print(F(":"));
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(F(","));
      }
    }
    else
    {
      Serial.print(F("AUX, "));

      for (int i = 0; i < 12; i++)
      {
        Serial.print(BMS_IC[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(F(","));
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes for SOC onto the serial port
 *****************************************************************************/
void print_sumofcells(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30, 4);
    Serial.print(F(","));
  }
  Serial.println("\n");
}

void print_avgofvoltage(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" AOV:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30 / 18, 4);
    Serial.print(F(","));
  }
  Serial.println("\n");
}

/*!****************************************************************
  \brief Function to check the MUX fail bit in the Status Register
   @return void
*******************************************************************/
void check_mux_fail(void)
{
  int8_t error = 0;
  for (int ic = 0; ic < TOTAL_IC; ic++)
  {
    Serial.print(" IC ");
    Serial.println(ic + 1, DEC);
    if (BMS_IC[ic].stat.mux_fail[0] != 0)
      error++;

    if (error == 0)
      Serial.println(F("Mux Test: PASS \n"));
    else
      Serial.println(F("Mux Test: FAIL \n"));
  }
}

/*!************************************************************
  \brief Prints Errors Detected during self test
   @return void
*************************************************************/
void print_selftest_errors(uint8_t adc_reg, int8_t error)
{
  if (adc_reg == 1)
  {
    Serial.println("Cell ");
  }
  else if (adc_reg == 2)
  {
    Serial.println("Aux ");
  }
  else if (adc_reg == 3)
  {
    Serial.println("Stat ");
  }
  Serial.print(error, DEC);
  Serial.println(F(" : errors detected in Digital Filter and Memory \n"));
}

/*!************************************************************
  \brief Prints the output of  the ADC overlap test
   @return void
*************************************************************/
void print_overlap_results(int8_t error)
{
  if (error == 0)
    Serial.println(F("Overlap Test: PASS \n"));
  else
    Serial.println(F("Overlap Test: FAIL \n"));
}

/*!************************************************************
  \brief Prints Errors Detected during Digital Redundancy test
   @return void
*************************************************************/
void print_digital_redundancy_errors(uint8_t adc_reg, int8_t error)
{
  if (adc_reg == 2)
  {
    Serial.println("Aux ");
  }
  else if (adc_reg == 3)
  {
    Serial.println("Stat ");
  }

  Serial.print(error, DEC);
  Serial.println(F(" : errors detected in Measurement \n"));
}

/*!****************************************************************************
  \brief Prints Open wire test results to the serial port
 *****************************************************************************/
void print_open_wires(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (BMS_IC[current_ic].system_open_wire == 65535)
    {
      Serial.print("No Opens Detected on IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.println();
    }
    else
    {
      Serial.print(F("There is an open wire on IC "));
      Serial.print(current_ic + 1, DEC);
      Serial.print(F(" Channel: "));
      Serial.println(BMS_IC[current_ic].system_open_wire);
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
   \brief Function to print the number of PEC Errors
   @return void
 *****************************************************************************/
void print_pec_error_count(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.println("");
    Serial.print(BMS_IC[current_ic].crc_count.pec_count, DEC);
    Serial.print(F(" : PEC Errors Detected on IC"));
    Serial.println(current_ic + 1, DEC);
  }
  Serial.println("\n");
}

/*!****************************************************
  \brief Function to select the S pin for discharge
  @return void
 ******************************************************/
int8_t select_s_pin(void)
{
  int8_t read_s_pin = 0;

  Serial.print(F("Please enter the Spin number: "));
  read_s_pin = (int8_t)read_int();
  Serial.println(read_s_pin);
  return (read_s_pin);
}

/*!****************************************************
  \brief Function to select the IC for discharge
  @return void
 ******************************************************/
int8_t select_ic(void)
{
  int8_t read_s_ic = 0;

  Serial.print(F("Please enter the ic number: "));
  read_s_ic = (int8_t)read_int();
  Serial.println(read_s_ic);
  return (read_s_ic);
}

/*!****************************************************************************
  \brief prints data which is written on PWM register onto the serial port
  @return void
 *****************************************************************************/
void print_wrpwm(void)
{
  int pwm_pec;

  Serial.println(F("Written PWM Configuration: "));
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwm.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    pwm_pec = pec15_calc(6, &BMS_IC[current_ic].pwm.tx_data[0]);
    serial_print_hex((uint8_t)(pwm_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(pwm_pec));
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints received data from PWM register onto the serial port
  @return void
 *****************************************************************************/
void print_rxpwm(void)
{
  Serial.println(F("Received pwm Configuration:"));
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F("IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwm.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].pwm.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].pwm.rx_data[7]);
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief prints data which is written on S Control register
  @return void
 *****************************************************************************/
void print_wrsctrl(void)
{
  int sctrl_pec;

  Serial.println(F("Written Data in Sctrl register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC: "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" Sctrl register group:"));
    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrl.tx_data[i]);
    }

    Serial.print(F(", Calculated PEC: 0x"));
    sctrl_pec = pec15_calc(6, &BMS_IC[current_ic].sctrl.tx_data[0]);
    serial_print_hex((uint8_t)(sctrl_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(sctrl_pec));
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief prints data which is read back from S Control register
  @return void
 *****************************************************************************/
void print_rxsctrl(void)
{
  Serial.println(F("Received Data:"));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[i]);
    }

    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[7]);
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints data which is written on PWM/S control register group B onto
  the serial port
   @return void
 *****************************************************************************/
void print_wrpsb(uint8_t type)
{
  int psb_pec = 0;

  Serial.println(F(" PWM/S control register group B: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (type == 1)
    {
      Serial.print(F(" IC: "));
      Serial.println(current_ic + 1, DEC);
      Serial.print(F(" 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.tx_data[0]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.tx_data[1]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.tx_data[2]);

      Serial.print(F(", Calculated PEC: 0x"));
      psb_pec = pec15_calc(6, &BMS_IC[current_ic].pwmb.tx_data[0]);
      serial_print_hex((uint8_t)(psb_pec >> 8));
      Serial.print(F(", 0x"));
      serial_print_hex((uint8_t)(psb_pec));
      Serial.println("\n");
    }
    else if (type == 2)
    {
      Serial.print(F(" IC: "));
      Serial.println(current_ic + 1, DEC);
      Serial.print(F(" 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.tx_data[3]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.tx_data[4]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.tx_data[5]);

      Serial.print(F(", Calculated PEC: 0x"));
      psb_pec = pec15_calc(6, &BMS_IC[current_ic].sctrlb.tx_data[0]);
      serial_print_hex((uint8_t)(psb_pec >> 8));
      Serial.print(F(", 0x"));
      serial_print_hex((uint8_t)(psb_pec));
      Serial.println("\n");
    }
  }
}

/*!****************************************************************************
  \brief Prints received data from PWM/S control register group B
   onto the serial port
   @return void
 *****************************************************************************/
void print_rxpsb(uint8_t type)
{
  Serial.println(F(" PWM/S control register group B:"));
  if (type == 1)
  {
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      Serial.print(F(" IC: "));
      Serial.println(current_ic + 1, DEC);
      Serial.print(F(" 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[0]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[1]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[2]);

      Serial.print(F(", Received PEC: 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[6]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[7]);
      Serial.println("\n");
    }
  }
  else if (type == 2)
  {
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      Serial.print(F(" IC: "));
      Serial.println(current_ic + 1, DEC);
      Serial.print(F(" 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[3]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[4]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[5]);

      Serial.print(F(", Received PEC: 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[6]);
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[7]);
      Serial.println("\n");
    }
  }
}

/*!****************************************************************************
  \brief prints data which is written on COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_wrcomm(void)
{
  int comm_pec;

  Serial.println(F("Written Data in COMM Register: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    comm_pec = pec15_calc(6, &BMS_IC[current_ic].com.tx_data[0]);
    serial_print_hex((uint8_t)(comm_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(comm_pec));
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints received data from COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_rxcomm(void)
{
  Serial.println(F("Received Data in COMM register:"));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC- "));
    Serial.print(current_ic + 1, DEC);

    for (int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].com.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].com.rx_data[7]);
    Serial.println("\n");
  }
}

/*!********************************************************************
  \brief Function to check the Mute bit in the Configuration Register
   @return void
**********************************************************************/
void check_mute_bit(void)
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" Mute bit in Configuration Register B: 0x"));
    serial_print_hex((BMS_IC[current_ic].configb.rx_data[1]) & (0x80));
    Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Function to print the Conversion Time
  @return void
 *****************************************************************************/
void print_conv_time(uint32_t conv_time)
{
  uint16_t m_factor = 1000; // to print in ms

  Serial.print(F("Conversion completed in:"));
  Serial.print(((float)conv_time / m_factor), 1);
  Serial.println(F("ms \n"));
}

/*!****************************************************************************
  \brief Function to check error flag and print PEC error message
  @return void
 *****************************************************************************/
void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

/*!************************************************************
  \brief Function to print text on serial monitor
  @return void
*************************************************************/
void serial_print_text(char data[])
{
  Serial.println(data);
}

/*!****************************************************************************
   \brief Function to print in HEX form
   @return void
 *****************************************************************************/
void serial_print_hex(uint8_t data)
{
  if (data < 16)
  {
    Serial.print("0");
    Serial.print((byte)data, HEX);
  }
  else
    Serial.print((byte)data, HEX);
}

/*!*****************************************************************************
 \brief Hex conversion constants
 *******************************************************************************/
char hex_digits[16] =
    {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

/*!************************************************************
 \brief Global Variables
 *************************************************************/
char hex_to_byte_buffer[5] =
    {
        '0', 'x', '0', '0', '\0'};

/*!************************************************************
 \brief Buffer for ASCII hex to byte conversion
 *************************************************************/
char byte_to_hex_buffer[3] =
    {
        '\0', '\0', '\0'};

/*!*****************************************************************************
 \brief Read 2 hex characters from the serial buffer and convert them to a byte
 @return char data Read Data
 ******************************************************************************/
char read_hex(void)
{
  byte data;
  hex_to_byte_buffer[2] = get_char();
  hex_to_byte_buffer[3] = get_char();
  get_char();
  get_char();
  data = strtol(hex_to_byte_buffer, NULL, 0);
  return (data);
}

/*!************************************************************
 \brief Read a command from the serial port
 @return char
 *************************************************************/
char get_char(void)
{
  // read a command from the serial port
  while (Serial.available() <= 0)
    ;
  return (Serial.read());
}
