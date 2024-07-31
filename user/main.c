/*
 * main.c
 *
 *  Created on: Mar 21, 2024
 *      Author: nov4ou
 */
#include "F2806x_Device.h"   // F2806x Headerfile
#include "F2806x_Examples.h" // F2806x Examples Headerfile
#include "SPLL_1ph_SOGI_F.h"
#include "Solar_F.h"
#include "adc.h"
#include "epwm.h"
#include "key.h"
#include "math.h"
#include "oled.h"
#include "spi.h"

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;

#pragma CODE_SECTION(cpu_timer1_isr, "ramfuncs");
#pragma CODE_SECTION(cpu_timer2_isr, "ramfuncs");
// // #pragma CODE_SECTION(OLED_Update, "ramfuncs");

#define ISR_FREQUENCY 20000
#define Kp 50.7089
// #define Ki 1063.14
#define Ki 5000.0
#define GRID_FREQ 50
#define CURRENT_RMS 1.5

_Bool flag = 0;
_Bool prev_flag = 0;
_Bool flag_voltage = 0;
_Bool prev_flag_voltage = 0;
float Kp_set = 1;

extern float grid_voltage;
extern float grid_inverter_current;
extern float grid_inverter_voltage;
extern float grid_current;
float V_rms = 0;
float V_rms_softstart = 1;
float V_rms_ref = 4;
float V_rms_in = 6;
float output = 0;
float dutycycle = 1000;
float normalized_voltage1;
float normalized_voltage2;
float ref_current;
float curr_error;
float curr_loop_out;
float V_mod;
float ratio = 1;
float compare_soft = 0; // For soft shutdown

Uint32 compare = 0;
Uint32 counter = 0;
Uint8 str[10];

typedef struct {
  float kp, ki, kd;
  float error, lastError;
  float integral, maxIntegral;
  float output, maxOutput;
} PID;
PID VoltageLoop, CurrentLoop;

SINEANALYZER_DIFF_F sineanalyzer_diff1;
SPLL_1ph_SOGI_F spll1;

void LED_Init(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);
void OLED_Update();
void ftoa(float f, int precision);
void itoa(int num, Uint8 *str);

int main() {
  memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
  InitFlash();

  InitSysCtrl();
  DINT;
  InitPieCtrl();
  IER = 0x0000;
  IFR = 0x0000;
  InitPieVectTable();
  EALLOW;
  PieVectTable.ADCINT1 = &adc_isr;
  PieVectTable.TINT2 = &cpu_timer2_isr;
  PieVectTable.TINT1 = &cpu_timer1_isr;
  EDIS;
  InitAdc(); // For this example, init the ADC
  AdcOffsetSelfCal();
  // ADC
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
  IER |= M_INT1;                     // Enable CPU Interrupt 1
  EINT;
  ERTM;
  PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
  IER |= M_INT8;
  EINT;

  ADC_Init();
  SINEANALYZER_DIFF_F_init(&sineanalyzer_diff1);
  sineanalyzer_diff1.SampleFreq = ISR_FREQUENCY;
  sineanalyzer_diff1.nsamplesMin = 363 / 2;
  sineanalyzer_diff1.nsamplesMax = 444 / 2;
  SPLL_1ph_SOGI_F_init(GRID_FREQ, ((float)(1.0 / ISR_FREQUENCY)), &spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);
  spll1.lpf_coeff.B0_lf = (float32)((2 * Kp + Ki / ISR_FREQUENCY) / 2);
  spll1.lpf_coeff.B1_lf = (float32)(-(2 * Kp - Ki / ISR_FREQUENCY) / 2);
  PID_Init(&VoltageLoop, 0.04, 0.02, 0, 50, 50);
  // PID_Init(&CurrentLoop, 1, 0.001, 0, 50, 50);
  PID_Init(&CurrentLoop, 1, 0.000, 0, 50, 50);

  LED_Init();
  EPWM2_Init(MAX_CMPA);
  EPWM3_Init(MAX_CMPA);
  EPWM5_Init(MAX_CMPA);
  EPWM6_Init(MAX_CMPA);
  // EPWM7_Init(MAX_CMPA);
  // EPWM8_Init(MAX_CMPA);
  EPwm1Regs.TBCTL.bit.SWFSYNC = 1;
  EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  // Set actions for rectifier
  EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
  EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
  // Set actions for rectifier
  EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
  EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

  KEY_Init();
  SPIB_Init();
  OLED_Init();
  OLED_Clear();
  InitCpuTimers();
  ConfigCpuTimer(&CpuTimer1, 90, 50);   // 20K ISR for sin analyze
  ConfigCpuTimer(&CpuTimer2, 90, 1000); // 10K ISR for PI calc
  CpuTimer1Regs.TCR.all = 0x4000;
  CpuTimer2Regs.TCR.all = 0x4000;
  IER |= M_INT13;
  IER |= M_INT14;
  EINT; // Enable Global interrupt INTM
  ERTM; // Enable Global realtime interrupt DBGM
  PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

  while (1) {
    // GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
    OLED_Update();
    // Test
    // EPwm5Regs.CMPA.half.CMPA = compare;
    // EPwm6Regs.CMPA.half.CMPA = compare;

    if (KEY_Read() != 0) {
      if (KEY_Read() == 1) {
        flag = 1 - flag;
        while (KEY_Read() == 1)
          ;
      }
      if (KEY_Read() == 2) {
        flag_voltage = 1 - flag_voltage;
        while (KEY_Read() == 2)
          ;
      }
      if (KEY_Read() == 4) {
        V_rms_ref += 0.5;
        if (V_rms_ref > 35)
          V_rms_ref = 35;
        while (KEY_Read() == 4)
          ;
      }
      if (KEY_Read() == 5) {
        V_rms_ref -= 0.5;
        if (V_rms_ref < 1)
          V_rms_ref = 1;
        while (KEY_Read() == 5)
          ;
      }
    }
    if (prev_flag != flag) {
      if (flag == 0) {
        EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
        EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
        // Set actions for rectifier
        EPwm5Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm5Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm5Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        // Set actions for rectifier
        EPwm6Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm6Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm6Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
      } else if (flag == 1) {
        // Active Low PWMs - Setup Deadband
        EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
        EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
        EPwm5Regs.DBRED = 1;
        EPwm5Regs.DBFED = 1;
        EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
        EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
        EPwm6Regs.DBRED = 1;
        EPwm6Regs.DBFED = 1;
        // Set actions for inverter
        EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
        EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
        // Set actions for inverter
        EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
        EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

        // EPwm5Regs.CMPA.half.CMPA = compare;
        // EPwm6Regs.CMPA.half.CMPA = compare;
      }
      prev_flag = flag;
    }
  }
}

void LED_Init(void) {
  EALLOW;
  //    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;

  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;

  GpioDataRegs.GPASET.bit.GPIO0 = 1;

  EDIS;
}

__interrupt void cpu_timer1_isr(void) {
  normalized_voltage1 = grid_voltage / (V_rms_ref * 1.414);
  sineanalyzer_diff1.Vin = normalized_voltage1;
  SINEANALYZER_DIFF_F_FUNC(&sineanalyzer_diff1);
  if (abs(sineanalyzer_diff1.SigFreq - 50) < 5)
    V_rms = sineanalyzer_diff1.Vrms * (V_rms_ref * 1.414);

  normalized_voltage2 = grid_inverter_voltage / 15;
  spll1.u[0] = normalized_voltage2;
  SPLL_1ph_SOGI_F_FUNC(&spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);

  V_mod = sin(spll1.theta[0]);
  EPwm2Regs.CMPA.half.CMPA = (Uint16)(fabs(V_mod) * MAX_CMPA);
}

__interrupt void cpu_timer2_isr(void) {
  GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;

  // EPwm5Regs.CMPA.half.CMPA = compare;
  // EPwm6Regs.CMPA.half.CMPA = compare;

  if (flag_voltage == 0) {
    V_rms_softstart = 0;
    compare_soft -= 0.66;
    if (compare_soft < 0)
      compare_soft = 0;
    EPwm5Regs.CMPA.half.CMPA = (Uint16)compare_soft;
  }

  // if (flag_voltage != prev_flag_voltage) {
  if (flag_voltage == 1) {
    // Soft Start
    V_rms_softstart += 0.001;
    if (V_rms_softstart >= V_rms_ref)
      V_rms_softstart = V_rms_ref;
    
    /********************* Voltage Loop ************************/
    PID_Calc(&VoltageLoop, V_rms_softstart, V_rms);
    output = VoltageLoop.output;
    if (output > 2 * 1.414)
      output = 2 * 1.414;
    if (output < -2 * 1.414)
      output = -2 * 1.414;

    PID_Calc(&CurrentLoop, (output + 2 * 1.414) * fabs(V_mod),
             fabs(grid_current));
    curr_loop_out = (CurrentLoop.output + V_rms_softstart) / V_rms_in;
    compare = (Uint32)(curr_loop_out * MAX_CMPA);
    if (compare >= 2200)
      compare = 2200;
    if (compare <= 50)
      compare = 50;
    compare_soft = compare;
    EPwm5Regs.CMPA.half.CMPA = compare;
    EPwm6Regs.CMPA.half.CMPA = compare;
    prev_flag_voltage = flag_voltage;
    /********************* Current Loop ************************/
    // ref_current = fabs(V_mod) * output * ratio;
    // curr_error = ref_current - fabs(grid_inverter_current);
    // // curr_loop_out = Kp_set * curr_error / 10;
    // curr_loop_out = Kp_set * curr_error / 40;
    // compare = (Uint32)(curr_loop_out * MAX_CMPA);
    // EPwm5Regs.CMPA.half.CMPA = compare;
    // EPwm6Regs.CMPA.half.CMPA = compare;
    /********************* Current Loop ************************/
  }

  // }
}

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut) {
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->maxIntegral = maxI;
  pid->maxOutput = maxOut;
  pid->error = 0;
  pid->lastError = 0;
  pid->integral = 0;
  pid->output = 0;
}

void PID_Calc(PID *pid, float reference, float feedback) {
  pid->lastError = pid->error;
  pid->error = reference - feedback;
  float dout = (pid->error - pid->lastError) * pid->kd;
  float pout = pid->error * pid->kp;
  pid->integral += pid->error * pid->ki;
  if (pid->integral > pid->maxIntegral)
    pid->integral = pid->maxIntegral;
  else if (pid->integral < -pid->maxIntegral)
    pid->integral = -pid->maxIntegral;
  pid->output = pout + dout + pid->integral;
  if (pid->output > pid->maxOutput)
    pid->output = pid->maxOutput;
  else if (pid->output < -pid->maxOutput)
    pid->output = -pid->maxOutput;
}

void OLED_Update() {
  counter++;
  if (counter == 5000) {
    OLED_ShowString(0, 0, "Master: ", 16);
    if (flag == 1) {
      OLED_ShowString(83, 0, " ON ", 16);
    } else {
      OLED_ShowString(83, 0, " OFF", 16);
    }

    OLED_ShowString(0, 2, "Dual Loop: ", 16);
    if (flag_voltage == 1) {
      OLED_ShowString(83, 2, " ON ", 16);
    } else {
      OLED_ShowString(83, 2, " OFF", 16);
    }

    // OLED_ShowString(0, 4, "K_RLC: ", 8);
    // if (K_RLC == 1) {
    //   OLED_ShowString(75, 4, "     1", 8);
    // } else {
    //   OLED_ShowString(75, 4, "    -1", 8);
    // }
    OLED_ShowString(0, 4, "V_rms:", 8);
    ftoa(V_rms_ref, 1);
    OLED_ShowString(75, 4, str, 4);

    OLED_ShowString(0, 6, "Compare: ", 8);
    // ftoa(compare, 0);
    itoa(compare, str);
    OLED_ShowString(70, 6, str, 4);
    // OLED_ShowNum(70, 6, compare, 5, 8);

    // OLED_ShowString(0, 2, "Eff:", 16);
    // if (Vol1Loop == 1 && Vol2Loop == 1) {
    //   ftoa(filteredEff * 100, 1);
    //   OLED_ShowString(45, 2, str, 16);
    // }

    // OLED_ShowString(0, 4, "Loop: ", 16);
    // if (Vol1Loop == 1 && Vol2Loop == 1)
    //   OLED_ShowString(45, 4, "ON ", 16);
    // else
    //   OLED_ShowString(45, 4, "OFF", 16);

    counter = 0;
  }
}

// This function converts a float to a string with a specified precision.
void ftoa(float f, int precision) {
  int j;

  // Split the float into integer and fractional parts
  int int_part = (int)f;
  float frac_part = f - int_part;

  int i = 0;
  // If the integer part is 0, add '0' to the string
  if (int_part == 0) {
    str[i++] = '0';
  } else {
    // Convert the integer part to string
    while (int_part) {
      str[i++] = int_part % 10 + '0';
      int_part /= 10;
    }
  }

  // Reverse the string to get the correct order
  for (j = 0; j < i / 2; j++) {
    char temp = str[j];
    str[j] = str[i - j - 1];
    str[i - j - 1] = temp;
  }

  // Add the decimal point
  str[i++] = '.';

  // Convert the fractional part to string with the specified precision
  for (j = 0; j < precision; j++) {
    frac_part *= 10;
    int digit = (int)frac_part;
    str[i++] = digit + '0';
    frac_part -= digit;
  }

  // Terminate the string
  str[i] = '\0';
}

void itoa(int num, Uint8 *str) {
  int i = 0;
  int isNegative = 0;

  // Handle negative numbers
  if (num < 0) {
    isNegative = 1;
    num = -num;
  }

  // Convert integer part to string
  if (num == 0) {
    str[i++] = '0';
  } else {
    while (num) {
      str[i++] = num % 10 + '0';
      num /= 10;
    }
  }

  // Add negative sign if the number is negative
  if (isNegative) {
    str[i++] = '-';
  }

  // Reverse the string to get the correct order
  str[i] = '\0'; // Null-terminate the string
  int j;
  for (j = 0; j < i / 2; j++) {
    char temp = str[j];
    str[j] = str[i - j - 1];
    str[i - j - 1] = temp;
  }
}
