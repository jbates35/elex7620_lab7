#include "msp.h"

//assumes system clock is at 48 MHz
//input dividers on Timer A = div 1, input to Timer A = 48 MHz
//Divide by 512 -> PWM frequency = 93.75 kHz

#define SMCLK   BIT0        //port 7.0
#define A8_IN   BIT5        //port 4.5 ADC input channel 8

#define PWM_PERIOD  512     //9-bit DAC (2^9 = 512)
#define ADCSCALE    5       //14-bit ADC; reduce by 5 bits to match DAC

#define SR_12kHz    4096
#define FSAMP       SR_12kHz //sampling frequency


void main(void)
{
    volatile unsigned int ADC_In=0;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    //P7->DIR |= SMCLK;
    //P7->SEL0 |= SMCLK;

    P6->DIR |= BIT0;        //sampling flag P6.0
    P6->OUT = 0;            //for measuring sampling frequency and ADC conversion time

    P5->DIR |= BIT6;        //PWM output P5.6
    P5->SEL0 |= BIT6;
    P5->SEL1 &= ~BIT6;

    P4->SEL0 |= A8_IN;      //use analog input #8 = P4.5 for ADC input
    P4->SEL1 |= A8_IN;

    //Disable conversion while setting up ADC registers
    ADC14->CTL0 &= ~ADC14_CTL0_ENC;

    ADC14->CTL0 = ADC14_CTL0_SHP | ADC14_CTL0_SSEL__SMCLK | ADC14_CTL0_DIV__4
            | ADC14_CTL0_SHT0__16 | ADC14_CTL0_ON;

    //Memory Control Register 0 to receive ADC samples on input #8
    ADC14->MCTL[0] = 8u;

    //Sets up timer to creating ADC sampling clock
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__1 | TIMER_A_CTL_MC__CONTINUOUS;
    TIMER_A0->CCR[0] = FSAMP-1;
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;

    //Sets up timer for PWM output
    TIMER_A2->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__1 | TIMER_A_CTL_MC__UP;
    TIMER_A2->CCR[0] = PWM_PERIOD-1;
    TIMER_A2->CCR[1] = PWM_PERIOD/2;
    TIMER_A2->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    //TIMER_A2->CTL |= TIMER_A_CTL_IE;

    NVIC_EnableIRQ(TA0_0_IRQn);
    //NVIC_EnableIRQ(TA2_N_IRQn);
    __enable_interrupts();


    //Filter variables needed
    int counter = 0;
    int M = 4;
    int filtered_val;

    // Create values for 5 cycles
    volatile unsigned int cycle_vals[5] =  { 0, 0, 0, 0, 0 }; // Assign starting values

    //Counter int - goes from 0-M
    while (1)  {

        if ((ADC14->IFGR0 & ADC14_IFGR0_IFG0) != 0)  {
            P6->OUT = 0x00;
            ADC_In = ((ADC14->MEM[0]) >> ADCSCALE);

            cycle_vals[counter] = ADC_In; //Insert new value in ADC_In at the counter position
            filtered_val = 0;

            //Averaging filter
            int i;
            for(i=0; i<=M; i++) {
                filtered_val += ( cycle_vals[i] / (1+M) ) ; // Sums average of all cycle values
            }

            TIMER_A2->CCR[0] = 0;               //disable timer
            TIMER_A2->CCR[1] = filtered_val;    //load new duty cycle value
            TIMER_A2->CCR[0] = PWM_PERIOD-1;    //enable timer

            // Increment counter from 0-5
            counter += 1;
            if(counter>M) counter=0;
        }

    }

}


void TA0_0_IRQHandler(void)  {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    TIMER_A0->CCR[0] += FSAMP;
    P6->OUT = 0x01;
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
}


