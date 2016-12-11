/*
 * Adam Matusiak 2016
 *
 * DSP lab2
 */
#include "ezdsp5535.h"
#include "ezdsp5535_gpio.h"
#include "ezdsp5535_i2c.h"
#include "ezdsp5535_i2s.h"
#include "ezdsp5535_sar.h"
#include "ezdsp5535_led.h"

#include "csl_i2c.h"

#include <dsplib.h>

void AIC3204_config();

#define AIC3204_I2C_ADDR 0x18
Int16 AIC3204_rset( Uint16 regnum, Uint16 regval )
{
    Uint16 cmd[2];
    cmd[0] = regnum & 0x007F;       // 7-bit Device Register
    cmd[1] = regval;                // 8-bit Register Data

    EZDSP5535_waitusec( 300 );

    return EZDSP5535_I2C_write( AIC3204_I2C_ADDR, cmd, 2 );
}

#define HWAFFT_SCALE_ON_FLAG 0
#define HWAFFT_SCALE_OFF_FLAG 1
#define HWAFFT_FFT_FLAG 0
#define HWAFFT_IFFT_FLAG 1

extern Uint16 hwafft_br(Int32*, Int32*, Uint16);
extern Uint16 hwafft_512pts(Int32*,Int32*,Uint16,Uint16);

#define N 512

#pragma DATA_SECTION(samples, ".input"); //DARAM3 block
Int32 samples[N];

#pragma DATA_SECTION(brev_samples, ".brev"); //DARAM2 block
#pragma DATA_ALIGN(brev_samples, 2*N); //log2(4*N) LSBs zeros
Int32 brev_samples[N];

#pragma DATA_SECTION(scratch, ".input");
Int32 scratch[N];

Int16* result = (Int16*)brev_samples;

int sample = 0;

#define SAMPLE_FREQ_FLOAT 8000.0
#define DIFF_FREQ (SAMPLE_FREQ_FLOAT/N)
const Int16 DTMF_freq_index_array[] = {	697.0/DIFF_FREQ + 0.5,
										770.0/DIFF_FREQ + 0.5,
										852.0/DIFF_FREQ + 0.5,
										941.0/DIFF_FREQ + 0.5,
										1209.0/DIFF_FREQ + 0.5,
										1336.0/DIFF_FREQ + 0.5,
										1477.0/DIFF_FREQ + 0.5,
										1633.0/DIFF_FREQ + 0.5};
const char DTFM_sign_array[4][4] =	{
										{'1','2','3','A'},
										{'4','5','6','B'},
										{'7','8','9','C'},
										{'*','0','#','D'}
									};
Int16 DTFM_amplitudes[8];

char DTFM_string[] = {"0000000000000000"};
char* dtfm_p = DTFM_string;
int dtfm_i = 0;

static inline int s_max(int start_index);
int DTFM_detection(char *c);

int no_dtfm_flag = 0;

int main(void) {
    Int16 data1, data2;
    Int16 sample_index = 0;

    *(ioport volatile unsigned*)0x0001 = 0x000E; //Idle configuration register właczenie zegara
    asm("	idle");

    Int16 EZDSP5535_init();
	EZDSP5535_I2C_init();
	AIC3204_config();
    /* Initialize I2S */
    EZDSP5535_I2S_init();

    /* Play Tone for 5 seconds*/
    for ( ;; )
    {
                /* Write 16-bit left channel Data */
                EZDSP5535_I2S_readLeft(&data1);
                /* Write 16-bit right channel Data */
                EZDSP5535_I2S_readRight(&data2);

                /* Write 16-bit left channel Data */
                EZDSP5535_I2S_writeLeft(data1);
                /* Write 16-bit right channel Data */
                EZDSP5535_I2S_writeRight(data2);


                samples[sample_index++] = (Int32)data2 << 16;
                if(sample_index == N)
                {
                	while(sample_index != N)
                		samples[sample_index++] = 0;

                	hwafft_br(samples, brev_samples, N);
                	Uint16 result_flag = hwafft_512pts(	brev_samples,
                										scratch,
                										HWAFFT_FFT_FLAG,
														HWAFFT_SCALE_ON_FLAG);
                	result 	=	result_flag
                			?	(Int16*)scratch
                			:	(Int16*)brev_samples;
                	asm("	NOP");
                	for(sample_index=0; sample_index<2*N; sample_index++) //2N 16-bitowych danych
                	{
                		Int32 temp = (Int32)result[sample_index] * (Int32)result[sample_index];
                		result[sample_index] = (Int16)(temp >> 15);
                	}
                	asm("	NOP");
                	for(sample_index=0; sample_index<N; sample_index++)
                	{
                		Int32 temp = result[2*sample_index] + result[2*sample_index+1];
                		if(temp > 0x7FFF)
                			result[sample_index] = 0x7FFF;
                		else if(temp < (Int32)0xFFFF8000)
                			result[sample_index] = (Int16)(Int32)0xFFFF8000;
                		else
                			result[sample_index] = (Int16)temp;
                	}
                	asm("	NOP");
                	sqrt_16(result,result,N/2);
                	sample_index = 0;

                	char c = 0;
                	if(DTFM_detection(&c) < 0)
                	{
                		no_dtfm_flag = 1;
                	}
                	else if(no_dtfm_flag == 1)
                	{
                		/*if(*dtfm_p == '\0')
                			dtfm_p = DTFM_string;

                		*(dtfm_p++) = c;*/

                		if(DTFM_string[dtfm_i] == '\0')
                		    dtfm_i = 0;

                		DTFM_string[dtfm_i++] = c;

                		if(c == '*')
                			dtfm_i = 0;

                		no_dtfm_flag =  0;
                	}
                }
    }
    EZDSP5535_I2S_close();    // Disble I2S
    AIC3204_rset( 0,  0x00 );  // Select page 0
    AIC3204_rset( 1,  0x01 );  // Reset codec

    return 0;
}

int DTFM_detection(char *c)
{
	int i = 0;
	for(; i<8; i++)
	{
		DTFM_amplitudes[i] = result[DTMF_freq_index_array[i]];
	}
	int low_i = 0, hig_i = 4;
	if((low_i = s_max(low_i)) < 0)
		return -1;
	if((hig_i = s_max(hig_i)) < 0)
		return -1;

	*c = DTFM_sign_array[low_i][hig_i-4];
	return 0;
}

static inline int s_max(int start_index)
{
	int i = 1, max_i = start_index, max = DTFM_amplitudes[max_i];
	while(i < start_index + 4) // znajdz najwiekszą wartość
	{
		if( DTFM_amplitudes[i] > DTFM_amplitudes[max_i] )
		{
			max_i = i;
			max = DTFM_amplitudes[max_i];
		}
		i++;
	}
	DTFM_amplitudes[max_i] = 0;
	i = 0;
	while(i < start_index + 4) // sprawdz czy tylko jeden ton w low
	{
		if( DTFM_amplitudes[i]  >= (max>>4)  ) //czy wiekszy od reszty o 2^5
			return -1;

		i++;
	}
	return max_i;
}

void AIC3204_config()
{
    /* Configure AIC3204 */
    AIC3204_rset( 0,  0x00 );  // Select page 0
    AIC3204_rset( 1,  0x01 );  // Reset codec
    EZDSP5535_waitusec(1000);  // Wait 1ms after reset
    AIC3204_rset( 0,  0x01 );  // Select page 1
    AIC3204_rset( 1,  0x08 );  // Disable crude AVDD generation from DVDD
    AIC3204_rset( 2,  0x01 );  // Enable Analog Blocks, use LDO power
    AIC3204_rset( 123,0x05 );  // Force reference to power up in 40ms
    EZDSP5535_waitusec(50000); // Wait at least 40ms
    AIC3204_rset( 0,  0x00 );  // Select page 0

    /* PLL and Clocks config and Power Up  */
    AIC3204_rset( 27, 0x0d );  // BCLK and WCLK are set as o/p; AIC3204(Master)
    AIC3204_rset( 28, 0x00 );  // Data ofset = 0
    AIC3204_rset( 4,  0x03 );  // PLL setting: PLLCLK <- MCLK, CODEC_CLKIN <-PLL CLK
    AIC3204_rset( 6,  0x09 );  // PLL setting: J=9
    AIC3204_rset( 7,  0x15 );  // PLL setting: HI_BYTE(D=5570)
    AIC3204_rset( 8,  0xC2 );  // PLL setting: LO_BYTE(D=5570)
    AIC3204_rset( 30, 0x88 );  // For 32 bit clocks per frame in Master mode ONLY
                               // BCLK=DAC_CLK/N =(12288000/8) = 1.536MHz = 32*fs
    AIC3204_rset( 5,  0x81 );  // PLL setting: Power up PLL, P=8 and R=1
    EZDSP5535_waitusec(10000); // Wait for PLL to come up
    AIC3204_rset( 13, 0x00 );  // Hi_Byte(DOSR) for DOSR = 128 decimal or 0x0080 DAC oversamppling
    AIC3204_rset( 14, 0x80 );  // Lo_Byte(DOSR) for DOSR = 128 decimal or 0x0080
    AIC3204_rset( 20, 0x80 );  // AOSR for AOSR = 128 decimal or 0x0080 for decimation filters 1 to 6
    AIC3204_rset( 11, 0x82 );  // Power up NDAC and set NDAC value to 2
    AIC3204_rset( 12, 0x87 );  // Power up MDAC and set MDAC value to 7
    AIC3204_rset( 18, 0x87 );  // Power up NADC and set NADC value to 7
    AIC3204_rset( 19, 0x82 );  // Power up MADC and set MADC value to 2

    /* DAC ROUTING and Power Up */
    AIC3204_rset( 0,  0x01 );  // Select page 1
    AIC3204_rset( 12, 0x08 );  // LDAC AFIR routed to HPL
    AIC3204_rset( 13, 0x08 );  // RDAC AFIR routed to HPR
    AIC3204_rset( 0,  0x00 );  // Select page 0
    AIC3204_rset( 64, 0x02 );  // Left vol=right vol
    AIC3204_rset( 65, 0x00 );  // Left DAC gain to 0dB VOL; Right tracks Left
    AIC3204_rset( 63, 0xd4 );  // Power up left,right data paths and set channel
    AIC3204_rset( 0,  0x01 );  // Select page 1
    AIC3204_rset( 16, 0x00 );  // Unmute HPL , 0dB gain
    AIC3204_rset( 17, 0x00 );  // Unmute HPR , 0dB gain
    AIC3204_rset( 9 , 0x30 );  // Power up HPL,HPR
    EZDSP5535_waitusec(100 );  // Wait

    /* ADC ROUTING and Power Up */
    AIC3204_rset( 0,  0x01 );  // Select page 1
    AIC3204_rset( 52, 0x30 );  // STEREO 1 Jack
                               // IN2_L to LADC_P through 40 kohm
    AIC3204_rset( 55, 0x30 );  // IN2_R to RADC_P through 40 kohmm
    AIC3204_rset( 54, 0x03 );  // CM_1 (common mode) to LADC_M through 40 kohm
    AIC3204_rset( 57, 0xc0 );  // CM_1 (common mode) to RADC_M through 40 kohm
    AIC3204_rset( 59, 0x00 );  // MIC_PGA_L unmute
    AIC3204_rset( 60, 0x00 );  // MIC_PGA_R unmute
    AIC3204_rset( 0,  0x00 );  // Select page 0
    AIC3204_rset( 81, 0xc0 );  // Powerup Left and Right ADC
    AIC3204_rset( 82, 0x00 );  // Unmute Left and Right ADC
    AIC3204_rset( 0,  0x00 );  // Select page 0
    EZDSP5535_waitusec(100 );  // Wait
}
