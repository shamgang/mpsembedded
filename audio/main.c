/*
 	main.c

	Author: Shamik Ganguly

 	Main file for Music Performance Augmenter audio module.
 	Handles digital effects and artificial accompaniment.
 	Effects:
 	Reads and parses control data over I2C as master, uses
 	control data to change effects selection and parameters,
 	applies bitcrusher effect using precision manipulation,
 	applies tremolo effect using sinusoidal envelope,
 	applies flanger effect using sinusoidally varying delay,
 	and applies pitch-shifted delay using downsampled delay.
 	Accompaniment:
 	Prepares frames of signal for autocorrelation, calculates
 	autocorrelation, finds maximum peak in autocorrelation,
 	calculates period and then frequency from peak, averages
 	frequency guesses over short time, quantizes average to MIDI,
 	stores MIDI notes in short-term histogram, cross-correlates
 	with Krumhansl key profiles to guess key, outputs a closed
 	major triad in that key using direct digital pure sine
 	synthesis, and ignores silent signal for a period of time
 	to make accompaniment less intermittent.
 	NOTE: Currently analyzes only right channel of input
*/

// TI DSP library
// Functions acorr is used for autocorrelation
#include <Dsplib.h>
// Spectrum Digital definitions and prototypes for C5515
// Function USBSTK5515_init used for initialization
#include <usbstk5515.h>
// C5515 AIC library made by EECS 452 GSI
// Functions AIC_init, AIC_read2, and AIC_write2 are used for
// initialization of audio input, and reads and writes of the buffer
#include <AIC_func.h>
// Interrupt-related definitions by EECS 452 GSI
#include <usbstk5515_interrupts.h>
// SPI-related definitions by EECS 452 GSI
#include <usbstk5515_spi.h>
// Spectrum Digital definitions and prototypes for C5515 I2C
// Functions USBSTK5515_I2C_init AND USBSTK5515_I2C_read are
// used for initialization and reading from I2C
#include <usbstk5515_i2c.h>
// Standard library includes
#include <stdio.h>
#include <math.h>
// Helper header file containing sine table
#include "sinetable.h"

// Sampling rate
#define fs 48000.0
// Size of input buffer
#define BUFSIZE 1024
// Log2(BUFSIZE)
#define LOG2BUFSIZE 10
// Number of frequency guesses to average
#define AVG_LENGTH 4
// Log2(AVG_LENGTH)
#define LOG2_AVG_LENGTH 2
// Number of past MIDI note guesses
// to store in histogram
#define SIZE_HIST 20
// Size of delay buffer
#define DELAY_LENGTH 16384 // 2^14
// Number of silent frames to ignore
#define HOLD_FRAMES 50
// Number of unique I2C messages
#define NUMPOTS 7
// Pre-autocorrelation noise gate level
// with reference to maximum 2^15
#define CLIP_LEVEL 8192 // 1/4 amplitude

// Setup for I2S/AIC interrupt routine by EECS 452 GSI
void I2S_interrupt_setup(void);
// Attenuates bitcrushed signal based on bit depth
// for normalization purposes. Values hardcoded
// based on ear testing
short normalizecrush(short in, char numbits);
// Interprets I2C message as control data
// and edits global control variables
void I2C_Interpret();
// Scale I2C control value for smoother mapping
char potscale(char pre);
// Scale I2C control value for better bitcrusher mapping
char potscalecrush(char pre);
// Full-scale normalize, half-wave rectify and center clip
// signal x to level CLIP_LEVEL, then shrink by BUFSIZE
void rectify_clip_scale(DATA x[], ushort n);
// Returns index of highest peak. Invalidates input array.
Uint16 findpeak(DATA x[], ushort n);
// Anchor for vector table
void Reset();

// Bitcrusher bit depth
Uint16 numbits = 15;

// Tremolo oscillating envelope synthesis variables
Uint16 trem_counter = 0, trem_temp, trem_FTV = 65536/fs; // FTV starts at 1 Hz
// Tremolo envelope
Int16 amp; // Q15

// Flanger delay samples
Int16 flange_wetsample_l, flange_wetsample_r;
// Flanger oscillating delay synthesis variables
Uint16 flange_counter = 0, flange_temp, flange_FTV = 65536/fs; // FTV starts at 1 Hz
// Flanger delay buffer variables
// Initialize delay to 5 ms or 240 samples at 48K sampling rate
Uint16 flange_delay_samples = 240, flange_delay_index, flange_buffer_index;

// Pitch-shifted delay samples
Int16 pitch_wetsample_l, pitch_wetsample_r;
// Pitch-shifted delay buffer variables
// Initialize delay to 1/4 second or 12000 samples at 48k sampling rate
Uint16 pitch_delay_index = 12000, pitch_buffer_index;
// Pitch-shifted delay downsampling variables
Uint16 shift_index, skip = 0;

// Delay buffer index mask
const Int32 mask = DELAY_LENGTH - 1;
// Circular delay buffers
Int16 buffer_l[DELAY_LENGTH];
Int16 buffer_r[DELAY_LENGTH];
// Delay buffer head
Uint16 buffer_start = 0;

// I2C message counter
char potcode = 0;

// I2C input buffer
Int8 readbuf = 0x00;

// Control value 0-16
unsigned short ctrl = 16;

// Effect selector enum
typedef enum Effect {
	BITCRUSH = 0,
	TREMOLO,
	FLANGER,
	PITCHDELAY,
	MAXEFFECTS
} Effect_t;
// Effect selection vector
char effectsel[MAXEFFECTS] = {0};
// Master bypass switch
char master_bypass = 0;
// Dry/wet level 0-16
char drywet = 16;

// Input sample counter, resets when buffer fills
Uint32 Counter;

// Rotating input buffers for AIC
DATA bOne	[BUFSIZE];
DATA bTwo 	[BUFSIZE];
// FFT output buffer
DATA bOut	[BUFSIZE];
// Buffer pointers
DATA *pAIC, *pIN, *pOUT;

// Circular buffer for storing frequency guesses
float freq_guesses[AVG_LENGTH];
Uint16 guesses_head, guesses_mask = AVG_LENGTH - 1;

// A4-normalized frequency
float hertz_ratio_float;
// Fixed point log of hert_ratio_float
Int16 log_ratio_fixed;
// MIDI note
Uint16 note;
// Silent frame counter
Uint16 zero_count = 0;

// Circular buffer of recent MIDI notes
DATA history[SIZE_HIST];
Uint16 front = 0;
// Histogram of recent MIDI notes
Uint16 histogram[12] = {0};

// Krumhansl key profiles: Q13
Uint16 kmajor[12] = {52019, 18268, 28508, 19087, 35881, 33505, 20644, 42516, 19579, 29983, 18760, 23593};
Uint16 kminor[12] = {51855, 21955, 28836, 44073, 21299, 28918, 20808, 38912, 32604, 22036, 27361, 25969};
// FTVs for low-register notes, C-B
Uint16 keyFTVs[12] = {179, 189, 200, 212, 225, 238, 253, 268, 284, 150, 159, 169};

// Accompaniment bypass switch
char artificial_bypass = 0;
// Accompaniment volume
char artificial_volume = 16;

// Array of key cross-correlations,
// C major - B major followed by C minor through B minor
Uint32 correlate[24];
Uint32 maxcorr;
Uint16 maxcorrind;

interrupt void I2S_ISR()
{
	// Input, synthesized, effected, and output samples
	Int16 inr, inl, outl, outr, dds, effr, effl;

	// Use log2(TABLE_SIZE) MSBs for sine table index
	temp = counter >> 6;
	temp2 = counter2 >> 6;
	temp3 = counter3 >> 6;
	// Access sine table for each synthesis wave
	dds = (sinetable[temp] >> 2) + (sinetable[temp2] >> 2) + (sinetable[temp3] >> 2);

	// Read sample from input
	AIC_read2(&inr, &inl);

	// Update delay buffer
	buffer_l[buffer_start] = inl;
	buffer_r[buffer_start] = inr;
	buffer_start = (Uint16)(((Uint32)buffer_start + (Uint32)DELAY_LENGTH - (Uint32) 1) & mask);

	// Apply effects if master_bypass is not on
	effl = inl;
	effr = inr;
	if(!master_bypass) {
		if(effectsel[FLANGER]) {
			// Use log2(TABLE_SIZE) = 10 MSBs for sine table index
			flange_temp = flange_counter >> 6;

			// Calculate oscillating delay time
			flange_delay_index = flange_delay_samples +
								(Uint16)(((Int32)20 * (Int32)sinetable[flange_temp] + 0x00004000) >> 15);

			// If we exceed delay buffer, use oldest available sample
			if(flange_delay_index >= DELAY_LENGTH) {
				flange_delay_index = DELAY_LENGTH - 1;
			}

			// Get sample from delay buffer
			flange_buffer_index = (flange_delay_index + buffer_start) % DELAY_LENGTH;
			flange_wetsample_l = buffer_l[flange_buffer_index];
			flange_wetsample_r = buffer_r[flange_buffer_index];

			// 100% mix
			effl = flange_wetsample_l;
			effr = flange_wetsample_r;

			// Increment counter by FTV
			// FTV determined by I2C_Interpret
			flange_counter = flange_counter + flange_FTV;
		}
		if(effectsel[PITCHDELAY]) {
			// Downsample delay
			// If downsampled delay has caught up, reset
			if(skip > pitch_delay_index) {
				skip = 0;
			}
			else {
				// Downsample by 2
				shift_index = pitch_delay_index - skip;
				skip++;
			}

			// Get sample from delay buffer
			pitch_buffer_index = (Uint16)(((Uint32)shift_index + (Uint32)buffer_start) % (Uint32)DELAY_LENGTH);
			pitch_wetsample_l = buffer_l[pitch_buffer_index];
			pitch_wetsample_r = buffer_r[pitch_buffer_index];

			// 50% mix
			effl = (effl >> 1) + (pitch_wetsample_l >> 1);
			effr = (effr >> 1) + (pitch_wetsample_r >> 1);
		}
		if(effectsel[BITCRUSH]) {
			// Crush bits
			// Bit depth determined by I2C_Interpret
			effr = effr & (0xFFFF << (16 - numbits));
			effl = effl & (0xFFFF << (16 - numbits));

			// Normalize loudness based on bit depth
			effr = normalizecrush(effr, numbits);
			effl = normalizecrush(effr, numbits);
		}
		if(effectsel[TREMOLO]) {
			// use log2(TABLE_SIZE) = 10 MSBs for sine table index
			trem_temp = trem_counter >> 6;

			// Calculate oscillating envelope
			amp = sinetable[trem_temp];

			// Apply envelope to input signal
			effl = (((Int32)amp*(Int32)effl)+0x00008000)>>15;
			effr = (((Int32)amp*(Int32)effr)+0x00008000)>>15;

			// Increment counter by FTV
			// FTV determined by I2C_Interpret
			trem_counter = trem_counter + trem_FTV;
		}
		// Mix dry and wet signal based on control from I2C_Interpret
		effr = (((Int32)inr * (Int32)(16-drywet)) + ((Int32)effr * (Int32)drywet)) >> 4;
		effl = (((Int32)inl * (Int32)(16-drywet)) + ((Int32)effl * (Int32)drywet)) >> 4;
	}

	// Output effected signal
	outr = effr;
	outl = effl;

	// If not bypassed, add accompaniment to output
	if(!artificial_bypass) {
		outr = outr + (((Int32)dds * (Int32)artificial_volume) >> 4);
		outl = outl + (((Int32)dds * (Int32)artificial_volume) >> 4);
	}

	// Write to output buffer
	AIC_write2(outr, outl);

	// Store right channel sample in buffer unless full, then skip
	if(Counter < BUFSIZE)
	{
		pAIC[Counter] = inr;
		Counter++;
	}

	// Increment DDS counters by FTVs
	counter = counter + FTV;
	counter2 = counter2 + FTV2;
	counter3 = counter3 + FTV3;

	// Clear interrupt flag
	IFR0 &= (1 << I2S_BIT_POS);
}

void main(void)
{
	// Temporary variable
	DATA *temp1;
	// Assign buffer pointers
	pAIC = &bOne[0];
	pIN = &bTwo[0];
	pOUT = &bOut[0];
	// Index of autocorrelation peak
	Uint16 peakind;
	// Frequency guess based on peak
	// and moving average current guess
	float freq_guess, cur_guess = 0.0;
	// Circular buffer head
	guesses_head = 0;
	// Clear sample counter
	Counter = 0;
	// Loop indices
	Int32 i, j;
	// DDS frequency tuning values
	FTV = FTV2 = FTV3 = 0;
	// Initializations
	USBSTK5515_init();
	USBSTK5515_I2C_init ( );
	AIC_init();
	I2S_interrupt_setup();
	// Enable interrupts
	_enable_interrupts();

	// Zero-fill buffers
	Int16 it;
	for(it = 0; it < AVG_LENGTH; ++it) {
		freq_guesses[it] = 0.0;
	}
	for(it = 0; it < SIZE_HIST; ++it) {
		history[it] = 0;
	}
	for(it = 0; it < 12; ++it) {
		histogram[it] = 0;
	}

	while(1)
	{
		// Collect samples through interrupt until buffer full
		if(Counter >= BUFSIZE)
		{
			// Read 1-byte message from I2C as master
			USBSTK5515_I2C_read( 0x69, &readbuf, 1);
			// Interpret message and apply effects controls
			I2C_Interpret();

			// Rotate buffers
			temp1 = pIN;
			pIN = pAIC;
			pAIC = temp1;

			// Reset sample counter
			Counter = 0;

			// Half-wave rectify, center clip, and scale input signal
			rectify_clip_scale(pIN, BUFSIZE);

			// Calculate autocorrelation of input buffer
			acorr(pIN, pOUT, BUFSIZE, BUFSIZE, raw);
			// Find maximum peak in autocorrelation
			peakind = findpeak(pOUT, BUFSIZE);
			// Find frequency from peak index
			if(peakind) {
				freq_guess = fs / peakind;
			}
			else freq_guess = 0;
			// Calculate running average of frequency guesses
			// using circular buffer
			cur_guess -= freq_guesses[guesses_head] / (float)AVG_LENGTH;
			cur_guess += freq_guess / (float)AVG_LENGTH;
			freq_guesses[guesses_head] = freq_guess;
			guesses_head = (guesses_head + 1) & guesses_mask;

			// MIDI conversion
			// Normalize by A4
			hertz_ratio_float = cur_guess / 440.;
			// NOTE: because our log function must handle outputs from -6 to 7,
			// our output must be signed 16 bit Q12.
			// Take log and convert to fixed-point
			log_ratio_fixed = (Int16)round(log2f(hertz_ratio_float) * 4096.);
			// If silent, count silent frames until HOLD_FRAMES
			// and then begin reporting silent notes
			// else, reset silent frame counter and finish MIDI conversion
			if(log_ratio_fixed == 0x8000) {
				if(zero_count < HOLD_FRAMES) {
					++zero_count;
					continue;
				}
				note = -1;
			}
			else {
				zero_count = 0;
				note = ((Int16)69 + (Int16)((Int32)12 * (Int32)log_ratio_fixed + (1 << 11) >> 12)) % 12;
			}

			// Remove oldest notes from histogram, ignoring silences
			if(history[front] != -1 && histogram[history[front]] > 0) --histogram[history[front]];
			// Add new note to histogram, ignoring silences
			if(note != -1) ++histogram[note];
			// Store notes in circular buffer for later histogram removal
			history[front] = note;
			front = (front + SIZE_HIST - 1) % SIZE_HIST;

			// Cross-correlate histogram with Krumhansl key profiles to find key
			maxcorr = 0;
			// Cross-correlate with major profile
			for(i = 0; i < 12; ++i) {
				correlate[i] = 0;
				for(j = 0; j < 12; ++j) {
					correlate[i] += (Uint32)histogram[(i + j) % 12] * (Uint32)kmajor[j];
				}
				if(correlate[i] > maxcorr) {
					maxcorr = correlate[i];
					maxcorrind = i;
				}
			}
			// Cross-correlate with minor profile
			for(i = 12; i < 24; ++i) {
				correlate[i] = 0;
				for(j = 0; j < 12; ++j) {
					correlate[i] += histogram[(i - 12 + j) % 12]*kminor[j];
				}
				if(correlate[i] > maxcorr) {
					maxcorr = correlate[i];
					maxcorrind = i;
				}
			}

			// Stop DDS if histogram is silent,
			// or else choose root FTV based on maximum
			// key correlation and add major third and
			// perfect fifth FTVs
			if(maxcorr == 0) {
				FTV = 0;
			}
			else {
				if(maxcorrind >= 12) {
					FTV = keyFTVs[maxcorrind - 12];
				}
				else {
					FTV = keyFTVs[maxcorrind];
				}
			}
			FTV2 = 5*FTV/4;
			FTV3 = 3*FTV/2;

		}
	}
}

void I2S_interrupt_setup(void)
{
	// Set up vector table
	Uint32 reset_loc = (Uint32)Reset;
	IVPD = reset_loc >> 8; // pointer table points to Reset
	IVPH = reset_loc >> 8;

	// Point to I2S_ISR
	*((Uint32*)((reset_loc + I2S_ISR_OFFSET)>>1)) = (Uint32)I2S_ISR;

	// Set the External bus select SP0Mode to I2S
	SYS_EXBUSSEL |= (0x1 << 8);
	// Set up the Global interrupt register to flag on I2S receive flag
	IER0 |= (1 << I2S_BIT_POS);
	IFR0 &= (1 << I2S_BIT_POS);
}

short normalizecrush(short in, char numbits) {
	if(numbits == 1) return ((Int32)in * (Int32) 21845) >> 16; // 1/3
	else if(numbits == 2) return ((Int32)in * (Int32) 32768) >> 16; // 1/2
	else if(numbits == 3) return ((Int32)in * (Int32) 43691) >> 16; // 2/3
	else if(numbits == 4) return ((Int32)in * (Int32) 49152) >> 16; // 3/4
	else return in;
}

void I2C_Interpret()
{
	// Last message in sequence contains bypass switch values
	// Bits 6-0 are bypasses for master effects, accompaniment,
	// bitcrusher, tremolo, flanger, and pitch-shifted delay respectively
	if(potcode == NUMPOTS - 1) {
		char effectcode = (readbuf & 0x7E) >> 1;
		master_bypass = (effectcode & 0x01) ? 1 : 0;
		artificial_bypass = (effectcode & 0x02) ? 1 : 0;
		effectsel[BITCRUSH] = (effectcode & 0x04) ? 1 : 0;
		effectsel[TREMOLO] = (effectcode & 0x08) ? 1 : 0;
		effectsel[FLANGER] = (effectcode & 0x10) ? 1 : 0;
		effectsel[PITCHDELAY] = (effectcode & 0x20) ? 1 : 0;
	}

	// All other messages are potentiometers
	// Messages 0-5 are for dry/wet, accompaniment volume,
	// bitcrusher bit depth, tremolo FTV, flanger FTV,
	// and pitch-shifted delay delay time respectively
	// Re-map continuous value for smoother, less noisy operation

	// Read continuous control value from message bits 6-0
	ctrl = (readbuf & 0x7F) >> 1;

	if(potcode == 0) drywet = potscale(ctrl);
	if(potcode == 1) artificial_volume = potscale(ctrl);
	if(potcode == 2) numbits = potscalecrush(ctrl);
	// f0 = fs*FTV/2^bcounter = 0.73*ctrl Hz if fs = 48k, counter bits = 16
	if(potcode == 3) trem_FTV = potscale(ctrl);
	// f0 = fs*FTV/2^bcounter = 0.73*ctrl Hz if fs = 48k, counter bits = 16
	if(potcode == 4) flange_FTV = potscale(ctrl);
	// delay time = 250 + 31*potscale(ctrl) milliseconds
	if(potcode == 5) pitch_delay_index = 12000 + 1500*potscale(ctrl);

	// Bit 7 of message is sync bit
	// Reset message counter if sync bit is high,
	// else increment message counter
	if(readbuf & 0x80) potcode = 0;
	else ++potcode;
}

char potscale(char pre) {
	return pre >> 2;
}

char potscalecrush(char pre) {
	short temp = (pre >> 2) + 1;
	if(temp <= 2) return 8;
	else if(temp <= 8) return 6;
	else if(temp <= 12) return 4;
	else if(temp <= 14) return 3;
	else if(temp <= 15) return 2;
	else return 1;
}

void rectify_clip_scale(DATA x[], ushort n) {
	Int16 i;
	Int16 scale = 0, test1, test2;
	for(i = 0; i < n; ++i) {
		test1 = x[i] & (1 << scale);
		test2 = x[i] & (1 << (scale + 1));
		if((test1 && !test2) || (!test1 && test2)) {
			++scale;
			if(scale > 14) break;
		}
	}
	for(i = 0; i < n; ++i) {
		if(scale < 6) x[i] = 0;
		else
			x[i] = (x[i] < (CLIP_LEVEL >> (15 - scale))) ? 0 : x[i] >> (LOG2BUFSIZE - (15 - scale));
	}
}

Uint16 findpeak(DATA x[], ushort n)
{
	Int16 i = 0, j, store = 0;
	while(i < n - 1) {
		// Fall to bottom of slope
		while(i < n - 1 && x[i] >= x[i + 1]) ++i;
		// Climb to top of slope
		while(i < n - 1 && x[i] <= x[i + 1]) ++i;
		// Walk back to edge of plateau
		j = i;
		while(j > 1 && x[j] == x[j - 1]) --j;
		// Store center of plateau as peak
		x[store++] = (i + j) >> 1;
	}
	// Try to eliminate false edge peaks
	if(i == n - 1 && (x[i] == 0 || x[i] < x[i - 1])) x[store - 1] = 0;
	// Sentinel
	x[store] = 0;
	// Find max peak
	DATA ind = x[0];
	DATA max = x[j];
	for(i = 0; i < store && x[i] != 0; ++i) {
		if(x[x[i]] > max) {
			ind = x[i];
			max = x[ind];
		}
	}
	return ind;
}
