/****************************************************************
**	OrangeBot Project
*****************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************
**	Test Quad Encoder Decoding routine
*****************************************************************
**  Feed an 8bit test vector to simulate inputs for a quad channel
**
**	FEATURES:
**
**		double event
**	Encoder ISR can detect twin events and compute correctly encoder reading
**	based on previous direction.
**	a double event flag is generated when a double event occour, signaling the
**	the microcontroller is overtaxed and is missing some events
**	this is advisory if the warning is rare, but should be treated as error if it's consistent
**	if let's say 10% of execution lead to a double event it should be fine
**	if 50% of execution lead to double events, some of the single event might actually be
**	misclassified triple events in the opposite direction.
**
**		quad channel
**	Encoder ISR can detect four encoder channel at once
**
**		local counters
**	Encoder ISR update couters locally and sync them with the main
**	only if the main is not accessing the registers and either the main request
**	an update or the local counters are getting too stuffed.
**	Since local counters are 8bit and global counters 32bit, this saves many instructions.
****************************************************************/

/****************************************************************************
**	INCLUDE
****************************************************************************/


#include <stdint.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define ENABLE_DEBUG
#include "debug.h"

/****************************************************************************
**	TESTS
****************************************************************************/

//#define TEST_FOUR_RAMPS

//Generate four sin like encoder waveforms to fully test all phasing
#define TEST_FOUR_SIN

/****************************************************************************
**	DEFINE
****************************************************************************/

//
#define ENCODER_TEST_VECTOR_LENGTH	128
//Force the ISR to update global counters
#define ENCODER_TEST_FORCE_UPDATE	110
//Number of quadrature encoders channels
#define ENC_NUM						4
//When a local counter is above this threshold, the ISR tries to sync local counters with the main counters
#define ENC_UPDATE_TH				1

/****************************************************************************
**	MACRO
****************************************************************************/

//generate a mask with '1' in position shift value
#define MASK( shift_value ) \
	( 0x01 << (shift_value) )

//generate inverted mask '0' in position shift value
#define INV_MASK( shift_value )	\
	(~MASK(shift_value))

/****************************************************************************
**	NAMESPACES
****************************************************************************/

using std::cout;

/****************************************************************************
**	STRUCT
****************************************************************************/

//Data structure to encode the full status of the encoder encoding and decoding
typedef struct _data_store
{
	uint8_t enc_in;
	int32_t enc_count[ENC_NUM];
	bool f_enc_err;
} Data_store;

/****************************************************************************
**	FUNCTION PROTOTYPES
****************************************************************************/

//Generate four encoder ramps with a given slope. Slope can be be an a float and bigger than 1
extern void generate_enc_input( uint8_t *enc_input, float *period  );

//Generate sin like encoder inputs
extern void generate_sin_enc_input( uint8_t *enc_input, float *period, float *slope  );

extern void decode_encoder_input( uint8_t portc_in );

/****************************************************************************
**	GLOBAL VARS
****************************************************************************/

//Length of a cycle in a quadrature encoder
#define ENC_QUADRATURE_SAMPLES		4
//Scan of B and A channels on LSB
uint8_t quadrature_enc[ENC_QUADRATURE_SAMPLES] =
{
	0x00,
	0x01,
	0x03,
    0x02
};

//Number of samples of the encoder quadrature decoding LUT
#define LUT_ENC_QUADRATURE_DECODING	32
// LUT: Encoder Quadrature Decoding
// Bit 765 | unused, hold at zero
// Bit 4 | Previous direction | 0 = clockwise | 1 = counterclockwise
// Bit 32 | old encoder reading | B channel A channel
// Bit 10 | new encoder reading | B channel A channel
int8_t enc_lut[LUT_ENC_QUADRATURE_DECODING] =
{
	+0,	//00=No Change: +0
	+1,	//01=A Rise with B=0: +1 (Clockwise)
	-1,	//02=B Rise with A=0: -1 (Counter Clockwise)
	+2,	//03=A Rise B Rise: double event +2
	-1,	//04=A Fall with B=0: -1 (Counter Clockwise)
	+0,	//05=No Change: +0
	+2,	//06=A Rise B Fall: double event +2
	+1,	//07=B Rise with A=1: +1 (Clockwise)
	+1,	//08=B Fall with A=0: +1 (Clockwise)
	+2,	//09=A Fall B Rise: double event +2
	+0,	//0a=No Change: +0
	-1,	//0b=A Rise with B=1: -1 (Counter Clockwise)
	+2,	//0c=A Fall B Fall: double event +2
	-1,	//0d=B Fall with A=1: -1 (Counter Clockwise)
	+1,	//0e=A Fall with B=1: +1 (Clockwise)
	+0,	//0f=No Change: +0
	+0,	//10=No Change: +0
	+1,	//11=A Rise with B=0: +1 (Clockwise)
	-1,	//12=B Rise with A=0: -1 (Counter Clockwise)
	-2,	//13=A Rise B Rise: double event -2
	-1,	//14=A Fall with B=0: -1 (Counter Clockwise)
	+0,	//15=No Change: +0
	-2,	//16=A Rise B Fall: double event -2
	+1,	//17=B Rise with A=1: +1 (Clockwise)
	+1,	//18=B Fall with A=0: +1 (Clockwise)
	-2,	//19=A Fall B Rise: double event -2
	+0,	//1a=No Change: +0
	-1,	//1b=A Rise with B=1: -1 (Counter Clockwise)
	-2,	//1c=A Fall B Fall: double event -2
	-1,	//1d=B Fall with A=1: -1 (Counter Clockwise)
	+1,	//1e=A Fall with B=1: +1 (Clockwise)
	+0	//1f=No Change: +0
};	//End LUT: Encoder Quadrature Decoding

//Global encoder counters
volatile int32_t g_enc_cnt[ ENC_NUM ];
//Detect double events
volatile bool g_f_enc_err = false;
//raise when the main is reading any 32b encoder channel counter
volatile bool g_f_enc_sem = false;
//main raises this flag to force an update of the 32 bit encoder channel counters
volatile bool g_f_enc_updt = false;

/****************************************************************************
**	MAIN
****************************************************************************/

int main()
{
	//Counter
	uint8_t t, ti;
	//Encoder input vector
	uint8_t enc_input[ ENCODER_TEST_VECTOR_LENGTH ];

	Data_store my_data[ ENCODER_TEST_VECTOR_LENGTH ];

	//period vector
	float period[ENC_NUM] = { 100.0, 50.0, 30.0, 20.0 };
	//slope vector
	float slope[ENC_NUM] = { 1.0, 0.5, -0.5, 1.2 };

	DSTART( 0 );
	DENTER();

	//Call ISR in order to initialize local static variables and not miss any edge the first call
	decode_encoder_input( enc_input[0] );


	#ifdef TEST_RAMP
	generate_enc_input( &enc_input[0], slope );
	#endif

	#ifdef TEST_FOUR_SIN
	generate_sin_enc_input( &enc_input[0], period, slope );
	#endif

    //Scan vector
    for (t = 0;t < ENCODER_TEST_VECTOR_LENGTH;t++)
    {
		//Save input vector on the data store
		my_data[t].enc_in = enc_input[t];
		//If threshold count is achieved
		if (t == ENCODER_TEST_FORCE_UPDATE)
		{
			//After a number of cycles, the main tries to force update of the encoder reading.
			g_f_enc_updt = true;
			//Disable interrupts

			//manually call the ISR to execute the update
			decode_encoder_input( enc_input[t] );
			//Here the ISR has updated the global counters
		}

		//Feed vector to ISR
		decode_encoder_input( enc_input[t] );
		//For: each encoder channel
		for (ti = 0;ti < ENC_NUM;ti++)
		{
			my_data[t].enc_count[ti] = g_enc_cnt[ti];
		}

    }

	DPRINT("IN  | ENC0  | ENC1  | ENC2  | ENC3\n");
	//Scan vector
    for (t = 0;t < ENCODER_TEST_VECTOR_LENGTH;t++)
    {
		DPRINT("%3x | %5d | %5d | %5d | %5d\n", my_data[t].enc_in, my_data[t].enc_count[0], my_data[t].enc_count[1], my_data[t].enc_count[2], my_data[t].enc_count[3]);
    }

	DRETURN();
    DSTOP();
    return 0;
}

/****************************************************************************
**	FUNCTION DEFINITIONS
****************************************************************************/

/****************************************************************************
**  Function
**  generate_enc_input
****************************************************************************/
//! @param enc_input	| vector in which save the four encoded encoder waveforms
//! @param period		| vector of four elements. Period of each of four encoder channels
//! @brief Encode a four channel test vector for the encoder ISR
//! @details
/***************************************************************************/

void generate_enc_input( uint8_t *enc_input, float *period  )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//counter
	int t, ti;
	float f_index;
	int index;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	DENTER();

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	DPRINT("  ROW | ENC0 | ENC1 | ENC2 | ENC3 | WORD\n");
	//Scan vector
    for (t = 0;t < ENCODER_TEST_VECTOR_LENGTH;t++)
    {
		DPRINT( "%5d | ", t);
		//Initialize input row
		enc_input[ t ] = 0;
		//For: scan all encoder channels
		for (ti = 0;ti < ENC_NUM;ti++)
		{
			//Generate encoder channel step based on the channel period
			f_index = (float)period[ ti ] *t;
			//Process encder step to get an index to the quadrature encoder LUT
			index = (int)1 *f_index;
			index = index %ENC_QUADRATURE_SAMPLES;
			//residual of a negative number is negative. I need to make it positive and flip it so that table is executed in the right order
			index = (index>=0)?(index):(ENC_QUADRATURE_SAMPLES+index);
			//Add the encoded quadrature channel code to the input vector
			enc_input[ t ] |= quadrature_enc[ index  ] << (ti*2);
			DPRINT_NOTAB( "%4.1f | ", f_index);
		}	//End For: scan all encoder channels
		DPRINT_NOTAB( "%3x\n", enc_input[ t ]);

    }

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();

	return;
}

/****************************************************************************
**  Function
**  generate_sin_enc_input
****************************************************************************/
//! @param enc_input	| vector in which save the four encoded encoder waveforms
//! @param period		| vector of four elements. Period of each of four encoder channels
//! @param slope		| vector of four elements. maximum slope of the sin waveform. can be slightly more than one
//! @brief Encode a four channel test vector for the encoder ISR
//! @details
/***************************************************************************/

void generate_sin_enc_input( uint8_t *enc_input, float *period, float *slope  )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//counter
	int t, ti;
	float f_index;
	int index;

	float amplitude[ENC_NUM];

	float index_accumulator[ENC_NUM];

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	DENTER();

	//compute amplitude of the sin waves
	//For: scan all encoder channels
	for (ti = 0;ti < ENC_NUM;ti++)
	{
		//amplitude[ti] = period[ti] *slope[ti] /2.0 /M_PI;
		amplitude[ti] = 1.0;
		//initialize f_index
		index_accumulator[ti] = (float)0.0;
	}

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	DPRINT("  ROW | ENC0 | ENC1 | ENC2 | ENC3 | WORD\n");
	//Scan vector
    for (t = 0;t < ENCODER_TEST_VECTOR_LENGTH;t++)
    {
		DPRINT( "%5d | ", t);
		//Initialize input row
		enc_input[ t ] = 0;
		//For: scan all encoder channels
		for (ti = 0;ti < ENC_NUM;ti++)
		{
			//accumulate local sin
			index_accumulator[ti] += amplitude[ti] *sin( 2.0* M_PI *t /period[ti]);

			//Generate encoder channel step based on the channel period
			f_index = index_accumulator[ti];
			//Process encder step to get an index to the quadrature encoder LUT
			index = (int)1 *f_index;
			index = index %ENC_QUADRATURE_SAMPLES;
			//residual of a negative number is negative. I need to make it positive and flip it so that table is executed in the right order
			index = (index>=0)?(index):(ENC_QUADRATURE_SAMPLES+index);
			//Add the encoded quadrature channel code to the input vector
			enc_input[ t ] |= quadrature_enc[ index  ] << (ti*2);
			DPRINT_NOTAB( "%4.1f | ", f_index);
		}	//End For: scan all encoder channels
		DPRINT_NOTAB( "%3x\n", enc_input[ t ]);

    }

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();

	return;
}


/****************************************************************************
**  ISR
**  PORTC_PORT_vect
****************************************************************************/
//! @brief Decode four quadrature encoder channels on an edge on each of the channel
//! @details
//!	PIN	| ENC0	| ENC1	| ENC2	| ENC3	|
//!	-------------------------------------
//!	CHA	| PC0	| PC2	| PC4	| PC6	|
//!	CHB	| PC1	| PC3	| PC5	| PC7	|
//!
//!	FEATURES:
//!
//!		combined ISR
//! option 1 was to write four smaller ISRs one for channel
//! option 2 was to write one bigger ISR triggered by each edge
//!	which one is better depends on ISR overhead and load distribution between channels
//! since all encoders go at the same speed, it make sense to write just one routine?
//!
//! 	global sync
//!	ISR only updates a local smaller faster counter all of the times.
//! synchronization between this counter and the main routine only happens when
//! the main routine is not sampling it, when the main routine is requesting it or when local counters are getting too full
//! this feature is meant to reduce the overhead of the ISR encoder routine significantly by not updating 32bit registers
//!
//!     double event
//!	A LUT allows handling of tricky double events that happen when the ISR can't keep up and skip a beat
//! double event can be handled with no error in count and allow to warn the main that the encoders are getting out of hand
//! and stalling the microcontroller
//!
//! ALGORITHM:
//! >Fetch new pin configuration
//! >For each channel
//!		>Build an index to the encoder LUT
//!		>Decode the increment to be added to the 16b relative counter
//!		>Save direction and detect double events to raise warnings
//!	>Write new configuration into old configuration
/***************************************************************************/

void decode_encoder_input( uint8_t portc_in )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//relative encoder counters
	static int8_t enc_cnt[ENC_NUM] = { 0 };
	//Memory of the previous direction of the encoders. each bit is one encoder channel. false=+ true=-
	static uint8_t enc_dir;
	//Memory of previous encoder pin configuration. Initialize to current one at first cycle
	static uint8_t enc_pin_old = portc_in;
	//Fetch pin configuration
	uint8_t enc_pin = portc_in;
	//Counter used to scan the encoders
	uint8_t t;
	//index to the LUT
	uint8_t index;
	//increment decoded from the LUT
	int8_t increment;
	//temporary error counter
	bool f_err = false;
	//
	bool f_update;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	DENTER_ARG( "in: %x\n",portc_in );

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For: each encoder channel
	for (t = 0;t < ENC_NUM;t++)
	{
		//! Build address to the encoder LUT
		// | 4		| 3		| 2		| 1		| 0
		// | dir	| old B	| old A	| B		| A
		//Inject new AB and clear index
		index = (((enc_pin) >> (2*t)) & 0x03);
		//Inject old AB
		index |= ((t==0)?((enc_pin_old<<2)&0x0c):(((enc_pin_old) >> (2*(t-1))) & 0x0c));
		//Inject old direction
		index |= (((enc_dir) << (4 -t)) & 0x10);

		//! Decode the increment through the LUT
		//Use the index as address for the encoder LUT, applying the complex truth table
		increment = enc_lut[ index ];

		//! Apply increment to local relative memory and compute special
		//Apply increment
		enc_cnt[t] += increment;
		//Compute new direction flag and write it back to the correct bit of the encoder direction memory
		enc_dir = (enc_dir & INV_MASK(t)) | (((increment < 0) & 0x01) << t);
		//Detect if a double event happened and remember it. Serves as over speed warning
		f_err |= ((increment == +2) || (increment == -2));
		//overflow update flag. if at least a counter is getting dangerously large
		f_update |= ((enc_cnt[t] >= ENC_UPDATE_TH) || (enc_cnt[t] <= -ENC_UPDATE_TH));

	} //End For: each encoder channel


	DPRINT( "%5d | %5d | %5d | %5d\n", enc_cnt[3], enc_cnt[2], enc_cnt[1], enc_cnt[0]);

	//! Write back double event error flag
	g_f_enc_err |= f_err;

	//! Write back ISR counters to global 32bit counters
	//Only write back if main requests it, if at least one counter is above threshold. withhold if
	if ((g_f_enc_sem == false) && ((g_f_enc_updt == true) || (f_update == true)))
	{
		//For: each encoder channel
		for (t = 0;t < ENC_NUM;t++)
		{
			g_enc_cnt[t] += enc_cnt[t];
			enc_cnt[t] = 0;
			//Clear update flag
			f_update = false;
			//notify the main that sync happened
			g_f_enc_updt = false;
		} //End For: each encoder channel
	}

	//Save pin configuration
	enc_pin_old = enc_pin;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	DRETURN();
}

