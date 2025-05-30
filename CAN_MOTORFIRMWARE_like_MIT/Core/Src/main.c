/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lower-level-Controller firmware
  * Last Updated by : Vishnudev Kurumbaparambil
  * Datum			: 10.02.2025
  * Base Author		: Dave Mangatter
  * Description		: This is the lower level program that runs in STM. This handles the data exchange between the UP board
  * 				  and Robot Leg motors. It contains a SPI and CAN part.
  * License 		: This software component is licensed by ST under BSD 3-Clause license
  ******************************************************************************
  */



/***********************************************************************************************************
*										I N C L U D E S
************************************************************************************************************/
#include "main.h"
#include "stdio.h"
#include "math.h"
#include <stdbool.h>



/***********************************************************************************************************
*										M A C R O S
************************************************************************************************************/

/****************************    RANGES AND LIMITS      **********************/
// Motor range control
#define MOTOR_P_MIN -12.5f
#define MOTOR_P_MAX  12.5f
#define MOTOR_V_MIN -50.0f
#define MOTOR_V_MAX  50.0f
#define MOTOR_T_MIN -65.0f
#define MOTOR_T_MAX  65.0f
#define MOTOR_KP_MIN 0.0f
#define MOTOR_KP_MAX 500.0f
#define MOTOR_KD_MIN 0.0f
#define MOTOR_KD_MAX 5.0f

// Max torque that can be requested from the motor
#define TRQ_REQ_MAX 1.5f

// STM limits for P V T Kd Kp
#define P_MIN -12.5f
#define P_MAX  12.5f
#define V_MIN -26.0f
#define V_MAX  26.0f
#define T_MIN -TRQ_REQ_MAX
#define T_MAX  TRQ_REQ_MAX
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

/// Joint Soft Stops
#define AB_LIM_P 	 1.5708f 	//90°
#define AB_LIM_N 	-1.5708f 	//-90°
#define HIP_LIM_P 	 2.0944f 	//120°
#define HIP_LIM_N 	-2.0944f 	//120°
#define KNEE_LIM_P	 4.01426f 		//-230°
#define KNEE_LIM_N 	-4.01426f  //-0.1f	//5.7° //todo proper limits, consider direction change
#define KP_SOFTSTOP  100.0f
#define KD_SOFTSTOP  0.4f



/****************************    HANDLING DEVIATIONS	**********************/
#define KNEE_GEARRATIO 1.5 //1.25 //todo for testing

/*
 *  STM 2 on Right/Front A[0]A[1]  a[0]a[1]  FR FL
 *  STM 1 on Left /Rear  A[2]A[3]  a[0]a[1]  RR RL
 */

#ifdef STM2_FRONT
	const int ab_mitdirection[2]    = { 1,  1};
	const int hip_mitdirection[2] 	= { 1,  1};
	const int knee_mitdirection[2] 	= { 1,  1};
#endif

#ifdef STM1_BACK
    const int ab_mitdirection[2]    = { 1,  1};
    const int hip_mitdirection[2] 	= { 1,  1};
    const int knee_mitdirection[2] 	= { 1,  1};
#endif



/****************************    DRIVER AND INTERFACES	**********************/
// length of receive/transmit buffers
#define RX_LEN 66
#define TX_LEN 66

// length of outgoing/incoming messages
#define STATE_LEN 30
#define CONTROL_LEN 66

//Can ID
#define Abad_CANID 	1
#define Hip_CANID   2
#define Knee_CANID  3
uint8_t CAN;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;


/****************************    STRUCTURES AND VARIABLES	**********************/

typedef struct {
    float ab_p[2],hip_p[2],knee_p[2];
    float ab_v[2],hip_v[2],knee_v[2];
  //float ab_t[2],hip_t[2],knee_t[2];
    uint32_t flags[2],checksum;
}spi_tx;

typedef struct {
    float ab_p[2], hip_p[2], knee_p[2];
    float ab_v[2], hip_v[2], knee_v[2];
    float ab_kp[2],hip_kp[2],knee_kp[2];
    float ab_kd[2],hip_kd[2],knee_kd[2];
    float ab_t[2], hip_t[2], knee_t[2];
    uint32_t flags[2],checksum;
}spi_rx;

typedef struct {
    float ab_t[2],hip_t[2],knee_t[2];
}torque_rx;

//structures
spi_rx 		valuesrec;
spi_rx 		control;
spi_tx 		state;
torque_rx 	torque;

//spi buffer
uint16_t spi_tx_buffer[TX_LEN];
uint16_t spi_rx_buffer[RX_LEN];

// CAN buffer
uint8_t CAN_TxData_buf[8];
uint8_t CAN_RxData_buf[8];

uint32_t 	currentControlMode = 99;
volatile uint32_t 	count_checksumerror = 0;

//State Variables
uint32_t checksum_calc; //to store calculated checksum

//input values
float p_in 	= 0.0f;
float v_in 	= 0.0f;
float kp_in = 2.0f;     //stifness
float kd_in = 0.4f;     //damper
float t_in 	= 0.0f;

//measured values
float p_out = 0.0f;     //position
float v_out = 0.0f;     //velocity
float t_out = 0.0f;     //torque


int receivedCanBus = 2;
int count=2;
int exit_command = 0;

//Time
uint32_t time;
uint32_t time2;


uint32_t Error_spi;
uint32_t CallbackError_spi;
uint32_t State_spi;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM8_Init(void);

// MOTOR RELATED
void motor_mode(uint8_t ID,CAN_TxHeaderTypeDef*Header,uint8_t*Data);
void exit_mode(uint8_t ID,CAN_TxHeaderTypeDef*Header,uint8_t*Data);
void zero(uint8_t ID,CAN_TxHeaderTypeDef*Header,uint8_t*Data);
void pack_message(uint8_t ID,CAN_TxHeaderTypeDef*Header,uint8_t*Data);
void unpack_replay(uint8_t* Data);

// Communication
void can_send_receive();
void spi_send_receive(void);
uint32_t xor_checksum(uint32_t*, int);
void CAN_Start(void);
void CAN_Exit(void);
void SPI_Exit(void);

int  softstop_joint(float *control,float state, float limit_p, float limit_n);
void safetycheck_reqTrq(float p_act, float v_act, float t_ff);

//Utilities
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int   float_to_uint(float x, float x_min, float x_max, int bits);
void  delay_us(uint16_t us);
uint32_t encode_floats(float a, float b, float c);
int check_nan_in_spi_rx(spi_rx *data);


// CAN Rx Callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_RxData_buf);
	if (RxHeader.DLC == 8)
	{
		receivedCanBus=0;
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, CAN_RxData_buf);
	if (RxHeader.DLC == 8)
	{
		receivedCanBus=1;
	}
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();

	//Initialize the spi buffers to avoid junk values
	for(int i = 0; i < TX_LEN ; i++)
		spi_tx_buffer[i] = 0;
	for(int i = 0; i < RX_LEN ; i++)
		spi_rx_buffer[i] = 0;



	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CAN1_Init();
	MX_TIM1_Init();
	MX_SPI1_Init();
	MX_CAN2_Init();
	MX_TIM8_Init();

	CAN_Start();
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim8);


	TxHeader.DLC = 8;  // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;



	printf("start\n");

    // ***** ARM SPI once, in interrupt mode *****
	HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);
	//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);




	// START MOTOR
	motor_mode(Abad_CANID, &TxHeader, CAN_TxData_buf);
	motor_mode(Hip_CANID, &TxHeader, CAN_TxData_buf);
	motor_mode(Knee_CANID, &TxHeader, CAN_TxData_buf);
 	delay_us(1000);


//zero(Abad_CANID, &TxHeader, CAN_TxData_buf);
//zero(Hip_CANID, &TxHeader, CAN_TxData_buf);
//zero(Knee_CANID, &TxHeader, CAN_TxData_buf);
//delay_us(1000);



//
////	 Only CAN
//	count=1;
//	  while (exit_command == 0)
//	  {
//
//
//			can_send_receive();
//			time=__HAL_TIM_GET_COUNTER(&htim8);
//	  }


 	// Loop until exited
	while (exit_command == 0)
	{
		//count = 1 executes the CAN
		if(count==1)
		{
			can_send_receive();
			count=2;
		}

	}//end of while


	// STOP MOTOR
	exit_mode(Abad_CANID, &TxHeader, CAN_TxData_buf);
	exit_mode(Hip_CANID, &TxHeader, CAN_TxData_buf);
	exit_mode(Knee_CANID, &TxHeader, CAN_TxData_buf);


	// Wait before shutting down communication
	delay_us(1000);

	// Exit communication
	CAN_Exit();
	SPI_Exit();

}// end of main


							/***************************************************
							 *  				 S P I
							 ***************************************************/

void spi_send_receive(void)
{
	//Pack the torques into the flag.
	 state.flags[0] = encode_floats(torque.ab_t[0], torque.hip_t[0], torque.knee_t[0]) | (state.flags[0]& 0x03);
	 state.flags[1] = encode_floats(torque.ab_t[1], torque.hip_t[1], torque.knee_t[1]) | (state.flags[1]& 0x03);


	//calculate the checksum
	state.checksum = xor_checksum((uint32_t*)&state,14);

	//pack the status variables into the tx buffer
	for(int i = 0; i < STATE_LEN ; i++)
	{
		spi_tx_buffer[i] = ((uint16_t*)(&state))[i];
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if (hspi->Instance == SPI1 && count == 2)
	{
		uint8_t validData = 1;
		//unpack the received bytes from rx buffer into †he valuesrec structur

		for(int i = 0; i < RX_LEN; i++)
		{
			((uint16_t*) &valuesrec)[i] = spi_rx_buffer[i];

		}

		if (check_nan_in_spi_rx(&valuesrec))
		{
			validData = 0;
		}

		if(validData == 1)
		{
			//if the communication has no issues the values will write in the control structure
			checksum_calc = xor_checksum((uint32_t*)&valuesrec,32);

			//Retrieve the current control Mode stored at higher 16 bits and reset flags to its value
			currentControlMode = (valuesrec.flags[0]>>16);
			valuesrec.flags[0] = (valuesrec.flags[0] & 0xFFFF);
			valuesrec.flags[1] = (valuesrec.flags[1] & 0xFFFF);

			if(valuesrec.checksum == checksum_calc && (valuesrec.flags[0]<=3 || valuesrec.flags[1]<=3))
			{
				for(int i = 0; i < CONTROL_LEN; i++)
				{
					((uint16_t*) &control)[i] = ((uint16_t*) &valuesrec)[i];
				}
			}
			else
			{
				count_checksumerror = count_checksumerror + 1;
			}
		}

		// ***** Now prepare for the NEXT transaction *****
		// 1. Update spi_tx_buffer with fresh data
		spi_send_receive();

		// 2. Re-arm the SPI in interrupt mode
		HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);
		//HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);

		//Start CAN
		count = 1;
	}//If it is SPI1
}

void HAL_SPI_ErrorCallback (SPI_HandleTypeDef* hspi){
	CallbackError_spi=HAL_SPI_GetError(&hspi1);

	HAL_SPI_DeInit(&hspi1);
	HAL_SPI_Init(&hspi1);
	HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)spi_tx_buffer, (uint8_t *)spi_rx_buffer, RX_LEN);

	//Start CAN
	count = 1;
}

void SPI_Exit(void) {
    // Disable SPI Interrupts
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn); // If SPI uses DMA Rx
    HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn); // If SPI uses DMA Tx

    // Abort any ongoing SPI transactions
    HAL_SPI_Abort(&hspi1);

    // Stop SPI DMA if enabled
    if (hspi1.hdmatx) {
        HAL_DMA_Abort(hspi1.hdmatx);
    }
    if (hspi1.hdmarx) {
        HAL_DMA_Abort(hspi1.hdmarx);
    }

    // Disable SPI Peripheral
    HAL_SPI_DeInit(&hspi1);
}

							/***************************************************
							 *  				 C A N
							 ***************************************************/

void CAN_Start(void) {
    // Stop CAN to prevent any ongoing transmissions
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_Stop(&hcan2);

    // Abort any pending transmissions safely using HAL functions
    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0);
    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX1);
    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX2);

    HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0);
    HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX1);
    HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX2);

    // Restart CAN
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    // Reactivate CAN notifications (if used)
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CAN_Exit(void) {

    // Disable CAN interrupts
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_SCE_IRQn);

    // Abort any pending CAN transmissions
    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0);
    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX1);
    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX2);
    HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0);
    HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX1);
    HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX2);

    // Disable CAN
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_Stop(&hcan2);

	HAL_CAN_DeInit(&hcan1);
	HAL_CAN_DeInit(&hcan2);
}

void can_send_receive(){

	// Reset the flags before each CAN communication
	state.flags[0] = 0;
	state.flags[1] = 0;

	CAN=0;
	pack_message(Abad_CANID, &TxHeader, CAN_TxData_buf);
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_TxData_buf, &TxMailbox);
    delay_us(300);
	if (receivedCanBus==0){
		unpack_replay(CAN_RxData_buf);
	}
	pack_message(Hip_CANID, &TxHeader, CAN_TxData_buf);
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_TxData_buf, &TxMailbox);
    delay_us(300);
	if (receivedCanBus==0){
		unpack_replay(CAN_RxData_buf);
	}
	pack_message(Knee_CANID, &TxHeader, CAN_TxData_buf);
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_TxData_buf, &TxMailbox);
    delay_us(300);
	if (receivedCanBus==0){
		unpack_replay(CAN_RxData_buf);
	}

	CAN=1;
	pack_message(Abad_CANID, &TxHeader, CAN_TxData_buf);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, CAN_TxData_buf, &TxMailbox);
    delay_us(300);
	if (receivedCanBus==1){
		unpack_replay(CAN_RxData_buf);
	}
	pack_message(Hip_CANID, &TxHeader, CAN_TxData_buf);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, CAN_TxData_buf, &TxMailbox);
    delay_us(300);
	if (receivedCanBus==1){
		unpack_replay(CAN_RxData_buf);
	}
	pack_message(Knee_CANID, &TxHeader, CAN_TxData_buf);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, CAN_TxData_buf, &TxMailbox);
    delay_us(300);
	if (receivedCanBus==1){
		unpack_replay(CAN_RxData_buf);
	}

}

/////////////////////////pack and unpack//////////////////////////

void pack_message(uint8_t ID,CAN_TxHeaderTypeDef*Header,uint8_t*Data)
{

	if(ID == Abad_CANID)
	{
		p_in 	= (control.ab_p[CAN] * ab_mitdirection[CAN]);
		v_in 	= (control.ab_v[CAN] * ab_mitdirection[CAN]);
		kp_in 	=  control.ab_kp[CAN];   //stifness
		kd_in 	=  control.ab_kd[CAN];     //damper
		t_in 	= (control.ab_t[CAN] * ab_mitdirection[CAN]);

		if(softstop_joint(&control.ab_p[CAN],state.ab_p[CAN],AB_LIM_P, AB_LIM_N))
		{	//Incase of wrong request
			state.flags[CAN] |= 0b01;
			p_in = p_in * ab_mitdirection[CAN]; // Direction update
//			t_in = t_in * ab_mitdirection[CAN]; // Direction update
		}

		// Safety Limit
		safetycheck_reqTrq(state.ab_p[CAN], state.ab_v[CAN], torque.ab_t[CAN]);
	}
	if(ID == Hip_CANID)
	{
		p_in 	= (control.hip_p[CAN] * hip_mitdirection[CAN]);
		v_in 	= (control.hip_v[CAN] * hip_mitdirection[CAN]);
		kp_in	=  control.hip_kp[CAN];   //stifness
		kd_in	=  control.hip_kd[CAN];     //damper
		t_in	= (control.hip_t[CAN] * hip_mitdirection[CAN]);

		if(softstop_joint(&control.hip_p[CAN],state.hip_p[CAN], HIP_LIM_P, HIP_LIM_N))
		{	//Incase of wrong request
			state.flags[CAN] |= 0b10;
			p_in = p_in * hip_mitdirection[CAN]; // Direction update
//			t_in = t_in * hip_mitdirection[CAN]; // Direction update
		}

		// Safety Limit
		safetycheck_reqTrq(state.hip_p[CAN], state.hip_v[CAN], torque.hip_t[CAN]);
	}
	if(ID == Knee_CANID)
	{
		p_in 	= (control.knee_p[CAN] * knee_mitdirection[CAN]) * KNEE_GEARRATIO;
		v_in 	= (control.knee_v[CAN] * knee_mitdirection[CAN]) / KNEE_GEARRATIO;
		kp_in 	=  control.knee_kp[CAN];   //stifness
		kd_in	=  control.knee_kd[CAN];     //damper
		t_in 	= (control.knee_t[CAN] * knee_mitdirection[CAN]) * KNEE_GEARRATIO;

		if(softstop_joint(&control.knee_p[CAN], state.knee_p[CAN], KNEE_LIM_P, KNEE_LIM_N))
		{	//Incase of wrong request
			state.flags[CAN] |= 0b11;
			p_in = (p_in * knee_mitdirection[CAN]) * KNEE_GEARRATIO; // Direction update
//			t_in = (t_in * knee_mitdirection[CAN]) * KNEE_GEARRATIO; // Direction update
		}

		// Safety Limit
		safetycheck_reqTrq(state.knee_p[CAN], state.knee_v[CAN], torque.knee_t[CAN]);
	}

	Header->StdId = ID;

    /// limit data to be within bounds ///
	float p_des = fminf(fmaxf(P_MIN, p_in), P_MAX);
	float v_des = fminf(fmaxf(V_MIN, v_in), V_MAX);
	float kp = fminf(fmaxf(KP_MIN, kp_in), KP_MAX);
	float kd = fminf(fmaxf(KD_MIN, kd_in), KD_MAX);
	float t_ff = fminf(fmaxf(T_MIN, t_in), T_MAX);

    /// convert floats to unsigned ints ///
    uint16_t p_int = float_to_uint(p_des, MOTOR_P_MIN, MOTOR_P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des, MOTOR_V_MIN, MOTOR_V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, MOTOR_KP_MIN, MOTOR_KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, MOTOR_KD_MIN, MOTOR_KD_MAX, 12);
    uint16_t t_int = float_to_uint(t_ff, MOTOR_T_MIN, MOTOR_T_MAX, 12);

    /// pack ints into the can buffer ///
    Data[0] = p_int>>8;
    Data[1] = p_int&0xFF;
    Data[2] = v_int>>4;
    Data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
    Data[4] = kp_int&0xFF;
    Data[5] = kd_int>>4;
    Data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
    Data[7] = t_int&0xff;

}


void unpack_replay(uint8_t*Data){
	/// unpack ints from can buffer ///

	uint16_t id = Data[0];
	uint16_t p_int = (Data[1]<<8)|Data[2];
	uint16_t v_int = (Data[3]<<4)|(Data[4]>>4);
	uint16_t i_int = ((Data[4]&0xF)<<8)|Data[5];
	/// convert uints to floats ///
	p_out = uint_to_float(p_int, MOTOR_P_MIN, MOTOR_P_MAX, 16);
	v_out = uint_to_float(v_int, MOTOR_V_MIN, MOTOR_V_MAX, 12);
	t_out = uint_to_float(i_int, MOTOR_T_MIN, MOTOR_T_MAX, 12);

	if(id == Abad_CANID)
	{
		state.ab_p[receivedCanBus]=  (p_out * ab_mitdirection[receivedCanBus]);
		state.ab_v[receivedCanBus]=  (v_out * ab_mitdirection[receivedCanBus]);
		torque.ab_t[receivedCanBus]= (t_out * ab_mitdirection[receivedCanBus]);
	}
	if(id == Hip_CANID)
	{
		state.hip_p[receivedCanBus]=  (p_out * hip_mitdirection[receivedCanBus]);
		state.hip_v[receivedCanBus]=  (v_out * hip_mitdirection[receivedCanBus]);
		torque.hip_t[receivedCanBus]= (t_out * hip_mitdirection[receivedCanBus]);
	}
	if(id == Knee_CANID)
	{
		state.knee_p[receivedCanBus]=  (p_out * knee_mitdirection[receivedCanBus])/ KNEE_GEARRATIO;
		state.knee_v[receivedCanBus]=  (v_out * knee_mitdirection[receivedCanBus])* KNEE_GEARRATIO;
		torque.knee_t[receivedCanBus]= (t_out * knee_mitdirection[receivedCanBus])/ KNEE_GEARRATIO;
    }
}



////////////////////////softstop//////////////////////////////////
//actuator only can move to a limit position

int softstop_joint(float *control,float state, float limit_p, float limit_n)
{
	if(*control>limit_p)
	{
		//*control = limit_p;
		p_in = limit_p;
//		v_in = 0.0f;
//		kp_in = 0.0f;
//		kd_in = KD_SOFTSTOP;
//		t_in += KP_SOFTSTOP*(limit_p - state);
		return 1;
	}
	if(*control<limit_n)
	{
		//*control = limit_n;
		p_in = limit_n;
//		v_in = 0.0f;
//		kp_in = 0.0f;
//		kd_in = KD_SOFTSTOP;
//		t_in += KP_SOFTSTOP*(limit_n - state);
		return 1;
	}

  return 0;
}


////////////////////////safetycheck_reqTrq//////////////////////////////////
//To add additional check on the torque requested to the motor. Calculated based on the PID model of motor.
void safetycheck_reqTrq(float p_act, float v_act, float t_ff)
{
			 //𝜏 = 𝜏ff   + 𝐾𝑝(𝑞des − 𝑞) + 𝐾𝑑(𝑞_des − 𝑞_) (2.2) Software and Control Design for the MIT Cheetah Quadruped Robots by Jared Di Carlo
	float trqreq = t_in + kp_in*(p_in - p_act) + kd_in*(v_in - v_act);

	// Incase the Trq to be calculated at the motor is too high, cancel the req by setting Kp, Kd to zero
	if (trqreq >= TRQ_REQ_MAX || trqreq <= -TRQ_REQ_MAX)
	{
		//Only reset kp, kd. Let other signals remain same. Trq will be limited in the next check
		kp_in = 0.0f;
		kd_in = 0.0f;
	} 

	// Limit `t_in` to max/min allowed torque
	if (t_in > TRQ_REQ_MAX)
	{
		t_in = TRQ_REQ_MAX;
	} else if (t_in < -TRQ_REQ_MAX)
	{
		t_in = -TRQ_REQ_MAX;
	}
}



									/***************************************************
									 *  		M O T O R    M O D E S
									 ***************************************************/


//Start motor
void motor_mode(uint8_t ID,CAN_TxHeaderTypeDef*Header,uint8_t*Data){
	Header->StdId = ID;
	Data[0] = 0xFF;
	Data[1] = 0xFF;
	Data[2] = 0xFF;
	Data[3] = 0xFF;
	Data[4] = 0xFF;
	Data[5] = 0xFF;
	Data[6] = 0xFF;
	Data[7] = 0xFC;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Data, &TxMailbox);
	delay_us(300);
}

//stop motor
void exit_mode(uint8_t ID,CAN_TxHeaderTypeDef*Header,uint8_t*Data){
	Header->StdId = ID;
	Data[0] = 0xFF;
	Data[1] = 0xFF;
	Data[2] = 0xFF;
	Data[3] = 0xFF;
	Data[4] = 0xFF;
	Data[5] = 0xFF;
	Data[6] = 0xFF;
	Data[7] = 0xFD;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Data, &TxMailbox);
	delay_us(300);
}

//set motorposition to zero
void zero(uint8_t ID,CAN_TxHeaderTypeDef*Header,uint8_t*Data){
	Header->StdId = ID;
	Data[0] = 0xFF;
	Data[1] = 0xFF;
	Data[2] = 0xFF;
	Data[3] = 0xFF;
	Data[4] = 0xFF;
	Data[5] = 0xFF;
	Data[6] = 0xFF;
	Data[7] = 0xFE;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Data, &TxMailbox);
	delay_us(300);
}




									/***************************************************
									 *  		 U T I L I T I E S
									 ***************************************************/


//math
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

//delay
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) < us);
}

//spi checksum
uint32_t xor_checksum(uint32_t* data, int len)
{
    uint32_t t = 0;
    for(int i = 0; i < len; i++)
        t = t ^ data[i];
    return t;
}


/*
 * This funciton encodes the torque value into high 30 bits of a 32bit variable.
 * This is used to encode torque into flag for logging it in UP
 */
uint32_t encode_floats(float a, float b, float c)
{
    uint32_t encoded = 0;

    // Clamp each value to the range [-70, 70] using conditional statements
    if (a < -70.0f) a = -70.0f;
    if (a > 70.0f) a = 70.0f;

    if (b < -70.0f) b = -70.0f;
    if (b > 70.0f) b = 70.0f;

    if (c < -70.0f) c = -70.0f;
    if (c > 70.0f) c = 70.0f;


    // Normalize and encode each float to 10 bits
    uint16_t a_enc = (uint16_t)round((a + 70) * (1023.0 / 140.0));
    uint16_t b_enc = (uint16_t)round((b + 70) * (1023.0 / 140.0));
    uint16_t c_enc = (uint16_t)round((c + 70) * (1023.0 / 140.0));

    // Pack into 32 bits, leaving the first 2 bits unused
    encoded |= (a_enc & 0x3FF) << 2;        // First 10 bits start at bit 2
    encoded |= (b_enc & 0x3FF) << 12;       // Next 10 bits start at bit 12
    encoded |= (c_enc & 0x3FF) << 22;       // Last 10 bits start at bit 22

    return encoded;
}


int check_nan_in_spi_rx(spi_rx *data)
{
	
    float *values_ptr = (float *)data;  // Treat struct as float array
    int num_floats = (sizeof(spi_rx) - sizeof(data->flags) - sizeof(data->checksum)) / sizeof(float);

    for (int i = 0; i < num_floats; i++)
    {
        if (isnan(values_ptr[i]))
        {
            // printf("ERROR: NaN detected at index %d! Value: %.9g\n", i, values_ptr[i]);
            return 1;  // Return error if NaN is found
        }
    }
    return 0;  // No NaN detected
}

//Printfunction
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		//__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}








										/***************************************************
										*  				H A L    B A S I C S
										***************************************************/






/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;  // which filter bank to use from the assigned ones 18
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0xFFFF;
  canfilterconfig.FilterMaskIdLow = 0xFFFC;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can) 20

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  CAN_FilterTypeDef canfilterconfig2;

  canfilterconfig2.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig2.FilterBank = 15;  // which filter bank to use from the assigned ones
  canfilterconfig2.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  canfilterconfig2.FilterIdHigh = 0x0000;
  canfilterconfig2.FilterIdLow = 0;
  canfilterconfig2.FilterMaskIdHigh = 0xFFFF;
  canfilterconfig2.FilterMaskIdLow = 0xFFFC;
  canfilterconfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig2.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig2);

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 180;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure PA15 for SPI1_NSS in Alternate Function mode */
  GPIO_InitStruct.Pin       = GPIO_PIN_15;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;     // Or AF_OD if required, usually AF_PP is correct
  GPIO_InitStruct.Pull      = GPIO_NOPULL;         // Typically no pull for NSS
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW; // Speed not critical for NSS
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;       // On STM32F446, AF5 is SPI1_NSS
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



//  /* EXTI interrupt init > Vishnu : Now this interrupt is not required as NSS hanldes the SPI enable/disable*/
//  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    CAN_Exit();  // Clean up CAN and SPI before entering infinite loop
	SPI_Exit();
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
