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
#include "stdio.h"
#include <stdint.h>   
#include "math.h"
#include <stdbool.h>
#include <string.h>


// Dummy definitions for HAL types and constants
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
// Dummy CAN handle structure (empty for simulation)
typedef struct {
    // No members needed for simulation
} CAN_HandleTypeDef;
// Dummy CAN Tx Header structure (as defined in STM HAL)
typedef struct {
    uint32_t StdId;             // Standard Identifier
    uint32_t ExtId;             // Extended Identifier
    uint32_t IDE;               // Identifier Extension
    uint32_t RTR;               // Remote Transmission Request
    uint32_t DLC;               // Data Length Code
    uint32_t TransmitGlobalTime; // Global time stamp (if used)
} CAN_TxHeaderTypeDef;

// Global dummy CAN handles
CAN_HandleTypeDef hcan1, hcan2;

// Dummy definitions for hardware-dependent structures and functions
CAN_TxHeaderTypeDef TxHeader = {0};
uint32_t TxMailbox = 0;

// Dummy delay function for simulation (does nothing)
void delay_us(uint16_t us) {
    (void)us; // No delay during simulation
}

// Dummy function to simulate CAN message transmission.
// Instead of sending the message, it prints the header and data.
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, 
										CAN_TxHeaderTypeDef *pHeader, 
										uint8_t aData[], 
										uint32_t *pTxMailbox) {
	(void)hcan;      // Unused in simulation
	(void)pTxMailbox; // Unused in simulation

	printf("Simulated CAN Tx: ID = %d, Data = ", pHeader->StdId);
	for (int i = 0; i < 8; i++) {
		printf("%02X ", aData[i]);
		}
	printf("\n");
	return HAL_OK;
}



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
#define TRQ_REQ_MAX 3.0f

// STM limits for P V T Kd Kp
#define P_MIN -12.5f
#define P_MAX  12.5f
#define V_MIN -26.0f
#define V_MAX  26.0f
#define T_MIN -1.5f
#define T_MAX  1.5f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

/// Joint Soft Stops
#define AB_LIM_P 	 1.5708f 	//90°
#define AB_LIM_N 	-1.5708f 	//-90°
#define HIP_LIM_P 	 2.0944f 	//120°
#define HIP_LIM_N 	-2.0944f 	//120°
#define KNEE_LIM_P	 4.01426f 		//5.7°
#define KNEE_LIM_N 	 -0.1f	//-230°
#define KP_SOFTSTOP  100.0f
#define KD_SOFTSTOP  0.4f



/****************************    HANDLING DEVIATIONS	**********************/
#define KNEE_GEARRATIO 1.5 //1.25 //todo for testing

/*
 *  STM 2 on Right/Front A[0]A[1]  a[0]a[1]  FR FL
 *  STM 1 on Left /Rear  A[2]A[3]  a[0]a[1]  RR RL
 */

 int ab_mitdirection[2]  ;
 int hip_mitdirection[2];
 int knee_mitdirection[2] ;



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





spi_tx 		state;
torque_rx 	torque;

//spi buffer
uint16_t spi_tx_buffer[TX_LEN];
uint16_t spi_rx_buffer[RX_LEN];

// CAN buffer
uint8_t CAN_TxData_buf[8];
uint8_t CAN_RxData_buf[8];

uint32_t 	currentControlMode = 99;

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

//Time
uint32_t time;
uint32_t time2;


uint32_t Error_spi;
uint32_t CallbackError_spi;
uint32_t State_spi;

// Define the `control` structure with hardcoded values
spi_rx control = {
    .ab_p 		= {1.5f, 1.6f},  // Abad motor positions (first and second index)
    .hip_p 		= {2.0f, 2.1f}, // Hip motor positions
    .knee_p 	= {0.8f, 0.9f}, // Knee motor positions
    
    .ab_v 		= {0.5f, 0.6f},  // Abad motor velocities
    .hip_v 		= {1.0f, 1.1f}, // Hip motor velocities
    .knee_v 	= {0.4f, 0.5f}, // Knee motor velocities
    
    .ab_kp 		= {100.0f, 110.0f}, // Abad motor stiffness (kp)
    .hip_kp 	= {120.0f, 130.0f}, // Hip motor stiffness
    .knee_kp 	= {90.0f, 95.0f},  // Knee motor stiffness
    
    .ab_kd 		= {0.2f, 0.3f}, // Abad motor damping (kd)
    .hip_kd 	= {0.4f, 0.5f}, // Hip motor damping
    .knee_kd 	= {0.1f, 0.15f}, // Knee motor damping
    
    .ab_t 		= {0.1f, 0.2f},  // Abad motor torque
    .hip_t 		= {0.3f, 0.4f},  // Hip motor torque
    .knee_t 	= {0.05f, 0.06f}, // Knee motor torque

    .flags 		= {0, 0},  // Initialize flags to 0
    .checksum 	= 0     // Initialize checksum to 0
};



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	
 ab_mitdirection[0]    =  1;
 hip_mitdirection[0] 	= -1;
 knee_mitdirection[0] 	= -1;
 
 
// Control Inputs (Slightly exceeding positive limits)
control.ab_p[CAN]   = AB_LIM_P - 0.1f;   // 12.6
control.hip_p[CAN]  = HIP_LIM_P - 0.1f;  // 15.1
control.knee_p[CAN] = KNEE_LIM_P + 0.2f; // 10.1

// Other control parameters can be set to nominal values
control.ab_v[CAN]   = 0.0f;
control.ab_kp[CAN]  = 100.0f;
control.ab_kd[CAN]  = 0.5f;
control.ab_t[CAN]   = 0.0f;

control.hip_v[CAN]  = 0.0f;
control.hip_kp[CAN] = 100.0f;
control.hip_kd[CAN] = 0.5f;
control.hip_t[CAN]  = 0.0f;

control.knee_v[CAN]  = 0.0f;
control.knee_kp[CAN] = 100.0f;
control.knee_kd[CAN] = 0.5f;
control.knee_t[CAN]  = 0.0f;

// Measured Outputs
p_out = 1.0f;  // Arbitrary measured position
v_out = 0.0f;  // Arbitrary measured velocity
t_out = 0.0f;  // Arbitrary measured torque




			
		// Set CAN index for Abad motor (0 or 1, depending on your configuration)
		CAN = 0;



        // printf("\n=====================================\n");
        // printf("Summary of Entered Values\n");
        // printf("=====================================\n");
    

		// printf("\n>>> Control Parameters Entered for Abad Motor:\n");
		// printf("Position (ab_p):        %.3f\n", control.ab_p[CAN]);
		// printf("Velocity (ab_v):        %.3f\n", control.ab_v[CAN]);
		// printf("Stiffness (ab_kp):      %.3f\n", control.ab_kp[CAN]);
		// printf("Damping (ab_kd):        %.3f\n", control.ab_kd[CAN]);
		// printf("Torque Feedforward:     %.3f\n", control.ab_t[CAN]);

		// printf("\n>>> Control Parameters Entered for Hip Motor:\n");
		// printf("Position (hip_p):       %.3f\n", control.hip_p[CAN]);
		// printf("Velocity (hip_v):       %.3f\n", control.hip_v[CAN]);
		// printf("Stiffness (hip_kp):     %.3f\n", control.hip_kp[CAN]);
		// printf("Damping (hip_kd):       %.3f\n", control.hip_kd[CAN]);
		// printf("Torque Feedforward:     %.3f\n", control.hip_t[CAN]);

		// printf("\n>>> Control Parameters Entered for Knee Motor:\n");
		// printf("Position (knee_p):      %.3f\n", control.knee_p[CAN]);
		// printf("Velocity (knee_v):      %.3f\n", control.knee_v[CAN]);
		// printf("Stiffness (knee_kp):    %.3f\n", control.knee_kp[CAN]);
		// printf("Damping (knee_kd):      %.3f\n", control.knee_kd[CAN]);
		// printf("Torque Feedforward:     %.3f\n", control.knee_t[CAN]);
	
	// printf("\n========================================\n");


    
        // printf("\n>>> Measured Values Entered:\n");
        // printf("Measured Position (p_out):  %.3f\n", p_out);
        // printf("Measured Velocity (v_out):  %.3f\n", v_out);
        // printf("Measured Torque (t_out):    %.3f\n", t_out);
        // printf("\n=====================================\n");


        // Only CAN
        can_send_receive();







}// end of main



							/***************************************************
							 *  				 C A N
							 ***************************************************/


void can_send_receive(){

	// Reset the flags before each CAN communication
	state.flags[0] = 0;
	state.flags[1] = 0;
    
	CAN=0;
	
	printf("\n\n\n             ABAD           \n");
	
		printf("\n>>> Control Parameters Entered for Abad Motor:\n");
		printf("Position (ab_p):        %.3f\n", control.ab_p[CAN]);
		printf("Velocity (ab_v):        %.3f\n", control.ab_v[CAN]);
		printf("Stiffness (ab_kp):      %.3f\n", control.ab_kp[CAN]);
		printf("Damping (ab_kd):        %.3f\n", control.ab_kd[CAN]);
		printf("Torque Feedforward:     %.3f\n", control.ab_t[CAN]);
		
    
        printf("\n>>> Measured Values Entered:\n");
        printf("Measured Position (p_out):  %.3f\n", p_out);
        printf("Measured Velocity (v_out):  %.3f\n", v_out);
        printf("Measured Torque (t_out):    %.3f\n", t_out);
        printf("\n=====================================\n");

		uint8_t CAN_RxData_buf[6] = {0x01, 0, 0, 0, 0, 0}; 
	receivedCanBus=0;
    CAN_RxData_buf[0] = Abad_CANID;
    unpack_replay(CAN_RxData_buf);
	pack_message(Abad_CANID, &TxHeader, CAN_TxData_buf);
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_TxData_buf, &TxMailbox);
    delay_us(300);
    
	
	
	
	// printf("\n\n\n             HIP           \n");
	
		// printf("\n>>> Control Parameters Entered for Hip Motor:\n");
		// printf("Position (hip_p):       %.3f\n", control.hip_p[CAN]);
		// printf("Velocity (hip_v):       %.3f\n", control.hip_v[CAN]);
		// printf("Stiffness (hip_kp):     %.3f\n", control.hip_kp[CAN]);
		// printf("Damping (hip_kd):       %.3f\n", control.hip_kd[CAN]);
		// printf("Torque Feedforward:     %.3f\n", control.hip_t[CAN]);
		
        // printf("\n>>> Measured Values Entered:\n");
        // printf("Measured Position (p_out):  %.3f\n", p_out);
        // printf("Measured Velocity (v_out):  %.3f\n", v_out);
        // printf("Measured Torque (t_out):    %.3f\n", t_out);
        // printf("\n=====================================\n");
		
    // CAN_RxData_buf[0] = Hip_CANID;
	    // unpack_replay(CAN_RxData_buf);
	// pack_message(Hip_CANID, &TxHeader, CAN_TxData_buf);
	// HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_TxData_buf, &TxMailbox);
    // delay_us(300);

	
	
	
	
	
	// printf("\n\n\n             KNEE           \n");
		// printf("\n>>> Control Parameters Entered for Knee Motor:\n");
		// printf("Position (knee_p):      %.3f\n", control.knee_p[CAN]);
		// printf("Velocity (knee_v):      %.3f\n", control.knee_v[CAN]);
		// printf("Stiffness (knee_kp):    %.3f\n", control.knee_kp[CAN]);
		// printf("Damping (knee_kd):      %.3f\n", control.knee_kd[CAN]);
		// printf("Torque Feedforward:     %.3f\n", control.knee_t[CAN]);
		
        // printf("\n>>> Measured Values Entered:\n");
        // printf("Measured Position (p_out):  %.3f\n", p_out);
        // printf("Measured Velocity (v_out):  %.3f\n", v_out);
        // printf("Measured Torque (t_out):    %.3f\n", t_out);
        // printf("\n=====================================\n");
		
    // CAN_RxData_buf[0] = Knee_CANID;
    // unpack_replay(CAN_RxData_buf);
	// pack_message(Knee_CANID, &TxHeader, CAN_TxData_buf);
	// HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_TxData_buf, &TxMailbox);
    // delay_us(300);


	// CAN=1;
	// pack_message(Abad_CANID, &TxHeader, CAN_TxData_buf);
	// HAL_CAN_AddTxMessage(&hcan2, &TxHeader, CAN_TxData_buf, &TxMailbox);
    // delay_us(300);
	// if (receivedCanBus==1){
	// 	unpack_replay(CAN_RxData_buf);
	// }
	// pack_message(Hip_CANID, &TxHeader, CAN_TxData_buf);
	// HAL_CAN_AddTxMessage(&hcan2, &TxHeader, CAN_TxData_buf, &TxMailbox);
    // delay_us(300);
	// if (receivedCanBus==1){
	// 	unpack_replay(CAN_RxData_buf);
	// }
	// pack_message(Knee_CANID, &TxHeader, CAN_TxData_buf);
	// HAL_CAN_AddTxMessage(&hcan2, &TxHeader, CAN_TxData_buf, &TxMailbox);
    // delay_us(300);
	// if (receivedCanBus==1){
	// 	unpack_replay(CAN_RxData_buf);
	// } 

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

        // Print the control values before applying soft stop and safety checks
		printf("\n=== Control Inputs for Abad Motor (Before Soft Stop & Safety Check) ===\n");
		printf("Position (p_in):        %.3f\n", p_in);
		printf("Velocity (v_in):        %.3f\n", v_in);
		printf("Stiffness (kp_in):      %.3f\n", kp_in);
		printf("Damping (kd_in):        %.3f\n", kd_in);
		printf("Torque Feedforward (t_in): %.3f\n", t_in);
		printf("\n");

		if(softstop_joint(&control.ab_p[CAN],state.ab_p[CAN],AB_LIM_P, AB_LIM_N))
		{	//Incase of wrong request
			state.flags[CAN] |= 0b01;
			p_in = p_in * ab_mitdirection[CAN]; // Direction update
			t_in = t_in * ab_mitdirection[CAN]; // Direction update
            printf(">>> Soft Stop Applied! Adjusted Values:\n");
            printf("Updated Position (p_in): %.3f\n", p_in);
            printf("Updated Torque (t_in):   %.3f\n", t_in);
            printf("\n");

		}


		// Safety Limit
		safetycheck_reqTrq(state.ab_p[CAN], state.ab_v[CAN], torque.ab_t[CAN]);

        // Print final control values after all adjustments
		printf("\n=== Final Control Values After Soft Stop & Safety Check ===\n");
		printf("Final Position (p_in):  %.3f\n", p_in);
		printf("Final Velocity (v_in):  %.3f\n", v_in);
		printf("Final Stiffness (kp_in): %.3f\n", kp_in);
		printf("Final Damping (kd_in):  %.3f\n", kd_in);
		printf("Final Torque (t_in):    %.3f\n", t_in);
		printf("\n=====================================\n");
	}
	if(ID == Hip_CANID)
	{
		p_in 	= (control.hip_p[CAN] * hip_mitdirection[CAN]);
		v_in 	= (control.hip_v[CAN] * hip_mitdirection[CAN]);
		kp_in	=  control.hip_kp[CAN];   //stifness
		kd_in	=  control.hip_kd[CAN];     //damper
		t_in	= (control.hip_t[CAN] * hip_mitdirection[CAN]);
		
		
        // Print the control values before applying soft stop and safety checks
		printf("\n=== Control Inputs for Abad Motor (Before Soft Stop & Safety Check) ===\n");
		printf("Position (p_in):        %.3f\n", p_in);
		printf("Velocity (v_in):        %.3f\n", v_in);
		printf("Stiffness (kp_in):      %.3f\n", kp_in);
		printf("Damping (kd_in):        %.3f\n", kd_in);
		printf("Torque Feedforward (t_in): %.3f\n", t_in);
		printf("\n");
		

		if(softstop_joint(&control.hip_p[CAN],state.hip_p[CAN], HIP_LIM_P, HIP_LIM_N))
		{	//Incase of wrong request
			state.flags[CAN] |= 0b10;
			p_in = p_in * hip_mitdirection[CAN]; // Direction update
			t_in = t_in * hip_mitdirection[CAN]; // Direction update
            printf(">>> Soft Stop Applied! Adjusted Values:\n");
            printf("Updated Position (p_in): %.3f\n", p_in);
            printf("Updated Torque (t_in):   %.3f\n", t_in);
            printf("\n");
		}

		// Safety Limit
		safetycheck_reqTrq(state.hip_p[CAN], state.hip_v[CAN], torque.hip_t[CAN]);
		
        // Print final control values after all adjustments
		printf("\n=== Final Control Values After Soft Stop & Safety Check ===\n");
		printf("Final Position (p_in):  %.3f\n", p_in);
		printf("Final Velocity (v_in):  %.3f\n", v_in);
		printf("Final Stiffness (kp_in): %.3f\n", kp_in);
		printf("Final Damping (kd_in):  %.3f\n", kd_in);
		printf("Final Torque (t_in):    %.3f\n", t_in);
		printf("\n=====================================\n");
	}
	if(ID == Knee_CANID)
	{
		p_in 	= (control.knee_p[CAN] * knee_mitdirection[CAN]) * KNEE_GEARRATIO;
		v_in 	= (control.knee_v[CAN] * knee_mitdirection[CAN]) / KNEE_GEARRATIO;
		kp_in 	=  control.knee_kp[CAN];   //stifness
		kd_in	=  control.knee_kd[CAN];     //damper
		t_in 	= (control.knee_t[CAN] * knee_mitdirection[CAN]) * KNEE_GEARRATIO;
		
		
        // Print the control values before applying soft stop and safety checks
		printf("\n=== Control Inputs for Abad Motor (Before Soft Stop & Safety Check) ===\n");
		printf("Position (p_in):        %.3f\n", p_in);
		printf("Velocity (v_in):        %.3f\n", v_in);
		printf("Stiffness (kp_in):      %.3f\n", kp_in);
		printf("Damping (kd_in):        %.3f\n", kd_in);
		printf("Torque Feedforward (t_in): %.3f\n", t_in);
		printf("\n");
		

		if(softstop_joint(&control.knee_p[CAN], state.knee_p[CAN], KNEE_LIM_P, KNEE_LIM_N))
		{	//Incase of wrong request
			state.flags[CAN] |= 0b11;
			p_in = (p_in * knee_mitdirection[CAN]) * KNEE_GEARRATIO; // Direction update
			t_in = (t_in * knee_mitdirection[CAN]) * KNEE_GEARRATIO; // Direction update
            printf(">>> Soft Stop Applied! Adjusted Values:\n");
            printf("Updated Position (p_in): %.3f\n", p_in);
            printf("Updated Torque (t_in):   %.3f\n", t_in);
            printf("\n");
		}

		// Safety Limit
		safetycheck_reqTrq(state.knee_p[CAN], state.knee_v[CAN], torque.knee_t[CAN]);
		
        // Print final control values after all adjustments
		printf("\n=== Final Control Values After Soft Stop & Safety Check ===\n");
		printf("Final Position (p_in):  %.3f\n", p_in);
		printf("Final Velocity (v_in):  %.3f\n", v_in);
		printf("Final Stiffness (kp_in): %.3f\n", kp_in);
		printf("Final Damping (kd_in):  %.3f\n", kd_in);
		printf("Final Torque (t_in):    %.3f\n", t_in);
		printf("\n=====================================\n");
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
	// /// unpack ints from can buffer ///
    uint16_t id = Data[0];
	uint16_t p_int = (Data[1]<<8)|Data[2];
	uint16_t v_int = (Data[3]<<4)|(Data[4]>>4);
	uint16_t i_int = ((Data[4]&0xF)<<8)|Data[5];
	//// convert uints to floats ///
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

    printf("\n\n\n=== READING State and Torque Values ===\n");

	if (id == Abad_CANID) {
		printf("\n>>> Abad Motor State & Torque:\n");
		printf("State Position (ab_p):  %.3f\n", state.ab_p[receivedCanBus]);
		printf("State Velocity (ab_v):  %.3f\n", state.ab_v[receivedCanBus]);
		printf("Torque (ab_t):          %.3f\n", torque.ab_t[receivedCanBus]);
	} 
	else if (id == Hip_CANID) {
		printf("\n>>> Hip Motor State & Torque:\n");
		printf("State Position (hip_p):  %.3f\n", state.hip_p[receivedCanBus]);
		printf("State Velocity (hip_v):  %.3f\n", state.hip_v[receivedCanBus]);
		printf("Torque (hip_t):          %.3f\n", torque.hip_t[receivedCanBus]);
	} 
	else if (id == Knee_CANID) {
		printf("\n>>> Knee Motor State & Torque:\n");
		printf("State Position (knee_p):  %.3f\n", state.knee_p[receivedCanBus]);
		printf("State Velocity (knee_v):  %.3f\n", state.knee_v[receivedCanBus]);
		printf("Torque (knee_t):          %.3f\n", torque.knee_t[receivedCanBus]);
	} 
	else {
		printf("\n>>> ERROR: Invalid Motor ID Received! ID: %d\n", id);
	}
	printf("\n================================\n");


}



////////////////////////softstop//////////////////////////////////
//actuator only can move to a limit position

int softstop_joint(float *control,float state, float limit_p, float limit_n)
{
	if(*control>=limit_p)
	{
		//*control = limit_p;
		p_in = limit_p;
		v_in = 0.0f;
		kp_in = 0.0f;
		kd_in = KD_SOFTSTOP;
		t_in += KP_SOFTSTOP*(limit_p - state);
		return 1;
	}
	if(*control<=limit_n)
	{
		//*control = limit_n;
		p_in = limit_n;
		v_in = 0.0f;
		kp_in = 0.0f;
		kd_in = KD_SOFTSTOP;
		t_in += KP_SOFTSTOP*(limit_n - state);
		return 1;
	}

  return 0;
}


////////////////////////safetycheck_reqTrq//////////////////////////////////
//To add additional check on the torque requested to the motor. Calculated based on the PID model of motor.
void safetycheck_reqTrq(float p_act, float v_act, float t_ff)
{
	// Print current values and calculated torque request
	printf("\n=== Torque Request Safety Check ===\n");
	printf("Actual Position (p_act): %.3f\n", p_act);
	printf("Actual Velocity (v_act): %.3f\n", v_act);
	printf("Torque Feedforward (t_ff): %.3f\n", t_ff);

	float trqreq = (p_in - p_act)*kp_in + (v_in - v_act)*kd_in + t_ff;



	// Print equation format with values
	printf("\n=== Torque Request Calculation ===\n");
	printf("Values:   (%.3f - %.3f) * %.3f + (%.3f - %.3f) * %.3f + %.3f\n",
		   p_in, p_act, kp_in, v_in, v_act, kd_in, t_ff);

	// Compute intermediate results
	float position_term = (p_in - p_act) * kp_in;
	float velocity_term = (v_in - v_act) * kd_in;

	// Print step-by-step additions
	printf("Steps:    %.3f + %.3f + %.3f = %.3f\n",
       position_term, velocity_term, t_ff, trqreq);


    printf("Torque request is   %.3f\n", trqreq);

	// Incase the Trq to be calculated at the motor is too high, cancel the req by setting everything to zero
	if (trqreq >= TRQ_REQ_MAX || trqreq <= -TRQ_REQ_MAX)
	{
        printf("\n>>> Control Inputs Reset Due to Excessive Torque Request:\n");
		p_in = 0.0f;
		v_in = 0.0f;
		kp_in = 0.0f;
		kd_in = 0.0f;
		t_in = 0.0f;
	}
	    // Else, limit only the torque (`t_in`) to max or min
    else if (t_in >= TRQ_REQ_MAX || t_in <= -TRQ_REQ_MAX) {
        printf(">>> Torque request is high but within limits. Clamping `t_in`.\n");

        // Limit `t_in` to max/min allowed torque
        if (t_in > TRQ_REQ_MAX) {
            t_in = TRQ_REQ_MAX;
        } else if (t_in < -TRQ_REQ_MAX) {
            t_in = -TRQ_REQ_MAX;
        }

        // Print updated torque value
        printf("Updated Torque (t_in): %.3f\n", t_in);
    } 
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



