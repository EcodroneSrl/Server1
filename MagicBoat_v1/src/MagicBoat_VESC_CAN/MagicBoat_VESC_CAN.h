#ifndef MagicBoat_VESC_CAN_h
#define MagicBoat_VESC_CAN_h

#include "Arduino.h"
#include "FlexCAN_T4.h"
#include "KalmanOneFlo.h"

#define SerialDeb Serial

#define DESCR_ON						1
#define HEAD_ON							1

#define DESCR_OFF						0
#define HEAD_OFF						0

#define CURRENT         				0x01
#define ERPM            				0x03

#define STATUS_PACKET_ONE				0x09
#define STATUS_PACKET_TWO				0x0E
#define STATUS_PACKET_THREE				0x0F
#define STATUS_PACKET_FOUR				0x10
#define STATUS_PACKET_FIVE				0x1B

#define AVG_SIZE						100

// Setting Type definitions
#define TYPE_CURRENT					0x01
#define TYPE_ERPM						0x03

// Defines for average
#define MOTOR_CURRENT					0
#define CURRENT_IN						1

#define CAN_BPS             			1000000



class MagicBoat_VESC_CAN
{
	private:

	KalmanOneFlo f_motor_current;     //istanza per una singola misura 
	KalmanOneFlo f_rpm;     //istanza per una singola misura
	
	float qn_i = 0.561858359; //Coovarianza del rumore di processo(Cercate di capire di quanto si discosta la misura rispetto alle condizioni al contorno di quello che misurate)process noise covariance
	float rn_i = 20; //Coovarianza del rumore di misura(cercate di capire quanto si discosta la misura dal valore che volete misurare)measurement noise covariance
	float xn_i = 0;    //valore iniziale , messo a zero la stima convergerà lentamente al valore della misura. value
	float pn_i = 1;    //Coovarianza dell'errore stimato. Si aggiorna da sola quindi un valore vale laltro a meno di valori strampalati. estimation error covariance
	float kn_i = 0.1;  //Guadagno di kalman, si aggiorna da solo, ma un guadagno alto fa andare la stima a stimare il rumore. kalman gain
	
	float qn_r = 32.01708617; //Coovarianza del rumore di processo(Cercate di capire di quanto si discosta la misura rispetto alle condizioni al contorno di quello che misurate)process noise covariance
	float rn_r = 300; //Coovarianza del rumore di misura(cercate di capire quanto si discosta la misura dal valore che volete misurare)measurement noise covariance
	float xn_r = 0;    //valore iniziale , messo a zero la stima convergerà lentamente al valore della misura. value
	float pn_r = 1;    //Coovarianza dell'errore stimato. Si aggiorna da sola quindi un valore vale laltro a meno di valori strampalati. estimation error covariance
	float kn_r = 0.1;  //Guadagno di kalman, si aggiorna da solo, ma un guadagno alto fa andare la stima a stimare il rumore. kalman gain

	
	public:
	//Variabili
	
	FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can1;
	
	uint8_t baudrate;
	uint8_t a_CAN_num;
	uint8_t a_CAN_bps;
	uint8_t motorID;
	
	uint32_t time_CAN_stat, time_request, time_CANBuffer;
	uint32_t time_update_tel;
	uint16_t index1 = 0;
	uint16_t index2 = 0;
	float motor_curr_array[AVG_SIZE];
	float curr_in_array[AVG_SIZE];
	float sum0;
	float sum1;
	
	// Status variables
	
	// Packet 1
	int32_t RPM;
	int32_t fk_RPM;
	float current;
	float duty;
	float fk_current;
	float avg_current;
	// Packet 2
	float amp_hours;
	float amp_hours_charged;
	// Packet 3
	float watt_hours;
	float watt_hours_charged;
	// Packet 4
	float temp_fet;
	float temp_motor;
	uint16_t current_in;
	float pid_pos_now;
	float avg_current_in;
	// Packet 5
	float tacho_value;
	float v_in;
	
	// Variabili long buffer
	uint8_t tel_1[8];
	uint8_t tel_2[8];
	uint8_t tel_3[8];
	uint8_t tel_4[8];
	uint8_t tel_5[8];
	uint8_t tel_6[8];
	uint8_t tel_7[8];
	uint8_t tel_8[8];
	uint8_t tel_9[8];
	uint8_t tel_10[8];
	uint8_t tel_11[8];
	
	
	int32_t fk_rpm;
	float fk_motor_current;
	
	struct __attribute__((packed)) bufferTel
	{
		float temp_fet1;
		float temp_motor1;
		float avg_motor_current;
		float avg_input_current;
		float i_D;
		float i_Q;
		float duty_cycle;
		int32_t rpm;
		float voltage;
		float amp_h;
		float amp_h_ch;
		float watt_h;
		float watt_h_ch;
		uint32_t odometer;
		uint32_t odometer_abs;
		float fault;
		float pid_pos;
		float controller_id;
		float mostemp1;
		float mostemp2;
		float mostemp3;
		float vd;
		float vq;
	} bufferTelData;
	
	// Metodi
	
	MagicBoat_VESC_CAN();
	void begin(uint8_t IDMotor);
	void initCAN();
	
	void writeCAN(uint8_t type, int32_t value);
	void printCANrx();
	
	void setCurrent(int32_t value);
	void setERPM(int32_t value);
	
	uint32_t getu32buffer(CAN_message_t msg, uint8_t start_byte);
	uint16_t getu16buffer(CAN_message_t msg, uint8_t start_byte);
	int32_t get32buffer(CAN_message_t msg, uint8_t start_byte);
	int16_t get16buffer(CAN_message_t msg, uint8_t start_byte);
	
	
	void sendTelemetryRequest(uint32_t timer_request);
	void sendRequest();
	uint8_t saveLongBuffer(CAN_message_t msg);
	uint8_t parseLongBuffer(CAN_message_t msg);
	
	uint8_t updateCANStatus();
	uint8_t updateCAN_Tel();
	void setTelTime(uint32_t time_tel);
	// void printCAN_Status(uint8_t description, uint16_t timer_CAN_stat, uint8_t update, uint8_t head);
	// void printCAN_Buffer(uint8_t description, uint16_t timer_CAN_stat, uint8_t update, uint8_t head);
	float shift_buffer(uint8_t current_type);
	void average(uint8_t current_type);
};

#endif