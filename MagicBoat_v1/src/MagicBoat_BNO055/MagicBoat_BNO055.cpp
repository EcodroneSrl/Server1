#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "MagicBoat_BNO055.h"

bool MagicBoat_BNO055::begin()
{ 
	Wire.begin(); //setup dell'I2C
	// Wire.setClock(400000); // I2C clock rate ,You can delete it but it helps the speed of I2C (default rate is 100000 Hz)
	return true;
}

// Legge la versione del firmware attualmente flashata
void MagicBoat_BNO055::getFWVersion()
{
  uint8_t LSBfw = readRegisterU8(BNO055_SW_REV_ID_LSB_ADDR);
  uint8_t MSBfw = readRegisterU8(BNO055_SW_REV_ID_MSB_ADDR);
  SerialDeb.print(MSBfw,HEX);
  SerialDeb.print(".");
  SerialDeb.println(LSBfw,HEX);
}

// Entra in config mode e aspetta 100ms per non avere errori nelle letture dati successive
int8_t MagicBoat_BNO055::set_config_mode()
{
	//Entering BNO055 configuration mode
	writeRegister8(BNO055_OPR_MODE_ADDR, CONFIG_MODE);
	delay(100);
	return readRegister8(BNO055_OPR_MODE_ADDR);
}

// Setta la power mode desiderata
uint8_t MagicBoat_BNO055::set_power_mode(uint8_t pw_mode)
{
	writeRegister8(BNO055_PWR_MODE_ADDR, pw_mode);
	return readRegister8(BNO055_PWR_MODE_ADDR);
}

// Setta la operation mode desiderata e aspetta 100ms per non avere errori nelleletture dati successive
uint8_t MagicBoat_BNO055::set_operation_mode(uint8_t op_mode)
{
	writeRegister8(BNO055_OPR_MODE_ADDR, op_mode);
	delay(100);
	return readRegisterU8(BNO055_OPR_MODE_ADDR);
}

// Legge l'attuale operation mode e la salva per poterla riattivare dopo aver usato la config mode 
uint8_t MagicBoat_BNO055::get_operation_mode()
{
	return readRegisterU8(BNO055_OPR_MODE_ADDR); 
}

// Configurazione iniziale di pwrmode e opmode per il sensore di indirizzo bnoAddress
bool MagicBoat_BNO055::configure(uint8_t pw_mode, uint8_t op_mode, uint8_t bnoAddress)
{
	
	//Entering BNO055 configuration mode
	set_config_mode();
	
	//Setting BNO055 Power Mode
	a_pw_mode = set_power_mode(pw_mode);

	//Setting BNO055 Operation Mode
	a_op_mode = set_operation_mode(op_mode);
	
	return true;
}

// Functions to select Register Page (Page0 or Page1)

void MagicBoat_BNO055::setPage0()
{
	writeRegister8(BNO055_Page_ID_ADDR, PAGE0);
}

void MagicBoat_BNO055::setPage1()
{
	writeRegister8(BNO055_Page_ID_ADDR, PAGE1);
}

// Function to write stored values into calibration registers

int16_t MagicBoat_BNO055::setCal(uint8_t regLSB, uint8_t regMSB, int16_t value) {
	uint8_t LSB=(value&(0X00FF));
	uint8_t MSB=(value >> 8);
	writeRegister8(regLSB, LSB);
	writeRegister8(regMSB, MSB);
	return value;
}

int16_t MagicBoat_BNO055::set_cal_ACC_OFFSET_X(int16_t value) 
{
	return setCal(ACC_OFFSET_X_LSB_ADDR, ACC_OFFSET_X_MSB_ADDR, value); 
}

int16_t MagicBoat_BNO055::set_cal_ACC_OFFSET_Y(int16_t value) {
	return setCal(ACC_OFFSET_Y_LSB_ADDR, ACC_OFFSET_Y_MSB_ADDR, value);
}

int16_t MagicBoat_BNO055::set_cal_ACC_OFFSET_Z(int16_t value) {
	return setCal(ACC_OFFSET_Z_LSB_ADDR, ACC_OFFSET_Z_MSB_ADDR, value); 
}

int16_t MagicBoat_BNO055::set_cal_MAG_OFFSET_X(int16_t value) {
	return setCal(MAG_OFFSET_X_LSB_ADDR, MAG_OFFSET_X_MSB_ADDR, value); 
}

int16_t MagicBoat_BNO055::set_cal_MAG_OFFSET_Y(int16_t value) {
	return setCal(MAG_OFFSET_Y_LSB_ADDR, MAG_OFFSET_Y_MSB_ADDR, value); 
}

int16_t MagicBoat_BNO055::set_cal_MAG_OFFSET_Z(int16_t value) {
	return setCal(MAG_OFFSET_Z_LSB_ADDR, MAG_OFFSET_Z_MSB_ADDR, value); 
}

int16_t MagicBoat_BNO055::set_cal_GYR_OFFSET_X(int16_t value) {
	return setCal(GYR_OFFSET_X_LSB_ADDR, GYR_OFFSET_X_MSB_ADDR, value); 
}

int16_t MagicBoat_BNO055::set_cal_GYR_OFFSET_Y(int16_t value) {
	return setCal(GYR_OFFSET_Y_LSB_ADDR, GYR_OFFSET_Y_MSB_ADDR, value); 
}

int16_t MagicBoat_BNO055::set_cal_GYR_OFFSET_Z(int16_t value) {
	return setCal(GYR_OFFSET_Z_LSB_ADDR, GYR_OFFSET_Z_MSB_ADDR, value); 
}

int16_t MagicBoat_BNO055::set_cal_ACC_RADIUS(int16_t value) {
	return setCal(ACC_RADIUS_LSB_ADDR, ACC_RADIUS_MSB_ADDR, value); 
}

int16_t MagicBoat_BNO055::set_cal_MAG_RADIUS(int16_t value) {
	return setCal(MAG_RADIUS_LSB_ADDR, MAG_RADIUS_MSB_ADDR, value); 
}


// Funzione che scrive i valori degli offset di calibrazione //
// contenuti in una struct nei rispettivi registri della IMU //

int16_t MagicBoat_BNO055::setCalAll() 
{
	uint8_t mode = get_operation_mode();
	set_config_mode();
	
	set_cal_ACC_OFFSET_X(cal_values.acc_off_x);
	set_cal_ACC_OFFSET_Y(cal_values.acc_off_y);
	set_cal_ACC_OFFSET_Z(cal_values.acc_off_z);
	
	set_cal_MAG_OFFSET_X(cal_values.mag_off_x);
	set_cal_MAG_OFFSET_Y(cal_values.mag_off_y);
	set_cal_MAG_OFFSET_Z(cal_values.mag_off_z);
	
	set_cal_GYR_OFFSET_X(cal_values.gyr_off_x);
	set_cal_GYR_OFFSET_X(cal_values.gyr_off_y);
	set_cal_GYR_OFFSET_X(cal_values.gyr_off_z);
	
	set_cal_ACC_RADIUS(cal_values.acc_rad);
	set_cal_MAG_RADIUS(cal_values.mag_rad);
	
	set_operation_mode(mode);
	
	return 1;
}

// Functions to write configurations for sensors
// Switchano alla Page1 di registri e poi ritornano alla Page0

void MagicBoat_BNO055::set_configuration_Acc(uint8_t range, uint8_t bw, uint8_t o_mode)
{
	uint8_t bno_mode = get_operation_mode();
	
	if ((bno_mode == NDOF) | (bno_mode == NDOF_FMC_OFF) | (bno_mode == IMU) | (bno_mode == M4G) | (bno_mode == COMPASS))
	{
		SerialDeb.println("Can't set these parameters in Non-Fusion modes");
	}
	else 
	{
		setPage1();
		set_config_mode();
		uint8_t conf_bit;
		conf_bit = (o_mode << 5) | (bw << 2) | (range);	
		writeRegister8(ACC_CONFIG_ADDR, conf_bit);
		set_operation_mode(o_mode);
		setPage0();
	}
}

void MagicBoat_BNO055::set_configuration_Gyr(uint8_t range, uint8_t bw, uint8_t o_mode)
{
	uint8_t bno_mode = get_operation_mode();
	
	if ((bno_mode == NDOF) | (bno_mode == NDOF_FMC_OFF) | (bno_mode == IMU) | (bno_mode == M4G) | (bno_mode == COMPASS))
	{
		SerialDeb.println("Can't set these parameters in Non-Fusion modes");
	}
	else 
	{
		setPage1();
		set_config_mode();
		uint8_t conf_bit1;
		uint8_t conf_bit2;
		conf_bit2 = o_mode;
		conf_bit1 = (bw << 3) | (range);	
		writeRegister8(GYR_CONFIG_ADDR, conf_bit1);
		writeRegister8(GYR_MODE_CONFIG_ADDR, conf_bit2);
		set_operation_mode(o_mode);
		setPage0();
	}
}

void MagicBoat_BNO055::set_configuration_Mag(uint8_t data_rate, uint8_t o_mode, uint8_t p_mode)
{
	uint8_t bno_mode = get_operation_mode();
	
	if ((bno_mode == NDOF) | (bno_mode == NDOF_FMC_OFF) | (bno_mode == IMU) | (bno_mode == M4G) | (bno_mode == COMPASS))
	{
		SerialDeb.println("Can't set these parameters in Non-Fusion modes");
	}
	else 
	{
		setPage1();
		set_config_mode();
		uint8_t conf_bit;
		conf_bit = (p_mode << 5) | (o_mode << 3) | (data_rate);	
		writeRegister8(MAG_CONFIG_ADDR, conf_bit);
		set_operation_mode(o_mode);
		setPage0();
	}
}

//Function to enable interrupt and masking

void MagicBoat_BNO055::set_interrupt()
{
	uint8_t mode = get_operation_mode();
	setPage1();
	set_config_mode();	
	writeRegister8(INT_ADDR, FUSION_DRDY_INT_EN);
	writeRegister8(INT_MSK_ADDR, FUSION_DRDY_MSK_EN);
	set_operation_mode(mode);
	setPage0();
}

//Function to read calibration registers values

int16_t MagicBoat_BNO055::getCal(uint8_t regLSB, uint8_t regMSB) 
{
	int16_t value;
	uint8_t LSB=readRegisterU8(regLSB);
	uint8_t MSB=readRegisterU8(regMSB);
	value = (MSB << 8) | (LSB & 0xff);
	return value;
}

// Funzione che legge gli attuali valori di calibrazione e li scrive in una struct

void MagicBoat_BNO055::getCalAll()
{	
		uint8_t mode = get_operation_mode();
		set_config_mode();

		cal_values.acc_off_x = get_cal_ACC_OFFSET_X();
		cal_values.acc_off_y = get_cal_ACC_OFFSET_Y();
		cal_values.acc_off_z = get_cal_ACC_OFFSET_Z();
		cal_values.mag_off_x = get_cal_MAG_OFFSET_X();
		cal_values.mag_off_y = get_cal_MAG_OFFSET_Y();
		cal_values.mag_off_z = get_cal_MAG_OFFSET_Z();
		cal_values.gyr_off_x = get_cal_GYR_OFFSET_X();
		cal_values.gyr_off_y = get_cal_GYR_OFFSET_Y();
		cal_values.gyr_off_z = get_cal_GYR_OFFSET_Z();
		cal_values.acc_rad = get_cal_ACC_RADIUS();
		cal_values.mag_rad = get_cal_MAG_RADIUS();

		set_operation_mode(mode);
}

int16_t MagicBoat_BNO055::get_cal_ACC_OFFSET_X()
{
	return getCal(ACC_OFFSET_X_LSB_ADDR, ACC_OFFSET_X_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_cal_ACC_OFFSET_Y()
{
	return getCal(ACC_OFFSET_Y_LSB_ADDR, ACC_OFFSET_Y_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_cal_ACC_OFFSET_Z()
{
	return getCal(ACC_OFFSET_Z_LSB_ADDR, ACC_OFFSET_Z_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_cal_MAG_OFFSET_X()
{
	return getCal(MAG_OFFSET_X_LSB_ADDR, MAG_OFFSET_X_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_cal_MAG_OFFSET_Y()
{
	return getCal(MAG_OFFSET_Y_LSB_ADDR, MAG_OFFSET_Y_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_cal_MAG_OFFSET_Z()
{
	return getCal(MAG_OFFSET_Z_LSB_ADDR, MAG_OFFSET_Z_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_cal_GYR_OFFSET_X()
{
	return getCal(GYR_OFFSET_X_LSB_ADDR, GYR_OFFSET_X_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_cal_GYR_OFFSET_Y()
{
	return getCal(GYR_OFFSET_Y_LSB_ADDR, GYR_OFFSET_Y_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_cal_GYR_OFFSET_Z()
{
	return getCal(GYR_OFFSET_Z_LSB_ADDR, GYR_OFFSET_Z_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_cal_ACC_RADIUS()
{
	return getCal(ACC_RADIUS_LSB_ADDR, ACC_RADIUS_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_cal_MAG_RADIUS()
{
	return getCal(MAG_RADIUS_LSB_ADDR, MAG_RADIUS_MSB_ADDR); 
}


/*---------- Functions to read sensor configurations in Non-Fusion Modes ---------------------------- */

int8_t MagicBoat_BNO055::get_ACC_conf()
{
	setPage1();
	int8_t value = readRegisterU8(ACC_CONFIG_ADDR);
	setPage0();
	return value;
}

int8_t MagicBoat_BNO055::get_GYR_conf1()
{
	setPage1();
	int8_t value = readRegisterU8(GYR_CONFIG_ADDR);
	setPage0();
	return value;
}

int8_t MagicBoat_BNO055::get_GYR_conf2()
{
	setPage1();
	int8_t value = readRegisterU8(GYR_MODE_CONFIG_ADDR);
	setPage0();
	return value;
}

int8_t MagicBoat_BNO055::get_MAG_conf()
{
	setPage1();
	int8_t value = readRegisterU8(MAG_CONFIG_ADDR);
	setPage0();
	return value;
}

/*-------------------- Functions to read output data ---------------------------- */

int16_t MagicBoat_BNO055::getData(uint8_t regLSB, uint8_t regMSB) 
{
	int16_t value;
	uint8_t LSB=readRegisterU8(regLSB);
	uint8_t MSB=readRegisterU8(regMSB);
	value = (MSB << 8) | (LSB & 0xff);
	return value;
}

float MagicBoat_BNO055::conv_float(int16_t value, float div) 
{
	float fvalue = float(value)/div;
	return fvalue;
}

//Euler angles

float MagicBoat_BNO055::range_180(float value) 
{
	if(value>=180.0000)
	{
		value=value-360.0000;
    }
	return value;
}

int16_t MagicBoat_BNO055::get_data_EUL_HEADING() 
{
	return getData(BNO055_EUL_HEADING_LSB_ADDR, BNO055_EUL_HEADING_MSB_ADDR); 
}

int16_t MagicBoat_BNO055::get_data_EUL_ROLL() 
{
	return getData(BNO055_EUL_ROLL_LSB_ADDR, BNO055_EUL_ROLL_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_EUL_PITCH() 
{
	return getData(BNO055_EUL_PITCH_LSB_ADDR, BNO055_EUL_PITCH_MSB_ADDR);
}

// Acceleration vectors

int16_t MagicBoat_BNO055::get_data_ACC_DATA_X()
{
	return getData(BNO055_ACC_DATA_X_LSB_ADDR, BNO055_ACC_DATA_X_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_ACC_DATA_Y()
{
	return getData(BNO055_ACC_DATA_Y_LSB_ADDR, BNO055_ACC_DATA_Y_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_ACC_DATA_Z()
{
	return getData(BNO055_ACC_DATA_Z_LSB_ADDR, BNO055_ACC_DATA_Z_MSB_ADDR);
}

// Magnetometer vectors

int16_t MagicBoat_BNO055::get_data_MAG_DATA_X()
{
	return getData(BNO055_MAG_DATA_X_LSB_ADDR, BNO055_MAG_DATA_X_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_MAG_DATA_Y()
{
	return getData(BNO055_MAG_DATA_Y_LSB_ADDR, BNO055_MAG_DATA_Y_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_MAG_DATA_Z()
{
	return getData(BNO055_MAG_DATA_Z_LSB_ADDR, BNO055_MAG_DATA_Z_MSB_ADDR);
}

// Gyroscope vectors

int16_t MagicBoat_BNO055::get_data_GYR_DATA_X()
{
	return getData(BNO055_GYR_DATA_X_LSB_ADDR, BNO055_GYR_DATA_X_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_GYR_DATA_Y()
{
	return getData(BNO055_GYR_DATA_Y_LSB_ADDR, BNO055_GYR_DATA_Y_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_GYR_DATA_Z()
{
	return getData(BNO055_GYR_DATA_Z_LSB_ADDR, BNO055_GYR_DATA_Z_MSB_ADDR);
}

// Quaternions

int16_t MagicBoat_BNO055::get_data_QUA_DATA_W()
{
	return getData(BNO055_QUA_DATA_W_LSB_ADDR, BNO055_QUA_DATA_W_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_QUA_DATA_X()
{
	return getData(BNO055_QUA_DATA_X_LSB_ADDR, BNO055_QUA_DATA_X_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_QUA_DATA_Y()
{
	return getData(BNO055_QUA_DATA_Y_LSB_ADDR, BNO055_QUA_DATA_Y_MSB_ADDR);
}

int16_t MagicBoat_BNO055::get_data_QUA_DATA_Z()
{
	return getData(BNO055_QUA_DATA_Z_LSB_ADDR, BNO055_QUA_DATA_Z_MSB_ADDR);
}

// Funziona che aggiorna periodicamente i dati dell'IMU con tempi
// persoalizzabili per ogni variabile, contenuti in update_timer[]

void MagicBoat_BNO055::update()
{
	for(int i = 0; i < 10; i++)
	{
		if(update_par.update_enable[i])
		{
			polling(i, update_par.update_timer[i]);
		}
	}
}

void MagicBoat_BNO055::polling(uint8_t i, uint16_t update_timer)
{
	if((millis() - update_time[i]) > update_timer) 
	{
		update_time[i] = millis();
		
		switch(i)
		{
			case UPD_CALIB_STAT:
				update_calib_stat();
			break;
			
			case UPD_CALIB_OFFSET:
				getCalAll();
			break;
			
			case UPD_ACC_CONF:
				update_Acc_conf();
			break;
			
			case UPD_GYR_CONF:
				update_Gyr_conf();
			break;
			
			case UPD_MAG_CONF:
				update_Mag_conf();
			break;
			
			case UPD_EULER:
				update_Euler();
			break;
			
			case UPD_ACC:
				update_Acc();
			break;
			
			case UPD_GYR:
				update_Gyr();
			break;
			
			case UPD_MAG:
				update_Mag();
			break;
			
			case UPD_QUA:
				update_Qua();
			break;
		}
	}
}

// Function to update calibration status variables

void MagicBoat_BNO055::update_calib_stat()
{
	uint8_t cal_byte = readRegisterU8(BNO055_CALIB_STAT_ADDR);
	mag_cal=(cal_byte & 0b00000011);
	acc_cal=(cal_byte >> 2 & 0b00000011);
	gyr_cal=(cal_byte >> 4 & 0b00000011);
	sys_cal=(cal_byte >> 6 & 0b00000011);
}

// Function to update calibration offset variables
// Commentata perchè c'è quella uguale che salva direttamente
// nella stuct (getCalAll)

/* void MagicBoat_BNO055::update_calib_offset()
{		 
	uint8_t mode = get_operation_mode();
	set_config_mode();

	acc_off_x = get_cal_ACC_OFFSET_X();
	acc_off_y = get_cal_ACC_OFFSET_Y();    
	acc_off_z = get_cal_ACC_OFFSET_Z(); 

	mag_off_x = get_cal_MAG_OFFSET_X();    
	mag_off_y = get_cal_MAG_OFFSET_Y();    
	mag_off_z = get_cal_MAG_OFFSET_Z();  

	gyr_off_x = get_cal_GYR_OFFSET_X();    
	gyr_off_y = get_cal_GYR_OFFSET_Y();    
	gyr_off_z = get_cal_GYR_OFFSET_Z();  

	acc_rad = get_cal_ACC_RADIUS();    
	mag_rad = get_cal_MAG_RADIUS();


	set_operation_mode(mode);
	 
	 
} */

// Functions to update configuration sensor variables

void MagicBoat_BNO055::update_Acc_conf()
{
	uint8_t value = get_ACC_conf();
	acc_range = value & (0x03);
	acc_bw = (value >> 2) & (0x07);
	acc_op_mode = (value >> 5) & (0x07);
}

void MagicBoat_BNO055::update_Gyr_conf()
{
	uint8_t value1 = get_GYR_conf1();
	gyr_range = value1 & (0x07);
	gyr_bw = (value1 >> 3) & (0x07);
	uint8_t value2 = get_GYR_conf2();
	gyr_op_mode = value2 & (0x07);
}

void MagicBoat_BNO055::update_Mag_conf()
{
	uint8_t value = get_MAG_conf();
	mag_data_rate = value & (0x07);
	mag_op_mode = (value >> 3) & (0x03);
	acc_op_mode = (value >> 5) & (0x03);
}

// Functions to update output data variables

void MagicBoat_BNO055::update_Euler()

{
	EulerAccData.Yaw = range_180(conv_float(get_data_EUL_HEADING(), 16.00));
	EulerAccData.Roll = conv_float(get_data_EUL_ROLL(), 16.00);
	EulerAccData.Pitch = conv_float(get_data_EUL_PITCH(), 16.00);
}

void MagicBoat_BNO055::update_Acc()
{
	EulerAccData.Ax = conv_float(get_data_ACC_DATA_X(), 100.00);
	EulerAccData.Ay = conv_float(get_data_ACC_DATA_Y(), 100.00);
	EulerAccData.Az = conv_float(get_data_ACC_DATA_Z(), 100.00);
}

void MagicBoat_BNO055::update_Mag()
{
	Mx = conv_float(get_data_MAG_DATA_X(), 16.00);
	My = conv_float(get_data_MAG_DATA_Y(), 16.00);
	Mz = conv_float(get_data_MAG_DATA_Z(), 16.00);
}

void MagicBoat_BNO055::update_Gyr()
{
	Gx = conv_float(get_data_GYR_DATA_X(), 16.00);
	Gy = conv_float(get_data_GYR_DATA_Y(), 16.00);
	Gz = conv_float(get_data_GYR_DATA_Z(), 16.00);
}

void MagicBoat_BNO055::update_Qua()
{
	q0 = (int16_t)get_data_QUA_DATA_W()/(pow(2,14));
	q1 = (int16_t)get_data_QUA_DATA_X()/(pow(2,14));
	q2 = (int16_t)get_data_QUA_DATA_Y()/(pow(2,14));
	q3 = (int16_t)get_data_QUA_DATA_Z()/(pow(2,14));
}

// ------------------- Functions to read/write registers --------------------------------


void MagicBoat_BNO055::writeRegister16(uint8_t reg, uint16_t val)
{
    Wire.beginTransmission(bnoAddress);  		// Address the I2C device
    Wire.write(reg);                        	// Send register address to write
    Wire.write((uint8_t)(val >> 8));        	// Write the first (MSB) byte
    Wire.write((uint8_t)val);               	// and then the second
    Wire.endTransmission();                 	// Close transmission and actually send data
    delayMicroseconds(I2C_DELAY);           	// delay required for sync
}

void MagicBoat_BNO055::writeRegister8(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(bnoAddress);  		// Address the I2C device
    Wire.write(reg);                        	// Send register address to write
    Wire.write(val);               				// Write the first (MSB) byte
    Wire.endTransmission();                 	// Close transmission and actually send data
    delayMicroseconds(I2C_DELAY);           	// delay required for sync
}

int16_t MagicBoat_BNO055::readRegister16(uint8_t reg)
{
    int16_t value;

    Wire.beginTransmission(bnoAddress);        	// Address the I2C device
    Wire.write(reg);                            // Send register address to read
    Wire.endTransmission();                     // Close transmission
    delayMicroseconds(I2C_DELAY);               // delay required for sync
    Wire.requestFrom(bnoAddress, (uint8_t)2);   // Request 2 consecutive bytes
    value = Wire.read();                        // Read the msb
    value = value << 8;			                // shift the data over 8 bits
    value |= Wire.read();                       // Read the lsb
    return value;
}

int8_t MagicBoat_BNO055::readRegister8(uint8_t reg)
{
    int8_t value;

    Wire.beginTransmission(bnoAddress);        	// Address the I2C device
    Wire.write(reg);                            // Send register address to read
    Wire.endTransmission();                     // Close transmission
    delayMicroseconds(I2C_DELAY);               // delay required for sync
    Wire.requestFrom(bnoAddress,(uint8_t)1);   	// Request 2 consecutive bytes
    value = Wire.read();                        // Read the msb
	return value;
}

uint8_t MagicBoat_BNO055::readRegisterU8(uint8_t reg)
{
    uint8_t value;

    Wire.beginTransmission(bnoAddress);        	// Address the I2C device
    Wire.write(reg);                            // Send register address to read
    Wire.endTransmission();                     // Close transmission
    delayMicroseconds(I2C_DELAY);               // delay required for sync
    Wire.requestFrom(bnoAddress,(uint8_t)1);   	// Request 2 consecutive bytes
    value = Wire.read();                        // Read the msb
	return value;
}