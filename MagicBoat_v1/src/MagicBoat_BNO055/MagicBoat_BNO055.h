#ifndef MagicBoat_BNO055_h
#define MagicBoat_BNO055_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define BNO055_ADDRESS						0x28
#define SerialDeb Serial
//#define SerialDeb Serial1 ---- 

#define UPDATE_ON							1
#define DESCR_ON							1
#define HEAD_ON								1

#define UPDATE_OFF							0
#define DESCR_OFF							0
#define HEAD_OFF							0

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                 0x00
#define BNO055_ACC_REV_ID_ADDR              0x01
#define BNO055_MAG_REV_ID_ADDR              0x02
#define BNO055_GYR_REV_ID_ADDR              0x03
#define BNO055_SW_REV_ID_LSB_ADDR			0x04
#define BNO055_SW_REV_ID_MSB_ADDR			0x05
#define BNO055_BL_Rev_ID_ADDR				0X06
#define BNO055_Page_ID_ADDR				    0X07

/* Accel data register*/
#define BNO055_ACC_DATA_X_LSB_ADDR			0X08
#define BNO055_ACC_DATA_X_MSB_ADDR			0X09
#define BNO055_ACC_DATA_Y_LSB_ADDR			0X0A
#define BNO055_ACC_DATA_Y_MSB_ADDR			0X0B
#define BNO055_ACC_DATA_Z_LSB_ADDR			0X0C
#define BNO055_ACC_DATA_Z_MSB_ADDR			0X0D

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR			0X0E
#define BNO055_MAG_DATA_X_MSB_ADDR			0X0F
#define BNO055_MAG_DATA_Y_LSB_ADDR			0X10
#define BNO055_MAG_DATA_Y_MSB_ADDR			0X11
#define BNO055_MAG_DATA_Z_LSB_ADDR			0X12
#define BNO055_MAG_DATA_Z_MSB_ADDR			0X13

/*Gyro data registers*/
#define BNO055_GYR_DATA_X_LSB_ADDR			0X14
#define BNO055_GYR_DATA_X_MSB_ADDR			0X15
#define BNO055_GYR_DATA_Y_LSB_ADDR			0X16
#define BNO055_GYR_DATA_Y_MSB_ADDR			0X17
#define BNO055_GYR_DATA_Z_LSB_ADDR			0X18
#define BNO055_GYR_DATA_Z_MSB_ADDR			0X19

/*Euler data registers*/
#define BNO055_EUL_HEADING_LSB_ADDR			0X1A
#define BNO055_EUL_HEADING_MSB_ADDR			0X1B

#define BNO055_EUL_ROLL_LSB_ADDR			0X1C
#define BNO055_EUL_ROLL_MSB_ADDR			0X1D

#define BNO055_EUL_PITCH_LSB_ADDR			0X1E
#define BNO055_EUL_PITCH_MSB_ADDR			0X1F

/*Quaternion data registers*/
#define BNO055_QUA_DATA_W_LSB_ADDR			0X20
#define BNO055_QUA_DATA_W_MSB_ADDR			0X21
#define BNO055_QUA_DATA_X_LSB_ADDR			0X22
#define BNO055_QUA_DATA_X_MSB_ADDR			0X23
#define BNO055_QUA_DATA_Y_LSB_ADDR			0X24
#define BNO055_QUA_DATA_Y_MSB_ADDR			0X25
#define BNO055_QUA_DATA_Z_LSB_ADDR			0X26
#define BNO055_QUA_DATA_Z_MSB_ADDR			0X27

/* Linear acceleration data registers*/
#define BNO055_LIA_DATA_X_LSB_ADDR			0X28
#define BNO055_LIA_DATA_X_MSB_ADDR			0X29
#define BNO055_LIA_DATA_Y_LSB_ADDR			0X2A
#define BNO055_LIA_DATA_Y_MSB_ADDR			0X2B
#define BNO055_LIA_DATA_Z_LSB_ADDR			0X2C
#define BNO055_LIA_DATA_Z_MSB_ADDR			0X2D

/*Gravity data registers*/
#define BNO055_GRV_DATA_X_LSB_ADDR			0X2E
#define BNO055_GRV_DATA_X_MSB_ADDR			0X2F
#define BNO055_GRV_DATA_Y_LSB_ADDR			0X30
#define BNO055_GRV_DATA_Y_MSB_ADDR			0X31
#define BNO055_GRV_DATA_Z_LSB_ADDR			0X32
#define BNO055_GRV_DATA_Z_MSB_ADDR			0X33

/* Temperature data register*/

#define BNO055_TEMP_ADDR					0X34

/* Status registers*/
#define BNO055_CALIB_STAT_ADDR				0X35
#define BNO055_ST_RESULT_ADDR				0X36
#define BNO055_INT_STA_ADDR					0X37
#define BNO055_SYS_STATUS_ADDR				0X39
#define BNO055_SYS_ERR_ADDR					0X3A

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR				0X3B
#define BNO055_DATA_SEL_ADDR				0X3C

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR				0X3D
#define BNO055_PWR_MODE_ADDR				0X3E

#define BNO055_SYS_TRIGGER_ADDR				0X3F
#define BNO055_TEMP_SOURCE_ADDR				0X40
/* Axis remap registers*/
#define BNO055_AXIS_MAP_CONFIG_ADDR			0X41
#define BNO055_AXIS_MAP_SIGN_ADDR			0X42

/* SIC registers*/
#define BNO055_SIC_MATRIX_0_LSB_ADDR		0X43
#define BNO055_SIC_MATRIX_0_MSB_ADDR		0X44
#define BNO055_SIC_MATRIX_1_LSB_ADDR		0X45
#define BNO055_SIC_MATRIX_1_MSB_ADDR		0X46
#define BNO055_SIC_MATRIX_2_LSB_ADDR		0X47
#define BNO055_SIC_MATRIX_2_MSB_ADDR		0X48
#define BNO055_SIC_MATRIX_3_LSB_ADDR		0X49
#define BNO055_SIC_MATRIX_3_MSB_ADDR		0X4A
#define BNO055_SIC_MATRIX_4_LSB_ADDR		0X4B
#define BNO055_SIC_MATRIX_4_MSB_ADDR		0X4C
#define BNO055_SIC_MATRIX_5_LSB_ADDR		0X4D
#define BNO055_SIC_MATRIX_5_MSB_ADDR		0X4E
#define BNO055_SIC_MATRIX_6_LSB_ADDR		0X4F
#define BNO055_SIC_MATRIX_6_MSB_ADDR		0X50
#define BNO055_SIC_MATRIX_7_LSB_ADDR		0X51
#define BNO055_SIC_MATRIX_7_MSB_ADDR		0X52
#define BNO055_SIC_MATRIX_8_LSB_ADDR		0X53
#define BNO055_SIC_MATRIX_8_MSB_ADDR		0X54

/* Accelerometer Offset registers*/
#define ACC_OFFSET_X_LSB_ADDR				0X55
#define ACC_OFFSET_X_MSB_ADDR				0X56
#define ACC_OFFSET_Y_LSB_ADDR				0X57
#define ACC_OFFSET_Y_MSB_ADDR				0X58
#define ACC_OFFSET_Z_LSB_ADDR				0X59
#define ACC_OFFSET_Z_MSB_ADDR				0X5A

/* Magnetometer Offset registers*/
#define MAG_OFFSET_X_LSB_ADDR				0X5B
#define MAG_OFFSET_X_MSB_ADDR				0X5C
#define MAG_OFFSET_Y_LSB_ADDR				0X5D
#define MAG_OFFSET_Y_MSB_ADDR				0X5E
#define MAG_OFFSET_Z_LSB_ADDR				0X5F
#define MAG_OFFSET_Z_MSB_ADDR				0X60

/* Gyroscope Offset registers*/
#define GYR_OFFSET_X_LSB_ADDR				0X61
#define GYR_OFFSET_X_MSB_ADDR				0X62
#define GYR_OFFSET_Y_LSB_ADDR				0X63
#define GYR_OFFSET_Y_MSB_ADDR				0X64
#define GYR_OFFSET_Z_LSB_ADDR				0X65
#define GYR_OFFSET_Z_MSB_ADDR				0X66

/* Accelerometer Radius registers*/
#define ACC_RADIUS_LSB_ADDR					0X67
#define ACC_RADIUS_MSB_ADDR					0X68

/* Magnetometer Radius registers*/
#define MAG_RADIUS_LSB_ADDR					0X69
#define MAG_RADIUS_MSB_ADDR					0X6A

/* PAGE0 REGISTERS DEFINITION END*/

/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define ACC_CONFIG_ADDR						0X08
#define MAG_CONFIG_ADDR						0X09
#define GYR_CONFIG_ADDR						0X0A
#define GYR_MODE_CONFIG_ADDR				0X0B
#define ACC_SLEEP_CONFIG_ADDR				0X0C
#define GYR_SLEEP_CONFIG_ADDR				0X0D
#define MAG_SLEEP_CONFIG_ADDR				0x0E

/* Interrupt registers*/
#define INT_MSK_ADDR						0X0F
#define INT_ADDR							0X10
#define ACC_AM_THRES_ADDR					0X11
#define ACC_INT_SETTINGS_ADDR				0X12
#define ACC_HG_DURATION_ADDR				0X13
#define ACC_HG_THRES_ADDR					0X14
#define ACC_NM_THRES_ADDR					0X15
#define ACC_NM_SET_ADDR						0X16
#define GYR_INT_SETING_ADDR					0X17
#define GYR_HR_X_SET_ADDR					0X18
#define GYR_DUR_X_ADDR						0X19
#define GYR_HR_Y_SET_ADDR					0X1A
#define GYR_DUR_Y_ADDR						0X1B
#define GYR_HR_Z_SET_ADDR					0X1C
#define GYR_DUR_Z_ADDR						0X1D
#define GYR_AM_THRES_ADDR					0X1E
#define GYR_AM_SET_ADDR						0X1F

/* PAGE1 REGISTERS DEFINITION END */

/* Interrupt enable */

#define ACC_NM_INT							0
#define ACC_AM_INT							0
#define ACC_HIGH_G_INT						0
#define GYR_DRDY_INT						0
#define GYR_HIGH_RATE_INT					0
#define GYR_AM_INT							0
#define MAG_DRDY_INT						0
#define ACC_BSX_DRDY_INT					1
	
/* Interrupt masking */

#define ACC_NM_MSK							0
#define ACC_AM_MSK							0
#define ACC_HIGH_G_MSK						0
#define GYR_DRDY_MSK						0
#define GYR_HIGH_RATE_MSK					0
#define GYR_AM_MSK							0
#define MAG_DRDY_MSK						0
#define ACC_BSX_DRDY_MSK					1	

/* Interrupt only FUSION */
#define FUSION_DRDY_INT_EN					1
#define FUSION_DRDY_MSK_EN					1

/* Page IDs */

#define PAGE0								0x00
#define PAGE1								0x01

/* Operation modes values*/

#define CONFIG_MODE							0X00

//Non-Fusion Modes
#define ACCONLY								0X01
#define MAGONLY								0X02
#define GYRONLY								0X03
#define ACCMAG								0X04
#define ACCGYR								0X05
#define MAGGYR								0X06
#define AMG									0X07

// Fusion modes
#define IMU									0X08
#define COMPASS								0X09
#define M4G									0X0A
#define NDOF_FMC_OFF						0X0B
#define NDOF								0X0C	

/* Power modes values*/
#define NORMAL_PWR_MODE						0X00	
#define LOW_PWR_MODE						0X01
#define SUSPEND_PWR_MODE					0X02	

/* Sensors configuration parameters*/

//Accelerometer

# define G_2								00
# define G_4								01
# define G_8								10
# define G_16								11

# define H_781								000
# define H_1563								001
# define H_3125								010
# define H_625								011
# define H_125								100
# define H_250								101
# define H_500								110
# define H_1000								111

# define NORMAL_ACC_OP_MODE					000
# define SUSPEND_ACC_OP_MODE				001
# define LOW_PW1_ACC_OP_MODE				010
# define STANDBY							011
# define LOW_PW2_ACC_OP_MODE				100
# define DEEP_SUSPEND_ACC_OP_MODE			101

//Gyroscope

# define DPS_2000							000
# define DPS_1000							001
# define DPS_500							010
# define DPS_250							011
# define DPS_125							100

# define H_523								000
# define H_230								001
# define H_116								010
# define H_47								011
# define H_23								100
# define H_12								101
# define H_64								110
# define H_32								111

# define NORMAL_GYR_OP_MODE					000
# define FAST_PWUP_GYR_OP_MODE				001
# define DEEP_SUSPEND_GYR_OP_MODE			010
# define SUSPEND_GYR_OP_MODE				011
# define ADV_PWSAVE_GYR_OP_MODE				100

//Magnetometer

# define H_2								000
# define H_6								001
# define H_8								010
# define H_10								011
# define H_15								100
# define H_20								101
# define H_25								110
# define H_30								111

# define LOW_PW_MAG_OP_MODE					00
# define REGULAR_MAG_OP_MODE				01
# define ENH_REGULAR_MAG_OP_MODE			10
# define HIGH_ACCURACY_MAG_OP_MODE			11

# define NORMAL_MAG_PW_MODE					00
# define SLEEP_MAG_PW_MODE					01
# define SUSPEND_MAG_PW_MODE				10
# define FORCE_MODE_MAG_PW_MODE				11

/*------- Case per update -------*/

#define UPD_CALIB_STAT						0
#define UPD_CALIB_OFFSET					1
#define UPD_ACC_CONF						2
#define UPD_GYR_CONF						3
#define UPD_MAG_CONF						4
#define UPD_EULER							5
#define UPD_ACC								6
#define UPD_GYR								7
#define UPD_MAG								8
#define UPD_QUA								9

/*------- Case per Debug -------*/

#define DEB_CALIB_STAT						0
#define DEB_CALIB_OFFSET					1
#define DEB_ACC_CONF						2
#define DEB_GYR_CONF						3
#define DEB_MAG_CONF						4
#define DEB_EULER							5
#define DEB_ACC								6
#define DEB_GYR								7
#define DEB_MAG								8
#define DEB_QUA								9
#define DEB_EUL_QUA							10
#define DEB_X								11
#define DEB_Y								12
#define DEB_Z								13



#define I2C_DELAY 							10

class MagicBoat_BNO055
{
	public:		
	 
	struct __attribute__((packed)) cal
	{
		int16_t acc_off_x = 40;
		int16_t acc_off_y = -19;
		int16_t acc_off_z = -14;
		int16_t mag_off_x = -38;
		int16_t mag_off_y = -139;
		int16_t mag_off_z = 79;
		int16_t gyr_off_x = -1;
		int16_t gyr_off_y = -3;
		int16_t gyr_off_z = -1;
		int16_t acc_rad = 1000;
		int16_t mag_rad = 518;
	} cal_values;
	
	struct __attribute__((packed)) update
	{
		uint8_t update_enable[10] = {0,0,0,0,0,1,1,0,0,1};
		uint16_t update_timer[10] = {11,11,11,11,11,11,11,11,51,100};
	} update_par;
	
	uint32_t update_time[10] = {1,1,1,1,1,1,1,1,1,1};
	
	struct __attribute__((packed)) debug
	{
		uint8_t debug_enable[14] = {0,0,0,0,0,1,0,0,0,0,0,0,0,0};
		uint8_t	debug_en_descr[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		uint8_t	debug_en_update[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		uint16_t debug_timer[14] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100};
		uint8_t	debug_en_head[14] = {0,0,0,0,0,1,0,0,0,0,0,0,0,0};
	} debug_par;
	
	uint32_t debug_time[14] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	
	bool begin();
	bool configure(uint8_t pw_mode, uint8_t op_mode, uint8_t bnoAddress);
	int8_t set_config_mode();
	uint8_t set_power_mode(uint8_t pw_mode);
	uint8_t set_operation_mode(uint8_t op_mode);
	void getFWVersion();
	void writeRegister16(uint8_t reg, uint16_t val16);
	void writeRegister8(uint8_t reg, uint8_t val8);
	int16_t readRegister16(uint8_t reg);
	int8_t readRegister8(uint8_t reg);
	uint8_t readRegisterU8(uint8_t reg);
	uint8_t get_operation_mode();
	uint8_t a_pw_mode;
	uint8_t a_op_mode;
	
	struct __attribute__((packed)) EulerAcc
	{
		float Yaw = 0;
		float Pitch = 0;
		float Roll = 0;
		float Ax = 0;
		float Ay = 0;
		float Az = 0;
	} EulerAccData;
	
	float Gx, Gy, Gz;
	float Mx, My, Mz;
	float q0, q1, q2, q3;
	float Yq, Rq, Pq;
	uint8_t mag_cal, acc_cal, gyr_cal, sys_cal;
	// int16_t mag_off_x, mag_off_y, mag_off_z;
	// int16_t gyr_off_x, gyr_off_y, gyr_off_z;
	// int16_t acc_off_x, acc_off_y, acc_off_z;
	// int16_t acc_rad, mag_rad;
	uint32_t time_get_cal, time_calib_stat, time_calib_offset, time_euler, time_acc, time_mag, time_gyr, time_qua, time_eulqua, time_dataX, time_dataY, time_dataZ;
	uint32_t time_acc_conf, time_mag_conf, time_gyr_conf;
	uint32_t time_update;
	uint8_t acc_range, acc_bw, acc_op_mode, gyr_range, gyr_bw, gyr_op_mode, mag_data_rate, mag_op_mode, mag_pw_mode;


	int16_t getCal(uint8_t regLSB, uint8_t regMSB);
	void getCalAll();
	int16_t get_cal_ACC_OFFSET_X();
	int16_t get_cal_ACC_OFFSET_Y();
	int16_t get_cal_ACC_OFFSET_Z();
	int16_t get_cal_MAG_OFFSET_X();
	int16_t get_cal_MAG_OFFSET_Y();
	int16_t get_cal_MAG_OFFSET_Z();
	int16_t get_cal_GYR_OFFSET_X();
	int16_t get_cal_GYR_OFFSET_Y();
	int16_t get_cal_GYR_OFFSET_Z();
	int16_t get_cal_ACC_RADIUS();
	int16_t get_cal_MAG_RADIUS();

	int16_t setCal(uint8_t regLSB, uint8_t regMSB, int16_t value);
	int16_t setCalAll();	 
	int16_t set_cal_ACC_OFFSET_X(int16_t value);
	int16_t set_cal_ACC_OFFSET_Y(int16_t value);
	int16_t set_cal_ACC_OFFSET_Z(int16_t value);
	int16_t set_cal_MAG_OFFSET_X(int16_t value);
	int16_t set_cal_MAG_OFFSET_Y(int16_t value);
	int16_t set_cal_MAG_OFFSET_Z(int16_t value);
	int16_t set_cal_GYR_OFFSET_X(int16_t value);
	int16_t set_cal_GYR_OFFSET_Y(int16_t value);
	int16_t set_cal_GYR_OFFSET_Z(int16_t value);
	int16_t set_cal_ACC_RADIUS(int16_t value);
	int16_t set_cal_MAG_RADIUS(int16_t value);

	void set_configuration_Acc(uint8_t range, uint8_t bw, uint8_t o_mode);
	void set_configuration_Gyr(uint8_t range, uint8_t bw, uint8_t o_mode);
	void set_configuration_Mag(uint8_t data_rate, uint8_t o_mode, uint8_t p_mode);

	void set_interrupt();
	
	int16_t getData(uint8_t regLSB, uint8_t regMSB);

	float range_180(float value);
	float conv_float(int16_t value, float div); 

	int16_t get_data_EUL_HEADING();
	int16_t get_data_EUL_ROLL();
	int16_t get_data_EUL_PITCH(); 

	int16_t get_data_ACC_DATA_X();
	int16_t get_data_ACC_DATA_Y();
	int16_t get_data_ACC_DATA_Z();

	int16_t get_data_GYR_DATA_X();
	int16_t get_data_GYR_DATA_Y();
	int16_t get_data_GYR_DATA_Z();

	int16_t get_data_MAG_DATA_X();
	int16_t get_data_MAG_DATA_Y();
	int16_t get_data_MAG_DATA_Z();

	int16_t get_data_QUA_DATA_W();
	int16_t get_data_QUA_DATA_X();
	int16_t get_data_QUA_DATA_Y();
	int16_t get_data_QUA_DATA_Z();

	int8_t get_ACC_conf();
	int8_t get_GYR_conf1();
	int8_t get_GYR_conf2();
	int8_t get_MAG_conf();

	void update();
	void polling(uint8_t i, uint16_t update_timer);
	void update_calib_offset();
	void update_calib_stat();
	void update_Qua();
	void Qua_to_Eul(uint8_t update);
	void update_Euler();
	void update_Acc();
	void update_Mag();
	void update_Gyr();

	void update_Acc_conf();
	void update_Gyr_conf();
	void update_Mag_conf();

	void printCalibOffset(uint8_t description, uint16_t timer_calib_offset, uint8_t update, uint8_t head);
	void printCalibStat(uint8_t description, uint16_t timer_calib_stat, uint8_t update, uint8_t head);
	void printEuler(uint8_t description, uint16_t timer_euler, uint8_t update, uint8_t head);
	void printEulerQua(uint8_t description, uint16_t timer_eulqua, uint8_t update, uint8_t head);
	void printAcc(uint8_t description, uint16_t timer_acc, uint8_t update, uint8_t head);
	void printMag(uint8_t description, uint16_t timer_mag, uint8_t update, uint8_t head);
	void printGyr(uint8_t description, uint16_t timer_gyr, uint8_t update, uint8_t head);
	void printQua(uint8_t description, uint16_t timer_qua, uint8_t update, uint8_t head);
	void printDataX(uint8_t description, uint16_t timer_DataX, uint8_t update, uint8_t head);
	void printDataY(uint8_t description, uint16_t timer_DataY, uint8_t update, uint8_t head);
	void printDataZ(uint8_t description, uint16_t timer_DataZ, uint8_t update, uint8_t head);
	void printAccConfig(uint8_t description, uint16_t timer_acc_conf, uint8_t update, uint8_t head);
	void printGyrConfig(uint8_t description, uint16_t timer_gyr_conf, uint8_t update, uint8_t head);	 
	void printMagConfig(uint8_t description, uint16_t timer_mag_conf, uint8_t update, uint8_t head);	 

	void setPage0();
	void setPage1();
	 
	private:
	
	uint8_t bnoAddress=BNO055_ADDRESS;
	uint8_t pw_mode;
	uint8_t op_mode;
};


#endif
