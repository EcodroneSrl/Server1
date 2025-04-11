#ifndef MagicBoat_GPS_F9X_h
#define MagicBoat_GPS_F9X_h

#define lengthBuff 				92
#define lengthBuff2 			64
#define init_buffer_length 		6

#define SerialGPS Serial1
#define SerialDeb Serial

#define DEB_GPS_PVT				0
#define DEB_GPS_RELPOSNED		1

#define PVT_OK			1
#define RELPOSNED_OK	2

#include <inttypes.h>

class MagicBoat_GPS_F9X
{
	public:
	
  // Internal variables
  int timediff;
	uint8_t ck_a;     // Packet checksum
	uint8_t ck_b;
	uint8_t serial_num;
	uint8_t MTK_payload_counter;
	uint32_t GPS_timer;
	uint8_t GPS_step;
  uint8_t MTK_buffer[lengthBuff+2];
	uint32_t time_print;
	uint8_t printClearGPS = 1;
	uint8_t ucenter = 0;
	uint8_t sync_char_0 = 0xB5;
	uint8_t sync_char_1 = 0x62;
	uint8_t sync_char_2 = 0x01;
	uint8_t sync_char_PVT_3 = 0x07;
	uint8_t sync_char_PVT_4 = 0x5C;
	uint8_t sync_char_PVT_5 = 0x00;
	
	uint8_t sync_char_RELPOSNED_3 = 0x3C;
	uint8_t sync_char_RELPOSNED_4 = 0x40;
	uint8_t sync_char_RELPOSNED_5 = 0x00;
	
	uint8_t init_buffer[init_buffer_length];
	
    int32_t Bytes2Long(unsigned char Buffer[]);

    int32_t int32parse(unsigned char Buffer[]);
    uint32_t uint32parse(unsigned char Buffer[]);
    int16_t int16parse(unsigned char Buffer[]);
    uint16_t uint16parse(unsigned char Buffer[]);

	struct __attribute__((packed)) debug
	{
		uint8_t debug_enable[2] = {0,0};
		uint16_t debug_timer[2] = {100,100};
		uint8_t	debug_en_head[2] = {0,0};
	} debug_par;
	
	uint32_t debug_time[2] = {1,1};
	
	union long_union
	{
		int32_t dword;
		uint8_t  byte[4];
	} longUnion;

    union int32_union
    {
        int32_t int32value;
        uint8_t byte[4];
    }int32Union;

    union uint32_union
    {
        uint32_t uint32value;
        uint8_t byte[4];
    }uint32Union;

    union int16_union
    {
        int16_t int16value;
        uint8_t byte[2];
    }int16Union;

    union uint16_union
    {
        uint16_t uint16value;
        uint8_t byte[2];
    }uint16Union;

    // Methods
	MagicBoat_GPS_F9X();
	virtual void begin(uint32_t baudrate);
	virtual uint8_t Read();
	virtual void End();
	virtual void serialdata(uint8_t data);
	virtual void serialdatafromUSB();
	virtual void printClearData(uint8_t printClearGPS);
	virtual void printClearGPS_ON();
	virtual void printClearGPS_OFF();
	virtual void ucenter_ON();
	virtual void ucenter_OFF();
	
	// Properties
	int16_t HDOP;//Horizontal Dilution of Precision Example:1.15 > 1.15*(10^2)=115Horizontal Dilution of Precision
	int32_t Lattitude;//Latitude (in decimal degrees)Example:23.0985721*(10^7)=230985721  
	int32_t Longitude;//Longitude (in decimal degrees)Example:120.2843832*(10^7)=1202843832
	int32_t Altitude;//MSL Altitude (meter)Example:34.82*(10^2)=3482
	int32_t gSpeed;//Ground Speed(m/s)Example:0.324(km/hr) > 0.324*1000/3600 =0.09 (m/s) > 0.09*100=9
	int32_t Heading;//Heading(degrees)Example:123.12 > 123.12*(10^2)=12312
	uint8_t Sats;//Number of visible satelites
	uint8_t Fix;// 0x01 > GPS no fix 0x02 > GPS 2D fix 0x03 > GPS 3D fix
	uint32_t UTC;//UTC Time Example: 03:35:23.10 > 33523.10 > 33523.10*(10^2) > 3352310
	uint32_t UTC2;//UTC Time Example: 03:35:23.10 > 33523.10 > 33523.10*(10^2) > 3352310
	uint32_t Date;
	/*
	FAT32 compliant format:
	bit31:25
	Year origin from the 1980 (0..127, e.g. 39 for 2019)
	bit24:21
	Month (1..12)
	bit20:16
	Day of the month (1..31)
	bit15:11
	Hour (0..23)
	bit10:5
	Minute (0..59)
	bit4:0
	Second / 2 (0..29, e.g. 25 for 50)
	*/



  //NAV-PVT variables
  struct __attribute__((packed)) pvt
  {
    uint32_t iTOW;//ms
    uint16_t year;//UTC
    uint8_t month;//1_12
    uint8_t day;//1_31_UTC
    uint8_t hour;//0_23_UTC
    uint8_t min;//0_59_UTC
    uint8_t sec;//0_59UTC
    
    // Validity Flags
    uint8_t validDate;
    uint8_t validTime;
    uint8_t fullyResolved;
    uint8_t validMag;
    
    uint32_t tAcc;//time accuracy
    int32_t nano;//ns frazione di secondo;
    uint8_t fixType = 0;//uguale a FIX
    
    // Fix Status Flags
    uint8_t gnssFixOK;
    uint8_t diffSoln;
    uint8_t psmState;
    uint8_t headVehValid;
    uint8_t carrSoln;
    
    // Additional Flags 1
    uint8_t confirmedAvai;
    uint8_t confirmedDate;
    uint8_t confirmedTime;
    
    uint8_t numSV;//Uguale a Sats: numero di sat usati in NAV
    int32_t lon;
    int32_t lat;
    int32_t height;//altitudine dall'ellissoide [mm]
    int32_t hMSL; //altezza dal livello del mare[mm]
    uint32_t hAcc;//accuratezza orizzontale stimata[mm]
    uint32_t vAcc;//accuratezza verticale
    int32_t velN;//[mm/s] velocit� verso nord
    int32_t velE;//verso est
    int32_t velD;//verso down
    int32_t gSpeed; // Speed 2d velocita
    int32_t headMot;//uguale HEADING 2Dprecisione 1e-5;
    uint32_t sAcc;//accuratezza speed
    uint32_t headAcc;//accuratezza heading
    uint16_t pDOP; //PositionDOP
    
    // Additional Flags 2
    uint8_t invalidL1h;
    
    int32_t headVeh;//1e-5 deg tipo bussola orientazione del veicolo
    int16_t magDec;//1e-2 magnetico declination
    uint16_t magAcc;//1e-2 accuratezza declinazione magnetica
	
	uint32_t millisNow;
	uint32_t microsNow;
  } pvt1;
    
    //NAV-RELPOSNED variables
  struct __attribute__((packed)) relposned
  {
	uint8_t version;
	uint8_t refStationId;
	uint32_t iTOW;//ms

	int32_t relPosN;//North component of relative position vector ~ cm
	int32_t relPosE;//East component of relative position vector ~ cm
	int32_t relPosD;//Down component of relative position vector ~ cm
	int32_t relPosLength; //Length of relative position vector ~ cm
	int32_t relPosHeading; //Heading of relative position vector ~ deg 1e-5

	int8_t relPosHPN;//High Precision North component of relative position vector ~ 0.1mm
	int8_t relPosHPE;//High Precision East component of relative position vector ~ 0.1mm
	int8_t relPosHPD;//High Precision Down component of relative position vector ~ 0.1mm
	int8_t relPosHPLength;//High Precision Length of relative position vector ~ 0.1mm
	uint32_t accN;//Accuracy of relative position North component ~ 0.1mm
	uint32_t accE;//Accuracy of relative position East component ~ 0.1mm
	uint32_t accD;//Accuracy of relative position Down component ~ 0.1mm
	uint32_t accLength;//Accuracy of Length of the relative position vector ~ 0.1mm
	uint32_t accHeading;//Accuracy of Heading of the relative position vector ~ deg 1e-5

	//Flags
	uint8_t gnssFixOK; //Valid Fix
	uint8_t diffSoln; //1 if differential corrections applied
	uint8_t relPosValid;//1 if relative position components and accuracies valid
	uint8_t carrSoln;//0=no carrier; 1=carrier with floating ambiguities; 2= carrier with fixed ambiguities
	uint8_t isMoving;//1 if receier in moving base mode
	uint8_t refPosMiss;//1 if extrapolated reference position used to compute moving base in this epoch
	uint8_t refObsMiss;//1 if extrapolated reference observations used to compute moving base in this epoch
	uint8_t relPosHeadingValid;//1 if relative position components and accuracies valid
	uint8_t relPosNormalized;//1 if components of relative position vector are normalized
	
	uint32_t millisNow;
	uint32_t microsNow;
  } relposned1;

};

extern MagicBoat_GPS_F9X GPS;

#endif
