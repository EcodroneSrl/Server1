#include "MagicBoat_GPS_F9X.h"
#include <avr/interrupt.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
//NewSoftSerial GPS_Serial = NewSoftSerial(3,2);//Comment this line to use Hardware Serial

#define DEBM8N false //metti false se non vuoi stampare  nulla
#define DEBM8N_END false

// Constructors ////////////////////////////////////////////////////////////////
MagicBoat_GPS_F9X::MagicBoat_GPS_F9X()
{
	
}


// Public Methods //////////////////////////////////////////////////////////////

// Scrive dati sulla seriale di debug
void MagicBoat_GPS_F9X::serialdata(uint8_t data)
{
	SerialDeb.write(data);
}

// Legge dati dalla seriale di debug
void MagicBoat_GPS_F9X::serialdatafromUSB()
{
	if(SerialDeb.available() > 0 )
	{
		uint8_t dataUSB = SerialDeb.read();
		SerialGPS.write(dataUSB);
	}
}

//Inizializza la seriale del GPS e le variabili
void MagicBoat_GPS_F9X::begin(uint32_t baudrate)
{
	ck_a = 0;
	ck_b = 0;
	GPS_step = 0;
	// Initialize serial port
    GPS_timer = millis();//Restarting timer...
	SerialGPS.begin(baudrate);	
}

void MagicBoat_GPS_F9X::End()
{
	SerialGPS.end();	
}

//******* Acquisisce il buffer del GPS e ne fa il parsing *******//

uint8_t MagicBoat_GPS_F9X::Read()
{
	uint8_t Err = 0;
	uint8_t data;
	uint8_t numc;
	// Abilita la trasmissione da a ucenter al GPS
	if(ucenter)
	{
		serialdatafromUSB();
	}
	
	numc = SerialGPS.available();
		
	if(numc > 0)
	{
		for (uint16_t i=0; i < numc; i++)// Process bytes received
		{
			if((Err == PVT_OK) || (Err == RELPOSNED_OK))
			{
				break;
			}
			data = SerialGPS.read();	
			if(ucenter) // Abilita la trasmissione dal GPS a ucenter
			{
				serialdata(data);
			}
			if(DEBM8N) // Stampa dati grezzi debug
			{
				SerialDeb.print(data,HEX);
				SerialDeb.print(",");
			}
			switch(GPS_step)//Normally we start from zero. This is a state machine
			{
				case 0:				
					if(data == sync_char_0) // sync char 
					{
						GPS_step++;
						init_buffer[0] = sync_char_0;
						//SerialDeb.println("Step_0");
					}
					else
					{
						GPS_step = 0;
					}			
				break;
				
				case 1:	
					if(data == sync_char_1)// sync char 1
					{
						//SerialDeb.println("Step_1");
						GPS_step++;
						init_buffer[1] = sync_char_1;
					}
					else
					{
						GPS_step = 0;
					}
				break;
				
				case 2:	
					if(data == sync_char_2)// MTK sync char 3
					{
						//SerialDeb.println("Step_2");
						GPS_step++;//ooh! The third data packet is correct, add it to check sum
						MTK_payload_counter = 0;//Reset counter
						ck_b = ck_a = data;
						init_buffer[2] = sync_char_2;
					}
					else
					{
						GPS_step = 0;
					}
				break;
				
				case 3:				
					if(data == sync_char_PVT_3)// MTK sync char 3
					{
						//SerialDeb.println("Step_3");
						GPS_step++;//ooh! The third data packet is correct, add it to check sum
						ck_b += (ck_a += data);			
						init_buffer[3] = sync_char_PVT_3;
					}
					else if(data == sync_char_RELPOSNED_3)// MTK sync char 3
					{
						//SerialDeb.println("Step_3");
						GPS_step = 104;//ooh! The third data packet is correct, add it to check sum
						ck_b += (ck_a += data);						
						init_buffer[3] = sync_char_RELPOSNED_3;
					}
					else
					{
						GPS_step = 0;
					}		
				break;
				
				case 4:
				
				if(data == sync_char_PVT_4)// MTK sync char 3
					{
						//SerialDeb.println("Step_4");
						GPS_step++;//ooh! The third data packet is correct, add it to check sum
						ck_b += (ck_a += data);			
						init_buffer[4] = sync_char_PVT_4;
					}
					else
					{
						GPS_step = 0;
					}
				break;
				
				case 5:
					if(data == sync_char_PVT_5)// MTK sync char 3
					{
						//SerialDeb.println("Step_5");
						GPS_step++;//ooh! The third data packet is correct, add it to check sum
						ck_b += (ck_a += data);			
						init_buffer[5] = sync_char_PVT_5;
					}
					else
					{
						GPS_step = 0;
					}
				break;
				
				case 6:
					//SerialDeb.println("Step_6");
					MTK_buffer[MTK_payload_counter] = data;
					MTK_payload_counter++;
					ck_b += (ck_a += data);
					if (MTK_payload_counter == lengthBuff)
					{
						GPS_step++;
					}
				break;
				
				case 7:
					//SerialDeb.println("Step_4");
					GPS_step++;
					MTK_buffer[MTK_payload_counter] = data;
					MTK_payload_counter++;
					if (ck_a != data)   // First checksum byte
					{
						GPS_step = 0;
						Err = 3;//Failed CheckSUM_B
					}
				break;

				case 8: // Payload data read...
		
					//SerialDeb.println("Step_5");
					GPS_step = 0;
					MTK_buffer[MTK_payload_counter] = data;
					MTK_payload_counter++;
					if (ck_b != data)   // Second checksum byte
					{
						GPS_step = 0;
						Err = 3;//Failed CheckSUM_B
						//SerialDeb.println("Step_6");
						break;
					}
			
					else
					{
						//NAV-PVT protocollo m8n

						pvt1.iTOW     = uint32parse(&MTK_buffer[0]);//ms
						UTC = pvt1.iTOW;
						pvt1.year     = uint16parse(&MTK_buffer[4]);//UTC
						pvt1.month    = MTK_buffer[6];//1_12
						pvt1.day      = MTK_buffer[7];//1_31_UTC
						pvt1.hour     = MTK_buffer[8];//0_23_UTC
						pvt1.min      = MTK_buffer[9];//0_59_UTC
						pvt1.sec      = MTK_buffer[10];//0_60UTC
						if (pvt1.sec>=60)
						{
							pvt1.sec = 59;
						}
						Date = 0;
						
						uint8_t valid    = MTK_buffer[11];//dal terzo bit allo zero ValidMAG;FullyREsolved;ValidTime;ValidDate
						if (valid&0x3) // Time and Date Valid
						{
							//MSDOS FAT format 
							Date = (pvt1.year - 1980)&0x7f; //7bit
							Date = (Date<<4) + (pvt1.month&0x0f); //4bit
							Date = (Date<<5) + (pvt1.day&0x1f); //5bit
							Date = (Date<<5) + (pvt1.hour&0x1f); //5bit
							Date = (Date<<6) + (pvt1.min&0x3f); //6bit
							Date = (Date<<5) + ((pvt1.sec>>1)&0x1f); //5bit
						}

						//Validity flags
						uint8_t flags  = MTK_buffer[11];
						
						pvt1.validDate = (flags) & (0x01);
						pvt1.validTime = (flags >> 1) & (0x01);;
						pvt1.fullyResolved = (flags >> 2) & (0x01);;
						pvt1.validMag = (flags >> 3) & (0x01);;
						
						pvt1.tAcc      = uint32parse(&MTK_buffer[12]);// time accuracy
						pvt1.nano      = int32parse(&MTK_buffer[16]);// ns frazione di secondo;
						pvt1.fixType   = MTK_buffer[20];// uguale a FIX
						
						//Fix Status flags
						uint8_t flags1  = MTK_buffer[21];
						
						pvt1.gnssFixOK = (flags1) & (0x01);
						pvt1.diffSoln = (flags1 >> 1) & (0x01);
						pvt1.psmState = (flags1 >> 2) & (0x07);
						pvt1.headVehValid = (flags1 >> 5) & (0x01);
						pvt1.carrSoln = (flags1 >> 6) & (0x03);
						
						// Additional Flags 1
						uint8_t flags2 = MTK_buffer[22];
						pvt1.confirmedAvai = (flags2 >> 5) & (0x01);
						pvt1.confirmedDate = (flags2 >> 6) & (0x01);
						pvt1.confirmedTime = (flags2 >> 7) & (0x01);
						
						pvt1.numSV     = MTK_buffer[23];//uint8_t numSV;//Uguale a Sats: numero di sat usati in NAV
						pvt1.lon 	   = int32parse(&MTK_buffer[24]);
						pvt1.lat 	   = int32parse(&MTK_buffer[28]);
						pvt1.lon	   = pvt1.lon/10;
						pvt1.lat 	   = pvt1.lat/10;
						pvt1.height    = int32parse(&MTK_buffer[32]);//altitudine dall'ellissoide [mm]
						pvt1.hMSL	   = int32parse(&MTK_buffer[36]); //altezza dal livello del mare[mm]
						pvt1.hAcc      = uint32parse(&MTK_buffer[40]);//accuratezza orizzontale stimata[mm]
						pvt1.vAcc      = uint32parse(&MTK_buffer[44]);//accuratezza verticale
						pvt1.velN      = int32parse(&MTK_buffer[48]);//[mm/s] velocit� verso nord
						pvt1.velE      = int32parse(&MTK_buffer[52]);//verso est
						pvt1.velD      = int32parse(&MTK_buffer[56]);//verso down
						pvt1.gSpeed    = int32parse(&MTK_buffer[60]);//int32_t gSpeed// Speed 2d velocita
						pvt1.headMot   = int32parse(&MTK_buffer[64]);//uguale HEADING 2Dprecisione 1e-5;
						pvt1.sAcc      = uint32parse(&MTK_buffer[68]);//accuratezza speed
						pvt1.headAcc   = uint32parse(&MTK_buffer[72]);//accuratezza heading
						pvt1.pDOP      = uint16parse(&MTK_buffer[76]); //PositionDOP
						
						// Additional Flags 2
						uint8_t flags3 = MTK_buffer[78];
						pvt1.invalidL1h = (flags3) & (0x01);
						
						pvt1.headVeh   = int32parse(&MTK_buffer[84]);//1e-5 deg tipo bussola orientazione del veicolo
						pvt1.magDec    = int16parse(&MTK_buffer[88]);//1e-2 magnetico declination
						pvt1.magAcc    = uint16parse(&MTK_buffer[90]);//1e-2 accuratezza declinazione magnetica
						
						pvt1.millisNow = millis();
						pvt1.microsNow = micros();
						
						Err = PVT_OK;
							  
						printClearData(printClearGPS);
						
						if(DEBM8N_END)
						{
							SerialDeb.println();
							SerialDeb.print("BUFFER: ");
							SerialDeb.println();
						}

						GPS_step = 0;
					}
				break;
						
						
				case 104:
					if(data == sync_char_RELPOSNED_4)// MTK sync char 3
					{
						GPS_step++;//ooh! The third data packet is correct, add it to check sum
						ck_b += (ck_a += data);
						init_buffer[4] = sync_char_RELPOSNED_4;
					}
					else
					{
						GPS_step = 0;
					}	
				break;

				case 105:
					if(data == sync_char_RELPOSNED_5)// MTK sync char 3
					{
						GPS_step++;//ooh! The third data packet is correct, add it to check sum
						ck_b += (ck_a += data);
						init_buffer[5] = sync_char_RELPOSNED_5;
					}
					else
					{
						GPS_step = 0;
					}	
				break;
					
				case 106:
					//SerialDeb.println("Step_6");
					MTK_buffer[MTK_payload_counter] = data;
					MTK_payload_counter++;
					ck_b += (ck_a += data);
					if (MTK_payload_counter == lengthBuff2)
					{
						GPS_step++;
					}
				break;
					
				case 107:
					//SerialDeb.println("Step_4");
					GPS_step++;
					MTK_buffer[MTK_payload_counter] = data;
					MTK_payload_counter++;
					if (ck_a != data)   // First checksum byte
					{
						GPS_step = 0;
						Err = 3;	//Failed CheckSUM_B
					}
				break;
					
				case 108: // Payload data read...
					//SerialDeb.println("Step_5");
					GPS_step = 0;
					MTK_buffer[MTK_payload_counter] = data;
					MTK_payload_counter++;
					if (ck_b != data)   // Second checksum byte
					{
						GPS_step = 0;
						Err = 3;//Failed CheckSUM_B
						//SerialDeb.println("Step_6");
						break;
					}
					else
					{
						//NAV-RELPOSNED

						relposned1.version  	   	= MTK_buffer[0]; //Message Version [0x01 for this version]
						
						relposned1.refStationId 	= uint16parse(&MTK_buffer[2]);
						relposned1.iTOW     	   	= uint32parse(&MTK_buffer[4]);//ms
						//reserved0[4]
						relposned1.relPosN      	= int32parse(&MTK_buffer[8]);//North component of relative position vector ~ cm
						relposned1.relPosE      	= int32parse(&MTK_buffer[12]);//East component of relative position vector ~ cm
						relposned1.relPosD      	= int32parse(&MTK_buffer[16]);//Down component of relative position vector ~ cm
						relposned1.relPosLength 	= int32parse(&MTK_buffer[20]); //Length of relative position vector ~ cm 
						relposned1.relPosHeading	= int32parse(&MTK_buffer[24]); //Heading of relative position vector ~ deg 1e-5
						//reserved1 [4]
						relposned1.relPosHPN        = MTK_buffer[32];//High Precision North component of relative position vector ~ 0.1mm
						relposned1.relPosHPE        = MTK_buffer[33];//High Precision East component of relative position vector ~ 0.1mm
						relposned1.relPosHPD        = MTK_buffer[34];//High Precision Down component of relative position vector ~ 0.1mm
						relposned1.relPosHPLength   = MTK_buffer[35];//High Precision Length of relative position vector ~ 0.1mm
						relposned1.accN 			= uint32parse(&MTK_buffer[36]);//Accuracy of relative position North component ~ 0.1mm
						relposned1.accE 			= uint32parse(&MTK_buffer[40]);//Accuracy of relative position East component ~ 0.1mm	
						relposned1.accD 			= uint32parse(&MTK_buffer[44]);//Accuracy of relative position Down component ~ 0.1mm
						relposned1.accLength		= uint32parse(&MTK_buffer[48]);//Accuracy of Length of the relative position vector ~ 0.1mm
						relposned1.accHeading 		= uint32parse(&MTK_buffer[52]);//Accuracy of Heading of the relative position vector ~ deg 1e-5
						//reserved2 [4]
						
						//Flags
						uint32_t flags3				= uint32parse(&MTK_buffer[60]);
						relposned1.gnssFixOK 		= (flags3) & (0x01); //Valid Fix
						relposned1.diffSoln  		= (flags3 >> 1) & (0x01); //1 if differential corrections applied
						relposned1.relPosValid 		= (flags3 >> 2) & (0x01);//1 if relative position components and accuracies valid
						relposned1.carrSoln  		= (flags3 >> 3) & (0x03);//0=no carrier; 1=carrier with floating ambiguities; 2= carrier with fixed ambiguities
						relposned1.isMoving 		= (flags3 >> 5) & (0x01);//1 if receier in moving base mode
						relposned1.refPosMiss 		= (flags3 >> 6) & (0x01);//1 if extrapolated reference position used to compute moving base in this epoch
						relposned1.refObsMiss 		= (flags3 >> 7) & (0x01);//1 if extrapolated reference observations used to compute moving base in this epoch
						relposned1.relPosHeadingValid = (flags3 >> 8) & (0x01);//1 if relative position components and accuracies valid
						relposned1.relPosNormalized = (flags3 >> 9) & (0x01);//1 if components of relative position vector are normalized
						
						relposned1.millisNow = millis();
						relposned1.microsNow = micros();

						Err = RELPOSNED_OK;
						GPS_step = 0;

						printClearData(printClearGPS);
						  
						if(DEBM8N_END)
						{
							SerialDeb.println();
							SerialDeb.print("BUFFER: ");
							SerialDeb.println();
						}

					break;
					}
			}
		}
	}
	  
	timediff = millis() - GPS_timer;
	if(timediff > 50000)
	{
		Err = 4;//Time out (>5s) reinit GPS
		GPS_timer = millis();
		//SerialDeb.println(timediff);
	}
	return Err;//0 = OK	
}
/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long (Lo -> Hi)
int32_t MagicBoat_GPS_F9X::Bytes2Long(unsigned char Buffer[])
{
	longUnion.byte[3] = *(Buffer+3);
	longUnion.byte[2] = *(Buffer+2);
	longUnion.byte[1] = *(Buffer+1);
	longUnion.byte[0] = *Buffer;
	return(longUnion.dword);
}
int32_t MagicBoat_GPS_F9X::int32parse(unsigned char Buffer[])
{
    int32Union.byte[3] = *(Buffer+3);
    int32Union.byte[2] = *(Buffer+2);
    int32Union.byte[1] = *(Buffer+1);
    int32Union.byte[0] = *Buffer;
    return(int32Union.int32value);
}
uint32_t MagicBoat_GPS_F9X::uint32parse(unsigned char Buffer[])
{
    uint32Union.byte[3] = *(Buffer+3);
    uint32Union.byte[2] = *(Buffer+2);
    uint32Union.byte[1] = *(Buffer+1);
    uint32Union.byte[0] = *Buffer;
    return(uint32Union.uint32value);
}
int16_t MagicBoat_GPS_F9X::int16parse(unsigned char Buffer[])
{
    int16Union.byte[1] = *(Buffer+1);
    int16Union.byte[0] = *Buffer;
    return(int16Union.int16value);
}
uint16_t MagicBoat_GPS_F9X::uint16parse(unsigned char Buffer[])
{
    uint16Union.byte[1] = *(Buffer+1);
    uint16Union.byte[0] = *Buffer;
    return(uint16Union.uint16value);
}

void MagicBoat_GPS_F9X::printClearData(uint8_t on_off)
{
	if(on_off)
	{
		SerialDeb.write(init_buffer, init_buffer_length);
		SerialDeb.write(MTK_buffer, MTK_payload_counter);
	}
} 

void MagicBoat_GPS_F9X::printClearGPS_ON()
{
	printClearGPS = 1;
}

void MagicBoat_GPS_F9X::printClearGPS_OFF()
{
	printClearGPS = 0;
}

void MagicBoat_GPS_F9X::ucenter_ON()
{
	ucenter = 1;
}

void MagicBoat_GPS_F9X::ucenter_OFF()
{
	ucenter = 0;
}

MagicBoat_GPS_F9X GPS;
