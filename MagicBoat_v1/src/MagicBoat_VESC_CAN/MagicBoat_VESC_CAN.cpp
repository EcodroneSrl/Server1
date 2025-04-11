#include "MagicBoat_VESC_CAN.h"
#include "FlexCAN_T4.h"
//#include "Arduino.h"

//Constructor

MagicBoat_VESC_CAN::MagicBoat_VESC_CAN()
{
	
}

void MagicBoat_VESC_CAN::initCAN()
{
	Can1.begin();
	Can1.setBaudRate(CAN_BPS);
}

void MagicBoat_VESC_CAN::begin(uint8_t IDMotor) //Inizializza il CAN bus n.1 sulla Teensy 4.1, setta i bps e acquisisce l'ID del motore
												//Inizializza gli oggetti per fare filtro di Kalman sui valori di corrente e RPM
{
	f_motor_current.setParam(qn_i,rn_i,pn_i,xn_i,kn_i);//sett parametri kalman filter
	f_rpm.setParam(qn_r,rn_r,pn_r,xn_r,kn_r);//sett parametri kalman filter
	motorID = IDMotor;
}

void MagicBoat_VESC_CAN::writeCAN(uint8_t type, int32_t value) // Funzione che scrive sul CANBUS i messaggi per comandare il motore
{
	CAN_message_t msg;
	
	//L'ID per dare i comandi deve avere al penultimo byte il tipo di controllo (corrente, ERPM) e all'ultimo byte l'ID del motore.
	uint64_t msgID = (type << 8) | (motorID); 
	
	msg.len = 4;
	msg.flags.extended = 1;
	msg.id = (msgID);
	
	//SerialDeb.println(msg.id, HEX);	
	
	msg.buf[3] = 0x00; 
	msg.buf[2] = 0x00;
	msg.buf[1] = 0x00; 
	msg.buf[0] = 0x00; 
	
	msg.buf[3] = (value) & (0xFF);
	msg.buf[2] = (value >> 8) & (0xFF);
	msg.buf[1] = (value >> 16) & (0xFF);
	msg.buf[0] = (value >> 24) & (0xFF);

	//Scrivo il messaggio che ho creato sul bus
	Can1.write(msg);
	
}	

void MagicBoat_VESC_CAN::printCANrx()  //Funzione di debug che stampa sulla seriale qualsiasi messaggio venga ricevuto sul CAN bus
{
	CAN_message_t msg;
	
	if(Can1.read(msg)){
		
		SerialDeb.print("LEN: "); 
		SerialDeb.print(msg.len);
		SerialDeb.print(" ");
		SerialDeb.print(" ID: "); 
		SerialDeb.print(msg.id, HEX);
		SerialDeb.print(" ");
		SerialDeb.print(" EXT: "); 
		SerialDeb.print(msg.flags.extended);
		SerialDeb.print(" BUF: ");
	 
	  for (byte i = 0; i < sizeof(msg.buf); i++)
	  {
		SerialDeb.print(msg.buf[i], HEX); 
		SerialDeb.print(" ");
	  }
	  SerialDeb.println("");
	}
}

void MagicBoat_VESC_CAN::setCurrent(int32_t value) //Funzione che pilota il motore in corrente
{
	writeCAN(TYPE_CURRENT, value);
}

void MagicBoat_VESC_CAN::setERPM(int32_t value)	//Funzione che pilota il motore in ERPM
{
	writeCAN(TYPE_ERPM, value);	
}


//Funzione che acquisisce i messaggi di telemetria 1,2,3,4,5 inviati dal VESC (da questi la misura di corrente non dava valori accettabili)
//Acquisisce anche un secondo buffer di telemetria più accurato, inviando periodicamente una richiesta

uint8_t MagicBoat_VESC_CAN::updateCANStatus()
{
	uint8_t update_status = 0;
	CAN_message_t msg;
	if(Can1.read(msg))
	{
		uint8_t stat_msg_type = (msg.id >> 8) & (0xFF);
		uint8_t motor_num = (msg.id) & (0xFF);
		if(motor_num == motorID)				
		switch(stat_msg_type)
			{
				case STATUS_PACKET_ONE:
					RPM     = get32buffer(msg,0);
					current = (float)get16buffer(msg,4)/10.0;
					duty    = (float)(get16buffer(msg,6)/1000.0);
					update_status = 1;
					break;
					
				case STATUS_PACKET_TWO:
					amp_hours     	  = (float)get32buffer(msg,0)/1e4;
					amp_hours_charged = (float)get32buffer(msg,4)/1e4;
					break;
					update_status = 2;
					
				case STATUS_PACKET_THREE:
					watt_hours     	   = (float)get32buffer(msg,0)/1e4;
					watt_hours_charged = (float)get32buffer(msg,4)/1e4;
					break;
					update_status = 3;
			
				case STATUS_PACKET_FOUR:
					temp_fet 	= (float)get16buffer(msg,0)/10.0;
					temp_motor  = (float)get16buffer(msg,2)/10.0;
					current_in  = (float)get16buffer(msg,4)/10.0;
					pid_pos_now = (float)get16buffer(msg,6)/50.0;
					break;
					update_status = 4;
					
				case STATUS_PACKET_FIVE:
					tacho_value     = (float)get32buffer(msg,0);
					v_in = (float)get16buffer(msg,4)/1e1;
					break;
					update_status = 5;
					
			}
	}
	return update_status;
}

uint8_t MagicBoat_VESC_CAN::updateCAN_Tel()
{
	uint8_t is_updated = 0;
	sendTelemetryRequest(time_update_tel); //Funzione che richiede ogni tot ms i dati di telemetria Long Buffer
	is_updated = 1;
	return is_updated;
}

void MagicBoat_VESC_CAN::setTelTime(uint32_t time_tel)
{
	time_update_tel = time_tel;
}

void MagicBoat_VESC_CAN::sendRequest() //Funzione che manda una richiesta al driver per ricevere il buffer di telemetria accurato
{
		
	CAN_message_t msg;
	msg.id = (0x00000800)|(motorID);
	msg.flags.extended = 1;
	msg.len = 3;
	
	msg.buf[0] = motorID;
	msg.buf[1] = 0x00;
	msg.buf[2] = 0x04;
	
	Can1.write(msg);
	
}

void MagicBoat_VESC_CAN::sendTelemetryRequest(uint32_t timer_request)
{
if((millis()-time_request) > timer_request) 
	{
		time_request = millis();
		sendRequest();
	}
}



uint8_t MagicBoat_VESC_CAN::saveLongBuffer(CAN_message_t msg) // Funzione che prende gli 11 messaggi di risposta alla richiesta di telemetria e li salva in altrettanti array
{	
	uint8_t is_saved = 0;
	//CAN_message_t msg;
	uint64_t telemetryID = (0x00000500)|(motorID);

  //Serial.println("saveLongBuffer");
  if(msg.id == telemetryID)
  {
    if(msg.buf[0] == 0x00 && msg.buf[1] == 0x04)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_1[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x07)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_2[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x0E)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_3[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x15)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_4[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x1C)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_5[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x23)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_6[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x2A)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_7[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x31)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_8[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x38)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_9[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x3f)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_10[i] = msg.buf[i]; 
      }
    }
    else if(msg.buf[0] == 0x46)
    {
      for(int i = 0; i < 8; i++)
      {
        tel_11[i] = msg.buf[i]; 
      }
      is_saved = 1;
    }		
  }
	return is_saved;
}

uint8_t MagicBoat_VESC_CAN::parseLongBuffer(CAN_message_t msg) //Funzione che fa il parsing dei valori salvati negli array e tira fuori le corrispondenti grandezze misurate
{	
	uint8_t is_parsed = 0;
	
	if(saveLongBuffer(msg))	
	{
		bufferTelData.temp_fet1 = ((tel_1[2]<<8)|(tel_1[3]))/10.0;
		bufferTelData.temp_motor1 = ((tel_1[4]<<8)|(tel_1[5]))/10.0;
		bufferTelData.avg_motor_current = ((tel_1[6]<<24)|(tel_1[7]<<16)|(tel_2[1]<<8)|(tel_2[2]))/100.0;
		bufferTelData.avg_input_current = ((tel_2[3]<<24)|(tel_2[4]<<16)|(tel_2[5]<<8)|(tel_2[6]))/100.0;
		bufferTelData.i_D = ((tel_2[7]<<24)|(tel_3[1]<<16)|(tel_3[2]<<8)|(tel_3[3]))/100.0;
		bufferTelData.i_Q = ((tel_3[4]<<24)|(tel_3[5]<<16)|(tel_3[6]<<8)|(tel_3[7]))/100.0;
		bufferTelData.duty_cycle = ((tel_4[1]<<8)|(tel_4[2]))/1000.0;
		bufferTelData.rpm = (tel_4[3]<<24)|(tel_4[4]<<16)|(tel_4[5]<<8)|(tel_4[6]);
		bufferTelData.voltage = ((tel_4[7]<<8)|(tel_5[1]))/10.0;
		bufferTelData.amp_h = ((tel_5[2]<<24)|(tel_5[3]<<16)|(tel_5[4]<<8)|(tel_5[5]))/10000.0;
		bufferTelData.amp_h_ch = ((tel_5[6]<<24)|(tel_5[7]<<16)|(tel_6[1]<<8)|(tel_6[2]))/10000.0;
		bufferTelData.watt_h = ((tel_6[3]<<24)|(tel_6[4]<<16)|(tel_6[5]<<8)|(tel_6[6]))/10000.0;
		bufferTelData.watt_h_ch = ((tel_6[7]<<24)|(tel_7[1]<<16)|(tel_7[2]<<8)|(tel_7[3]))/10000.0;
		bufferTelData.odometer = (tel_7[4]<<24)|(tel_7[5]<<16)|(tel_7[6]<<8)|(tel_7[7]);
		bufferTelData.odometer_abs = (tel_8[1]<<24)|(tel_8[2]<<16)|(tel_8[3]<<8)|(tel_8[4]);
		bufferTelData.fault = tel_8[5];
		bufferTelData.pid_pos = ((tel_8[6]<<24)|(tel_8[7]<<16)|(tel_9[1]<<8)|(tel_9[2]))/1e6;
		bufferTelData.controller_id = (tel_9[3]);
		bufferTelData.mostemp1 = ((tel_9[4]<<8)|(tel_9[5]))/10.0;
		bufferTelData.mostemp2 = ((tel_9[6]<<8)|(tel_9[7]))/10.0;
		bufferTelData.mostemp3 = ((tel_10[1]<<8)|(tel_10[2]))/10.0;
		bufferTelData.vd = ((tel_10[3]<<24)|(tel_10[4]<<16)|(tel_10[5]<<8)|(tel_10[6]))/1000.0;
		bufferTelData.vq = ((tel_10[7]<<24)|(tel_11[1]<<16)|(tel_11[2]<<8)|(tel_11[3]))/1000.0;
	
	//Filtraggio Kalman dei valori di corrente e ERPM
	fk_rpm = f_rpm.getStima(bufferTelData.rpm);
	fk_motor_current = f_motor_current.getStima(bufferTelData.avg_motor_current);
	is_parsed = 1;
	}
	
	return is_parsed;
}

int32_t MagicBoat_VESC_CAN::get32buffer(CAN_message_t msg, uint8_t start_byte) //Funzione per acquisire un buffer int32_t dei messaggi di stato
{
		int32_t buf0, buf1, buf2, buf3, buffer32;
		buf0 = ((uint32_t)msg.buf[start_byte]) << 24;
		buf1 = ((uint32_t)msg.buf[start_byte+1]) << 16;
		buf2 = ((uint32_t)msg.buf[start_byte+2]) << 8;
		buf3 = ((uint32_t)msg.buf[start_byte+3]);
		
		buffer32 = (buf0 | buf1 | buf2 | buf3);
		
		return buffer32;
}

int16_t MagicBoat_VESC_CAN::get16buffer(CAN_message_t msg, uint8_t start_byte) //Funzione per acquisire un buffer int16_t dei messaggi di stato
{
		int16_t buf0, buf1, buffer16;
		buf0 = (uint16_t)msg.buf[start_byte] << 8;
		buf1 = (uint16_t)(msg.buf[start_byte+1]);
		
		buffer16 = (buf0 | buf1);
		
		return buffer16;
}

uint32_t MagicBoat_VESC_CAN::getu32buffer(CAN_message_t msg, uint8_t start_byte) //Funzione per acquisire un buffer uint32_t dei messaggi di stato
{
		uint32_t buf0, buf1, buf2, buf3, buffer32;
		buf0 = ((uint32_t)msg.buf[start_byte]) << 24;
		buf1 = ((uint32_t)msg.buf[start_byte+1]) << 16;
		buf2 = ((uint32_t)msg.buf[start_byte+2]) << 8;
		buf3 = ((uint32_t)msg.buf[start_byte+3]);
		
		buffer32 = (buf0 | buf1 | buf2 | buf3);
		
		return buffer32;
}

uint16_t MagicBoat_VESC_CAN::getu16buffer(CAN_message_t msg, uint8_t start_byte) //Funzione per acquisire un buffer uint32_t dei messaggi di stato
{
		uint16_t buf0, buf1, buffer16;
		buf0 = (uint16_t)(msg.buf[start_byte] << 8) & (0xFF00);
		buf1 = (uint16_t)(msg.buf[start_byte+1]) & (0x00FF);
		
		buffer16 = (buf0 | buf1);
		
		return buffer16;
}

/* float MagicBoat_VESC_CAN::shift_buffer(uint8_t current_type)
{	
	uint16_t n = AVG_SIZE;
	switch(current_type)
	{
	case 0:
		
		for(int i = 0; i < n; i++)
		{
		motor_curr_array[i] = motor_curr_array[i+1];
		}
		
	motor_curr_array[n-1] = current;
	sum0 = 0;
	for(int i = 0; i<n; i++)
	{
		sum0 = sum0 + motor_curr_array[i];
	}
	
	if(index1 < n-1)
	{
		index1++;
	}
	else
	{
		index1 = 1;
	}
	return sum0;
	break;
	
	case 1:
		
		for(int i = 0; i < n; i++)
		{
		curr_in_array[i] = curr_in_array[i+1];
		}
		
	curr_in_array[n-1] = current_in;
	sum1 = 0;
	for(int i = 0; i<n; i++)
	{
		sum1 = sum1 + curr_in_array[i];
	}
	
	if(index1 < n-1)
	{
		index2++;
	}
	else
	{
		index2 = 1;
	}
	return sum1;
	break;
	}
} */

/* void MagicBoat_VESC_CAN::average(uint8_t current_type)
{
	uint16_t n = AVG_SIZE;
	
	switch(current_type)
	{
	case 0:
	sum0 = shift_buffer(current_type);
	avg_current = sum0/n;
	break;
	
	case 1:
	sum1 = shift_buffer(current_type);
	avg_current_in = sum1/n;
	break;
	}
} */


// Print personalizzabile dei messaggi di stato 1,2,3,4,5

/* void MagicBoat_VESC_CAN::printCAN_Status(uint8_t description, uint16_t timer_CAN_stat, uint8_t update, uint8_t head)
{
	if(update == UPDATE_ON)
	{
		updateCANStatus();
	}
	
	if ((millis()-time_CAN_stat)>timer_CAN_stat) 
	{
	time_CAN_stat = millis();
	
	if (description == DESCR_ON) 
	{
	SerialDeb.print("RPM:");
	}
    SerialDeb.print(RPM);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Curr:");
	}
    SerialDeb.print(current);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Duty:");
	}
    SerialDeb.print(duty);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Amp_h:");
	}
    SerialDeb.print(amp_hours);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Amp_h_ch:");
	}
    SerialDeb.print(amp_hours_charged);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Watt_h:");
	}
    SerialDeb.print(watt_hours);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Watt_h_ch:");
	}
    SerialDeb.print(watt_hours_charged);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Temp_FET:");
	}
    SerialDeb.print(temp_fet);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Temp_motor:");
	}
    SerialDeb.print(temp_motor);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Curr_in:");
	}
    SerialDeb.print(current_in);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("PID_pos:");
	}
    SerialDeb.print(pid_pos_now);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Tacho:");
	}
    SerialDeb.print(tacho_value);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("V_in:");
	}
    SerialDeb.print(v_in);
	SerialDeb.print("; ");
	
	
	if (head == HEAD_ON) 
	{
	SerialDeb.println("; ");
	}
	else
	{
	SerialDeb.print("; ");
	}
	}
}

// Print personalizzabile del buffer più accurato

void MagicBoat_VESC_CAN::printCANBuffer(uint8_t description, uint16_t timer_CANBuffer, uint8_t update, uint8_t head)
{
	if(update == UPDATE_ON)
	{
		updateCANStatus();
	}
	
	if ((millis()-time_CANBuffer)>timer_CANBuffer) 
	{
	time_CANBuffer = millis();
	
	if (description == DESCR_ON) 
	{
	SerialDeb.print("RPM:");
	}
    SerialDeb.print(rpm);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Curr:");
	}
    SerialDeb.print(avg_motor_current);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Duty:");
	}
    SerialDeb.print(duty_cycle);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Amp_h:");
	}
    SerialDeb.print(amp_h);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Amp_h_ch:");
	}
    SerialDeb.print(amp_h_ch);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Watt_h:");
	}
    SerialDeb.print(watt_h);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Watt_h_ch:");
	}
    SerialDeb.print(watt_h_ch);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Temp_FET:");
	}
    SerialDeb.print(temp_fet1);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Temp_motor:");
	}
    SerialDeb.print(temp_motor);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Curr_in:");
	}
    SerialDeb.print(avg_input_current);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Odo:");
	}
    SerialDeb.print(odometer);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("Odo abs:");
	}
    SerialDeb.print(odometer_abs);
	SerialDeb.print("; ");
	if (description == DESCR_ON) 
	{
	SerialDeb.print("V:");
	}
    SerialDeb.print(voltage);
	SerialDeb.print("; ");
	
	
	if (head == HEAD_ON) 
	{
	SerialDeb.println("; ");
	}
	else
	{
	SerialDeb.print("; ");
	}
	}
} */
