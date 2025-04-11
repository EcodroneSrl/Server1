#include "MagicBoat_BMS.h"

MagicBoat_BMS::MagicBoat_BMS(uint8_t num)
{
	num_cell_set = num;
}

void MagicBoat_BMS::request_basic_info() // Request for total voltage, current, Residual capacity, Balanced state, MOSFET control status (REG 0x03)
{
	if((!start_read_eprom))
	{
		uint8_t data[7] = {221, 165, 3, 0, 255, 253, 119}; //  DD  A5 03 00  FF  FD  77
		SerialBMS.write(data, 7);
	}
}

void MagicBoat_BMS::request_vcell_info() // Request for cells voltages (REG 0x04)
{

	if((!start_read_eprom))
	{
		uint8_t data[7] = {221, 165, 4, 0, 255, 252, 119}; // DD  A5  4 0 FF  FC  77
		SerialBMS.write(data, 7);
	}
}

void MagicBoat_BMS::setCellNum(uint8_t numcell_set) // Sets the battery pack number of cells 
{
	write_request_start();
	write_register(CELL_CNT_REG,numcell_set);
	write_request_end();
}

void MagicBoat_BMS::setCutOff(float povp, float povp_r, float puvp, float puvp_r) //Set over/under voltage cutoff thresholds and release thresholds
{
	write_request_start();
	write_register(POVP_REG,povp*100.0);
	write_register(POVP_REL_REG,povp_r*100.0);
	write_register(PUVP_REG,puvp*100.0);
	write_register(PUVP_REL_REG,puvp_r*100.0);
	write_request_end();
}

void MagicBoat_BMS::setMOSFET(uint8_t charge, uint8_t discharge) //Set over/under voltage cutoff thresholds and release thresholds
{
	uint8_t chg_dsg_set;
	chg_dsg_set = ((discharge << 1) | (charge));
	write_request_start();
	write_register(MOSFET_CONTROL_REG,chg_dsg_set);
	e_write_request_end();
}


uint8_t MagicBoat_BMS::get_bms_feedback()
{
	read_ok = 0;
	uint8_t incByte = 0;
	uint8_t num = SerialBMS.available();
		for(int i = 0; i < num; i++)
		{
 			incByte = SerialBMS.read();
             //Serial.print(incByte);
             //Serial.print(",");
			switch(stepBMS)
			{
				case 0:
					if(incByte == SOP)
					{
						stepBMS++;
						checksum = 0x10000;
						cks0 = 0;
						cks1 = 0;
					}
					else
					{
						stepBMS = 0;
					}
				break;
				
				case 1:
					if((incByte >= START_REG_RESP) && (incByte <= END_REG_RESP))
					{
						stepBMS++;
						regAddrResp = incByte;
					}
					
					else if((incByte == 1) && (start_register_write == 1))
					{
						stepBMS = 100;
					}
					
					else 
					{
						stepBMS = 0;
					}
				break;
				
				case 2:
					if(incByte == 0)
					{
						stepBMS++;
						checksum -= incByte;
					}
					else if(incByte == ERR )
					{
						Serial.println("Error: 0x80");
						stepBMS = 0;
					}
					else
					{
						stepBMS = 0;
					}
				break;
				
				case 3:
					Length = incByte;
					stepBMS++;
					index = 0;
					checksum -= incByte;
				break;
				
				case 4:
					if(index < Length)
					{
						inArr[index] = incByte;
						checksum -= incByte;
						index++;
						if(index == (Length))
						{
							stepBMS++;
						}
					}
					else if(Length == 0)
					{
						stepBMS = 0;
					}
				break;
				
				case 5:
					cks0 = ((checksum >> 8) & (0xFF));
					cks1 = ((checksum) & (0xFF));	
						// Serial.print(cks0);
						// Serial.print(",");
						// Serial.println(incByte);			
					if(incByte == cks0)
					{
						stepBMS++;
					}
					else
					{

						//Serial.println("checksum error0");
						stepBMS = 0;
					}
				break;
				
				case 6:		
						// Serial.print(cks1);
						// Serial.print(",");
						// Serial.println(incByte);		
					if(incByte == cks1)
					{
						stepBMS++;
					}
					else
					{
						stepBMS = 0;
					}
				break;
				
				case 7:
					if(incByte == EOP)
					{
						read_ok = 1;
						checksum = 0;
						// Serial.println("EOP");
						analBuffer();
						stepBMS = 0;
					}
				break;
				
				case 100:
					//Serial.println("CASE100");
				
				if((incByte == EOP) || (timer_start_register_write > time_start_register_write))
				{
					stepBMS = 0;
					start_register_write = 0;
					// Serial.println("re enable ");
				}
				break;
				
			}
			if(read_ok)
			{
				break;
			}
		}	
	return read_ok;
}

uint8_t MagicBoat_BMS::analBuffer()
{	
	switch(regAddrResp)
	{
		
		case REG_BASIC_INFO: // Raccoglie la risposta del registro Basic Info (0x03)
		
			// PACK VOLTAGE, bytes 0 and 1
			uint16_t pack_v_int; 
			pack_v_int = parse16(inArr[0], inArr[1]);
			BasicData.PackVoltage = ((float)(pack_v_int)) / 100.0; // convert to float and leave at 2 dec places


			// PACK CURRENT, bytes 2 and 3
			int16_t pack_c_int;
			pack_c_int = parse16(inArr[2], inArr[3]);
			BasicData.PackCurrent = ((float)(pack_c_int))/ 100.0; // convert to float and leave at 2 dec places
			

			//REMAINING CAPACITY, bytes 4 and 5
			uint16_t rem_cap_int;
			rem_cap_int = parse16(inArr[4], inArr[5]);
			BasicData.RemainCapacity = ((float)(rem_cap_int))/ 100.0; // convert to float and leave at 2 dec places


			//FULL CAPACITY, bytes 6 and 7
			uint16_t full_cap_int;
			full_cap_int = parse16(inArr[6], inArr[7]);
			BasicData.FullCapacity = ((float)(full_cap_int)) / 100.0; // convert to float and leave at 2 dec places

			
			//CHARGE/DISCHARGE CYCLES, bytes 8 and 9
			BasicData.CyclesNum = parse16(inArr[8], inArr[9]);
			
			//CELL BALANCE STATUS, bytes 12,13,14,15
			uint16_t bal_stat_int0;
			uint16_t bal_stat_int1;
			bal_stat_int0 = parse16(inArr[12], inArr[13]);
			bal_stat_int1 = parse16(inArr[14], inArr[15]);
			if(num_cell_set <= 16)
			{
				for(int i = 0; i < num_cell_set; i++)
				{
					BasicData.balance_status[i] = (bal_stat_int0 >> i) & (0x01);
				}
			}
			else if(num_cell_set > 16)
			{
				for(int i = 16; i < num_cell_set; i++)
				{
					BasicData.balance_status[i] = (bal_stat_int1 >> i) & (0x01);
				}
				for(int i = 0; i < 15; i++)
				{
					BasicData.balance_status[16-i+1] = (bal_stat_int0 >> i) & (0x01);
				}
			}
			
			//CURRENT ERRORS FLAGS, bytes 16 and 17
			uint16_t error_flags; 
			error_flags = parse16(inArr[16], inArr[17]);
			BasicData.cell_ov = ((error_flags) & (0x01));
			BasicData.cell_uv = ((error_flags >> 1) & (0x01));
			BasicData.pack_ov = ((error_flags >> 2) & (0x01));
			BasicData.pack_uv = ((error_flags >> 3) & (0x01));
			BasicData.charge_ot = ((error_flags >> 4) & (0x01));
			BasicData.charge_ut = ((error_flags >> 5) & (0x01));
			BasicData.discharge_ot = ((error_flags >> 6) & (0x01));
			BasicData.discharge_ut = ((error_flags >> 7) & (0x01));
			BasicData.charge_oc = ((error_flags >> 8) & (0x01));
			BasicData.discharge_oc = ((error_flags >> 9) & (0x01));
			BasicData.short_circuit = ((error_flags >> 10) & (0x01));
			BasicData.frontend_error = ((error_flags >> 11) & (0x01));
			BasicData.locked_FET = ((error_flags >> 12) & (0x01)); 

			//RSOC, byte 19
			BasicData.RSOC = (inArr[19]);

			
			//FET_STATUS, byte 20
			uint8_t FET_status;
			FET_status = (inArr[20]);
			BasicData.FET_charge 	  = ((FET_status) & (0x01)); 
			BasicData.FET_discharge = ((FET_status >> 1) & (0x01)); 
			


			//NTC 1, bytes 23 and 24
			uint16_t NTC1_int;
			NTC1_int = parse16(inArr[23], inArr[24]);
			BasicData.NTC1 = ((float)(NTC1_int - 2731)) / 10.00; // convert to float and leave at 2 dec places


			//NTC 2, bytes 25 and 26
			uint16_t NTC2_int; 
			NTC2_int = parse16(inArr[25], inArr[26]);
			BasicData.NTC2 = ((float)(NTC2_int - 2731)) / 10.00; // convert to float and leave at 2 dec places
		break;
			
		case REG_VCELL: // Raccoglie la risposta del registro VCell (0x04)
			
			for(int i = 0; i < num_cell_set; i++)
			{
				uint16_t var = parse16(inArr[2*i],inArr[2*i+1]);
				vcell[i] = ((float)var)/1000.0;
			}
		break;
		
		// ---------------------------------------------------------------------------------- //
		// ----- Da ora in poi i case per tutti i possibili registri letti della EEPROM ----- //
		// ---------------------------------------------------------------------------------- //
		
		case DESIGN_CAP_REG: //Pack Capacity, as designed
			uint16_t design_cap_int;
			design_cap_int	= parse16(inArr[0], inArr[1]);
			EepromData.design_cap = ((float)(design_cap_int)) / 100.0;
		break;
		
		case CYCLE_CAP_REG: //Cycle Capacity, as designed
			uint16_t cycle_cap_int;
			cycle_cap_int = parse16(inArr[0], inArr[1]);
			EepromData.cycle_cap = ((float)(cycle_cap_int)) / 100.0;
		break;
		
		case TEN_CAP_VOLTAGE_REG:
			uint16_t cap100_int;
			cap100_int = parse16(inArr[0], inArr[1]);
			EepromData.cap100 = ((float)(cap100_int)) / 1000.0;
		
		break;
		
		case EIGHT_CAP_VOLTAGE_REG:
			uint16_t cap80_int;
			cap80_int = parse16(inArr[0], inArr[1]);
			EepromData.cap80 = ((float)(cap80_int)) / 1000.0;
		break;
		
		case SIX_CAP_VOLTAGE_REG:
			uint16_t cap60_int;
			cap60_int = parse16(inArr[0], inArr[1]);
			EepromData.cap60 = ((float)(cap60_int)) / 1000.0;
		break;
		
		case FOUR_CAP_VOLTAGE_REG:
			uint16_t cap40_int;
			cap40_int = parse16(inArr[0], inArr[1]);
			EepromData.cap40 = ((float)(cap40_int)) / 1000.0;		
		break;
		
		case TWO_CAP_VOLTAGE_REG:
			uint16_t cap20_int;
			cap20_int = parse16(inArr[0], inArr[1]);
			EepromData.cap20 = ((float)(cap20_int)) / 1000.0;		
		break;
		
		case ZERO_CAP_VOLTAGE_REG:
			uint16_t cap0_int;
			cap0_int = parse16(inArr[0], inArr[1]);
			EepromData.cap0 = ((float)(cap0_int)) / 1000.0;	
		break;
		
		case DISCHARGE_RATE_REG:
			uint16_t dsg_rate_int;
			dsg_rate_int = parse16(inArr[0], inArr[1]);
			EepromData.dsg_rate = ((float)(dsg_rate_int)) / 10.0;	
		break;
		
		case CYCLE_CNT_REG:
			EepromData.cyc_cnt = parse16(inArr[0], inArr[1]);	
		break;
		
		case CHG_OT_REG:
			uint16_t chg_ot_int;
			chg_ot_int = parse16(inArr[0], inArr[1]);
			EepromData.chg_ot = ((float)(chg_ot_int - 2731)) / 10.00;	
		break;
		
		case CHG_OT_REL_REG:
			uint16_t chg_ot_rel_int;
			chg_ot_rel_int = parse16(inArr[0], inArr[1]);
			EepromData.chg_ot_rel = ((float)(chg_ot_rel_int - 2731)) / 10.00;	
		break;
		
		case CHG_UT_REG:
			int16_t chg_ut_int;
			chg_ut_int = parse16(inArr[0], inArr[1]);
			EepromData.chg_ut = ((float)(chg_ut_int - 2731)) / 10.00;	
		break;
		
		case CHG_UT_REL_REG:
			int16_t chg_ut_rel_int;
			chg_ut_rel_int = parse16(inArr[0], inArr[1]);
			EepromData.chg_ut = ((float)(chg_ut_rel_int - 2731)) / 10.00;	
		break;
		
		case DSG_OT_REG:
			uint16_t dsg_ot_int;
			dsg_ot_int = parse16(inArr[0], inArr[1]);
			EepromData.dsg_ot = ((float)(dsg_ot_int - 2731)) / 10.00;	
		break;
		
		case DSG_OT_REL_REG:
			uint16_t dsg_ot_rel_int;
			dsg_ot_rel_int = parse16(inArr[0], inArr[1]);
			EepromData.dsg_ot_rel = ((float)(dsg_ot_rel_int - 2731)) / 10.00;	
		break;
		
		case DSG_UT_REG:
			int16_t dsg_ut_int;
			dsg_ut_int = parse16(inArr[0], inArr[1]);
			EepromData.dsg_ut = ((float)(dsg_ut_int - 2731)) / 10.00;	
		break;
		
		case DSG_UT_REL_REG:
			int16_t dsg_ut_rel_int;
			dsg_ut_rel_int = parse16(inArr[0], inArr[1]);
			EepromData.dsg_ut = ((float)(dsg_ut_rel_int - 2731)) / 10.00;		
		break;
		
		case CHGT_DELAY_REG:
			EepromData.chg_ut_delay = inArr[0];
			EepromData.chg_ot_delay = inArr[1];		
		break;
		
		case DSGT_DELAY_REG:
			EepromData.dsg_ut_delay = inArr[0];
			EepromData.dsg_ot_delay = inArr[1];		
		break;
		
		case POVP_REG:
			uint16_t povp_int;
			povp_int = parse16(inArr[0], inArr[1]);
			EepromData.povp = ((float)(povp_int)) / 100.000;		
		break;
		
		case POVP_REL_REG:
			uint16_t povp_rel_int;
			povp_rel_int = parse16(inArr[0], inArr[1]);
			EepromData.povp_rel = ((float)(povp_rel_int)) / 100.000;		
		break;
		
		case PUVP_REG:
			uint16_t puvp_int;
			puvp_int = parse16(inArr[0], inArr[1]);
			EepromData.puvp = ((float)(puvp_int)) / 100.000;		
		break;
		
		case PUVP_REL_REG:
			uint16_t puvp_rel_int;
			puvp_rel_int = parse16(inArr[0], inArr[1]);
			EepromData.puvp_rel = ((float)(puvp_rel_int)) / 100.000;	
		break;
		
		case COVP_REG:
			uint16_t covp_int;
			covp_int = parse16(inArr[0], inArr[1]);
			EepromData.covp = ((float)(covp_int)) / 1000.000;
		break;
		
		case COVP_REL_REG:
			uint16_t covp_rel_int;
			covp_rel_int = parse16(inArr[0], inArr[1]);
			EepromData.covp_rel = ((float)(covp_rel_int)) / 1000.000;		
		break;
		
		case CUVP_REG:
			uint16_t cuvp_int;
			cuvp_int = parse16(inArr[0], inArr[1]);		
			EepromData.cuvp = ((float)(cuvp_int)) / 1000.000;			
		break;
		
		case CUVP_REL_REG:
			uint16_t cuvp_rel_int;
			cuvp_rel_int = parse16(inArr[0], inArr[1]);
			EepromData.cuvp_rel = ((float)(cuvp_rel_int)) / 1000.000;		
		break;
		
		case PACK_DELAY_REG:
			EepromData.puvp_delay = inArr[0];
			EepromData.povp_delay = inArr[1];			
		break;
		
		case CELL_DELAY_REG:
			EepromData.dsg_ut_delay = inArr[0];
			EepromData.dsg_ot_delay = inArr[1];		
		break;
		
		case COCP_REG:
			int16_t cocp_int;
			cocp_int = parse16(inArr[0], inArr[1]);
			EepromData.cocp = ((float)(cocp_int)) / 100.000;		
		break;
		
		case DOCP_REG:
			int16_t docp_int;
			docp_int = parse16(inArr[0], inArr[1]);
			EepromData.docp = ((float)(docp_int)) / 100.000;			
		break;
		
		case CHGC_DELAY_REG:
			EepromData.cocp_delay = inArr[0];
			EepromData.cocp_rel_delay = inArr[1];		
		break;
		
		case DSGC_DELAY_REG:
			EepromData.docp_delay = inArr[0];
			EepromData.docp_rel_delay = inArr[1];		
		break;
		
		case BAL_START_REG:
			uint16_t bal_start_int;
			bal_start_int = parse16(inArr[0], inArr[1]);
			EepromData.bal_start = ((float)(bal_start_int)) / 1000.000;		
		break;
		
		case BAL_WINDOW_REG:
			uint16_t bal_window_int;
			bal_window_int = parse16(inArr[0], inArr[1]);
			EepromData.bal_window = ((float)(bal_window_int)) / 1000.000;		
		break;
		
		case SHUNT_RES_REG:
			uint16_t shunt_res_int;
			shunt_res_int = parse16(inArr[0], inArr[1]);
			EepromData.shunt_res = ((float)(shunt_res_int)) / 10.00;	
		break;
		
		case FUNC_CONFIG_REG:
			uint16_t func_config_bits; 
			func_config_bits = parse16(inArr[0], inArr[1]);
			EepromData.switch_en = ((func_config_bits) & (0x01));
			EepromData.scrl_en = ((func_config_bits >> 1) & (0x01));
			EepromData.bal_en = ((func_config_bits >> 2) & (0x01));
			EepromData.ch_bal_en = ((func_config_bits >> 3) & (0x01));
			EepromData.led_en = ((func_config_bits >> 4) & (0x01));
			EepromData.led_num_en = ((func_config_bits >> 5) & (0x01));		
		break;
		
		case NTC_CONFIG_REG:
			uint16_t ntc_config_bits; 
			ntc_config_bits = parse16(inArr[0], inArr[1]);
			for(int i = 0; i<8; i++)
			{
				EepromData.ntc[i] = ((ntc_config_bits >> (i)) & (0x01));
			}			
		break;
		
		case CELL_CNT_REG:
			EepromData.cell_cnt = parse16(inArr[0], inArr[1]);		
		break;
		
		case COVP_HIGH_REG:
			uint16_t covp_high_int;
			covp_high_int = parse16(inArr[0], inArr[1]);
			EepromData.covp_high = ((float)(covp_high_int)) / 1000.00;		
		break;
		
		case CUVP_HIGH_REG:
			uint16_t cuvp_high_int;
			cuvp_high_int = parse16(inArr[0], inArr[1]);
			EepromData.cuvp_high = ((float)(cuvp_high_int)) / 1000.00;		
		break;
		
		case SHORT_DSGOC_HIGH_REG:
			uint8_t byte1;
			uint8_t byte2;
			byte1 = inArr[0];
			byte2 = inArr[1];
			
			EepromData.sc_dsgoc_x2 = ((byte1 >> 7) & (0x01));
			EepromData.sc_delay = ((byte1 >> 3) & (0x03));
			EepromData.sc = ((byte1) & (0x07));
			
			EepromData.dsgoc2_delay = ((byte2 >> 4) & (0x0F));
			EepromData.dsgoc2 = ((byte2) & (0x0F));
		break;
		
		case CXVP_HIGH_DELAY_SC_REG:
			uint8_t byte;
			byte = inArr[0];
			EepromData.sc_rel_time = inArr[1];
			EepromData.cuvp_high_delay = ((byte >> 6) & (0x03));
			EepromData.covp_high_delay = ((byte >> 4) & (0x03));
		break;
	}
	
	if(regAddrResp >= 0x03)
	{
		start_read_eprom = 0;
	}
	
	return regAddrResp;
}

uint8_t MagicBoat_BMS::en_dis_read_eprom(uint8_t en_dis)
//TOFIX: Nome confusionario, meglio set_read_eeprom(bool/uint8_t enable)
//TOFIX: Viene settato solo dalla eeprom, non viene mai impostato. Attualmente sembra inutile.
{
	if(en_dis == 0)
	{
		enable_read_all_eeprom = 0;
	}
	else
	{
		enable_read_all_eeprom = 1;
	}
	return 1;
}

uint8_t MagicBoat_BMS::readAllParam()
{
	//TOFIX: ne legge solo uno. ciclo o è voluto?
	//TOFIX: return sempre 1, meglio farla void.
	//TOFIX: il flag 
	if(enable_read_all_eeprom)
	{
	index_read_eeprom++;

		switch(index_read_eeprom)
		{
		case 1:
			read_eprom(DESIGN_CAP_REG);
		break;
		
		case 2:
			read_eprom(CYCLE_CAP_REG);
		break;
		
		case 3:
			read_eprom(TEN_CAP_VOLTAGE_REG);
		break;
		
		case 4:
			read_eprom(EIGHT_CAP_VOLTAGE_REG);
		break;
		
		case 5:
			read_eprom(SIX_CAP_VOLTAGE_REG);
		break;
		
		case 6:
			read_eprom(FOUR_CAP_VOLTAGE_REG);
		break;
		
		case 7:
			read_eprom(TWO_CAP_VOLTAGE_REG);
		break;
		
		case 8:
			read_eprom(ZERO_CAP_VOLTAGE_REG);
		break;
		
		case 9:
			read_eprom(DISCHARGE_RATE_REG);
		break;
		
		case 10:
			read_eprom(CYCLE_CNT_REG);
		break;
		
		case 11:
			read_eprom(CYCLE_CAP_REG);
		break;
		
		case 12:
			read_eprom(CHG_OT_REG);
		break;
		
		case 13:
			read_eprom(CHG_OT_REL_REG);
		break;
		
		case 14:
			read_eprom(CHG_UT_REG);
		break;
		
		case 15:
			read_eprom(CHG_UT_REL_REG);
		break;
		
		case 16:
			read_eprom(DSG_OT_REG);
		break;
		
		case 17:
			read_eprom(DSG_OT_REL_REG);
		break;
		
		case 18:
			read_eprom(DSG_UT_REG);
		break;
		
		case 19:
			read_eprom(DSG_UT_REL_REG);
		break;
		
		case 20:
			read_eprom(CHGT_DELAY_REG);
		break;
		
		case 21:
			read_eprom(DSGT_DELAY_REG);
		break;
		
		case 22:
			read_eprom(POVP_REG);
		break;
		
		case 23:
			read_eprom(POVP_REL_REG);
		break;
		
		case 24:
			read_eprom(PUVP_REG);
		break;
		
		case 25:
			read_eprom(PUVP_REL_REG);
		break;
		
		case 26:
			read_eprom(COVP_REG);
		break;
		
		case 27:
			read_eprom(COVP_REL_REG);
		break;
		
		case 28:
			read_eprom(CUVP_REG);
		break;
		
		case 29:
			read_eprom(CUVP_REL_REG);
		break;
		
		case 30:
			read_eprom(PACK_DELAY_REG);
		break;
		
		case 31:
			read_eprom(CELL_DELAY_REG);
		break;
		
		case 32:
			read_eprom(COCP_REG);
		break;
		
		case 33:
			read_eprom(DOCP_REG);
		break;
		
		case 34:
			read_eprom(CHGC_DELAY_REG);
		break;
		
		case 35:
			read_eprom(DSGC_DELAY_REG);
		break;
		
		case 36:
			read_eprom(BAL_START_REG);
		break;

		case 37:
			read_eprom(BAL_WINDOW_REG);
		break;
		
		case 38:
			read_eprom(SHUNT_RES_REG);
		break;
		
		case 39:
			read_eprom(FUNC_CONFIG_REG);
		break;
		
		case 40:
			read_eprom(NTC_CONFIG_REG);
		break;
		
		case 41:
			read_eprom(CELL_CNT_REG);
		break;
		
		case 42:
			read_eprom(COVP_HIGH_REG);
		break;
		
		case 43:
			read_eprom(CUVP_HIGH_REG);
		break;

		case 44:
			read_eprom(SHORT_DSGOC_HIGH_REG);
		break;	
		
		case 45:
			read_eprom(CXVP_HIGH_DELAY_SC_REG);
		break;
		
		}
		
		if(index_read_eeprom >= index_read_eeprom_max)
		{
			enable_read_all_eeprom = 0;
			index_read_eeprom = 0;
		}
	}
	return 1;
}


uint16_t MagicBoat_BMS::parse16(uint8_t highbyte, uint8_t lowbyte) // turns two bytes into a single long integer
{
  uint16_t a16bitvar = (highbyte);
  a16bitvar <<= 8; //Left shift 8 bits,
  a16bitvar = (a16bitvar | lowbyte); //OR operation, merge the two
  return a16bitvar;
}


void MagicBoat_BMS::read_eprom(uint8_t reg_Addr)
{
	start_read_eprom = 1;
	write_request_start();
	uint8_t data_read[7] = {221, 165, reg_Addr, 0, 255, uint8_t(0x100 - reg_Addr), 119};
	SerialBMS.write(data_read, 7);
	write_request_end();
}

//----------------------------------------------------------------------------

void MagicBoat_BMS::write_register(uint8_t regAddr, uint16_t value) //Write value on EEPROM register
{
	start_register_write = 1;
	uint8_t data_write[9] = {DW0, DW1, 0, DW3, 0, 0, 0, 0, DW8};
	uint16_t valuehigh = ((value) & (0xFF00)) >> 8;
	uint16_t valuelow  = ((value) & (0x00FF));
	
	data_write[2] = regAddr;
	data_write[4] = valuehigh;
	data_write[5] = valuelow;
	
	checksum_calc(data_write);
	
	data_write[6] = cks_temp0;
	data_write[7] = cks_temp1;
	SerialBMS.write(data_write, 9);
	delay(100);
	
	//  for(int i = 0; i<9; i++)
	// {
	// 	Serial.println(data_write[i],HEX);
	// } 
	
}

//----------------------------------------------------------------------------

void MagicBoat_BMS::write_request_start()
{
  
  //Serial.println("start");
  timer_start_register_write = millis();
  uint8_t data[9] = {221, 90, 0, 2, 86, 120, 255, 48, 119}; // DD 5A 00 02 56 78 FF 30 77
  SerialBMS.write(data, 9);
  delay(100);
}

//----------------------------------------------------------------------------

void MagicBoat_BMS::write_request_end()
{
  
  //Serial.println("end");
  uint8_t data[9] = {221, 90, 1, 2, 0, 0, 255, 253, 119}; // DD 5A 01 02 00 00 FF FD 77
  SerialBMS.write(data, 9);
  delay(100);
}

//----------------------------------------------------------------------------

void MagicBoat_BMS::e_write_request_end()
{

  uint8_t data[9] = {221, 90, 1, 2, 40, 40, 255, 173, 119};   // DD 5A 01 02 28 28 FF AD 77
  SerialBMS.write(data, 9);
  delay(100);
}

//----------------------------------------------------------------------------

uint8_t MagicBoat_BMS::checksum_calc(uint8_t *data)
{
	uint16_t cks_temp = 0xFFFF - data[2] - data[3] - data[4] - data[5]+1;
	cks_temp0 = (((cks_temp)&(0xFF00)) >> 8);
	cks_temp1 = (((cks_temp)&(0x00FF)));
	
	return 1;
}
