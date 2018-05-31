#if WILO 

const Pgrup wlp[] = {
	{"***;"," WILO-PUMPEN    ","       ", P&Wilomod,														ASCII_FORM,  0, P&hid1,	V0, 0, 0},
	{" 00;"," PUMPEN-INDEX   "," # 1-8 ", P&SerialDeviceNr,										 EXPAND_CHAR,  0, P&hid1,	V0, 0, EXP_WLP},	// Norbert 19.06.2014

	{"_01;"," BEZEICHNUNG    ","       ", P&BusPuPara[0].Name,							DYN_ASCII_FORM,  0, P&hid1,	V1, 0, EXP_WLP},
                                                                  			
	{" 02;"," BUS EINSCHALTEN"," 1=JA  ", P&BusPuPara[0].Funktion,					 	 		 US_CHAR,  0, P&hid1,	V0, 0, EXP_WLP},
                                                                  			
	{"*03;"," BUS            ","       ", P&BusPuPara[0].Funktion,						PU_FUNC_FORM,  0, P&hid1,	V0, 0, EXP_WLP},
	{" 04;"," ADRESSE        ","       ", P&BusPuPara[0].Adresse,					 	 		 	 US_CHAR,  0, P&hid1,	V0, 0, EXP_WLP},
                                                                  			
	{"*05."," Betr.-Art AUTO ","       ", P&BusPuPara[0].Betrieb,	  				 	 PU_OPR_FORM,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*->."," Betr.-Art Auto "," 0..3  ", P&BusPuPara[0].Betrieb,	 				 	 		 	 US_CHAR,  0, P&hid2,	V0, 0, EXP_WLP},
                                                                  			
	{"*06."," Regelart AUTO  ","       ", P&BusPuPara[0].Regelart, 					 	 PU_CTR_FORM,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*->."," Regelart Auto  "," 0..3  ", P&BusPuPara[0].Regelart,					 	 		 US_CHAR,  0, P&hid2,	V0, 0, EXP_WLP},
                                                                  			
	{"*07."," Sollwert AUTO  "," %     ", P&BusPuPara[0].Sollwert,					  			US_INT,  1, P&hid2,	V0, 0, EXP_WLP},

  {"*20;"," BETR.-ART SOLL ","       ", P&BusPuData[0].betriebsArtSoll,	 	DYN_ASCII_FORM, 15, P&hid1,	V1, 0, EXP_WLP},
  {"*->."," Betr.-Art Soll "," 0-3   ", P&modb_data[0].output_value1_temp,		 	 	US_INT,  0, P&hid2,	V0, 0, EXP_WLP},
  {"*->."," B-Art Soll Wilo","       ", P&modb_data[0].output_value[1],		 	 			US_INT,  0, P&hid4,	V0, 0, EXP_WLP},
  {"*21;"," BETR.-ART IST  ","       ", P&BusPuData[0].betriebsArtIst,	 	DYN_ASCII_FORM, 15, P&hid1,	V1, 0, EXP_WLP},
  {"*->."," Betr.-Art Ist  "," 0-3   ", P&modb_data[0].Operation_Input, 			 		US_INT,  0, P&hid2,	V0, 0, EXP_WLP},

	{"*22;"," REGELART SOLL  ","       ", P&BusPuData[0].regelArtSoll,	 		DYN_ASCII_FORM, 15, P&hid1,	V1, 0, EXP_WLP},
	{"*->."," Regelart Soll  "," 0-3   ", P&modb_data[0].output_value2_temp, 		 	 	US_INT,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*->."," Regel.Soll Wilo","       ", P&modb_data[0].output_value[2],		 	 		 	US_INT,  0, P&hid4,	V0, 0, EXP_WLP},
	{"*23;"," REGELART IST   ","       ", P&BusPuData[0].regelArtIst,	 			DYN_ASCII_FORM, 15, P&hid1,	V1, 0, EXP_WLP},
	{"*->."," Regelart Ist   "," 0-3   ", P&modb_data[0].Control_Input,	 		 				US_INT,  0, P&hid2,	V0, 0, EXP_WLP},

	{"*24;"," SOLLWERT SOLL  "," %     ", P&modb_data[0].output_value0_temp,  			US_INT,  1, P&hid1,	V1, 0, EXP_WLP},	// mit Komma !
	{"*25."," SOLLWERT IST   "," %     ", P&modb_data[0].SetPoint_Input, 		 				US_INT,  1, P&hid1,	V1, 0, EXP_WLP},	// mit Komma !

	{"*30;"," DREHZAHL       "," 1/min ", P&modb_data[0].input_reg5, 								US_INT,  0, P&hid1,	V1, 0, EXP_WLP},
	{"*31;"," DIFFERENZDRUCK "," mWS   ", P&modb_data[0].input_reg0, 								US_INT,	 1, P&hid1,	V1, 0, EXP_WLP},
	{"*32;"," FOERDERSTROM   "," m/h  ", P&modb_data[0].input_reg1, 								US_INT,	 1, P&hid1,	V1, 0, EXP_WLP},
	{"*33;"," MEDIEN-TEMP.   "," C    ", P&modb_data[0].input_reg6_celsius, 				 S_INT,  1, P&hid1,	V1, 0, EXP_WLP},
	{"*->."," Medien-Temp.   "," K     ", P&modb_data[0].input_reg6, 								US_INT,  1, P&hid2,	V0, 0, EXP_WLP},
	{"*34;"," LEISTUNG       "," W     ", P&modb_data[0].input_reg3, 								US_INT,  0, P&hid1,	V1, 0, EXP_WLP},
	{"*35;"," ENERGIEVERBR.  "," kWh   ", P&modb_data[0].input_reg2, 								US_INT,  0, P&hid1, V1, 0, EXP_WLP},
	{"*36;"," BETRIEBSZEIT   "," h     ", P&modb_data[0].op_hours, 									US_INT,  0, P&hid1, V0, 0, EXP_WLP},

	{"*40;"," SERVICE-MELDUNG","       ", P&modb_data[0].message1_text,			DYN_ASCII_FORM, 15, P&hid1,	V1, 0, EXP_WLP},	// 28.10.2014 AnFre
	{"*->."," Service Message","       ", P&modb_data[0].input_reg15,   				 DIGR_FORM, 16, P&hid2,	V0, 0, EXP_WLP},
	{"*41;"," ALARM-MELDUNG  ","       ", P&modb_data[0].message2_text,		  DYN_ASCII_FORM, 15, P&hid1,	V1, 0, EXP_WLP},
	{"*->."," Alarm-Meldung  ","       ", P&modb_data[0].input_reg16,   				 DIGR_FORM, 16, P&hid2,	V0, 0, EXP_WLP},
	{"*42;"," ALARM-STATUS   ","       ", P&modb_data[0].message3_text,   	DYN_ASCII_FORM, 15, P&hid1,	V1, 0, EXP_WLP},
	{"*->."," Alarm-Status   ","       ", P&modb_data[0].input_reg17,   				 DIGR_FORM, 16, P&hid2,	V0, 0, EXP_WLP},
	{"*43;"," PUMPEN-STATUS  ","       ", P&modb_data[0].message4_text,	    DYN_ASCII_FORM, 15, P&hid1,	V1, 0, EXP_WLP},
	{"*->."," Pumpen-Status  ","       ", P&modb_data[0].input_reg18,   				 DIGR_FORM, 16, P&hid2,	V0, 0, EXP_WLP},
	{"*44;"," STATUS DIAGNOSE","       ", P&modb_data[0].message5_text,  		DYN_ASCII_FORM, 15, P&hid1,	V1, 0, EXP_WLP},
	{"*->."," Status Diagnose","       ", P&modb_data[0].input_reg19,   				 DIGR_FORM, 16, P&hid2,	V0, 0, EXP_WLP},
	{"*45."," Mehrfach-MSG   ","       ", P&modb_data[0].message0,						 JANEIN_FORM,  0, P&hid2,	V0, 0, EXP_WLP},
	{" 47;"," RM-VERZOEGERUNG"," min   ", P&BusPuPara[0].RMverz,						 	 		 US_CHAR,  0, P&hid1,	V0, 0, EXP_WLP},
	{" ->."," RM-Verzoeg. ctr"," min/2 ", P&BusPuData[0].betriebRmCtr,	 						US_INT,  0, P&hid2,	V0, 0, EXP_WLP},
	{" 48."," BusCommandTimer","       ", P&BusPuPara[0].HoldingReg300,		 	 		 	 US_CHAR,  0, P&hid2,	V0, 0, EXP_WLP},

	{"*50."," Mains Current  "," A     ", P&modb_data[0].input_reg4,   							US_INT,  1, P&hid2,	V0, 0, EXP_WLP},
	{"*51."," Current Op-Mode","       ", P&modb_data[0].input_reg7,  							US_INT,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*52."," Pump Module    ","       ", P&modb_data[0].input_reg8,  							US_INT,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*53."," Pump Type      ","       ", P&modb_data[0].input_reg9,  							US_INT,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*54."," Max Speed      "," 1/min ", P&modb_data[0].input_reg10,  							US_INT,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*55."," Min Speed      "," 1/min ", P&modb_data[0].input_reg11,  							US_INT,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*56."," Max PWR Rating "," W     ", P&modb_data[0].input_reg14,  							US_INT,  0, P&hid2,	V0, 0, EXP_WLP},

	{"*64."," Support.Errors ","       ", P&modb_data[0].input_reg12,   				 DIGR_FORM, 16, P&hid2,	V0, 0, EXP_WLP},
	{"*65."," Supported SM   ","       ", P&modb_data[0].input_reg13,   				 DIGR_FORM, 16, P&hid2,	V0, 0, EXP_WLP},

//	{" 70."," w-modbus status","       ", P&modb_control[0].rx_errorflag,		BUS_ERROR_FORM,  0, P&hid2,	V0, 0, EXP_WLP},
	{" 70."," bus-timeout    ","       ", P&modb_control[0].bus_timeout, 		 			 US_CHAR,	 0, P&hid2,	V0, 0, EXP_WLP},
	{"*71."," exception?     ","       ", P&modb_control[0].exeption_code, 	 EXEPTION_FORM,  0, P&hid2,	V0, 0, EXP_WLP},	
	{"*72."," tx-error-status","       ", P&modb_control[0].tx_errorflag, 	BUS_ERROR_FORM,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*73."," tx-error-status","       ", P&modb_control[0].tx_errorflag, 				 US_CHAR,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*74."," rx-error status","       ", P&modb_control[0].rx_errorflag,		BUS_ERROR_FORM,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*75."," rx-error-status","       ", P&modb_control[0].rx_errorflag, 		 		 US_CHAR,  0, P&hid2,	V0, 0, EXP_WLP},
	{"*76."," rx-error-count ","       ", P&modb_control[0].error_count, 		 			 US_CHAR,  0, P&hid2,	V0, 0, EXP_WLP},

	{"*80;"," SM WILO-PUMPE ?","       ", P&BusPuData[0].puAlarm,							  JANEIN_FORM, 0, P&hid1, V1, 0, EXP_WLP},
	{"*81;"," SM MODBUS ?    ","       ", P&BusPuData[0].busSm,				 					JANEIN_FORM, 0, P&hid1, V1, 0, EXP_WLP},
	{"*->."," SM Modbus war  ","       ", P&modb_data[0].busSmWar,				 					US_CHAR, 0, P&hid2,	V0, 0, EXP_WLP},
	{"*82;"," SM RM PUMPE ?  ","       ", P&BusPuData[0].betriebSm,		 					JANEIN_FORM, 0, P&hid1, V1, 0, EXP_WLP},

	{"*89;"," HANDBETRIEB ?  ","       ", P&BusPuPara[0].Hand,		 			 				JANEIN_FORM, 0, P&hid1,	V1, 0, EXP_WLP},
// ***AnFre 29.08.2013: HAND-Einstellungen in der jeweiligen Parameter-Gruppe                                
//	{" 99;"," HAND/AUTOMATIK "," HAND=1", P&BusPuPara[0].Hand,				 		 	 				US_CHAR, 0, P&hid1,	V1, 0 0},
//	{" ->;"," BETR.-ART HAND "," 0..1  ", P&BusPuPara[0].BetriebHand,		 	 					US_CHAR, 0, P&hid1,	V1, 0 0},
//	{" ->;"," REGELART  HAND "," 0..3  ", P&BusPuPara[0].RegelartHand,		 	 				US_CHAR, 0, P&hid1,	V1, 0 0},
//	{" ->;"," SOLLWERT HAND  "," %     ", P&BusPuPara[0].SollHand,		 							 US_INT, 1, P&hid1,	V1, 0 0},
// 63 Parameter-Zeilen mit Expansion. In UserKonv.c: 	if( pnum > 1 && pnum <= 64 )	

// von WiloServParli
	{"*A1."," ANHANG WILO    ","       ", P&Anhang_Wilo,												 ASCII_FORM, 0, P&hid2,	V0, 0, 0},
	{"*->."," Pumpen-Param.A ","       ", P&ModbusBaudWilo,							 						US_CHAR, 0, P&hid2,	V0, 0, 0},
	{" ->."," Pumpen-Param.C ","       ", P&ModbusSioWilo,							 						US_CHAR, 0, P&hid2,	V0, 0, 0},
	{" ->."," Bus-Neustart   ","       ", P&Bus_restart, 		 				 		 						US_CHAR, 0, P&hid2,	V0, 0, 0},

  {"*->."," bus-timeout    ","       ", P&bus_timeout_com, 									 JANEIN_FORM,  0, P&hid2,	V0, 0, 0},
  {"*->."," pumpen-timeout ","       ", P&pu_timeout_com, 									 JANEIN_FORM,  0, P&hid2,	V0, 0, 0},
	{"*->."," slave-adresse  ","       ", P&modb_curr_adr, 		 					 		 				US_CHAR, 0, P&hid2,	V0, 0, 0},
	{" ->."," state-control  ","       ", P&modb_state_control, 		 				 				US_CHAR, 0, P&hid2,	V0, 0, 0},
	{"*->."," pu-nr/tsk-state","       ",						0,											MODB_STATE_FORM, 0, P&hid2,	V0, 0, 0},
	{"*->."," modb-rx-status ","       ", P&modb_rx_status, 				 								US_CHAR, 0, P&hid2,	V0, 0, 0},
	{" ->."," task-timer     ","       ", P&TaskTimer1,															US_CHAR, 0, P&hid2,	V0, 0, 0},
};
#endif