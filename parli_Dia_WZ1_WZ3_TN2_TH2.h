const Pgrup dia[] = {
	{"***."," DIAGNOSE       ","       ", P&Diamod,							ASCII_FORM, 0, P&hid2,	V0, 0, 0},

	{" 01."," DIAGNOSE BEGINN","       ", P&dis.begin,		 		 	 DATE_FORM, 1, P&hid2,	V0, 0, 0},	
	{" 02."," DIAGNOSE ENDE  ","       ", P&dis.end, 			 		 	 DATE_FORM, 1, P&hid2,	V0, 0, 0}, 
	
	{" **."," WZ1: KESSEL    ","       ", P&DiaLeistmod,				ASCII_FORM, 0, P&hid2,	V0, 0, 0},
	{" 10."," LEI-BEREICH 1.1"," kW    ", P&dis.leistgBereich[0],		US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT LEIBER 1.1"," h     ", P&did.leistgBereich_h[0],	US_INT,	0, P&hid2,	V0, 0, 0},
	{" 11."," LEI-BEREICH 1.2"," kW    ", P&dis.leistgBereich[1],		US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT LEIBER 1.2"," h     ", P&did.leistgBereich_h[1],	US_INT,	0, P&hid2,	V0, 0, 0},
	{" 12."," LEI-BEREICH 1.3"," kW    ", P&dis.leistgBereich[2],		US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT LEIBER 1.3"," h     ", P&did.leistgBereich_h[2],	US_INT,	0, P&hid2,	V0, 0, 0},
	{" 13."," LEI-BEREICH 1.4"," kW    ", P&dis.leistgBereich[3],		US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT LEIBER 1.4"," h     ", P&did.leistgBereich_h[3],	US_INT,	0, P&hid2,	V0, 0, 0},
	{"*14."," LEISTUNG 1 MAX "," kW    ", P&did.wmLeistg_max,				US_INT,	0, P&hid2,	V0, 0, 0},	
	
	{" **."," WZ1: KESSEL    ","       ", P&DiaVolstmod,				ASCII_FORM, 0, P&hid2,	V0, 0, 0},
	{" 15."," VOL-BEREICH 1.1"," m/h  ", P&dis.flowBereich[0],		 US_LONG,	3, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT VOLBER 1.1"," h     ", P&did.flowBereich_h[0],		US_INT,	0, P&hid2,	V0, 0, 0},
	{" 16."," VOL-BEREICH 1.2"," m/h  ", P&dis.flowBereich[1],		 US_LONG,	3, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT VOLBER 1.2"," h     ", P&did.flowBereich_h[1],		US_INT,	0, P&hid2,	V0, 0, 0},										 	
	{" 17."," VOL-BEREICH 1.3"," m/h  ", P&dis.flowBereich[2],		 US_LONG,	3, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT VOLBER 1.3"," h     ", P&did.flowBereich_h[2],		US_INT,	0, P&hid2,	V0, 0, 0},										 	
	{" 18."," VOL-BEREICH 1.4"," m/h  ", P&dis.flowBereich[3],		 US_LONG,	3, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT VOLBER 1.4"," h     ", P&did.flowBereich_h[3],		US_INT,	0, P&hid2,	V0, 0, 0},	
	{"*19."," VOL-STROM 1 MAX"," m/h  ", P&did.flow_max,	   		 	 US_LONG, 3, P&hid2,	V0, 0, 0}, 

	{" **."," WZ1: KESSEL    ","       ", P&DiaVorlmod,					ASCII_FORM, 0, P&hid2,	V0, 0, 0},
	{"*20."," TVKES MAXIMUM  "," C    ", P&did.tvpMax.temp, 				 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*->."," DATUM TVKES MAX","       ", P&did.tvpMax.tag,		 	 DATE_FORM, 1, P&hid2,	V0, 0, 0},		
	{"*->."," Ta    TVKES MAX"," C    ", P&did.tvpMax.ta,				ANA_FORM, 1, P&hid2,	V0, 0, 0},
	{"*21."," TVKES MINIMUM  "," C    ", P&did.tvpMin.temp, 				 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*->."," DATUM TVKES MIN","       ", P&did.tvpMin.tag,		 	 DATE_FORM, 1, P&hid2,	V0, 0, 0},		
	{"*->."," Ta    TVKES MIN"," C    ", P&did.tvpMin.ta,				ANA_FORM, 1, P&hid2,	V0, 0, 0},														 	
	
	{" **."," WZ3: NETZ      ","       ", P&DiaLeistmod,				ASCII_FORM, 0, P&hid2,	V0, 0, 0},
	{" 25."," LEI-BEREICH 3.1"," kW    ", P&dis.leistgBereich2[0],		US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT LEIBER 3.1"," h     ", P&did.leistgBereich_h2[0],	US_INT,	0, P&hid2,	V0, 0, 0},
	{" 26."," LEI-BEREICH 3.2"," kW    ", P&dis.leistgBereich2[1],		US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT LEIBER 3.2"," h     ", P&did.leistgBereich_h2[1],	US_INT,	0, P&hid2,	V0, 0, 0},
	{" 27."," LEI-BEREICH 3.3"," kW    ", P&dis.leistgBereich2[2],		US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT LEIBER 3.3"," h     ", P&did.leistgBereich_h2[2],	US_INT,	0, P&hid2,	V0, 0, 0},
	{" 28."," LEI-BEREICH 3.4"," kW    ", P&dis.leistgBereich2[3],		US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT LEIBER 3.4"," h     ", P&did.leistgBereich_h2[3],	US_INT,	0, P&hid2,	V0, 0, 0},
	{"*29."," LEISTUNG 3 MAX "," kW    ", P&did.wmLeistg_max2,				US_INT,	0, P&hid2,	V0, 0, 0},	
	
	{" **."," WZ3: NETZ      ","       ", P&DiaVolstmod,				ASCII_FORM, 0, P&hid2,	V0, 0, 0},
	{" 30."," VOL-BEREICH 3.1"," m/h  ", P&dis.flowBereich2[0],		 US_LONG,	3, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT VOLBER 3.1"," h     ", P&did.flowBereich_h[0],		US_INT,	0, P&hid2,	V0, 0, 0},
	{" 31."," VOL-BEREICH 3.2"," m/h  ", P&dis.flowBereich2[1],		 US_LONG,	3, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT VOLBER 3.2"," h     ", P&did.flowBereich_h2[1],		US_INT,	0, P&hid2,	V0, 0, 0},										 	
	{" 32."," VOL-BEREICH 3.3"," m/h  ", P&dis.flowBereich2[2],		 US_LONG,	3, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT VOLBER 3.3"," h     ", P&did.flowBereich_h2[2],		US_INT,	0, P&hid2,	V0, 0, 0},										 	
	{" 33."," VOL-BEREICH 3.4"," m/h  ", P&dis.flowBereich2[3],		 US_LONG,	3, P&hid2,	V0, 0, 0},
	{"*->."," ZEIT VOLBER 3.4"," h     ", P&did.flowBereich_h2[3],		US_INT,	0, P&hid2,	V0, 0, 0},	
	{"*34."," VOL-STROM 3 MAX"," m/h  ", P&did.flow_max2,	   		 	 US_LONG, 3, P&hid2,	V0, 0, 0}, 

	{" **."," WZ3: NETZ      ","       ", P&DiaVorlmod,					ASCII_FORM, 0, P&hid2,	V0, 0, 0},
	{"*35."," TN1 MAXIMUM    "," C    ", P&did.tvpMax2.temp, 				 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*->."," DATUM  TN1 MAXI","       ", P&did.tvpMax2.tag,		 	 DATE_FORM, 1, P&hid2,	V0, 0, 0},		
	{"*->."," Ta     TN1 MAXI"," C    ", P&did.tvpMax2.ta,				ANA_FORM, 1, P&hid2,	V0, 0, 0},
	{"*36."," TN1 MINIMUM    "," C    ", P&did.tvpMin2.temp, 				 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*->."," DATUM  TN1 MINI","       ", P&did.tvpMin2.tag,		 	 DATE_FORM, 1, P&hid2,	V0, 0, 0},		
	{"*->."," Ta     TN1 MINI"," C    ", P&did.tvpMin2.ta,				ANA_FORM, 1, P&hid2,	V0, 0, 0},														 	

#if WWANZ > 0
//	{"*30."," ANZ. SP.-LADUNG","       ", P&did.wwlad_anz, 					US_INT, 0, P&hid2,	V0, 0, 0},
//	{"*31."," SP.-LADUNG MAX "," min   ", P&did.wwlad_time,					US_INT, 0, P&hid2,	V0, 0, 0},										 	
//	{"*32."," ANZ. WW-VORRANG","       ", P&did.wwvor_anz, 					US_INT, 0, P&hid2,	V0, 0, 0},
//	{"*33."," WW-VORRANG MAX "," min   ", P&did.wwvor_time,					US_INT, 0, P&hid2,	V0, 0, 0},
#endif

	{" **."," MITTLERE TEMP. ","       ", P&DiaTH2mod,					ASCII_FORM, 0, P&hid2,	V0, 0, 0},
	{"*41."," TH2 JANUAR     "," C    ", P&TH2Mittel[0], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*42."," TH2 FEBRUAR    "," C    ", P&TH2Mittel[1], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*43."," TH2 MAERZ      "," C    ", P&TH2Mittel[2], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*44."," TH2 APRIL      "," C    ", P&TH2Mittel[3], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*45."," TH2 MAI        "," C    ", P&TH2Mittel[4], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*46."," TH2 JUNI       "," C    ", P&TH2Mittel[5], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*47."," TH2 JULI       "," C    ", P&TH2Mittel[6], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*48."," TH2 AUGUST     "," C    ", P&TH2Mittel[7], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*49."," TH2 SEPTEMBER  "," C    ", P&TH2Mittel[8], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*50."," TH2 OKTOBER    "," C    ", P&TH2Mittel[9], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*51."," TH2 NOVEMBER   "," C    ", P&TH2Mittel[10], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*52."," TH2 DEZEMER    "," C    ", P&TH2Mittel[11], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},

	{"*53."," TH2 VJ JANUAR  "," C    ", P&TH2MittelVJ[0], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*54."," TH2 VJ FEBRUAR "," C    ", P&TH2MittelVJ[1], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*55."," TH2 VJ MAERZ   "," C    ", P&TH2MittelVJ[2], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*56."," TH2 VJ APRIL   "," C    ", P&TH2MittelVJ[3], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*57."," TH2 VJ MAI     "," C    ", P&TH2MittelVJ[4], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*58."," TH2 VJ JUNI    "," C    ", P&TH2MittelVJ[5], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*59."," TH2 VJ JULI    "," C    ", P&TH2MittelVJ[6], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*60."," TH2 VJ AUGUST  "," C    ", P&TH2MittelVJ[7], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*61."," TH2 VJ SEPTEMBE"," C    ", P&TH2MittelVJ[8], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*62."," TH2 VJ OKTOBER "," C    ", P&TH2MittelVJ[9], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*63."," TH2 VJ NOVEMBER"," C    ", P&TH2MittelVJ[10], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*64."," TH2 VJ DEZEMER "," C    ", P&TH2MittelVJ[11], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},

	{" **."," MITTLERE TEMP. ","       ", P&DiaTN2mod,					ASCII_FORM, 0, P&hid2,	V0, 0, 0},
	{"*71."," TN2 JANUAR     "," C    ", P&TN2Mittel[0], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*72."," TN2 FEBRUAR    "," C    ", P&TN2Mittel[1], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*73."," TN2 MAERZ      "," C    ", P&TN2Mittel[2], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*74."," TN2 APRIL      "," C    ", P&TN2Mittel[3], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*75."," TN2 MAI        "," C    ", P&TN2Mittel[4], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*76."," TN2 JUNI       "," C    ", P&TN2Mittel[5], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*77."," TN2 JULI       "," C    ", P&TN2Mittel[6], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*78."," TN2 AUGUST     "," C    ", P&TN2Mittel[7], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*79."," TN2 SEPTEMBER  "," C    ", P&TN2Mittel[8], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*80."," TN2 OKTOBER    "," C    ", P&TN2Mittel[9], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*81."," TN2 NOVEMBER   "," C    ", P&TN2Mittel[10], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*82."," TN2 DEZEMER    "," C    ", P&TN2Mittel[11], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},

	{"*83."," TN2 VJ JANUAR  "," C    ", P&TN2MittelVJ[0], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*84."," TN2 VJ FEBRUAR "," C    ", P&TN2MittelVJ[1], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*85."," TN2 VJ MAERZ   "," C    ", P&TN2MittelVJ[2], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*86."," TN2 VJ APRIL   "," C    ", P&TN2MittelVJ[3], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*87."," TN2 VJ MAI     "," C    ", P&TN2MittelVJ[4], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*88."," TN2 VJ JUNI    "," C    ", P&TN2MittelVJ[5], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*89."," TN2 VJ JULI    "," C    ", P&TN2MittelVJ[6], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*90."," TN2 VJ AUGUST  "," C    ", P&TN2MittelVJ[7], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*91."," TN2 VJ SEPTEMBE"," C    ", P&TN2MittelVJ[8], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*92."," TN2 VJ OKTOBER "," C    ", P&TN2MittelVJ[9], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*93."," TN2 VJ NOVEMBER"," C    ", P&TN2MittelVJ[10], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*94."," TN2 VJ DEZEMER "," C    ", P&TN2MittelVJ[11], 				 	 S_INT, 1, P&hid2,	V0, 0, 0},

};
