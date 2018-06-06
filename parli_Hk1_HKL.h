/*------------------------------ Heizkreis 1 -------------------------------------------*/
const Pgrup hk1[] = {
//-------------------
	{"_**:"," HEIZKREIS      ","       ", P&hkd[HK1].raumname,	ASCII_FORM, 0, P&vis,		V0, 0, 0},
	{"*01:"," VL-HEIZUNG  TVH"," C    ", P&TVS[HK1],						 ANA_FORMP, 1, P&vis,		V1, 0, 0},      
	{"*01:"," VL-HEIZUNG  TVH"," C    ", P&TVS[HK1],					ANA_FORMP, 0x81, P&kom,		E1, FUEHLER, 0},	// ***AnFre

	#if TRLBEG_HK1==1
	{"*02:"," RL-HEIZUNG  TRH"," C    ", P&TRS[HK1],						 ANA_FORMP, 1, P&vis,		V1, 0, 0},      
	{"*02:"," RL-HEIZUNG  TRH"," C    ", P&TRS[HK1],					ANA_FORMP, 0x81, P&kom,		E1, FUEHLER, 0},	// ***AnFre
	#endif
// ***AnFre 06.06.2012
	{"*03:"," SPEI. OB.TSPo-L"," C    ", P&TW2_[0],						 ANA_FORMP, 1, P&vis,		V1, 0, 0},
	{"*03:"," SPEI. OB.TSPo-L","       ", P&TW2_[0],					ANA_FORMP, 0x81, P&kom,		E1, FUEHLER, 0},
	{"*04:"," SPEI. MI.TSPm-L"," C    ", P&TW3_[0],						 ANA_FORMP, 1, P&vis,		V1, 0, 0},
	{"*04:"," SPEI. MI.TSPm-L","       ", P&TW3_[0],					ANA_FORMP, 0x81, P&kom,		E1, FUEHLER, 0},
	{"*05:"," SPEI. UN.TSPu-L"," C    ", P&TW4_[0],						 ANA_FORMP, 1, P&vis,		V1, 0, 0},
	{"*05:"," SPEI. UN.TSPu-L","       ", P&TW4_[0],					ANA_FORMP, 0x81, P&kom,		E1, FUEHLER, 0},

	{" 11;"," ANHEBUNG LADUNG"," K     ", P&hks[HK1].Tvpa,					US_INT, 1, P&hid1,	V1, 0, 0},
	{" 12;"," VORLAUF   MAX  "," C    ", P&hks[HK1].Tvma,					US_INT, 1, P&hid1,	V1, 0, 0},
	{" 13;"," VORLAUF   MIN  "," C    ", P&hks[HK1].Tvmi,					US_INT, 1, P&hid1,	V1, 0, 0},

	{"*14:"," ANFORDER.EXT 1 "," C    ", P&hkd[HK1].ExtAnf1Hk,			ANA_FORM, 1, P&vis,	V1, 0, 0},
	{"*14:"," ANFORDER.EXT 1 "," C    ", P&hkd[HK1].ExtAnf1Hk,	 ANA_FORM, 0x81, P&kom,	E1, FUEHLER, 0},
	{" ->."," ANFORD.GUELTIG "," 1=JA  ", P&hks[HK1].ExtAnf1_Ja,	 	 US_CHAR, 0, P&hid2,V0, 0, 0},
	{"*15:"," ANFORDER.EXT 2 "," C    ", P&hkd[HK1].ExtAnf2Hk,			ANA_FORM, 1, P&vis,	V1, 0, 0},
	{"*15:"," ANFORDER.EXT 2 "," C    ", P&hkd[HK1].ExtAnf2Hk,	 ANA_FORM, 0x81, P&kom,	E1, FUEHLER, 0},
	{" ->."," ANFORD.GUELTIG "," 1=JA  ", P&hks[HK1].ExtAnf2_Ja,	   US_CHAR, 0, P&hid2,V0, 0, 0},
	{" 16;"," ANHEB. EXT.ANF."," K     ", P&hks[HK1].TvAnhExt,				US_INT, 1, P&hid1,V1, 0, 0},

	#if TRLBEG_HK1==1
	{" 18;"," RUECKLAUF MAX  "," C    ", P&hks[HK1].Trma,					US_INT, 1, P&hid1,	V1, 0, 0},
	{" 19;"," Xp-RUECKL.BEGR.","       ", P&hks[HK1].Kpr,						US_INT, 2, P&hid1,	V1, 0, 0},
	#endif

	{"*20:"," VORLAUF BERECHN"," C    ", P&hkd[HK1].tvsb,					US_INT, 1, P&vis,		V1, 0, 0},
//***AnFre 06.06.2012	{"*->."," Vorlauf ber.HKl"," C    ", P&hkd[HK1].tvsb_hkl,			US_INT, 1, P&hid2,	V0, 0, 0},
	{"*->."," Korrektur RL   "," K     ", P&hkd[HK1].rlKorr,			 	 S_INT, 1, P&hid2,	V0, 0, 0},
// Funktion entfernt!!! 06.01.2017
//	{"*->."," Absenkung Solar"," K     ", P&hkd[HK1].absSolBer,		 	 S_INT, 1, P&hid2,	V0, 0, 0},
	{" ->."," Vorlauf Filt.ZK"," s     ", P&hks[HK1].Fzk,						US_INT, 1, P&hid2,	V0, 0, 0},

	{" 50;"," PU LK SOLL MIN "," %     ", P&hks[HK1].PuDzMin,				US_INT, 1, P&hid1,	V1, 0, 0},
//	{" 51;"," Ta L-PU.SOLL MI"," C    ", P&hks[HK1].PuDzTaMax,		 	 S_INT, 1, P&hid1,	V1, 0, 0},	06.01.2017
	{" 51;"," PU LK SOLL MAX "," %     ", P&hks[HK1].PuDzMax,				US_INT, 1, P&hid1,	V1, 0, 0},
//	{" 53;"," Ta L-PU.SOLL MA"," C    ", P&hks[HK1].PuDzTaMin,		 	 S_INT, 1, P&hid1,	V1, 0, 0},	06.01.2017
	{" 52;"," DT SOLL-IST    "," K     ", P&hks[HK1].DTPuDzMin,		 	 S_INT, 1, P&hid1,	V1, 0, 0},	//06.01.2017
	{" 53."," PU LK DZ-STEIG."," %/30s ", P&hks[HK1].PuDzSteig,		 	 S_INT, 1, P&hid2,	V1, 0, 0},	//06.01.2017
	{" ->."," Dz-Steig. ein? ","       ", P&hkd[HK1].puDzSteiEin,JANEIN_FORM, 0, P&hid2,V0, 0, 0},
	{"*->."," Sollwert berech"," %     ", P&hkd[HK1].puSollBer,			 US_INT, 1, P&hid2,	V0, 0, 0},
	{" 55;"," REGELART SOLL  "," 0..3  ", P&BusPuPara[PU_BUS_HK1 - 1].Regelart,	US_CHAR,  0, P&wilo3,	V1, 0, IDX0_FELD},

// ***AnFre 06.06.2012 Absenkungen in HKN:

	{" 60;"," SO/WI-AUTOMATIK"," JA=1  ", P&hks[HK1].Swhk,				 US_CHAR, 0, P&hid1,	V1, 0, 0},
	{" 61;"," WW-VORRANG     "," JA=1  ", P&hks[HK1].WwVorrang,		 US_CHAR, 0, P&hid1,	V1, 0, 0},
	{" 62;"," dT LADUNG EIN  "," K     ", P&hks[HK1].DTLadEin,			 S_INT, 1, P&hid1,	V1, 0, 0},

	{" 65;"," VENT.MAX SOLAR "," %     ", P&hks[HK1].VentMaxSolar,	US_INT, 1, P&hid1,	V2, 0, 0},
// Funktion entfernt!!! 06.01.2017
//	{" 66;"," ABSENKUNG SOLAR"," K     ", P&hks[HK1].AbsenkSol,			US_INT, 1, P&hid1,	V2, 0, 0},	06.01.2017
//	{" 67;"," FAKTOR ABSENK. ","       ", P&hks[HK1].AbsenkFaktor,	US_INT, 2, P&hid1,	V2, 0, 0},	06.01.2017

	{" 68."," TDiff.Alarm Ein"," K     ", P&hks[HK1].TabwMax,			 US_CHAR, 1, P&hid2,	V0, 0, 0},
	{" 69."," ZVerz.Alarm Ein"," min   ", P&hks[HK1].TabwZeit,		 US_CHAR, 0, P&hid2,	V0, 0, 0},

	{"*70:"," BEDARFSLADUNG ?","       ", P&hkd[HK1].bedLadung,	 JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*71:"," EXT.ANF. AKTIV?","       ", P&hkd[HK1].extAnfAktiv,JANEIN_FORM, 0, P&vis,		V1, 0, 0},

	{"*73:"," PU LK      EIN?","       ", P&hkd[HK1].puEinAnz, JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*->."," Pumpen-Laufzeit"," h     ", P&hkd[HK1].puLzAnz,			 US_LONG, 0, P&hid2,	V1, 0, 0},
	{" ->."," Pu-Nachlaufzeit"," min   ", P&hks[HK1].Puna,				 US_CHAR, 0, P&hid2,	V0, 0, 0},
	{" ->."," Pum-Nachl.  Cnt"," *30 s ", P&hkd[HK1].zpu,						US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->:"," PUMPENSOLLWERT "," %     ", P&AA_UNI[U2],					AOUT_FORMP, 1, P&vis,		V1, 0, 0},
	{"*->:"," BM PU LK   EIN?","       ", P&hkd[HK1].puBmAnz,  JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*->:"," WILO-BUS LAD-PU","       ", P&BusPuPara[PU_BUS_HK1 - 1].Funktion,	PU_FUNC_FORM,0, P&vis,	V1, 0, IDX0_FELD},
	{"*->;"," BUS-ADRESSE    ","       ", P&BusPuPara[PU_BUS_HK1 - 1].Adresse,		 US_CHAR,  0, P&wilo3,V1, 0, IDX0_FELD},
	{"*->:"," REGELART IST   ","       ", P&BusPuData[PU_BUS_HK1 - 1].regelArtIst,DYN_ASCII_FORM,15,P&wilo3,V1,0, IDX0_FELD},
	{"*->;"," DREHZAHL IST   "," 1/min ", P&modWiloPu[PU_BUS_HK1 - 1].wiloInReg.mwspSpeed.messw, 	US_INT,  0, P&wilo3,V1, 0, IDX0_FELD},
	{"*->;"," DIFFERENZDRUCK "," mWS   ", P&modWiloPu[PU_BUS_HK1 - 1].wiloInReg.mwspActualDifferentialPressure.messw, 	US_INT,	 1, P&wilo3,V1, 0, IDX0_FELD},
	{"*->;"," FOERDERSTROM   "," m/h  ", P&modWiloPu[PU_BUS_HK1 - 1].wiloInReg.mwspFlowRate.messw, 	US_INT,	 1, P&wilo3,V1, 0, IDX0_FELD},

//#if PU_BUS_HK1 > 0								// Bus-Pumpe
//	{"*75:"," BUS-PUMPE EIN ?","       ", P&hkd[HK1].busPuEin, 			 JANEIN_FORM, 0, P&vis,		V1, 0, 0},	//Ausgang nach Pumpe
//	{" 76;"," PU-SOLL NORMAL "," %     ", P&hks[HK1].BusPuSollwert,		 		US_INT, 1, P&hid1,	V1, 0, 0},
//	{" 77;"," PU-SOLL ABSENK "," %     ", P&hks[HK1].BusPuSollwertAbsenk,	US_INT, 1, P&hid1,	V1, 0, 0},
//#endif	

	{"*80;"," BETRIEBS-REGIE ","       ", P&hkd[HK1].regie_txt, DYN_ASCII_FORM, 0, P&hid4,	V0, 0, 0},
	{" ->."," Absenkphase ?  ","       ", P&hkd[HK1].absen,		 JANEIN_FORM, 0, P&hid4,	V0, 0, 0},
	{" ->."," Absenkwert abs "," K     ", P&abs_ram[HK1].tvab,			US_INT, 1, P&hid4,	V0, 0, 0},
	{" ->."," Absenkzeit abs "," *10 s ", P&abs_ram[HK1].zasd,			US_INT, 0, P&hid4,	V0, 0, 0},
	{" ->."," Abschaltphase ?","       ", P&hkd[HK1].absch,		 JANEIN_FORM, 0, P&hid4,	V0, 0, 0},
	{" ->."," Abschalt   abs "," *10 s ", P&abs_ram[HK1].zAbsch,		US_INT, 0, P&hid4,	V0, 0, 0},
	{" ->."," Aufheizphase ? ","       ", P&hkd[HK1].aufhe,		 JANEIN_FORM, 0, P&hid4,	V0, 0, 0},
	{" ->."," Aufheiz    abs "," *10 s ", P&abs_ram[HK1].zahd,			US_INT, 0, P&hid4,	V0, 0, 0},
	{" ->."," Sommerbetrieb ?","       ", P&hkd[HK1].somme,	 	 JANEIN_FORM, 0, P&hid4,	V0, 0, 0},
	{" ->."," Stuetzbetrieb ?","       ", P&hkd[HK1].stuetz,	 JANEIN_FORM, 0, P&hid4,	V0, 0, 0},
	{" ->."," Frostbetrieb ? ","       ", P&hkd[HK1].frost,		 JANEIN_FORM, 0, P&hid4,	V0, 0, 0},

// ***AnFre	{" ->:"," absenkwert bed "," K     ", P&abs_bed[HK1].tvab,			US_INT, 1, P&hid2,	V0, 0, 0},
// ***AnFre	{" ->:"," absenkzeit bed "," s/10  ", P&abs_bed[HK1].zasd,			US_INT, 0, P&hid2,	V0, 0, 0},
// ***AnFre	{" ->:"," abschalt   bed "," s/10  ", P&abs_bed[HK1].zAbsch,		US_INT, 0, P&hid2,	V0, 0, 0},
// ***AnFre	{" ->:"," aufheiz    bed "," s/10  ", P&abs_bed[HK1].zahd,			US_INT, 0, P&hid2,	V0, 0, 0},


	{"*85:"," RUECKL.BEGR. ? ","       ", P&hkd[HK1].rlbegr,	 JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*86:"," SM REGELABW.?  ","       ", P&hkd[HK1].abwAlarm, JANEIN_FORM, 0, P&vis, 	A1, EINZEL,   0},
	{"*88:"," SM PU LK      ?","       ", P&hkd[HK1].puSmAnz,	 JANEIN_FORM, 0, P&vis,		E1, STANDARD, 0},

//	#if PU_BUS_HK1 > 0
//	{"*88:"," SM BUS LADE-PU?","       ", P&hkd[HK1].busPuSm,	JANEIN_FORM, 0, P&vis,		A1, EINZEL,   0},
//	#endif

	{"*89:"," HANDBETRIEB ?  ","       ", P&hks[HK1].Haut,		 JANEIN_FORM, 0, P&vis,		E1, EREIGNIS, 0},

	{" 90;"," P-VERSTAERKUNG "," %/K   ", P&hks[HK1].Kp,						US_INT, 2, P&hid1,	V0, 0, 0},
	{" 91;"," D-VERSTAERKUNG ","       ", P&hks[HK1].Kd,						US_INT, 2, P&hid1,	V0, 0, 0},
	{" 92;"," TASTZEIT       "," s     ", P&hks[HK1].Ts,						US_INT, 1, P&hid1,	V0, 0, 0},
	{" 93;"," NACHSTELLZEIT  "," s     ", P&hks[HK1].Tn,						US_INT, 1, P&hid1,	V0, 0, 0},

	{" 94;"," VENTILSTELL.MIN"," %     ", P&hks[HK1].Y_rel_min,	 		US_INT, 1, P&hid1,	V0, 0, 0},
	{" 95;"," OEFFNUNGSBEGINN"," %     ", P&hks[HK1].Y_rel_beg,	 		US_INT, 1, P&hid1,	V0, 0, 0},

	{"*96:"," SOLLWERT       "," C    ", P&hkd[HK1].tsol,					US_INT, 1, P&vis,	V1, 0, 0},
	{"*97:"," REGELABWEICHUNG"," K     ", P&hkd[HK1].ei,						 S_INT, 1, P&vis,	V1, 0, 0},
	{"*->."," dy_rel         ","       ", P&hkd[HK1].dy_rel,				 S_INT, 3, P&hid2,	V0, 0, 0},

	{"*98:"," V.STELLUNG RV H"," %     ", P&RVENT[HK1],					AOUT_FORMP, 1, P&vis,	V1, 0, 0},
	{"*98;"," Vent.Min Berech"," %     ", P&hkd[HK1].ventMinBer, 		US_INT, 1, P&hid4,	V0, 0, 0},
	{" ->;"," RL-Temp Mindern"," K     ", P&hks[HK1].RLKorrMin,			 S_INT, 1, P&hid4,	V0, 0, 0},

	//-------------------------------------------------------------------------------------------------
	{" 99;"," HAND/AUTOMATIK "," HAND=1", P&hks[HK1].Haut,				 US_CHAR, 0, P&hid1,	V1, 0, 0},

	{" ->;"," PU LK EIN/AUS  "," EIN=1 ", P&hks[HK1].Puea,		 		 US_CHAR, 0, P&hid1,	V1, 0, 0},
	{" ->;"," PU LK REGELART "," 0..3  ", P&BusPuPara[PU_BUS_HK1-1].RegelartHand,	US_CHAR, 0, P&wilo3,	V1, 0, IDX0_FELD},
	{" ->;"," PU LK SOLLWERT "," %     ", P&hks[HK1].PuDzHand,		  US_INT, 1, P&hid1,	V1, 0, 0},

	{" ->;"," VENTIL ST. RV H"," %     ", P&hks[HK1].Vstell,			  US_INT, 1, P&hid1,	V1, 0, 0},

};


/*------------------------------ Heizkurven-Adaption ----------------------------------------------*/
