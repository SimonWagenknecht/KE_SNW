/*------------------------------ Solarer Heizkreis ***AnFre-------------------------------------------*/
const Pgrup hkSoL[] = {
//-------------------
	{"***:"," HEIZKREIS      ","       ", P&Hkmod_SoL,								ASCII_FORM, 0, P&vis,		V0, 0, 0},
	{"*01:"," VL-HEIZUNG  TVH"," C    ", P&TVS[HK1],						 			ANA_FORMP, 1, P&vis,		V0, 0, 0},      
	{"*01:"," VL-HEIZUNG  TVH"," C    ", P&TVS[HK1],								ANA_FORMP, 0x81, P&kom,		V0, 0, 0},

	#if TRLBEG_HK1==1
	{"*02:"," RL-HEIZUNG  TRH"," C    ", P&TRS[HK1],									 ANA_FORMP, 1, P&vis,		V0, 0, 0},      
	{"*02:"," RL-HEIZUNG  TRH"," C    ", P&TRS[HK1],								ANA_FORMP, 0x81, P&kom,		V0, 0, 0},
	#endif
	{"*04:"," VOR PWT-HEI TS3"," C    ", P&TS3_[0],									 ANA_FORMP, 1, P&vis,		V0, 0, 0},      
	{"*04:"," VOR PWT-HEI TS3"," C    ", P&TS3_[0],								ANA_FORMP, 0x81, P&kom,		V0, 0, 0},

//	{" 11:"," VORL. ANHEBUNG "," K     ", P&hksSoL[HK1].TvAnh,						US_INT, 1, P&vis,		V1, 0, 0},
//	{" 11;"," VL-ABSENK. HK1:"," K     ", P&hksSoL[HK1].TvAbsenk,					US_INT, 1, P&hid1,		V0, 0, 0}, 06.06.2012 entfernt
	{" 11;"," VL-ANHEB. HKS: "," K     ", P&hksSoL[HK1].TvAnheb,						US_INT, 1, P&hid1, V2, 0, 0},		// 06.01.2017

	{"*20:"," VORLAUF BERECHN"," C    ", P&hkdSoL[HK1].tvsb,							US_INT, 1, P&vis,		V1, 0, 0},

	{" 30;"," dT SOL.LAD. EIN"," K     ", P&hksSoL[HK1].DTSoLad,						US_INT, 1, P&hid1,	V0, 0, 0},
	{" 31;"," MINDESTZEIT LAD"," min   ", P&hksSoL[HK1].SoLadMin,					 US_CHAR, 0, P&hid1,	V0, 0, 0},
	{" 32;"," SPERRZEIT LADUN"," min   ", P&hksSoL[HK1].SoLadSperr,				 US_CHAR, 0, P&hid1,	V0, 0, 0},
	{" 33;"," S. L. TSPo-LMAX"," C    ", P&hksSoL[HK1].TH3Max,							US_INT, 1, P&hid1,	V0,	0, 0},

	{" 50;"," PUMPE-SOLL MIN "," %     ", P&hksSoL[HK1].PuSoMin,						US_INT, 1, P&hid1,	V0,	0, 0},
	{" 51;"," PUMPE-SOLL MAX "," %     ", P&hksSoL[HK1].PuSoMax,						US_INT, 1, P&hid1,	V0,	0, 0},
	{" 52;"," P-VERST. PUMPE "," %/K   ", P&hksSoL[HK1].Pu_Kp,						US_INT, 2, P&hid1,	V0, 0, 0},
	{" 53;"," D-VERST. PUMPE ","       ", P&hksSoL[HK1].Pu_Kd,						US_INT, 2, P&hid1,	V0, 0, 0},
	{" 54;"," TASTZEIT PUMPE "," s     ", P&hksSoL[HK1].Pu_Ts,						US_INT, 1, P&hid1,	V0, 0, 0},
	{" 55;"," NACHST.ZEIT PU."," s     ", P&hksSoL[HK1].Pu_Tn,						US_INT, 1, P&hid1,	V0, 0, 0},
	{"*56:"," SOLLWERT PUMPE "," C    ", P&hkdSoL[HK1].pu_tsol,					US_INT, 1, P&vis,		V1, 0, 0},
	{"*->."," Regelabw.Pumpe "," K     ", P&hkdSoL[HK1].pu_ei,						 S_INT, 1, P&hid2,	V0, 0, 0},
	{"*->."," dy_rel   Pumpe ","       ", P&hkdSoL[HK1].pu_dy_rel,				 S_INT, 3, P&hid2,	V0, 0, 0},
	{"*57:"," PU-SOLL BERECHN"," %     ", P&hkdSoL[HK1].pu_y_rel,					 S_INT, 1, P&vis,		V1, 0, 0},

	{"*70:"," SOLARE LADUNG ?","       ", P&hkdSoL[HK1].solLadung,	JANEIN_FORM,	0, P&vis,		V1, 0, 0},
	{" ->."," Mindestzeit Cnt"," *30 s ", P&hkdSoL[HK1].zSoLadMin,			 US_INT,	0, P&hid2,	V0, 0, 0},
	{" ->."," Sperrzeit   Cnt"," *30 s ", P&hkdSoL[HK1].zSoLadSperr,		 US_INT,	0, P&hid2,	V0, 0, 0},
	{"*71:"," SOLARES HEIZEN?","       ", P&hkdSoL[HK1].hkSolJa,		JANEIN_FORM,	0, P&vis,		V1, 0, 0},
	{" ->."," HeizNachlaufCnt"," *30 s ", P&hkdSoL[HK1].zHkSolJa,				 US_INT,	0, P&hid2,	V0, 0, 0},

	{"*80:"," SPEICHER MAX ? ","       ", P&hkdSoL[HK1].th3Max,		 	JANEIN_FORM, 0, P&vis,		E1, EREIGNIS, 0},

	{"*89:"," HANDBETRIEB ?  ","       ", P&hksSoL[HK1].Haut,		 		JANEIN_FORM, 0, P&vis,		E1, EREIGNIS, 0},

	{" 90;"," P-VERSTAERKUNG "," %/K   ", P&hksSoL[HK1].Kp,						US_INT, 2, P&hid1,	V0, 0, 0},
	{" 91;"," D-VERSTAERKUNG ","       ", P&hksSoL[HK1].Kd,						US_INT, 2, P&hid1,	V0, 0, 0},
	{" 92;"," TASTZEIT       "," s     ", P&hksSoL[HK1].Ts,						US_INT, 1, P&hid1,	V0, 0, 0},
	{" 93;"," NACHSTELLZEIT  "," s     ", P&hksSoL[HK1].Tn,						US_INT, 1, P&hid1,	V0, 0, 0},
	{" 94;"," VENTILSTELL.MAX"," %     ", P&hksSoL[HK1].RVSoMax,			US_INT, 1, P&hid1,	V0,	0, 0},
	{" 95;"," OEFFNUNGSBEGINN"," %     ", P&hksSoL[HK1].Y_rel_beg,	 	US_INT, 1, P&hid1,	V0, 0, 0},

	{"*96;"," SOLLWERT       "," C    ", P&hkdSoL[HK1].tsol,					US_INT, 1, P&vis,		V1, 0, 0},
	{"*97;"," REGELABWEICHUNG"," K     ", P&hkdSoL[HK1].ei,						 S_INT, 1, P&vis,		V1, 0, 0},
	{"*->."," dy_rel         ","       ", P&hkdSoL[HK1].dy_rel,				 S_INT, 3, P&hid2,	V0, 0, 0},

	{"*98;"," V.STELLU. RV SH"," %     ", P&RVENTSO[HK1],					AOUT_FORMP, 1, P&vis,		V1, 0, 0},

	//-------------------------------------------------------------------------------------------------
	{" 99;"," HAND/AUTOMATIK "," HAND=1", P&hksSoL[HK1].Haut,				 US_CHAR, 0, P&hid1,	V1, 0, 0},
	{" ->;"," VENT. ST. RV SH"," %     ", P&hksSoL[HK1].Vstell,			  US_INT, 1, P&hid1,	V1, 0, 0},

};

