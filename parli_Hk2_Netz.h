/*------------------------------ Heizkreis 2 -------------------------------------------*/
const Pgrup hk2[] = {
//-------------------
	{"_**:"," HEIZKREIS      ","       ", P&hkd[HK2].raumname,	ASCII_FORM, 0, P&vis,		V1, 0, 0},
	{"*01:"," VORL.     TVNK1"," C    ", P&TVS[HK2],						 ANA_FORMP, 1, P&vis,		V1, 0, 0},      
	{"*01:"," VORL.     TVNK1"," C    ", P&TVS[HK2],					ANA_FORMP, 0x81, P&kom,		E1, FUEHLER, 0},

	#if TRLBEG_HK2==1
	{"*02:"," RUECKL.   TRNK1"," C    ", P&TRS[HK2],						 ANA_FORMP, 1, P&vis,		V1, 0, 0},      
	{"*02:"," RUECKL.   TRNK1"," C    ", P&TRS[HK2],					ANA_FORMP, 0x81, P&kom,		E1, FUEHLER, 0},
	#endif
	
	{" 06;"," SO/WI HK-EXTRA "," 1=JA  ", P&hks[HK2].SoWiExtra,		 US_CHAR, 0, P&hid1,	V1, 0, 0},
	{" 07;"," Ta-WINTER [EIN]"," C    ", P&hks[HK2].Tae,						US_INT, 1, P&hid1,	V1, 0, 0},
	{" 08;"," Ta-SOMMER [AUS]"," C    ", P&hks[HK2].Taa,						US_INT, 1, P&hid1,	V1, 0, 0},
	{"*->;"," Sommer ?       ","       ", P&hkd[HK2].sowi,		 JANEIN_FORM, 0, P&hid2,	V0, 0, 0},

	{" 10;"," STEIGUNG       ","       ", P&hks[HK2].Anst,					US_INT, 2, P&hid1,	V1, 0, 0},
	{" 11;"," NIVEAU         "," C    ", P&hks[HK2].Tvpa,					US_INT, 1, P&hid1,	V1, 0, 0},
	{" 12;"," VORLAUF   MAX  "," C    ", P&hks[HK2].Tvma,					US_INT, 1, P&hid1,	V1, 0, 0},
	{" 13;"," VORLAUF   MIN  "," C    ", P&hks[HK2].Tvmi,					US_INT, 1, P&hid1,	V1, 0, 0},

	{"*14:"," ANFORDER.EXT 1 "," C    ", P&hkd[HK2].ExtAnf1Hk,			ANA_FORM, 1, P&vis,	V1, 0, 0},
	{"*14:"," ANFORDER.EXT 1 "," C    ", P&hkd[HK2].ExtAnf1Hk,	 ANA_FORM, 0x81, P&kom,	E1, FUEHLER, 0},
	{" ->."," ANFORD.GUELTIG "," 1=JA  ", P&hks[HK2].ExtAnf1_Ja,	 	 US_CHAR, 0, P&hid2,V0, 0, 0},
	{"*15:"," ANFORDER.EXT 2 "," C    ", P&hkd[HK2].ExtAnf2Hk,			ANA_FORM, 1, P&vis,	V1, 0, 0},
	{"*15:"," ANFORDER.EXT 2 "," C    ", P&hkd[HK2].ExtAnf2Hk,	 ANA_FORM, 0x81, P&kom,	E1, FUEHLER, 0},
	{" ->."," ANFORD.GUELTIG "," 1=JA  ", P&hks[HK2].ExtAnf2_Ja,	   US_CHAR, 0, P&hid2,V0, 0, 0},
	{" 16;"," ANHEB. EXT.ANF."," K     ", P&hks[HK2].TvAnhExt,				US_INT, 1, P&hid1,V1, 0, 0},

	{" 17;"," SOLARE ANHEBUNG"," K     ", P&hks[HK2].SolAnhebung,		US_INT, 1, P&hid1,	V1, 0, 0},
	{" ->;"," SOL.AN. TSPo-L>"," C    ", P&hks[HK2].SolAnhebTH3,		US_INT, 1, P&hid1,	V0, 0, 0},
	{" ->;"," SOL.ANH TSPo-P>"," C    ", P&hks[HK2].SolAnhebTS5,		US_INT, 1, P&hid1,	V0, 0, 0},

	#if TRLBEG_HK2==1
	{" 18;"," RUECKLAUF MAX  "," C    ", P&hks[HK2].Trma,					US_INT, 1, P&hid1,		V1, 0, 0},
	{" 19;"," Xp-RUECKL.BEGR.","       ", P&hks[HK2].Kpr,						US_INT, 2, P&hid1,		V1, 0, 0},
	#endif

	{"*20:"," VORLAUF BERECHN"," C    ", P&hkd[HK2].tvsb,					US_INT, 1, P&vis,		V1, 0, 0},
	{"*->."," Vorlauf ber.HKN"," C    ", P&hkd[HK2].tvsb_hkl,			US_INT, 1, P&hid2,	V0, 0, 0},
	{"*->."," Korrektur RL   "," K     ", P&hkd[HK2].rlKorr,			 	 S_INT, 1, P&hid2,	V0, 0, 0},
// ***AnFre	{"*->:"," korrektur adapt"," K     ", P&hkd[HK2].adaptKorr,			 S_INT, 1, P&hid2,	V0, 0, 0},
// ***AnFre	{"*->:"," korrektur kaska"," K     ", P&hkd[HK2].kaskaKorr,			 S_INT, 1, P&hid2,	V0, 0, 0},
// ***AnFre	{"*->."," Korrektur wwvor"," K     ", P&hkd[HK2].wwvorKorr,			 S_INT, 1, P&hid2,	V0, 0, 0},
	{" ->."," Vorlauf Filt.ZK"," s     ", P&hks[HK2].Fzk,						US_INT, 1, P&hid2,	V0, 0, 0},

	#if LEIBEGR_HK2 == 1
	{"*21:"," ABSENK.LEI.BEGR"," K     ", P&hkd[HK2].leistBegrAbsenk,US_INT,1, P&vis,		V1,	0, 0},
	#endif

	#if FBH_HK2==1 // ***AnFre 06.06.2012 JA
	{" 25;"," ESTR.-PR.START ","       ", P&hks[HK2].StartAufheiz, DATE_FORM, 1, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG1 "," C    ", P&hks[HK2].EstrichTemp[0], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG2 "," C    ", P&hks[HK2].EstrichTemp[1], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG3 "," C    ", P&hks[HK2].EstrichTemp[2], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG4 "," C    ", P&hks[HK2].EstrichTemp[3], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG5 "," C    ", P&hks[HK2].EstrichTemp[4], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG6 "," C    ", P&hks[HK2].EstrichTemp[5], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG7 "," C    ", P&hks[HK2].EstrichTemp[6], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG8 "," C    ", P&hks[HK2].EstrichTemp[7], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG9 "," C    ", P&hks[HK2].EstrichTemp[8], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG10"," C    ", P&hks[HK2].EstrichTemp[9], US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG11"," C    ", P&hks[HK2].EstrichTemp[10],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG12"," C    ", P&hks[HK2].EstrichTemp[11],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG13"," C    ", P&hks[HK2].EstrichTemp[12],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG14"," C    ", P&hks[HK2].EstrichTemp[13],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG15"," C    ", P&hks[HK2].EstrichTemp[14],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG16"," C    ", P&hks[HK2].EstrichTemp[15],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG17"," C    ", P&hks[HK2].EstrichTemp[16],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG18"," C    ", P&hks[HK2].EstrichTemp[17],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG19"," C    ", P&hks[HK2].EstrichTemp[18],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG20"," C    ", P&hks[HK2].EstrichTemp[19],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG21"," C    ", P&hks[HK2].EstrichTemp[20],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG22"," C    ", P&hks[HK2].EstrichTemp[21],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG23"," C    ", P&hks[HK2].EstrichTemp[22],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG24"," C    ", P&hks[HK2].EstrichTemp[23],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG25"," C    ", P&hks[HK2].EstrichTemp[24],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG26"," C    ", P&hks[HK2].EstrichTemp[25],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG27"," C    ", P&hks[HK2].EstrichTemp[26],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG28"," C    ", P&hks[HK2].EstrichTemp[27],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG29"," C    ", P&hks[HK2].EstrichTemp[28],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{" ->;"," ESTR.TEMP.TAG30"," C    ", P&hks[HK2].EstrichTemp[29],US_CHAR, 0, P&hid4,	V0, 0, 0},
	{"*26:"," ESTRICH-PROGR.?","       ", P&hkd[HK2].estrichProg,JANEIN_FORM, 0, P&vis, 	V0, 0, 0},
	{"*->."," ESTRICH AKT.TAG","       ", P&hkd[HK2].estrichTag,		 US_CHAR, 0, P&hid4,	V0, 0, 0},
	{"*->."," ESTR.AKT.SOLLT."," C    ", P&hkd[HK2].estrichTemp,			US_INT, 0, P&hid4,	V0, 0, 0},
	#endif

	#if ( ANFORD > 0 || NTANZ > 0 || KEANZ > 0 )
	{" 28:"," ANFORD.ANHEBUNG"," K     ", P&hks[HK2].TvpAnh,				US_INT, 1, P&vis,			V1, 0, 0},
	#endif
	
	#if BEDARF_HK2==1
	{" 29:"," VORRANG ZEITABS"," 1=JA  ", P&hks[HK2].VorrangZeitabsenkung, US_CHAR, 0, P&vis,	V1, 0, 0},
	#endif

	{" 30;"," ABSENK-BEGIN WO"," h:min ", P&abs_tab[HK2][0].begin,ZEIT_FORM,0, P&hid1,		V1, 0, 0},
	{" 31;"," ABSENK-DAUER WO"," h     ", P&abs_tab[HK2][0].dauer, US_CHAR, 0, P&hid1,		V1, 0, 0},
	{" 32;"," ABSENK-WERT  WO"," K     ", P&abs_tab[HK2][0].abwert,	US_INT, 1, P&hid1,		V1, 0, 0},
	{" 34;"," GANZE WOCHE SET"," 1=JA  ", P&setwoche[HK2],				 US_CHAR, 0, P&hid1,		V1, 0, 0},

	{" 35;"," ABSENK-BEGIN Mo"," h:min ", P&abs_tab[HK2][1].begin,ZEIT_FORM,0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-DAUER Mo"," h     ", P&abs_tab[HK2][1].dauer, US_CHAR, 0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-WERT  Mo"," K     ", P&abs_tab[HK2][1].abwert,	US_INT, 1, P&hid1,		V1, 0, 0},

	{" 36;"," ABSENK-BEGIN Di"," h:min ", P&abs_tab[HK2][2].begin,ZEIT_FORM,0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-DAUER Di"," h     ", P&abs_tab[HK2][2].dauer, US_CHAR, 0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-WERT  Di"," K     ", P&abs_tab[HK2][2].abwert,	US_INT, 1, P&hid1,		V1, 0, 0},

	{" 37;"," ABSENK-BEGIN Mi"," h:min ", P&abs_tab[HK2][3].begin,ZEIT_FORM,0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-DAUER Mi"," h     ", P&abs_tab[HK2][3].dauer, US_CHAR, 0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-WERT  Mi"," K     ", P&abs_tab[HK2][3].abwert,	US_INT, 1, P&hid1,		V1, 0, 0},

	{" 38;"," ABSENK-BEGIN Do"," h:min ", P&abs_tab[HK2][4].begin,ZEIT_FORM,0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-DAUER Do"," h     ", P&abs_tab[HK2][4].dauer, US_CHAR, 0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-WERT  Do"," K     ", P&abs_tab[HK2][4].abwert,	US_INT, 1, P&hid1,		V1, 0, 0},

	{" 39;"," ABSENK-BEGIN Fr"," h:min ", P&abs_tab[HK2][5].begin,ZEIT_FORM,0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-DAUER Fr"," h     ", P&abs_tab[HK2][5].dauer, US_CHAR, 0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-WERT  Fr"," K     ", P&abs_tab[HK2][5].abwert,	US_INT, 1, P&hid1,		V1, 0, 0},

	{" 40;"," ABSENK-BEGIN Sa"," h:min ", P&abs_tab[HK2][6].begin,ZEIT_FORM,0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-DAUER Sa"," h     ", P&abs_tab[HK2][6].dauer, US_CHAR, 0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-WERT  Sa"," K     ", P&abs_tab[HK2][6].abwert,	US_INT, 1, P&hid1,		V1, 0, 0},

	{" 41;"," ABSENK-BEGIN So"," h:min ", P&abs_tab[HK2][7].begin,ZEIT_FORM,0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-DAUER So"," h     ", P&abs_tab[HK2][7].dauer, US_CHAR, 0, P&hid1,		V1, 0, 0},
	{" ->;"," ABSENK-WERT  So"," K     ", P&abs_tab[HK2][7].abwert,	US_INT, 1, P&hid1,		V1, 0, 0},

	{"*->."," Zeitabsenkphase","       ", P&abs_ram[HK2].absen,	 JANEIN_FORM, 0, P&hid2,V0, 0, 0},
//*** AnFre 06.06.2012 
	{" 42;"," LEGIONELLENTOD "," JA=1  ", P&hks[HK2].Legjn,				 US_CHAR, 0, P&vis,		V1, 0, 0},
	{" 43;"," LEGION.-TAGE   "," Mo=1..", P&hks[HK2].Legtag,	GWOTAG2_FORM, 0, P&hid1,	V1, 0, 0},
	{" 44;"," LEGION.-ZEIT   "," h:min ", P&hks[HK2].Legzeit,		 ZEIT_FORM, 0, P&hid1,	V1, 0, 0},
	{" 45;"," LEGION.-DAUER  "," h     ", P&hks[HK2].Legdau,			 US_CHAR, 0, P&hid1,	V1, 0, 0},
	{" 46;"," LEGION.-TEMP.  "," C    ", P&hks[HK2].Twwleg,				US_INT, 1, P&hid1,	V1, 0, 0},
	{" ->."," Legio Dauer Cnt"," *30 s ", P&hkd[HK2].zleg,			 		US_INT, 0, P&hid2,	V0, 0, 0},
	
	{" 50;"," PU-SOLL NORMAL "," %     ", P&hks[HK2].BusPuSollwert,		 		US_INT, 1, P&hid1,	V1, 0, 0},
	{" 51;"," PU-SOLL ABSENK "," %     ", P&hks[HK2].BusPuSollwertAbsenk,	US_INT, 1, P&hid1,	V1, 0, 0},
	{" 55;"," REGELART SOLL  "," 0..3  ", P&BusPuPara[PU_BUS_HK2 - 1].Regelart,	US_CHAR,  0, P&wilo4,	V1, 0, IDX0_FELD},

	{" 60;"," SOMMER AUS     "," JA=1  ", P&hks[HK2].Swhk,				 US_CHAR, 0, P&hid1,	V1, 0, 0},
	{" 61;"," HEIZGRENZE  Ta "," C    ", P&hks[HK2].TaHeizgrenze,		 S_INT, 1, P&hid1,	V1, 0, 0},
	{" 62."," Frostgrenze Ta "," C    ", P&hks[HK2].TaFrostgrenze,		 S_INT, 1, P&hid2,	V0, 0, 0},
	{" 63."," Absch-Zeitkonst"," h/K   ", P&hks[HK2].AbschZeitKonst,	US_INT, 1, P&hid2,	V0, 0, 0},
	{" 64."," Schnellaufheiz."," JA=1  ", P&hks[HK2].Sauf,				 	 US_CHAR, 0, P&hid2,	V0, 0, 0},

	{" 68."," TDiff.Alarm Ein"," K     ", P&hks[HK2].TabwMax,			 US_CHAR, 1, P&hid2,	V0, 0, 0},
	{" 69."," ZVerz.Alarm Ein"," min   ", P&hks[HK2].TabwZeit,		 US_CHAR, 0, P&hid2,	V0, 0, 0},

	{"*70:"," SOLARE ANHEB.? ","       ", P&hkd[HK2].solAnheb, JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*71:"," LEGION.BETRIEB?","       ", P&hkd[HK2].legio,		 JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*73:"," PU NK1     EIN?","       ", P&hkd[HK2].puEinAnz, JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*->."," Pumpen-Laufzeit"," h     ", P&hkd[HK2].puLzAnz,			 US_LONG, 0, P&hid2,	V1, 0, 0},
	{" ->."," Pu-Nachlaufzeit"," min   ", P&hks[HK2].Puna,					US_CHAR, 0, P&hid2,	V1, 0, 0},
	{" ->."," Pum-Nachl.  Cnt"," *30 s ", P&hkd[HK2].zpu,						 US_INT, 0, P&hid2,	V0, 0, 0},
	{"*->:"," PUMPENSOLLWERT "," %     ", P&AA_UNI[U3],					AOUT_FORMP, 1, P&vis,		V1, 0, 0},
	{"*->:"," BM PU NK1  EIN?","       ", P&hkd[HK2].puBmAnz,  JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*->:"," WILO-BUS NETZ-P","       ", P&BusPuPara[PU_BUS_HK2 - 1].Funktion,PU_FUNC_FORM,0, P&vis,	V1, 0, IDX0_FELD},
	{"*->;"," BUS-ADRESSE    ","       ", P&BusPuPara[PU_BUS_HK2 - 1].Adresse,		 US_CHAR,  0, P&wilo4,V1, 0, IDX0_FELD},
	{"*->:"," REGELART IST   ","       ", P&BusPuData[PU_BUS_HK2 - 1].regelArtIst,ASCII_FORM,15,P&wilo4,V1,0, IDX0_FELD},
	{"*->;"," DREHZAHL IST   "," 1/min ", P&modb_data[PU_BUS_HK2 - 1].input_reg5, 	US_INT,  0, P&wilo4,V1, 0, IDX0_FELD},
	{"*->;"," DIFFERENZDRUCK "," mWS   ", P&modb_data[PU_BUS_HK2 - 1].input_reg0, 	US_INT,	 1, P&wilo4,V1, 0, IDX0_FELD},
	{"*->;"," FOERDERSTROM   "," m/h  ", P&modb_data[PU_BUS_HK2 - 1].input_reg1, 	US_INT,	 1, P&wilo4,V1, 0, IDX0_FELD},

//#if PU_BUS_HK2 > 0								// Bus-Pumpe
//	{"*75:"," BUS-PUMPE EIN ?","       ", P&hkd[HK2].busPuEin, 			 JANEIN_FORM, 0, P&vis,		V1, 0, 0},	//Ausgang nach Pumpe
//	{" 76;"," PU-SOLL NORMAL "," %     ", P&hks[HK2].BusPuSollwert,		 		US_INT, 1, P&hid1,	V1, 0, 0},
//	{" 77;"," PU-SOLL ABSENK "," %     ", P&hks[HK2].BusPuSollwertAbsenk,	US_INT, 1, P&hid1,	V1, 0, 0},
//#endif	

	{"*80:"," BETRIEBS-REGIE ","       ", P&hkd[HK2].regie_txt, DYN_ASCII_FORM, 0, P&vis,		V1, 0, 0},
	{"*80:"," ABSENKPHASE ?  ","       ", P&hkd[HK2].absen,		 JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{" ->."," Absenkwert abs "," K     ", P&abs_ram[HK2].tvab,			US_INT, 1, P&hid2,	V0, 0, 0},
	{" ->."," Absenkzeit abs "," *10 s ", P&abs_ram[HK2].zasd,			US_INT, 0, P&hid2,	V0, 0, 0},
	{"*81:"," ABSCHALTPHASE ?","       ", P&hkd[HK2].absch,		 JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{" ->."," Abschalt   abs "," *10 s ", P&abs_ram[HK2].zAbsch,		US_INT, 0, P&hid2,	V0, 0, 0},
	{"*82;"," AUFHEIZPHASE ? ","       ", P&hkd[HK2].aufhe,		 JANEIN_FORM, 0, P&hid1,	V1, 0, 0},
	{" ->."," Aufheiz    abs "," *10 s ", P&abs_ram[HK2].zahd,			US_INT, 0, P&hid2,	V0, 0, 0},
	{"*83:"," SOMMER  AUS ?  ","       ", P&hkd[HK2].somme,	 	 JANEIN_FORM, 0, P&vis,		V1, 0, 0},
//	{"*->:"," STUETZBETRIEB ?","       ", P&hkd[HK2].stuetz,	 JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*84:"," FROSTBETRIEB ? ","       ", P&hkd[HK2].frost,		 JANEIN_FORM, 0, P&hid1,	V1, 0, 0},

	{"*85:"," RUECKL.BEGR. ? ","       ", P&hkd[HK2].rlbegr,	 JANEIN_FORM, 0, P&vis,		V1, 0, 0},
	{"*86:"," SM REGELABW.?  ","       ", P&hkd[HK2].abwAlarm, JANEIN_FORM, 0, P&vis, 	A1, EINZEL,   0},
	{"*87:"," SM STW NK1 ?   ","       ", P&STWHK[HK2], 		 JANEIN_FORMIP, 0, P&vis,		A1, EINZEL,   0},
	{"*88:"," SM PU NK1     ?","       ", P&hkd[HK2].puSmAnz,	 JANEIN_FORM, 0, P&vis,		E1, STANDARD, 0},

//	#if PU_BUS_HK2 > 0
//	{"*88:"," SM BUS NETZ-PU?","       ", P&hkd[HK2].busPuSm,	JANEIN_FORM, 0, P&vis,		A1, EINZEL,   0},
//	#endif

	{"*89:"," HANDBETRIEB ?  ","       ", P&hks[HK2].Haut,		 JANEIN_FORM, 0, P&vis,		E1, EREIGNIS, 0},

	{" 90;"," P-VERSTAERKUNG "," %/K   ", P&hks[HK2].Kp,						US_INT, 2, P&hid1,	V1, 0, 0},
	{" 91;"," D-VERSTAERKUNG ","       ", P&hks[HK2].Kd,						US_INT, 2, P&hid1,	V1, 0, 0},
	{" 92;"," TASTZEIT       "," s     ", P&hks[HK2].Ts,						US_INT, 1, P&hid1,	V1, 0, 0},
	{" 93;"," NACHSTELLZEIT  "," s     ", P&hks[HK2].Tn,						US_INT, 1, P&hid1,	V1, 0, 0},

	{" 94;"," VENTILSTELL.MIN"," %     ", P&hks[HK2].Y_rel_min,	 		US_INT, 1, P&hid1,	V1, 0, 0},
	{" 95;"," OEFFNUNGSBEGINN"," %     ", P&hks[HK2].Y_rel_beg,	 		US_INT, 1, P&hid1,	V1, 0, 0},

	{"*96;"," SOLLWERT       "," C    ", P&hkd[HK2].tsol,					US_INT, 1, P&hid1,	V1, 0, 0},
	{"*97;"," REGELABWEICHUNG"," K     ", P&hkd[HK2].ei,						 S_INT, 1, P&hid1,	V1, 0, 0},
	{"*->;"," dy_rel         ","       ", P&hkd[HK2].dy_rel,				 S_INT, 3, P&hid2,	V0, 0, 0},

	#if DREIP_HK2==0
	{"*98:"," V.STELL. RV NK1"," %     ", P&RVENT[HK2],					AOUT_FORMP, 1, P&vis,		V1, 0, 0},
	{"*98;"," VENT.MIN BERECH"," %     ", P&hkd[HK2].ventMinBer, 		US_INT, 1, P&hid1,	V1, 0, 0},
	{" ->;"," RL-Temp Mindern"," K     ", P&hks[HK2].RLKorrMin,			 S_INT, 1, P&hid2,	V0, 0, 0},

	#endif

	//-------------------------------------------------------------------------------------------------
	{" 99;"," HAND/AUTOMATIK "," HAND=1", P&hks[HK2].Haut,				 US_CHAR, 0, P&hid1,	V1, 0, 0},

	{" ->:"," PU NK1 EIN/AUS "," EIN=1 ", P&hks[HK2].Puea,		 		 US_CHAR, 0, P&hid1,	V1, 0, 0},
	{" ->;"," PU NK1 REGELART"," 0..3  ", P&BusPuPara[PU_BUS_HK2-1].RegelartHand,	US_CHAR, 0, P&wilo4,	V1, 0, IDX0_FELD},
	{" ->;"," PU NK1 SOLLWERT"," %     ", P&hks[HK2].PuDzHand,		  US_INT, 1, P&hid1,	V1, 0, 0},

	{" ->:"," VENT ST. RV NK1"," %     ", P&hks[HK2].Vstell,			  US_INT, 1, P&hid1,	V1, 0, 0},

};


/*------------------------------ Heizkurven-Adaption ----------------------------------------------*/
