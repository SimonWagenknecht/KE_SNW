
/***** Berechnung von Waermemenge, Volumenstrom und Leistung aus dem Volumenimpuls eines Zaehleingangs *****/

/* Test : VL 57°C / RL 50°C 1 Kubikmeter => 28969,8 kJ => 8,05 kWh 10s-Impuls => 2898 kW => 360 m3/h		*/

#include "struct.h"
#include "ustruct.h"
#include "defins.h"
#include "sramext.h"
#include "uramext.h"
#include "standdef.h"
#include "uconstext.h"
#include "wmeng.h"
#include "unoinitext.h"

#if WMENG > 0

/* Berechnet wird die Wärmemenge, die dem Volumen entspricht, welches mit einem Zaehleingang des R36 erfasst wird. 		*/
/* Die Nutzung von ZIN7 bis ZIN10 ist prinzipiell möglich																															*/
/* Die Berechnung erfolgt in kJ, die Abspeicherung in den EEPROM und die Anzeige in kWh 															*/
/* Die Task laeuft alle 50 ms, die Waermemengenberechnung erfolgt einmal pro Sekunde für jeden Zähler								  */
/* Berechnung des Volumenstroms alle 200 ms																																						*/
	
/*****;TABELLEN-ANHANG ************************* aus MRC
;
; FUNKTION DER ENTHALPIE h = f(Temp.) [ p = 1,6 MPa ]
; WERTE IN 10TEL KJ/KG  FORMAT 13.3
; STUETZWEITE 10,0 GRAD
;			  grdC   kJ/kg
TABEN:	DEFW	128	;  0,01	  1,6
	DEFW	3488	; 10,0	 43,6
	DEFW	6832	; 20,0	 85,4
	DEFW	10168	; 30,0	127,1
	DEFW	13512	; 40,0	168,9
	DEFW	16848	; 50,0	210,6
	DEFW	20192	; 60,0	252,4
	DEFW	23544	; 70,0	294,3
	DEFW	26888	; 80,0	336,1
	DEFW	30248	; 90,0	378,1
	DEFW	33616	;100,0	420,2
	DEFW	36992	;110,0	462,4
	DEFW	40376	;120,0	504,7
	DEFW	43776	;130,0	547,2
	DEFW	47192	;140,0	589,9
	DEFW	50624	;150,0	632,8
	DEFW	54088	;160,0	676,1
	DEFW	57568	;170,0	719,6
	DEFW	61072	;180,0	763,4
	DEFW	64616	;190,0	807,7
	DEFW	0FFFFH	;200,0	852,4	;68192
	DEFW	0FFFFH	;220,0
;
; FUNKTION DER DICHTE  RHO = f(Temp.) [ p = 1,6 MPa ]
; WERTE IN KG/M**3  FORMAT S10.5
; STUETZEITE 10,0 GRAD
;
TABDI	DEFW	32019	;  0,01	1000,6
	DEFW	32016	; 10,0	1000,5
	DEFW	31968
	DEFW	31885
;	DEFW	31774
	DEFW	31639	; 50,0	 988,7285
	DEFW	31484
;	DEFW	31308
;	DEFW	31113
;	DEFW	30906
	DEFW	30684	;100,0	 958,8647
;	DEFW	30444
;	DEFW	30194
;	DEFW	29929
;	DEFW	29649
	DEFW	29358	;150,0	 917,4312
;	DEFW	29051
;	DEFW	28730
;	DEFW	28394
;	DEFW	28041
	DEFW	27672	;200,0	 864,7527
	DEFW	25000	;210,0
;
*****/

/***** Dichte des Wassers als Funktion der Temperatur ( Quelle : MRC ) *****/
/* RHO = f(Temp.) [ p = 1,6 MPa ]																					 */
/* Werte in kG/l, Stuetzweite 10 Grad von 0,01°C .. 200°C									 */

const float Dichte[] = { 1.0006, 1.0005, 0.9990, 0.9964, 0.9929, 0.9887, 0.9839, 0.9784, 0.9723, 0.9658, 0.9588, 
															 0.9514, 0.9436, 0.9353, 0.9265, 0.9174, 0.9078, 0.8978, 0.8873, 0.8763, 0.8647 };


/***** Enthalpie des Wassers als Funktion der Temperatur ( Quelle : MRC ) *****/
/* h = f(Temp.) [ p = 1,6 MPa ]																								*/
/* Werte in kJ/kg, Stuetzweite 10 Grad von 0,01°C .. 200°C											*/

const float Enthalp[] = { 0.01, 43.6,  85.4, 127.1, 168.9, 210.6, 252.4, 294.3, 336.1, 378.1, 420.2, 
														 462.4, 504.7, 547.2, 589.9, 632.8, 676.1, 719.6, 763.4, 807.7, 852.4 }; 

;/* 
;_________________________________________
;*****************************************
;SOLAR-Absorber mit 40,0 Vol.% TYFOCOR L
;*****************************************
;------------------------------------------
;
TABENS:												kJ/(kg*K)

	DEFW	29	; 0,1	  0,361			3,50
	DEFW	2920	;10,0	 36,5 		3,64
	DEFW	5894	;20,0	 73,7			3,68
	DEFW	8926	;30,0	111,6			3,72
	DEFW	12029	;40,0	150,4			3,76
	DEFW	15196	;50,0	190,0			3,79
	DEFW	18379	;60,0	229,7			3,83
	DEFW	21666	;70,0	270,8			3,87
	DEFW	25018	;80,0	312,7			3,90
	DEFW	28361	;90,0	354,5			3,94
	DEFW	31832	;100	397,9			3,98
	DEFW	35367	;110	442,1			4,01
	DEFW	38880	;120	486,0			4,05
	DEFW	40000
	DEFW	40000
	DEFW	40000
	DEFW	40000
	DEFW	40000
	DEFW	40000
	DEFW	40000
	DEFW	40000
	DEFW	0FFFFH
	DEFW	0FFFFH
;
; FUNKTION DER DICHTE  RHO = f(Temp.) [ p = 1,6 MPa ]
; WERTE IN KG/M**3  FORMAT S10.5
; STUETZEITE 10,0 GRAD
;
TABDIS:													Solvis 40% Tyfocor
	DEFW	32767	;  0,01	1023			1045
	DEFW	32767
	DEFW	32767										1038
	DEFW	32767								
	DEFW	32767	; 40,0	1023			1028
	DEFW	32640	; 50,0  1020
	DEFW	32432	; 60,0  1013,5		1014
	DEFW	32208	; 70,0  1006,5
	DEFW	31984	; 80,0   999,5		999
	DEFW	31760	; 90,0   992,5
	DEFW	31504	;100,0   984,5		983
	DEFW	31280	;110,0   977,5
	DEFW	31040	;120,0	 970,0		965
	DEFW	30000
	DEFW	30000
	DEFW	30000
	DEFW	30000
	DEFW	30000
	DEFW	30000
	DEFW	30000
	DEFW	30000
	DEFW	30000
;
*****/

/***** Dichte 40% Tyfocor als Funktion der Temperatur ( Quelle : R36 ) *****/
/* RHO = f(Temp.) [ p = 1,6 MPa ]																					 */
/* Werte in kG/l, Stuetzweite 10 Grad von 0,01°C .. 200°C									 */

const float Dichte40[] = { 1.050, 1.045, 1.040, 1.034, 1.028, 1.022, 1.015, 1.008, 1.001, 0.994, 0.986, 
															 0.978, 0.970, 0.962, 0.953, 0.944, 0.934, 0.924, 0.913, 0.902, 0.890 };
/***** Enthalpie 40% Tyfocor als Funktion der Temperatur ( Quelle : R36 ) *****/
/* h = f(Temp.) [ p = 1,6 MPa ]																								*/
/* Werte in kJ/kg, Stuetzweite 10 Grad von 0,01°C .. 200°C											*/

const float Enthalp40[] = { 0.01, 36.4,  73.6, 111.6, 150.4, 189.5, 229.8, 270.9, 312.8, 354.6, 398.0, 
														 441.1, 486.0, 531.7, 576.8, 622.5, 668.8, 715.7, 763.2, 811.3, 862.4 }; 

/***** Dichte 50% Tyfocor als Funktion der Temperatur  Quelle : Parabel (Munzert)  *****/
const float Dichte50[] = { 1.060, 1.054, 1.048, 1.042, 1.035, 1.028, 1.021, 1.014, 1.006, 0.999, 0.991, 
															 0.983, 0.974, 0.966, 0.957, 0.948, 0.938, 0.928, 0.917, 0.906, 0.894 };
/***** Enthalpie 50% Tyfocor als Funktion der Temperatur ( Quelle : Parabel (Munzert) ) *****/
const float Enthalp50[] = { 0.01, 34.3,  69.6, 105.6, 142.4, 180.0, 218.4, 258.3, 298.4, 339.3, 381.0, 
														 423.5, 468.0, 512.2, 557.2, 603.0, 649.6, 697.0, 745.2, 794.2, 842.0 }; 


/***** Dichte 60% Tyfocor als Funktion der Temperatur  Quelle : Parabel (Munzert)  *****/
const float Dichte60[] = { 1.065, 1.059, 1.052, 1.045, 1.038, 1.031, 1.024, 1.016, 1.008, 1.000, 0.992, 
															 0.983, 0.974, 0.966, 0.957, 0.948, 0.938, 0.928, 0.917, 0.906, 0.894 };
/***** Enthalpie 60% Tyfocor als Funktion der Temperatur ( Quelle : Parabel (Munzert) ) *****/
const float Enthalp60[] = { 0.01, 32.4,  65.6, 99.6, 134.4, 170.0, 206.4, 243.6, 281.6, 320.4, 360.0, 
														 397.1, 441.6, 483.6, 526.4, 570.0, 614.4, 659.6, 705.6, 752.4, 800.0 }; 


#define   MAXZEIT 3								// Faktor fuer Zeit des zuletzt erfassten Impulses bis Volumenstrom = 0 wenn kein Impuls

#define TDIFF_MAX	54000						// 3 Stunden maximaler Impulsabstand für Berechnung des Volumenstroms


char DefInputs ( char w );
char LeistungMax ( char w );

UINT ImpulsWertigkeit ( char exp );


// Task läuft alle 200 ms, jeder Zähler wird einmal pro Sekunde behandelt !
void Wmeng ( void )
{
	char i;
	int tvl, trl;										// Temp. Vorlauf / Ruecklauf
	int iv,ir;											// Index in Tabellen fuer VL und RL
	UINT impulsWert;								// Impulswertigkeit 1, 10, 100, 1000 l für Exponent 0..3
	char anzImp;
	UINT wmDiff;
	static ULONG volumenAlt = 0;		// altes Volumen seit letztem Impuls
	static char neustart[4] = {1,1,1,1};
	static char min = 0xFF;					// taskeigener Minutenzähler wegen Sprüngen der Uhrzeit bei Synchr. durch R50
																	// benutzt zur Berechnung der max.Leistung	
	static char sec;
	static char msec200;
	
	static char w = 3;

	
	if ( ++w > 3 )									// w ist der Index für die Wärmemengen-Berechnung
		w = 0;

	if ( w == 0 )	
	{			

// die aktuelle Minute bildet der Regler selbst, um Probleme mit der Uhrzeit-Synchronsation zu vermeiden
		if ( min > 60 )
		{
			min = Min;									// Anfangs-Initialisierung
			sec = Sek;
			msec200 = 0;
		}
		else if ( ++msec200 >= 5 )
		{
			msec200 = 0;
			if ( ++sec >= 60 )
			{
				sec = 0;
				if ( ++min >= 60 )
					min = 0;
			}		
		
			if ( Std == 23 && Min == 58 && Sek <= 1 )
			{
				min = 58;																			// tägliche Synchronisation, kein Problem, wenn es mal nicht klappt
				sec = Sek;
			}	
		}	
	}

		
	if ( DefInputs ( w ) == FALSE )
	{
// ***AnFre		return;												// mindestens ein Eingang ( Zaehler oder Temperaturen ) nicht definiert oder Temperatur-Fühler defekt
	}
	
	if ( wmengCtr[w].monSichern == TRUE )
	{	
		bicbus ( EEPADR,	(char *)&wmengCtr[w].verbr[Mon-1], WMENG_MON_ADR+(UINT)w*SIZE_WMENG+(Mon-1)*sizeof(wmengCtr[w].verbr[0]),
																																				 sizeof(wmengCtr[w].verbr[0]), BICWR );
		wmengCtr[w].monSichern = FALSE;
	}
																																	

	if ( wmengCtr[w].zeitdiff < TDIFF_MAX )					// 3 Stunden bei 200 ms Takt
			wmengCtr[w].zeitdiff++;	

	if ( Std == 0 && Min == 1 && w == sec )
		wmengCtr[w].lastWmeng = wmengCtr[w].wmeng;		// für Berechnung der Durchschnitts-Leistung



// Prost Neujahr ( Monatsverbräuche werden zu denen des Vorjahres )
	if ( Mon == 12 && Tag == 31 && Std == 23 && Min < 57 && wmengCtr[w].monInVj == 1 )
	{
		wmengCtr[w].monInVj = 0;
		bicbus ( EEPADR,	(char *)( &wmengCtr[w].monInVj ), MON_IN_VJ+(UINT)w*SIZE_WMENG, 1, BICWR );
	}
	else if ( wmengCtr[w].monInVj == 0 && Mon == 1 && Tag == 1 && Std == 0 && Min < 3 )
	{
		for ( i = 0; i < 12; i++ )
		{
			wmengCtr[w].verbr_vj[i] = wmengCtr[w].verbr[i];	
			wmengCtr[w].verbr[i] = 0;
		}
	 	if ( bicbus ( EEPADR,	(char *)( &wmengCtr[w].verbr ), WMENG_MON_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].verbr), BICWR ) == 0 )
	 		bicbus ( EEPADR,	(char *)( &wmengCtr[w].verbr ), WMENG_MON_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].verbr), BICWR );	
	 	if ( bicbus ( EEPADR,	(char *)( &wmengCtr[w].verbr_vj ), WMENG_VJMON_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].verbr_vj), BICWR ) == 0 )
			bicbus ( EEPADR,	(char *)( &wmengCtr[w].verbr_vj ), WMENG_VJMON_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].verbr_vj), BICWR );				
		wmengCtr[w].monInVj = 1;
		bicbus ( EEPADR,	(char *)&wmengCtr[w].monInVj, MON_IN_VJ+(UINT)w*SIZE_WMENG, 1, BICWR );
		return;
	}

// Monatsanfang: Verbrauch reset (nicht im Januar)
// ist beim Jahreswechsel bereits erfolgt, hier vorsichtshalber nochmal
	if ( Mon != 1 && Tag == 1 && Std == 0 && Min < 2 )
	{
		if ( wmengCtr[w].monReset == 1 && wmengCtr[w].verbr[Mon-1] != 0 )
		{
			wmengCtr[w].verbr[Mon-1] = 0;
			bicbus ( EEPADR,	(char *)&wmengCtr[w].verbr[Mon-1], WMENG_MON_ADR+(UINT)w*SIZE_WMENG+(Mon-1)*sizeof(wmengCtr[w].verbr[0]),
																																				 sizeof(wmengCtr[w].verbr[0]), BICWR );
			wmengCtr[w].monReset = 0;
		}
	}
	else if ( Min > 3 )
		wmengCtr[w].monReset = 1;	


	if ( wmengCtr[w].pZaehlsp->zdim.dim_idx != 6 || wmengCtr[w].pZaehlsp->zdim.exponent > 3 )
		return;												// kein Volumenzaehler oder Wertigkeit nicht 1..1000 l 

	impulsWert = ImpulsWertigkeit ( wmengCtr[w].pZaehlsp->zdim.exponent );

// Neustart 
	if ( neustart[w] == 1 )
	{
		// ***AnFre 30.04.2014 Korrektur der Fühler
		bicbus ( EEPADR,	(char *)( &WZxTVKorr[w] ), WZxTVKORR_ADR +w*2 , 2, BICRD );					// Vorlauftemp
		bicbus ( EEPADR,	(char *)( &WZxTRKorr[w] ), WZxTRKORR_ADR +w*2 , 2, BICRD );					// Rücklauftemp

		wmengCtr[w].lastWert = wmengCtr[w].pZaehlsp->zwert;
		wmengCtr[w].pZaehlsp->zdim.dim_idx = 6;
		wmengCtr[w].zeitdiff = 0;	
		wmengCtr[w].zeitdiffZuletzt = 0xFFFF;	
		neustart[w] = 0;
		return;		
	}

// ***AnFre 30.04.2014 Korrektur der Fühler
	if ( WZxTVKorr[w] != WZxTVKorrAlt[w] )
	{
		WZxTVKorrAlt[w] = WZxTVKorr[w];
		bicbus ( EEPADR,	(char *)( &WZxTVKorr[w] ), WZxTVKORR_ADR +w*2 , 2, BICWR );					// Vorlauftemp
	}	
	if ( WZxTRKorr[w] != WZxTRKorrAlt[w] )
	{
		WZxTRKorrAlt[w] = WZxTRKorr[w];
		bicbus ( EEPADR,	(char *)( &WZxTRKorr[w] ), WZxTRKORR_ADR +w*2 , 2, BICWR );					// Vorlauftemp
	}	
		
// Waermezaehler synchronisieren
	if ( wmengCtr[w].syncFlag == 1 )
	{
		bicbus ( EEPADR,	(char *)&wmengCtr[w].wmeng,	WMENG_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].wmeng), BICWR );						
		bicbus ( EEPADR,	(char *)&wmengCtr[w].verbr,	WMENG_MON_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].verbr), BICWR);	
		bicbus ( EEPADR,	(char *)&wmengCtr[w].verbr_vj, WMENG_VJMON_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].verbr_vj), BICWR);					
		wmengCtr[w].syncFlag = 0;
		return;	
	}
	else
		wmengCtr[w].syncFlag = 0;									// Fehleingaben loeschen

	
// Maxima loeschen	
	if ( wmengCtr[w].resMax == 1 )
	{
		wmengCtr[w].flow_max = 0;
		bicbus ( EEPADR,	(char *)&wmengCtr[w].flow_max,	FLOWMAX_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].flow_max), BICWR);				
		wmengCtr[w].wmLeistg_max	= 0;
		bicbus ( EEPADR,	(char *)&wmengCtr[w].wmLeistg_max,	LEISTMAX_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].wmLeistg_max), BICWR);		

#if LEIST_BER == 1
		// löschen des Maximalwerts der mittleren Leistung und aller darauf bezogenen Einträge
		MaxLeistung[w].mittelMax = 0;
		MaxLeistung[w].mittel = 0;
		MaxLeistung[w].tag.jahr = 0;
		MaxLeistung[w].tag.monat = 1;
		MaxLeistung[w].tag.tag = 1;
		MaxLeistung[w].zeit.min = 0;
		MaxLeistung[w].zeit.std = 0;
		MaxLeistung[w].vl.stat = NICHTV;
		MaxLeistung[w].vl.messw = 0;	
		MaxLeistung[w].rl.stat = NICHTV;
		MaxLeistung[w].rl.messw = 0;																			
		MaxLeistung[w].ta.stat = NICHTV;
		MaxLeistung[w].ta.messw = 0;																			
		MaxLeistung[w].flow = 0;
		bicbus ( EEPADR, (char *)&MaxLeistung[w], MITT_LEIST+(UINT)w*SIZE_MITT_LEIST, sizeof(maxLeistung), BICWR );
		wmengCtr[w].lastWmeng = 0;																						
#endif

		wmengCtr[w].resMax = 0;	
	}
	else
		wmengCtr[w].resMax = 0;										// Fehleingaben loeschen

// Vorlauf-, Rücklauf-Temperatur
	tvl = ( (wmengCtr[w].pVl)->stat == 0 ) ? ( (wmengCtr[w].pVl)->messw + WZxTVKorr[w] ): 9999;
	WZxTV[w].messw	= tvl;
	WZxTV[w].stat		= (wmengCtr[w].pVl)->stat;
	trl = ( (wmengCtr[w].pRl)->stat == 0 ) ? ( (wmengCtr[w].pRl)->messw + WZxTRKorr[w] ) : 9999;
	WZxTR[w].messw	= trl;
	WZxTR[w].stat		= (wmengCtr[w].pRl)->stat;


// Volumenänderung	
	if ( ( wmengCtr[w].pZaehlsp->zwert - wmengCtr[w].lastWert ) / impulsWert > 3 )
	{																						// Fehler, maximal 3 Imp. pro 200 msec moeglich
		wmengCtr[w].lastWert = wmengCtr[w].pZaehlsp->zwert;
		return;
	}
	else
	{	// Volumen-Impulse erfassen
		// 06.01.2017: WZ2-Solar zählt nicht, wenn MVKOLEA auf Pufferweg AB-->B
		if ( w == 2 && MVKOLEA[SO1]->wert == 1 )
		{
			wmengCtr[w].pZaehlsp->zwert = wmengCtr[w].lastWert;		// im System aufgezählen Wert zurücksetzen
		}

		anzImp = (char)( ( wmengCtr[w].pZaehlsp->zwert - wmengCtr[w].lastWert ) / impulsWert );	// max. 3
		wmengCtr[w].volumen =	(UINT)( wmengCtr[w].pZaehlsp->zwert - wmengCtr[w].lastWert );			// in Liter
		wmengCtr[w].lastWert = wmengCtr[w].pZaehlsp->zwert;
	}
	
//-	if ( anzImp > 0 )
//-	{
//-		wmengCtr[w].lastWert = wmengCtr[w].pZaehlsp->zwert;
//-		// Ermittlung der Volumen-Änderung
//-		switch ( wmengCtr[w].pZaehlsp->zdim.komma )			
//-		{		
//-			case 0 :
//-				wmengCtr[w].volumen =	(UINT)anzImp * 1000;
//-				break;
//-			case 1 :
//-				wmengCtr[w].volumen =	(UINT)anzImp * 100;
//-				break;
//-			case 2 :
//-				wmengCtr[w].volumen =	(UINT)anzImp * 10;
//-				break;
//-			default :
//-				wmengCtr[w].volumen =	(UINT)anzImp;
//-				break;																			
//-		}
//-	}
//-
	if  ( anzImp > 0 )
	{
// Volumenstrom
		if ( wmengCtr[w].zeitdiffZuletzt < 0xFFFF )						// der erste Impuls nach dem Neustart wird ignoriert 
		{
			wmengCtr[w].flow = (ULONG)(wmengCtr[w].volumen) * 3600 * 5 / wmengCtr[w].zeitdiff;	// ***AnFre 36-->3600: 0,000 m3/h !!!		
			// flow [100l/h], Volumen [l], zeitdiff[200ms]; 1 h = 3600 * 1000 ms = 3600 * 5 * 200ms
			volumenAlt = (ULONG)(wmengCtr[w].volumen); // Volumen-Differenz retten für 'Kein Impuls'
			// Maximaler Volumenstrom
			if ( wmengCtr[w].flow > wmengCtr[w].flow_max )
			{
				wmengCtr[w].flow_max = wmengCtr[w].flow;		
				bicbus ( EEPADR,	(char *)&wmengCtr[w].flow_max,	FLOWMAX_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].flow_max), BICWR);				
			}	
		}	

// Waermeleistung
		if ( (trl < tvl) && (tvl < 1800 && trl < 1800) )			// // ***AnFre	Ruecklauf < Vorlauf
		{
// ***AnFre			return;
	
// ***AnFre			if ( tvl < 5 || tvl > 1900 || trl < 5 || trl > 1900 )	// return auch bei Messwertfehler (tvp=trp=9999)
// ***AnFre				return;												// Tabelle gilt nur von 0..200 grd C, bei R38-Abstuerzen ist tvl, trl fuer 1 sec = 0 Grad C			
		
			ir = trl / 100;									// Index in Dichte-Tabelle, fuer Masse-Ermittlung gilt die Ruecklauf-Temp.
			iv = tvl / 100;									// Schrittweite 10 K, aber trl in Zehntelgrad => 100 Schritte		

// ***AnFre !!! nur ZIN9 !!! Wärmemenge Solar	mit Tyfocor 40,50 oder 60 %	
			if ( w == 2 )
			{
				if ( Tyfocor > 55 )
				{
					wmengCtr[w].masse = (float)wmengCtr[w].volumen * ( Dichte60[ir] - ( (float)( trl - 100*ir ) / 100 ) * ( Dichte60[ir] - Dichte60[ir+1] ) );
	
					wmengCtr[w].wmeng_J = (ULONG)( wmengCtr[w].masse * ( ( Enthalp60[iv] + ( (float)( tvl - 100*iv ) / 100 ) * ( Enthalp60[iv+1] - Enthalp60[iv] ) )
																-	( Enthalp60[ir] + ( (float)( trl - 100*ir ) / 100 ) * ( Enthalp60[ir+1] - Enthalp60[ir] ) ) ) );
				}
				else
				{
					if ( Tyfocor > 45 )
					{
						wmengCtr[w].masse = (float)wmengCtr[w].volumen * ( Dichte50[ir] - ( (float)( trl - 100*ir ) / 100 ) * ( Dichte50[ir] - Dichte50[ir+1] ) );
			
						wmengCtr[w].wmeng_J = (ULONG)( wmengCtr[w].masse * ( ( Enthalp50[iv] + ( (float)( tvl - 100*iv ) / 100 ) * ( Enthalp50[iv+1] - Enthalp50[iv] ) )
																		-	( Enthalp50[ir] + ( (float)( trl - 100*ir ) / 100 ) * ( Enthalp50[ir+1] - Enthalp50[ir] ) ) ) );
					}
					else 
					{
						if ( Tyfocor > 20 )
						{
							wmengCtr[w].masse = (float)wmengCtr[w].volumen * ( Dichte40[ir] - ( (float)( trl - 100*ir ) / 100 ) * ( Dichte40[ir] - Dichte40[ir+1] ) );
			
							wmengCtr[w].wmeng_J = (ULONG)( wmengCtr[w].masse * ( ( Enthalp40[iv] + ( (float)( tvl - 100*iv ) / 100 ) * ( Enthalp40[iv+1] - Enthalp40[iv] ) )
																		-	( Enthalp40[ir] + ( (float)( trl - 100*ir ) / 100 ) * ( Enthalp40[ir+1] - Enthalp40[ir] ) ) ) );
						}
						else
						{
							wmengCtr[w].masse = (float)wmengCtr[w].volumen * ( Dichte[ir] - ( (float)( trl - 100*ir ) / 100 ) * ( Dichte[ir] - Dichte[ir+1] ) );
				
							wmengCtr[w].wmeng_J = (ULONG)( wmengCtr[w].masse * ( ( Enthalp[iv] + ( (float)( tvl - 100*iv ) / 100 ) * ( Enthalp[iv+1] - Enthalp[iv] ) )
																		-	( Enthalp[ir] + ( (float)( trl - 100*ir ) / 100 ) * ( Enthalp[ir+1] - Enthalp[ir] ) ) ) );
						}
					}
				}										
			}
			else
			{
				wmengCtr[w].masse = (float)wmengCtr[w].volumen * ( Dichte[ir] - ( (float)( trl - 100*ir ) / 100 ) * ( Dichte[ir] - Dichte[ir+1] ) );
	
				wmengCtr[w].wmeng_J = (ULONG)( wmengCtr[w].masse * ( ( Enthalp[iv] + ( (float)( tvl - 100*iv ) / 100 ) * ( Enthalp[iv+1] - Enthalp[iv] ) )
															-	( Enthalp[ir] + ( (float)( trl - 100*ir ) / 100 ) * ( Enthalp[ir+1] - Enthalp[ir] ) ) ) );
			}
		}
		else
		{
			wmengCtr[w].masse = 0;
			wmengCtr[w].wmeng_J = 0;
		}
			
		if ( wmengCtr[w].zeitdiffZuletzt < 0xFFFF )						// der erste Impuls nach dem Neustart wird ignoriert 
		{
			wmengCtr[w].wmLeistg = (UINT)( wmengCtr[w].wmeng_J * 5  / wmengCtr[w].zeitdiff );

			// maximale Leistung
			if ( wmengCtr[w].wmLeistg > wmengCtr[w].wmLeistg_max )
			{
				wmengCtr[w].wmLeistg_max = wmengCtr[w].wmLeistg;		
				bicbus ( EEPADR,	(char *)&wmengCtr[w].wmLeistg_max,	LEISTMAX_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].wmLeistg_max), BICWR);		
			}
		}

// Waermemenge mittels Enthalpie ermitteln, errechnet wird die Enthalpie-Differenz zwischen Vor- und Ruecklauf		
// Enthalpie-Bestimmung per Interpolation
/*			entv = Enthalp[iv] + ( (float)( tvp - 100*iv ) / 100 ) * ( Enthalp[iv+1] - Enthalp[iv] );	
			entr = Enthalp[ir] + ( (float)( trp - 100*ir ) / 100 ) * ( Enthalp[ir+1] - Enthalp[ir] );					
			
			wmeng_J = masse * (	entv - entr );
			wmeng_Jsum += (ULONG)wmeng_J;
*/			
		wmengCtr[w].wmeng_Jsum += wmengCtr[w].wmeng_J;
 		wmDiff = (UINT)( wmengCtr[w].wmeng_Jsum / 3600 );
 		if ( wmDiff )
 		{														// mehr als 1 kWh Zuwachs
	 		wmengCtr[w].wmeng_Jsum = wmengCtr[w].wmeng_Jsum % 3600;				// Rest merken	 		
	 		wmengCtr[w].wmeng += wmDiff;
	 		wmengCtr[w].verbr[Mon-1] += wmDiff;

			bicbus ( EEPADR,	(char *)&wmengCtr[w].wmeng,	WMENG_ADR+(UINT)w*SIZE_WMENG, sizeof(wmengCtr[w].wmeng), BICWR );	
			wmengCtr[w].monSichern = TRUE;					
		}			

		wmengCtr[w].zeitdiffZuletzt = ( wmengCtr[w].zeitdiffZuletzt != 0xFFFF ) ? wmengCtr[w].zeitdiff : 0;
		wmengCtr[w].zeitdiff = 0;
	}

// kein Impuls : Volumenstrom und Leistung schrittweise verringern
	else
	{
		if ( ( wmengCtr[w].zeitdiffZuletzt > 0 ) && ( wmengCtr[w].zeitdiff > wmengCtr[w].zeitdiffZuletzt ) )
		{
			if ( ( wmengCtr[w].zeitdiff > wmengCtr[w].zeitdiffZuletzt * MAXZEIT ) || wmengCtr[w].zeitdiff >= TDIFF_MAX )
			{
				wmengCtr[w].flow = 0;
				wmengCtr[w].wmLeistg = 0;
			}				
			else if ( ( wmengCtr[w].zeitdiff - wmengCtr[w].zeitdiffZuletzt ) % 10 == 0 )		// alle 2 Sek.wird die Anzeige aktualisiert (verringert)
			{
				wmengCtr[w].flow = volumenAlt * 3600 * 5 / wmengCtr[w].zeitdiff;	// ***AnFre 36==>3600
				wmengCtr[w].wmLeistg = (UINT)( wmengCtr[w].wmeng_J * 5  / wmengCtr[w].zeitdiff );
			}								
		}
	}	

	
// Berechnung des Maximums der mittleren Leistung
#if LEIST_BER == 1

	if ( sec == (30+w) && msec200 == w )
	{
		if ( LeistungMax ( w ) == TRUE )
		{
			i = MaxLeistung[w].MittelnLeistg;
			if ( i != 0 )
			{														// keine Berechnung bei MittelnLeistg = 0, sonst gehts einmal pro Minute weiter
				// Test und Korrektur
				if ( i != 60 && i != 30 && i != 20 && i != 15 && i != 10 && i != 5 )		// zugelassene Mittelungen in min 
				{
					i = 60;												// Standardwert
					MaxLeistung[w].MittelnLeistg = i;
				}
			
				if ( ( min % i ) == 0 )
				{
					if ( wmengCtr[w].lastWmeng != 0 )				// nach Neustart zuerst Anfangs-Init.von lastWmeng
					{
						if ( wmengCtr[w].wmeng >= wmengCtr[w].lastWmeng )
						{
							MaxLeistung[w].mittel = (UINT)( ( wmengCtr[w].wmeng - wmengCtr[w].lastWmeng ) * 60 / i );
							if ( MaxLeistung[w].mittel > MaxLeistung[w].mittelMax )
							{
								MaxLeistung[w].mittelMax = MaxLeistung[w].mittel;
								MaxLeistung[w].tag.jahr = Jahr;
								MaxLeistung[w].tag.monat = Mon;
								MaxLeistung[w].tag.tag = Tag;
								MaxLeistung[w].zeit.min = Min;
								MaxLeistung[w].zeit.std = Std;
								MaxLeistung[w].vl.stat = (wmengCtr[w].pVl)->stat;
								MaxLeistung[w].vl.messw = (wmengCtr[w].pVl)->messw;	
								MaxLeistung[w].rl.stat = (wmengCtr[w].pRl)->stat;
								MaxLeistung[w].rl.messw = (wmengCtr[w].pRl)->messw;																			
								MaxLeistung[w].ta.stat = ta1m.stat;
								MaxLeistung[w].ta.messw = ta1m.messw;																			
								MaxLeistung[w].flow = wmengCtr[w].flow;
								bicbus ( EEPADR, (char *)&MaxLeistung[w], MITT_LEIST + (UINT)w * SIZE_MITT_LEIST, sizeof( maxLeistung ), BICWR );					
							}
						}
					}	
					wmengCtr[w].lastWmeng = wmengCtr[w].wmeng;
				}
			}
		}
	}
	
#endif		
	
}

			
char DefInputs ( char w )
{	
	wmengCtr[w].pZaehlsp = 0;
	wmengCtr[w].pVl = 0;	
	wmengCtr[w].pRl = 0;
	
	if ( w == 0 && ZE7WM == 1 )
	{
		wmengCtr[w].pZaehlsp = ZE[4];
		wmengCtr[w].pVl = ZE7WMVL;
		wmengCtr[w].pRl = ZE7WMRL;
	}		
	else if ( w == 1 && ZE8WM == 1 )
	{
		wmengCtr[w].pZaehlsp = ZE[5];
		wmengCtr[w].pVl = ZE8WMVL;
		wmengCtr[w].pRl = ZE8WMRL;
	}		
	else if ( w == 2 && ZE9WM == 1 )
	{
		wmengCtr[w].pZaehlsp = ZE[6];
		wmengCtr[w].pVl = ZE9WMVL;
		wmengCtr[w].pRl = ZE9WMRL;
	}		
	else if ( w == 3 && ZE10WM == 1 )
	{
		wmengCtr[w].pZaehlsp = ZE[7];
		wmengCtr[w].pVl = ZE10WMVL;	
		wmengCtr[w].pRl = ZE10WMRL;
	}		

	if ( wmengCtr[w].pZaehlsp == 0 || wmengCtr[w].pVl == 0 || wmengCtr[w].pVl->stat != 0 || wmengCtr[w].pRl == 0 || wmengCtr[w].pRl->stat != 0 )
		return (FALSE);												// keine korrekte Zuweisung von Zähleingang oder Temperaturen oder Temperaturfühler gestört
	else
		return (TRUE);
}


char LeistungMax ( char w )
{
	char ret = FALSE;
	
	if ( w == 0 && LEIST_BER_ZE7 == 1 )
		ret = TRUE;
	else if ( w == 1 && LEIST_BER_ZE8 == 1 )
		ret = TRUE;
	else if ( w == 2 && LEIST_BER_ZE9 == 1 )
		ret = TRUE;
	else if ( w == 3 && LEIST_BER_ZE10 == 1 )
		ret = TRUE;
	
	return ( ret );
}



UINT ImpulsWertigkeit ( char exp )
{
	UINT wert;
		
	for ( wert = 1; exp > 0; exp-- )
		wert *= 10;
	return ( wert ); 	
}

#endif		// #if WMENG > 0																											
																											