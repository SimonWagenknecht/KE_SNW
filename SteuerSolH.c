/*	Task:	30s			SteuerSol		*/
/*	Solar-Puffer-Steuerung		*/

#include <stdlib.h>
#include "struct.h"
#include "ustruct.h"
#include "defins.h"
#include "sramext.h"
#include "uramext.h"
#include "standdef.h"
#include "unoinitext.h"

void PuSolLaufz ( void );
void PuPufLaufz ( void );
// void PuEin ( void );
// void PuAus ( void );
void SolarAus (void);
void SolarKollEin (void);
// void SolarKollLad (void);
void SolarPuffEin (void);
void SolarPuffLad (void);
void pusBm ( void ); 			// BM Solar-Pumpe
void pupBm ( void );			// BM Puffer-Pumpe
// int  KwVentil ( void );		// UV-Kaltwasser ersetzt den Überströmer

void Pumpenbus_Sol ( char so );	// Wilo-PumpenBus

/*---------------------- Task SteuerSol -----------------------------------*/
// Takt: 30 s

void SteuerSol(void)
{	
	char	i;
	static char neustart = 1;		// Programmstart
	static char neumess = 0;		// Berechnung Solarsensor 1mal pro Minute
//	int th2, th3, th4, th5; AnFre 06.06.2012 in den RAM verlegt war tw1 bis tw4
// AnFre 03.08.2011 in den RAM verlegt:	int ts1, ts2, ts3, ts4, ts5;
// AnFre 03.08.2011 in den RAM verlegt:	int ts6warm, ts7kalt;
	static float fl_ae_uni1 = 0;	//gedämpfter alter Messwert
	float	messwNeu = 0;						//Neuer Messwert mit Offset
	float	filter = 0;							// Filter-Konstante
	float fl_ss_Wh = 0;

	if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
	{
	//Laufzeit Solar-Pumpe ==> Solarbetrieb 
		PuSolLaufz ();
	// Betriebsmeldung Solarpumpe  überwachen
		pusBm();
	}

	if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
	{
	//Laufzeit Puffer-Pumpe ==> Beladen / Entladen
		PuPufLaufz ();
	// Betriebsmeldung Pufferpumpe  überwachen
		pupBm();
	}
	
//  CO2-Einsparung berechnen aus Wmeng(3) ZIN09 
	CO2_Einspar = ( wmengCtr[2].wmeng * CO2_Faktor ) / 1000;

/* =================== Messung der Solar-Strahlung ======================================== */

// Neustart : gespeicherte Werte aus nichtfluechtigem RAM	
	if ( neustart )
	{
 		bicbus(EEPADR,	(char *)&sod[SO1].jahrAlt, STRAHLUNG1_ADR, 42, BICRD);	//!!! feste Länge !!!
 		bicbus(EEPADR,	(char *)&sod[SO1].SS1_Monat[0], STRAHLUNG2_ADR, 48, BICRD);	//!!! feste Länge !!!
		sod[SO1].monAlt = Mon;
		sod[SO1].jahrAlt = Jahr;
		neustart = 0;
	}

// Alle SS-Zähler Synchronisieren
	if ( sod[SO1].SS1_Sync == 1 ) 
	{
		sod[SO1].fl_ss_Wh_g = (float)sod[SO1].SS1_kWh_g * 1000;	//Gesamtwert
		sod[SO1].fl_ss_Wh_j = (float)sod[SO1].SS1_kWh_j * 1000;	//Jahreswert
		sod[SO1].fl_ss_Wh_m = (float)sod[SO1].SS1_kWh_m * 1000;	//Monatswert
		sod[SO1].fl_ss_Wh_d = (float)sod[SO1].SS1_Wh_d;	//Tageswert
		sod[SO1].fl_ss_Wh_h = (float)sod[SO1].SS1_Wh_h;	//Stundenwert

		bicbus(EEPADR,	(char *)&sod[SO1].jahrAlt, STRAHLUNG1_ADR, 42, BICWR);	//!! feste Länge !!!
		bicbus(EEPADR,	(char *)&sod[SO1].SS1_Monat[0], STRAHLUNG2_ADR, 48, BICWR);	//!! feste Länge !!!
		sod[SO1].SS1_Sync = 0;
	}
	else
	{
// Berechnung Solar-Sensor-Strahlung 
		if ( neumess )	// 1mal pro Minute
		{
			neumess = 0;
		//StundenWerte
			if ( Min == 0 )
			{	//neue Stunde
				sod[SO1].fl_ss_Wh_h = 0;
				sod[SO1].SS1_Wh_vh = sod[SO1].SS1_Wh_h;
				//alle Stunden sichern
		 		bicbus(EEPADR,	(char *)&sod[SO1].jahrAlt, STRAHLUNG1_ADR, 42, BICWR);	//!! feste Länge !!!
		 		bicbus(EEPADR,	(char *)&sod[SO1].SS1_Monat[0], STRAHLUNG2_ADR, 48, BICWR);	//!!! feste Länge !!!
		// TagesWerte
				if ( Std == 0 )
				{	//neuer Tag
					sod[SO1].fl_ss_Wh_d = 0;
					sod[SO1].SS1_Wh_vd = sod[SO1].SS1_Wh_d;
				}
			}	//Ende Minute=0
		//Monatswerte
			if ( sod[SO1].monAlt == Mon )
			{ // Monatswerte speichern
				sod[SO1].SS1_Monat[Mon-1] = sod[SO1].SS1_kWh_m;
			}
			else			
			{	//neuer Monat
				sod[SO1].monAlt = Mon;
				sod[SO1].fl_ss_Wh_m = 0;
				sod[SO1].SS1_kWh_vm = sod[SO1].SS1_kWh_m;
			}
	//Jahreswerte
			if ( sod[SO1].jahrAlt < Jahr )
			{	//neues Jahr
				sod[SO1].jahrAlt = Jahr;
				sod[SO1].fl_ss_Wh_j = 0;
				sod[SO1].SS1_kWh_vj = sod[SO1].SS1_kWh_j;
				for ( i = 0; i < 12; i++ )
				{
					sod[SO1].SS1_VJ_Monat[i] = sod[SO1].SS1_Monat[i];		// SolarSensor Messung  Monate Vorjahr	[ kWh/m^2 ]
					sod[SO1].SS1_Monat[i] = 0;		// SolarSensor Messung  Monate	[ kWh/m^2 ]
				}	
			}
			if ( (AE_UNI[U1]->stat != UNTERL) && (AE_UNI[U1]->messw > 0) )
			{ //kein Unterlauf
				sod[SO1].SS1_Anz.stat = AE_UNI[U1]->stat;

				messwNeu = (float) Gerade_YvonX ( AE_UNI[U1]->messw, 0, sos[SO1].SS1_Skal0, 1000, sos[SO1].SS1_Skal10 );

				if ( messwNeu < 0 )
					messwNeu = 0;
		
					if ( sos[SO1].SS1_Filter > 1 )
					{
						filter = (float) sos[SO1].SS1_Filter;
						fl_ae_uni1 = ( fl_ae_uni1 * ( filter -1) + messwNeu) / filter; // Filtern
					}
					else
						fl_ae_uni1 = messwNeu;
						
					sod[SO1].SS1_Anz.messw = (int)(fl_ae_uni1);
					fl_ss_Wh = ( fl_ae_uni1 / 60 );
					sod[SO1].fl_ss_Wh_g = sod[SO1].fl_ss_Wh_g + fl_ss_Wh;
					sod[SO1].fl_ss_Wh_j = sod[SO1].fl_ss_Wh_j + fl_ss_Wh;
					sod[SO1].fl_ss_Wh_m = sod[SO1].fl_ss_Wh_m + fl_ss_Wh;
					sod[SO1].fl_ss_Wh_d = sod[SO1].fl_ss_Wh_d + fl_ss_Wh;
					sod[SO1].fl_ss_Wh_h = sod[SO1].fl_ss_Wh_h + fl_ss_Wh;
					sod[SO1].SS1_kWh_g = (long)(sod[SO1].fl_ss_Wh_g / 1000);
					sod[SO1].SS1_kWh_j = (int)(sod[SO1].fl_ss_Wh_j / 1000);
					sod[SO1].SS1_kWh_m = (int)(sod[SO1].fl_ss_Wh_m / 1000);
					sod[SO1].SS1_Wh_d = (int)sod[SO1].fl_ss_Wh_d;
					sod[SO1].SS1_Wh_h = (int)sod[SO1].fl_ss_Wh_h;
			}	//Ende kein UNTERL
			else
			{ // Unterlauf
				sod[SO1].SS1_Anz.messw = 0;
				sod[SO1].SS1_Anz.stat = UNTERL;
			}
		}	
		else
		{	//alle Minute messen
			neumess = 1;
		}
	}				 
/* ======================= Ende Messung der Solar-Strahlung ========================= */

// Zuordnug der Temperaturen
	if ( TRS[HK1]->stat == 0 )
		th2 = TRS[HK1]->messw;	//HK1 Rücklauf
	else
		th2 = TW4_[WW1]->messw;	//Speicher Unten

// ***AnFre 06.06.2012 TW1 nicht vorh.
	if ( TW2_[WW1]->stat == 0 )
		th3 = TW2_[WW1]->messw;	//Speicher Oben
	if ( TW3_[WW1]->stat == 0 )
		th4 = TW3_[WW1]->messw;	//Speicher Mitte
	if ( TW4_[WW1]->stat == 0 )
		th5 = TW4_[WW1]->messw;	//Speicher Unten
	
	if ( TS1_[SO1]->stat == 0 )
		ts1 = TS1_[SO1]->messw;
// ***AnFre 06.06.2012 TS2 nicht vorh.
//	if ( TS2_[SO1]->stat == 0 )
//		ts2 = TS2_[SO1]->messw;
//	else
//		ts2 = TS1_[SO1]->messw;
	if ( TS3_[SO1]->stat == 0 )
		ts3 = TS3_[SO1]->messw;
	if ( TS4_[SO1]->stat == 0 )
		ts4 = TS4_[SO1]->messw;
	if ( TS5_[SO1]->stat == 0 )
		ts5 = TS5_[SO1]->messw;

// Ermitteln der momentanen max. und min. Temperatur über 2 Speicherfühler
// im Normalfall liefert der obere Fuehler max, der untere min
// wenn 1 Fuehler nicht vorh. oder defekt, wird der andere fuer min und max verwendet	
	if ( TS6_[SO1]->stat == 0 && TS7_[SO1]->stat == 0 )
	{																			// beide Fuehler in Ordnung
		ts6warm = TS6_[SO1]->messw;
		ts7kalt = TS7_[SO1]->messw;			
	}
	else
	{ if ( TS6_[SO1]->stat == 0 )
				ts6warm = ts7kalt = TS6_[SO1]->messw; 	//nur oberer Fühler o.k.
		else
		{ if ( TS7_[SO1]->stat == 0 )
			ts6warm = ts7kalt = TS7_[SO1]->messw;//nur unterer Fühler o.k.
			else
			ts6warm = ts7kalt = 0;	//Kein Fühler o.k.
		}
	}

// ======================= Automatik-Betrieb ===========================================

// Solarbetrieb Zeitschaltung nach Zeit[h:min] und Dauer[h]
// Bei Abschaltung SolarAus() und return
	if ( sos[SO1].SolAb[0] == Min && sos[SO1].SolAb[1] == Std )
	{ sod[SO1].zSolAus = (int) sos[SO1].SolAbDau * 120;		//Abschaltdauer => Minuten
	}
	if ( sod[SO1].zSolAus > 0 )
	{ sod[SO1].zSolAus --;
		sod[SO1].solAus = 1;			//Solar abgeschaltet nach Zeit
		SolarAus( );
		goto SolarEnde;
	}
	sod[SO1].solAus = 0;				//Solar nicht abgeschaltet nach Zeit

/* --------------------------------------------------------------- */
// ***AnFre 10.07.2007
// Umschalten auf Puffer-Entladebetrieb wegen zu hoher Temp. an der Solar-Pumpe: TS5 > TS5Max
// Alarm wird in Alarme.c gebildet
	if ( pusoTempAl == 1 )
	{
		sod[SO1].kollEin = 0;	// Kollektor-Betrieb abschalten
		sod[SO1].puffEin = 1;	// Puffer-Betrieb einschalten
		sod[SO1].zPuffEin = 0;
		sod[SO1].pufLad = 0;
		sod[SO1].zPruefSperre	= sos[SO1].PruefSperre *2;	// nächste Prüfung Solar sperren [30s]
		SolarPuffLad ();
		goto SolarEnde;
	}

/* --------------------------------------------------------------- */
// ***AnFre	03.08.2011
//  Prüfen, ob Solar möglich ( "Schnupperschaltung ist in Betrieb" )
	if ( sod[SO1].zPruefDauer > 0 )
	{
		sod[SO1].zPruefDauer --;	// Prüfung läuft noch
		sod[SO1].pruefBetrieb = 1;
		MVKOLEA[SO1]->wert = 0;			// UV auf Kollektor-Weg 
		PUSOLEA[SO1]->wert = 1;			// Solarpumpe EIN
		sod[SO1].solarPu	=	1;			// Wilo-PumpenBus
		if ( BusPuPara[PU_BUS_PUF-1].Funktion == 1 )		// ***AnFre Wilo-PumpenBus
			PUPUFEA[SO1]->wert = 1;			// Relais immer Ein
		else
			PUPUFEA[SO1]->wert = 0;			// Pufferpumpe AUS
		sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus

		sod[SO1].zPruefSperre	= sos[SO1].PruefSperre *2;	// nächste Prüfung sperren [30s]
		goto SolarEnde;
	}
	else
	{
		sod[SO1].pruefBetrieb = 0;
	}

/* ---------------------------------------------------------------- */

	if ( sod[SO1].kollEin == 1 )
	{ //KollektorBetrieb ******************************************
		sod[SO1].zPruefDauer 	= 0;	// Prüfung aus ***AnFre 03:08:2011
		sod[SO1].pruefBetrieb = 0;
		sod[SO1].zPruefSperre	= sos[SO1].PruefSperre *2;	// nächste Prüfung sperren [30s]

		if( MVKOLEA[SO1]->wert == 1 )	//noch nicht AB->A KollektorWeg ?
		{	
			if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
				PUSOLEA[SO1]->wert = 0;			// Solarpumpe AUS während Ventil fährt
			sod[SO1].solarPu	=	0;			// Wilo-PumpenBus
		}
		else
		{	PUSOLEA[SO1]->wert = 1;		// Solarpumpe EIN
			sod[SO1].solarPu	=	1;		// Wilo-PumpenBus
		}
		MVKOLEA[SO1]->wert = 0;	// AB->A Kollektorweg
		if ( sod[SO1].zKollEin ++ < sos[SO1].VerzKollEin *2 )
		{
			goto SolarEnde;
		}
	//'MindestLaufZeit' ist abgelaufen
		if ( ts1 < sos[SO1].TSolEnd && ts3 < sos[SO1].TSolEnd )
		{ // KollektorBetrieb abschalten
			sod[SO1].kollEin = 0;
			goto SolarEnde;
		}

		if ( hkd[HK1].extAnfAktiv == 0 && hkd[HK1].bedLadung == 0 )
		{ // HK1 nicht aktiv: ==> TH5
			if ( (ts3 < th5) && (ts4 < ts7kalt) )
			{ // KollektorBetrieb abschalten
				sod[SO1].kollEin = 0;
				goto SolarEnde;
			}
		}
		else
		{ // HK1 aktiv: ==> TH2
			if ( (ts3 < th2) && (ts4 < ts7kalt) )
			{ // KollektorBetrieb abschalten
				sod[SO1].kollEin = 0;
				goto SolarEnde;
			}
		}
		
// ------- Abfrage PufferBeladung möglich ? ----------------------------

		if ( hkdSoL[HK1].hkSolJa == 0 && hkdSoL[HK1].solLadung == 0 )
		{ // Kein solares Heizen und Keine Solare Ladung
			if ( ts4 > (ts7kalt + sos[SO1].DTPufLad) )
			{//PufferBeladung ist möglich
				sod[SO1].pufLad = 1;
				if( MVPUFEA[SO1]->wert == 0 || sod[SO1].solarPu == 0 )	// noch nicht AB->B Beladen ?
				{
					if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
						PUPUFEA[SO1]->wert = 0;			// Pufferpumpe AUS während Ventil fährt
					sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
				}
				else
				{	PUPUFEA[SO1]->wert = 1;
					sod[SO1].pufferPu	=	1;			// Wilo-PumpenBus
				}
				MVPUFEA[SO1]->wert = 1;
			}
			else
			{//PufferBeladung nicht möglich
				if ( ts4 < (ts7kalt+10)	)
				{//PufferLadung abschalten
					sod[SO1].pufLad = 0;
				if ( BusPuPara[PU_BUS_PUF-1].Funktion == 1 )		// ***AnFre Wilo-PumpenBus
					PUPUFEA[SO1]->wert = 1;		// Relais immer Ein
				else
					PUPUFEA[SO1]->wert = 0;
				sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
				}
			}
		}
		else
		{
			if ( hkdSoL[HK1].hkSolJa == 1 )
			{ // Solares Heizen ist Ein
				if ( (ts4 > ts7kalt + sos[SO1].DTPufLad) &&
						((RVENTSO[HK1]->awert < sos[SO1].RvHksStart) ||
							(ts4 > sos[SO1].TS5Max - 100)) )
				{//PufferBeladung ist möglich
					sod[SO1].pufLad = 1;
					if( MVPUFEA[SO1]->wert == 0 || sod[SO1].solarPu == 0 )	// noch nicht AB->B Beladen ?
					{
						if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
							PUPUFEA[SO1]->wert = 0;			// Pufferpumpe AUS während Ventil fährt
						sod[SO1].pufferPu	 = 0;
					}
					else
					{
						PUPUFEA[SO1]->wert = 1;
						sod[SO1].pufferPu	 = 1;
					}
					MVPUFEA[SO1]->wert = 1;
				}
				else
				{//PufferBeladung nicht möglich
					if ( (ts4 < ts7kalt+10)	||
							 (RVENTSO[HK1]->awert > sos[SO1].RvHksEnde) )
					{//PufferLadung abschalten
						sod[SO1].pufLad = 0;
						if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
							PUPUFEA[SO1]->wert = 0;
						sod[SO1].pufferPu	 = 0;
					}
				}
			}
			else
			{ // Solare Ladung ist Ein
				if ( (ts4 > ts7kalt + sos[SO1].DTPufLad) &&
							((AA_UNI[U2]->awert > sos[SO1].PumStart) ||
							(ts4 > sos[SO1].TS5Max - 100)) )
				{//PufferBeladung ist möglich
					sod[SO1].pufLad = 1;
					if( MVPUFEA[SO1]->wert == 0 || sod[SO1].solarPu == 0 )	// noch nicht AB->B Beladen ?
					{
						if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
							PUPUFEA[SO1]->wert = 0;			// Pufferpumpe AUS während Ventil fährt
						sod[SO1].pufferPu	 = 0;
					}
					else
					{	PUPUFEA[SO1]->wert = 1;
						sod[SO1].pufferPu	 = 1;
					}
					MVPUFEA[SO1]->wert = 1;
				}
				else
				{//PufferBeladung nicht möglich
					if ( (ts4 < ts7kalt+10)	||
							 (AA_UNI[U2]->awert < sos[SO1].PumEnde) )
					{//PufferLadung abschalten
						sod[SO1].pufLad = 0;
						PUPUFEA[SO1]->wert = 0;
						sod[SO1].pufferPu	 = 0;
					}
				}
			}
		}
	}	//Ende KollektorBetrieb
/* ---------------------------------------------------------------------------------- */
	else
	{	// Kein KollektorBetrieb *************************************
		sod[SO1].pufLad = 0;
		if ( sod[SO1].puffEin == 0 || sod[SO1].zPuffEin > sos[SO1].VerzPuffEin *2 )
		{ // nicht in der Startphase für Pufferbetrieb
			if ( ts1 > sos[SO1].TSolBeg )
			{
				if ( hkd[HK1].extAnfAktiv == 0 && hkd[HK1].bedLadung == 0 )
				{ // HK1 nicht aktiv: ==> TH5
					if ( (ts1 > th5 + sos[SO1].DTKoll) || (ts1 > ts7kalt + sos[SO1].DTKoll) )
					{//KollektorBetrieb einschalten
						sod[SO1].zKollEin = 0;
						SolarKollEin ();
						goto SolarEnde;
					}
				}
				else
				{ // HK1 aktiv: ==> TH2
					if ( (ts1 > th2 + sos[SO1].DTKoll) || (ts1 > ts7kalt + sos[SO1].DTKoll) )
					{//KollektorBetrieb einschalten
						sod[SO1].zKollEin = 0;
						SolarKollEin ();
						goto SolarEnde;
					}
				}
			}
		}

		if ( sod[SO1].puffEin == 0 )				
		{// und Kein Pufferbetrieb ------------------------------------ 

			if ( hkd[HK1].extAnfAktiv == 0 && hkd[HK1].bedLadung == 0 )
			{ // HK1 nicht aktiv: ==> TH5
				if ( ts6warm > th5 + sos[SO1].DTPuff )
				{//PufferBetrieb einschalten
					sod[SO1].zPuffEin = 0;
					SolarPuffEin ();
					goto SolarEnde;
				}
			}
			else
			{ // HK1 aktiv: ==> TH2
				if ( ts6warm > th2 + sos[SO1].DTPuff )
				{//PufferBetrieb einschalten
					sod[SO1].zPuffEin = 0;
					SolarPuffEin ();
					goto SolarEnde;
				}
			}

			 // 03.08.2011 Prüfen, ob Solar möglich ( "Schnupperschaltung" )
			if ( (Std > 6 && Std < 21) && (sos[SO1].PruefDauer > 0) )
			{
				if ( sod[SO1].zPruefSperre > 0 )
				{
					sod[SO1].zPruefSperre --;
				}
				else
				{
					if ( (sod[SO1].SS1_Anz.stat == 0 && sod[SO1].SS1_Anz.messw > sos[SO1].PruefSS1) ||
							 (sod[SO1].SS1_Anz.stat != 0 && ts1 > sos[SO1].PruefTS1) )											//TS1 abfragen, wenn SS1 defekt ist
					{ //Prüfbetrieb einschalten
						sod[SO1].zPruefDauer	= sos[SO1].PruefDauer *2;	// Dauer Prüfung ein [30s]
						MVKOLEA[SO1]->wert = 0;													// UV auf Kollektor-Weg 
						if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
							PUSOLEA[SO1]->wert = 0;													// Solarpumpe Aus während Ventil fährt
						sod[SO1].solarPu	=	0;			// Wilo-PumpenBus
						if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
							PUPUFEA[SO1]->wert = 0;													// Pufferpumpe AUS
						sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
						goto SolarEnde;
					}
				}
			}
			else
			{
				sod[SO1].zPruefSperre = 0;
				sod[SO1].zPruefDauer	= 0;
				sod[SO1].pruefBetrieb = 0;
			}
			SolarAus( );
		}
		else
		{//PufferBetrieb (Entladen)**********************************************
			if( MVKOLEA[SO1]->wert == 0 )	//noch nicht AB->B PufferWeg ?
			{	
				if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
					PUSOLEA[SO1]->wert = 0;			// Solarpumpe AUS während Ventil fährt
				sod[SO1].solarPu	=	0;			// Wilo-PumpenBus
			}
			else
			{	PUSOLEA[SO1]->wert = 1;		// Solarpumpe EIN
				sod[SO1].solarPu	=	1;			// Wilo-PumpenBus
			}
			MVKOLEA[SO1]->wert = 1;	// AB->B Entladen Puffer
			if( MVPUFEA[SO1]->wert == 1 || sod[SO1].solarPu == 0 )		//noch nicht AB->A Entladen ?
			{	
				if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
					PUPUFEA[SO1]->wert = 0;				// Pufferpumpe AUS während Ventil fährt
				sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
			}
			else			
			{	PUPUFEA[SO1]->wert = 1; //PufferPumpe EIN
				sod[SO1].pufferPu	=	1;			// Wilo-PumpenBus
			}
			MVPUFEA[SO1]->wert = 0;	// AB->A Entladen
			if ( sod[SO1].zPuffEin ++ < sos[SO1].VerzPuffEin *2 )
			{
				goto SolarEnde;
			}
	//'Wartezeit' ist abgelaufen
	
//			 // 03.08.2011 Prüfen, ob Solar möglich ( "Schnupperschaltung" )
//			if ( (Std > 6 && Std < 21) && (sos[SO1].PruefDauer > 0) )
//			{
//				if ( sod[SO1].zPruefSperre > 0 )
//				{
//					sod[SO1].zPruefSperre --;
//				}
//				else
//				{
//					if ( (sod[SO1].SS1_Anz.stat == 0 && sod[SO1].SS1_Anz.messw > sos[SO1].PruefSS1) ||
//							 (sod[SO1].SS1_Anz.stat != 0 && ts1 > sos[SO1].PruefTS1) )											//TS1 abfragen, wenn SS1 defekt ist
//					{ //Prüfbetrieb einschalten
//						sod[SO1].zPruefDauer	= sos[SO1].PruefDauer *2;	// Dauer Prüfung ein [30s]
//						MVKOLEA[SO1]->wert = 0;													// UV auf Kollektor-Weg 
//						if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
//							PUSOLEA[SO1]->wert = 0;													// Solarpumpe Aus während Ventil fährt
//						sod[SO1].solarPu	=	0;			// Wilo-PumpenBus
//						if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
//							PUPUFEA[SO1]->wert = 0;													// Pufferpumpe AUS
//						sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
//						goto SolarEnde;
//					}
//				}	
//			}
//			else
//			{
//				sod[SO1].zPruefSperre = 0;
//				sod[SO1].zPruefDauer	= 0;
//				sod[SO1].pruefBetrieb = 0;
//			}

			if ( hkd[HK1].extAnfAktiv == 0 && hkd[HK1].bedLadung == 0 )
			{ // HK1 nicht aktiv: ==> TH5
				if ( ts3 < th5  )
				{ // PufferBetrieb abschalten
					sod[SO1].puffEin = 0;
					if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
						PUPUFEA[SO1]->wert = 0;		// Pufferpumpe aus
					sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
					goto SolarEnde;
				}
			}
			else
			{ // HK1 aktiv: ==> TH2
				if ( ts3 < th2  )
				{ // PufferBetrieb abschalten
					sod[SO1].puffEin = 0;
					if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
						PUPUFEA[SO1]->wert = 0;		// Pufferpumpe aus
					sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
					goto SolarEnde;
				}
			}
		}
	}	//Ende Kein KollektorBetrieb
/* ------------------------------------------------------------------------------------------ */

SolarEnde:
			
//Solar-Abschaltungen bei zu hohen Temperaturen
	// Puffer Max
	if ( ts6warm > sos[SO1].TS6Max + 50 && sod[SO1].puffEin == 0 )	// Puffer zu warm ? und 'Kollektor-Betrieb'
	{
		pufferMax = 1;
	}		
	else
	{
		if ( ts6warm > sos[SO1].TS6Max && ts7kalt > sos[SO1].TS6Max - 50 && sod[SO1].puffEin == 0 ) //Puffer zu warm und Beladen ?
		{
			if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
				PUPUFEA[SO1]->wert = 0;
			sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
			pufferMax = 1;
		}
		else
			pufferMax = 0;
	}
	if ( ts6warm > sos[SO1].TS6Max + 100 )	// Puffer zu warm ? 
	{
		pufferMax = 1;
	}		

	// Speicher Max	
//	if ( th3 > sos[SO1].TW3Max )	//Speicher zu warm ?
//		speicherMax = 1;
//	else	if ( th3 < sos[SO1].TW3Max - 30 )		// 07.05.2009 Hysterese 3,0 K
//		speicherMax = 0;

	// Zirkulation Max
//	if ( (th3 > sos[SO1].TW2Max) || (TW2_[WW1]->stat != 0) )	// VL-Zirk. zu warm ?
//		zirkMax = 1;
//	else	if ( (th3 < sos[SO1].TW2Max - 30) && (TW2_[WW1]->stat == 0) )		// 07.05.2009 Hysterese 3,0 K
//		zirkMax = 0;

//	if ( speicherMax == 1 )
//	{
//		UVWWEA[SO1]->wert = 1;				// UmschaltVentil TWW-Solar[19]: 1	 AB->A Speicherweg
//		MVKALTW[SO1]->awert = 0;			// Regelventil Kaltwasser[15]:	0,0% AB->B direkter Speicherweg	
//		MVWLEA[WW1]->wert = 0;				// Ladeventil[17] ZU
//		wwd[WW1].solWwLad = 0;
//	}
//	else
//	{
//		if ( zirkMax == 1 )
//		{
//			UVWWEA[SO1]->wert = 0;				// UmschaltVentil TWW-Solar: 0 AB->B Speicherweg
//			MVKALTW[SO1]->awert = 0;			// 0,0% AB->B Speicherweg über PWÜ-Solar	
//			MVWLEA[WW1]->wert = 0;				// Ladeventil ZU
//			wwd[WW1].solWwLad = 0;
//		}
//	}

// TS3 > TS2 - Alarm	(Rückerwärmung aus Speicher)
//	if ( (ts3 > (ts2 + 50)) && sod[SO1].solarPu > 0 )
//	{
//		if ( sod[SO1].zts23Al < sos[SO1].vts23Al )	// 10 Minuten Verzögerung
//			sod[SO1].zts23Al++;
//		else
//		{
//			sod[SO1].ts23Alarm = 1;
//			sod[SO1].ts23AlDauer++;
//		}
//	}
//	else
//	{
//		if ( ts3 < (ts2 + 10) )
//		{
//			sod[SO1].ts23Alarm = 0;
//			sod[SO1].zts23Al = 0;
//		}
//	}


// ------------ Drehzahl-Ausgabe der Pumpe Solarkreis -------------------------------------
	if ( sos[SO1].Haut == 0 && BusPuPara[PU_BUS_SOL-1].Funktion == 0 )
	{
		if( sod[SO1].solarPu == 1 )		// Solar-Pumpe EIN ?
		{
			if( MVKOLEA[SO1]->wert == 0 )
			{ // Kollektorweg
				AA_UNI[U1]->awert = sos[SO1].DzSoPuKol;
			}
			else
			{ // Pufferweg
				AA_UNI[U1]->awert = sos[SO1].DzSoPuPuf;
			}
		}
		else
		{
			AA_UNI[U1]->awert = 0;
		}
	}
// ----------- Wilo-PumpenBus -----------------------------------------------------
	#if ( BUS_PU_MAX > 0 )
	Pumpenbus_Sol ( SO1 );	// Wilo-Pumpen Solarkreis / Pufferkreis	steuern
	#endif
		
// ----- Handbetrieb --------------------------------------------------------------
// auch WILO ModBus
	BusPuPara[PU_BUS_SOL-1].Hand 	= sos[SO1].Haut;
	BusPuPara[PU_BUS_PUF-1].Hand 	= sos[SO1].Haut;

	if ( sos[SO1].Haut == TRUE )	// ***AnFre 29.08.2013: HAND-Einstellungen gelten auch für WILO-ModBus 
	{ 
//		sod[SO1].solarPu	 = sos[SO1].SolPuea;		// Solarpumpe (27) ein/aus
//		sod[SO1].pufferPu	 = sos[SO1].PufPuea;		// Pufferpumpe (28) ein/aus
		MVKOLEA[SO1]->wert = sos[SO1].KolMvea;		// UmschaltVentil Kollektor (26) Auf/Zu
		MVPUFEA[SO1]->wert = sos[SO1].PufMvea;		// UmschaltVentil Puffer (25) Auf/Zu
		UVWWEA[SO1]->wert  = sos[SO1].SolMvea;		// UmschaltVentil TWE-Solar (14) Zu/Auf  
		MVKALTW[SO1]->awert = sos[SO1].KwMvst;		// MotorVentil Solar (20)  Analog 0,0-100,0% 
		sod[SO1].zSolAus = 0; 										// Abschaltung löschen bei HAND

// Pumpe Solarkreis
		if ( sos[SO1].SolPuea > 3 )	// bei WILO 0...3 Aus, Ein, Min, Max
			sos[SO1].SolPuea = 0;
		if ( sos[SO1].SolPuea > 0 )
			sod[SO1].solarPu = 1;			// Solarpumpe (27) ein
		else
			sod[SO1].solarPu = 0;			// Solarpumpe (27) aus

	  AA_UNI[U1]->awert		= sos[SO1].DzSoPuHand;								// Drehzahl-Ausgabe Solarpumpe (27)

		if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )								// ***AnFre Kein Wilo-PumpenBus
		{
			PUSOLEA[SO1]->wert	= sod[SO1].solarPu;					// Solarpumpe ein/aus
			BusPuPara[PU_BUS_SOL-1].BetriebHand		= 0;
			BusPuPara[PU_BUS_SOL-1].SollHand			= 0;
		}
		else
		{
			PUSOLEA[SO1]->wert	= 1;																	// Solarpumpe (27) immer ein bei ModBus
			BusPuPara[PU_BUS_SOL-1].BetriebHand		= sos[SO1].SolPuea;			// Solarpumpe (27) ein/aus, Min.-, Max.-Drehzahl
			// RegelartHand wird direkt in parli eingetragen
			BusPuPara[PU_BUS_SOL-1].SollHand	= sos[SO1].DzSoPuHand;	// Sollwert Solarpumpe (27)
		}

// Pumpe Pufferkreis
		if ( sos[SO1].PufPuea > 3 )	// bei WILO 0...3 Aus, Ein, Min, Max
			sos[SO1].PufPuea = 0;
		if ( sos[SO1].PufPuea > 0 )
			sod[SO1].pufferPu = 1;			// Pufferpumpe (28) ein
		else
			sod[SO1].pufferPu = 0;			// Pufferrpumpe (28) aus

		if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )								// ***AnFre Kein Wilo-PumpenBus
		{
			PUPUFEA[SO1]->wert = sod[SO1].pufferPu;										// Pufferpumpe (28) ein/aus
			BusPuPara[PU_BUS_PUF-1].BetriebHand		= 0;
			BusPuPara[PU_BUS_PUF-1].SollHand			= 0;
		}
		else
		{
			PUPUFEA[SO1]->wert = 1;																		// Pufferpumpe (27) immer ein bei ModBus
			BusPuPara[PU_BUS_PUF-1].BetriebHand		= sos[SO1].PufPuea;			// Pufferpumpe (27) ein/aus, Min.-, Max.-Drehzahl
			// RegelartHand wird direkt in parli eingetragen
			// SollHand Pufferpumpe  wird direkt in parli eingetragen
		}

		sod[SO1].pusoBm = 0;				// kein Alarm
		sod[SO1].pusoBmVerz = 0;		//Verzögerung Soft-Alarm: 'kein Betriebsmeldungs-Signal' ***AnFre 06.06.12
		sod[SO1].pupuBm = 0;				// kein Alarm
		sod[SO1].pupuBmVerz = 0;		//Verzögerung Soft-Alarm: 'kein Betriebsmeldungs-Signal' ***AnFre 06.06.12
	}
	else
	{
		sos[SO1].Haut = FALSE;								// evtl. Fehleingabe korrigieren
	}

} //Programm-Ende
/* ------------------------------------------------------------------------------------- */
/*****************************************************/
/***** Funktion : Solarbetrieb ausschalten			 *****/
/*****************************************************/

void SolarAus (void)
{
	sod[SO1].pufLad = 0;
	sod[SO1].kollEin = 0;
//	sod[SO1].zKollEin = 0;
	sod[SO1].puffEin = 0;
//	sod[SO1].zPuffEin = 0;
//	wwd[WW1].solWwLad = 0;		// SolareLadung AUS: MVWLEA = wwlad		
	if ( BusPuPara[PU_BUS_SOL-1].Funktion == 1 )		// ***AnFre Wilo-PumpenBus
		PUSOLEA[SO1]->wert = 1;		// Relais immer Ein
	else
		PUSOLEA[SO1]->wert = 0;		// Solarpumpe aus
	sod[SO1].solarPu	=	0;			// Wilo-PumpenBus
	if ( BusPuPara[PU_BUS_PUF-1].Funktion == 1 )		// ***AnFre Wilo-PumpenBus
		PUPUFEA[SO1]->wert = 1;		// Relais immer Ein
	else
		PUPUFEA[SO1]->wert = 0;		// Pufferpumpe aus
	sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
	MVKOLEA[SO1]->wert = 1;		// MotorVentil Kollektor AB->B PufferWeg
	MVPUFEA[SO1]->wert = 1;		// MotorVentil Puffer AB->B BeladeWeg

//	UVWWEA[SO1]->wert = 0;		// UmschaltVentil TWW-Solar 0: B->AB Speicherweg
//	MVKALTW[SO1]->awert = (int) KwVentil();	// MotorVentil KW-Solar Analog 0,0 bis 100,0% 
//	if ( (PUKEEA[KE1]->wert == 0) && (RVENTWW[WW1]->awert == 0) )
//	{
//		if ( (int)(volstrCtr.flow / 100) < sos[SO1].KwSolVolMax - 2 )		// maximaler Zapfstrom -0,2 m^3
//		{
//			MVKALTW[SO1]->awert = 0;					// MotorVentil Kaltwasser über Solar-TWW 0,0% AB->B  (ZU)
//		}
//		else if ( (int)(volstrCtr.flow / 100) > sos[SO1].KwSolVolMax + 2 )		// maximaler Zapfstrom +0,2 m^3
//		{
//			MVKALTW[SO1]->awert = sos[SO1].KwSolVentMin;	//	UV-Kaltwasser x% auf Nachwärmerweg
//		}
//	}
}	//FunktionsEnde

/************************************************************/
/***** Funktion : Solar-Kollektorbetrieb einschalten		*****/
/************************************************************/

void SolarKollEin (void)
{
//	wwd[WW1].solWwLad = 0;
	sod[SO1].kollEin = 1;
	sod[SO1].puffEin = 0;
	if( MVKOLEA[SO1]->wert == 1 )	//noch nicht AB->A KollektorWeg ?
	{	
		if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
			PUSOLEA[SO1]->wert = 0;			// Solarpumpe AUS während Ventil fährt
		sod[SO1].solarPu	=	0;			// Wilo-PumpenBus
	}
	else
	{	PUSOLEA[SO1]->wert = 1;		// Solarpumpe EIN
		sod[SO1].solarPu	=	1;			// Wilo-PumpenBus
	}
	MVKOLEA[SO1]->wert = 0;			// AB->A Kollektorweg

//	UVWWEA[SO1]->wert = 0;			// UmschaltVentil TWW-Solar 0: B->AB Speicherweg
//	if ( (int)(volstrCtr.flow / 100) < sos[SO1].KwSolVolMax - 2 )		// maximaler Zapfstrom -0,2 m^3
//	{
//		MVKALTW[SO1]->awert = 0;					// MotorVentil Kaltwasser über Solar-TWW 0,0% AB->B  (ZU)
//	}
//	else if ( (int)(volstrCtr.flow / 100) > sos[SO1].KwSolVolMax + 2 )		// maximaler Zapfstrom +0,2 m^3
//	{
//		MVKALTW[SO1]->awert = sos[SO1].KwSolVentMin;	//	UV-Kaltwasser x% auf Nachwärmerweg
//	}
}	//FunktionsEnde

/////******************************************************/
/////***** Funktion : Solar-Kollektor-Ladebetrieb 		*****/
/////******************************************************/
////
//void SolarKollLad (void)
//{
//	wwd[WW1].solWwLad = 1;
//	sod[SO1].kollEin = 1;
//	sod[SO1].puffEin = 0;
//	if( MVKOLEA[SO1]->wert == 1 )	//noch nicht AB->A KollektorWeg ?
//	{	
//		if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
//			PUSOLEA[SO1]->wert = 0;			// Solarpumpe AUS während Ventil fährt
//		sod[SO1].solarPu	=	0;			// Wilo-PumpenBus
//	}
//	else
//	{	PUSOLEA[SO1]->wert = 1;		// Solarpumpe EIN
//		sod[SO1].solarPu	=	1;			// Wilo-PumpenBus
//	}
//	MVKOLEA[SO1]->wert = 0;	// AB->A Kollektorweg
//	UVWWEA[SO1]->wert = 1;			// UmschaltVentil TWW-Solar 1: A->AB Solarweg
//	if ( (int)(volstrCtr.flow / 100) < sos[SO1].KwSolVolMax - 2 )		// maximaler Zapfstrom -0,2 m^3
//	{
//		MVKALTW[SO1]->awert = 0;					// MotorVentil Kaltwasser über Solar-TWW 0,0% AB->B  (ZU)
//	}
//	else if ( (int)(volstrCtr.flow / 100) > sos[SO1].KwSolVolMax + 2 )		// maximaler Zapfstrom +0,2 m^3
//	{
//		MVKALTW[SO1]->awert = sos[SO1].KwSolVentMin;	//	UV-Kaltwasser x% auf Nachwärmerweg
//	}
//}	//FunktionsEnde

/********************************************************************/
/***** Funktion : Solar-Pufferbetrieb (Entladen) ohne LadeBetrieb einschalten		*****/
/********************************************************************/

void SolarPuffEin (void)
{
		sod[SO1].kollEin = 0;
		sod[SO1].puffEin = 1;
		if( MVKOLEA[SO1]->wert == 0 )	//noch nicht AB->B PufferWeg ?
		{
			if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
				PUSOLEA[SO1]->wert = 0;			// Solarpumpe AUS während Ventil fährt
			sod[SO1].solarPu	=	0;			// Wilo-PumpenBus
			sod[SO1].zPruefSperre	= sos[SO1].PruefSperre *2;	// nächste Prüfung sperren [30s] ***AnFre 03.08.2011
		}
		else
		{	PUSOLEA[SO1]->wert = 1;			// Solarpumpe EIN
			sod[SO1].solarPu	=	1;			// Wilo-PumpenBus
		}
		MVKOLEA[SO1]->wert = 1;				// MotorVentil Kollektor AB->B PufferWeg
		if( MVPUFEA[SO1]->wert == 1 || sod[SO1].solarPu == 0 )		//noch nicht AB->A Entladen ?
		{	
			if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
				PUPUFEA[SO1]->wert = 0;				// Pufferpumpe AUS während Ventil fährt
			sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
		}
		else			
		{	PUPUFEA[SO1]->wert = 1;			// Pufferpumpe EIN
			sod[SO1].pufferPu	=	1;			// Wilo-PumpenBus
		}
		MVPUFEA[SO1]->wert = 0;				// AB->A PufferEntladen

//		UVWWEA[SO1]->wert = 0;				// UmschaltVentil TWW-Solar 0: B->AB Speicherweg
//		if ( (int)(volstrCtr.flow / 100) < sos[SO1].KwSolVolMax - 2 )		// maximaler Zapfstrom -0,2 m^3
//		{
//			MVKALTW[SO1]->awert = 0;					// MotorVentil Kaltwasser über Solar-TWW 0,0% AB->B  (ZU)
//		}
//		else if ( (int)(volstrCtr.flow / 100) > sos[SO1].KwSolVolMax + 2 )		// maximaler Zapfstrom +0,2 m^3
//		{
//			MVKALTW[SO1]->awert = sos[SO1].KwSolVentMin;	//	UV-Kaltwasser x% auf Nachwärmerweg
//		}

}	//FunktionsEnde

/********************************************************************/
/***** Funktion : Solar-Puffer (Entladen) mit WW-Ladebetrieb		*****/
/********************************************************************/

void SolarPuffLad (void)
{
	wwd[WW1].solWwLad = 1;
	sod[SO1].kollEin = 0;
	sod[SO1].puffEin = 1;
	if( MVKOLEA[SO1]->wert == 0 )	//noch nicht AB->B PufferWeg ?
	{
		if ( BusPuPara[PU_BUS_SOL-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
			PUSOLEA[SO1]->wert = 0;			// Solarpumpe AUS während Ventil fährt
		sod[SO1].solarPu	=	0;			// Wilo-PumpenBus

		sod[SO1].zPruefSperre	= sos[SO1].PruefSperre *2;	// nächste Prüfung sperren [30s] ***AnFre 03.08.2011
	}
	else
	{	PUSOLEA[SO1]->wert = 1;			// Solarpumpe EIN
		sod[SO1].solarPu	=	1;			// Wilo-PumpenBus
	}
	MVKOLEA[SO1]->wert = 1;				// MotorVentil Kollektor AB->B PufferWeg
	if( MVPUFEA[SO1]->wert == 1 || sod[SO1].solarPu == 0 )		//noch nicht AB->A Entladen ?
	{	
		if ( BusPuPara[PU_BUS_PUF-1].Funktion == 0 )		// ***AnFre Wilo-PumpenBus
			PUPUFEA[SO1]->wert = 0;				// Pufferpumpe AUS während Ventil fährt
		sod[SO1].pufferPu	=	0;			// Wilo-PumpenBus
	}
	else			
	{	PUPUFEA[SO1]->wert = 1;			// Pufferpumpe EIN
		sod[SO1].pufferPu	=	1;			// Wilo-PumpenBus
	}
	MVPUFEA[SO1]->wert = 0;				// AB->A PufferEntladen

//	UVWWEA[SO1]->wert = 1;				// UmschaltVentil TWW-Solar 0: A->AB Solarweg
//	if ( (int)(volstrCtr.flow / 100) < sos[SO1].KwSolVolMax - 2 )		// -0,2 m^3
//	{
//		MVKALTW[SO1]->awert = 0;					// MotorVentil Kaltwasser über Solar-TWW 0,0% AB->B  (ZU)
//	}
//	else if ( (int)(volstrCtr.flow / 100) > sos[SO1].KwSolVolMax + 2 )		// +0,2 m^3
//	{
//		MVKALTW[SO1]->awert = sos[SO1].KwSolVentMin;	//	UV-Kaltwasser x% auf Nachwärmerweg
//	}
}	//FunktionsEnde

/************************************************************/
 /***** Funktion : Pumpenlaufzeit 										 *****/
  /***** laeuft alle 30 Sekunden fuer jeden Solarkreis*****/	 
	 /******************************************************/
void PuSolLaufz ( void )
{
		if ( PUSOLEA[SO1]->func == STATISCH && sod[SO1].solarPu )	
		{
			if ( ++sod[SO1].lzKolPuMin >= 120 )
			{
				sod[SO1].lzKolPuMin = 0;
				++sod[SO1].lzKolPuH;							// Stundenzaehler inkr.	
				bicbus(EEPADR, (char*)&sod[SO1].lzKolPuH, SOLH1_ADR+(SO1*2), 2, BICWR);
			}
		}
}				

/************************************************************/
/***** Funktion : Pumpenlaufzeit 											 *****/
/***** laeuft alle 30 Sekunden fuer jeden Solarkreis  *****/	 
/*********************************************************/
void PuPufLaufz ( void )
{
		if ( PUPUFEA[SO1]->func == STATISCH && sod[SO1].pufferPu )	
		{
			if ( ++sod[SO1].lzPufPuMin >= 120 )
			{
				sod[SO1].lzPufPuMin = 0;
				++sod[SO1].lzPufPuH;							// Stundenzaehler inkr.	
				bicbus(EEPADR, (char*)&sod[SO1].lzPufPuH, PUFH1_ADR+(SO1*2), 2, BICWR);
			}
		}
}				

///****************************************/
///***** Funktion : Pumpe einschalten *****/
///****************************************/		
//void PuEin ( void )		
//{
//	sod[SO1].zpu = (char)( sos[SO1].Puna + 1 );	
//	PUSOLEA[SO1]->wert = 1;								// Pumpenrelais einschalten			
//}
//
///*****************************************************/
///***** Funktion : Pumpe ausschalten mit Nachlauf *****/
///***** laeuft alle 30 sek fuer jeden Solarkreis  *****/	 
///*****************************************************/
//void PuAus ( void )
//{
//		if ( sod[SO1].zpu )
//			sod[SO1].zpu--;
//		if ( sod[SO1].zpu == 0 )	
//			PUSOLEA[SO1]->wert = 0;			// Pumpenrelais ausschalten
//}
									 
/*********************************************************/
/***** Betriebsmeldung Einer Solarpumpe überwachen   *****/
/*********************************************************/
void pusBm ( void )
{
	if(BMPUSOL[SO1]->bwert == 1)	// Betriebsmeldung  EIN ?
	{
		sod[SO1].pusoBm = 0;				// kein Alarm
		sod[SO1].pusoBmVerz = 0;		//Verzögerung Soft-Alarm: 'kein Betriebsmeldungs-Signal' ***AnFre 06.06.12
	}
	else
	{
		if(PUSOLEA[SO1]->func == STATISCH && sod[SO1].solarPu == 1)			// Solarpumpe EIN ?
		{
			if ( sod[SO1].pusoBmVerz < 3)
				sod[SO1].pusoBmVerz ++;
			else
			{
				sod[SO1].pusoBm = 1;			// Soft-Alarm: 'kein Betriebsmeldungs-Signal'
			}
		}
	}
}
									 
/*********************************************************/
/***** Betriebsmeldung Einer Pufferpumpe überwachen   *****/
/*********************************************************/
void pupBm ( void )
{
	if(BMPUPUF[SO1]->bwert == 1)	// Betriebsmeldung  EIN ?
	{
		sod[SO1].pupuBm = 0;				// kein Alarm
		sod[SO1].pupuBmVerz = 0;		//Verzögerung Soft-Alarm: 'kein Betriebsmeldungs-Signal' ***AnFre 06.06.12
	}
	else
	{
		if(PUPUFEA[SO1]->func == STATISCH && sod[SO1].pufferPu == 1)			// Pufferpumpe EIN ?
		{
			if ( sod[SO1].pupuBmVerz < 3)
				sod[SO1].pupuBmVerz ++;
			else
				sod[SO1].pupuBm = 1;			// Soft-Alarm: 'kein Betriebsmeldungs-Signal'
		}
	}
}

/*****************************************************************/	
/**** KW-Ventil entsprechend Volumenstrom stellen             ****/
/*****************************************************************/	
//int KwVentil ( void )
//{
//	int kwVentil = 0;		//KW-Ventil öffnen entsprechend Volumenstrom
//	int kwVolstr = 0;		// KW-Volumenstrom in m3*10
//	if ( kwzDefekt == 0 )
//	{
//		kwVolstr = (int)(volstrCtr.flow / 100);
////	testPoint[0] = kwVolstr;													//TestPunkt 1
//		if ( kwVolstr >= sos[SO1].KwVol0 )
//		{
//			kwVentil = sos[SO1].KwVentMin;
//		}
//		else if ( kwVolstr <= sos[SO1].KwVol100 )
//		{
//			kwVentil = 1000;
//		}
//		else
//		{
//			kwVentil = Gerade_YvonX ( kwVolstr, sos[SO1].KwVol100,1000, sos[SO1].KwVol0, sos[SO1].KwVentMin );
////	testPoint[1] = kwVentil;													//TestPunkt 2
//		}
//	}
//	else
//	{
//		kwVentil = sos[SO1].KwVentMin;
//	}
//
////	testPoint[2] = kwVentil;													//TestPunkt 3
//	return kwVentil;
//}

/*****************************************************/
/***** Funktion : Wilo-PumpenBus								 *****/
/*****************************************************/
void Pumpenbus_Sol ( char so )
{
#if BUS_PU_MAX > 0
	char pu = 0;
	
	if ( so == SO1 )
	{
// Pumpe Solarkreis
		pu = PU_BUS_SOL;
		
		if ( pu > 0 && pu <= BUS_PU_MAX )
		{
			pu -= 1;
		
			if ( BusPuPara[pu].Funktion == 1 )
			{
				if ( sod[so].solarPu == 1 )
				{
					BusPuPara[pu].Betrieb = 1;
					if( MVKOLEA[so]->wert == 0 )
					{ // Kollektorweg
						BusPuPara[pu].Sollwert = sos[so].DzSoPuKol;
					}
					else
					{ // Pufferweg
						BusPuPara[pu].Sollwert = sos[so].DzSoPuPuf;
					}
				}
				else
				{
					BusPuPara[pu].Betrieb = 0;
					BusPuPara[pu].Sollwert = 0;
				}				
			}

			#if GENI > 0	
			sod[so].busSolPuEin = pu_control[pu].pu_operation;	
			#endif
	
			#if WILO_MODBUS > 0	
			sod[so].busSolPuEin = modWiloPu[pu].output_value1_temp;	
			#endif
		}

// Pumpe Pufferkreis
		pu = PU_BUS_PUF;
		
		if ( pu > 0 && pu <= BUS_PU_MAX )
		{
			pu -= 1;
		
			if ( BusPuPara[pu].Funktion == 1 )
			{
				if ( sod[so].pufferPu == 1 )
				{
					BusPuPara[pu].Betrieb = 1;
					BusPuPara[pu].Sollwert = sos[so].BusPufPuSoll;		// Sollwert bei WILO Pumpenbus
				}
				else
				{
					BusPuPara[pu].Betrieb = 0;
					BusPuPara[pu].Sollwert = 0;	
				}				
			}

			#if GENI > 0	
			sod[so].busPufPuEin = pu_control[pu].pu_operation;	
			#endif
	
			#if WILO_MODBUS > 0	
			sod[so].busPufPuEin = modWiloPu[pu].output_value1_temp;	
			#endif
		}

	}
#endif
}
	