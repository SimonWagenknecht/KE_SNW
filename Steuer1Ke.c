/*	Task:					SteuerKe		*/
/*	Kesselsteuerung						*/

#include <stdlib.h>
#include "struct.h"
#include "ustruct.h"
#include "defins.h"
#include "sramext.h"
#include "uramext.h"
#include "standdef.h"

void kespu_schalt(char ke);	// Kessel / Pumpe schalten
void klaufz(char ke);
void ke_Bm(char ke, doutsp	*kesea);
void ke_Al(char ke);			// ***AnFre 06.03.2008
int kessel_rueckl_begr(char ke, int tvsb);

void Pumpenbus_KE ( char ke );

/*---------------------- Task SteuerKe -----------------------------------*/
// Takt: 30 s ***AnFre 09.10.2007

void SteuerKe(void)
{	
	char i;
	int x2, x1, y;
	int tvsbhk;			//Sollwert aus HK1
	int tvsbww;			//Sollwert aus TWE
	int tvsbmax;		//max. Sollwert aus HK1, TWE und ext. Anf.
	int tvsb;				//Sollwert für Kessel
	mwsp *tvkes;						// Steuergröße

	// Lokale Pointer auf Ke-Strukturen
	KeDynam    *pKed;	
	KeStandard *pKes;
	
	// Strukturpointer zuweisen
	pKed = &ked[KE1];	
	pKes = &kes[KE1];
	
	tvkes = TVKE[KE1];		// Steuergröße Kesselvorlauf

// ***AnFre 06.03.2008 Verzögerung Kessel-Alarm 
 ke_Al(KE1);			// ***AnFre 06.03.2008

// ***AnFre
// MotorVentil Kaltwasser MVKALTW im Winter Auf (100,0%)
// MotorVentil Kaltwasser MVKALTW im Sommer auf (100%), wenn
// 	Kessel ein und TKES1 > TW1 ;  sonst zu (0%) (auch wenn Kessel auf HAND)
// Achtung!!! wird in SteuerSol.c geschaltet !!!
	
// --------------------- Abfrage Hand-/Automatik ------------------------------
	if(pKes->Haut == TRUE)
	{	
		RVENTKE[KE1]->awert	= pKes->Kestell;		// Hand-Stellwert
		pKed->y_rel 			= pKes->Kestell;		// Anzeige
// Zubringer-Pumpe
		PUKEEA[KE1]->wert		= pKes->Kespuea;		// Hand KesselFreigabe oder KesselPumpe ein/aus
// ***AnFre
		pKed->tvsb = (int) ( pKes->Tvsol0 + ((long)( pKes->Tvsol10 - pKes->Tvsol0 ) * pKes->Kestell / 1000 ) );
		pKed->tvsb_kes 			= 0;								// Anzeige *20: ungültig

		#if ( BUS_PU_MAX > 0 )
		Pumpenbus_KE ( KE1 );
		#endif

		return;
	}
	else																			// Automatikbetrieb
	{	pKes->Haut 		= 0;									// evtl. Fehleingaben löschen
	}
// --------------------- Abfrage Hand-/Automatik Ende ------------------------------

	// Maximale Solltemp. ermitteln
	//-----------------------------

	//Solltemp. aus Heizung		!!!AnFre

	tvsbhk = hkd[HK1].tvsb;
	
/* ---------- ***AnFre	26.03.2008 Heiz-Bedarf melden für Kessel -------------------------- */
// wenn RV-Stellung länger als 5 min unter Minimal-Stellung
	if ( hkdSoL[HK1].heizBed == 0 )		// konventionelles RV von HK1 (hkd[HK1]
		tvsbhk = 0;								// Kein Heizungs-Bedarf

	if(tvsbhk > 0)
		tvsbhk += pKes->Tvpa;									// plus Kesselanhebung HK1
			
	//Solltemp. aus Warmwasser	***AnFre
//
//	tvsbww = prd[PR1].aktTwwsol;	 			//WW-Temp.-Profil	// Regel-Solltemp. als Sollwert
//
//	if(wwd[WW1].konvLegio > 0)								// konventionelle Legionelle ?
//		if( TW2_[0]->messw < wws[WW1].Twwleg ||	// Kessel aus, wenn TS2,TS3,TS4 > Twwleg
//		  	TW3_[0]->messw < wws[WW1].Twwleg ||
//		  	TW4_[0]->messw < wws[WW1].Twwleg )
//			tvsbww = wws[WW1].Twwleg;							// Legionellentemp. als Sollwert
//
//	if(wwd[WW1].wwvor > 0)										// Vorrang ?
//		if(tvsbww < prd[PR1].aktTwwVor)
//			tvsbww = prd[PR1].aktTwwVor;								// Regel-Vorrang-temp. als Sollwert
//
//	if(wwd[WW1].wwlad > 0)										// Ladebetrieb ?
//		if(tvsbww < prd[PR1].aktTwwlad)
//			tvsbww = prd[PR1].aktTwwlad;								// Regel-Ladetemp. als Sollwert
//
///* ---------- ***AnFre	26.03.2008 WW-Bedarf melden für Kessel -------------------------- */
//// wenn RV-Stellung länger als 5 min unter 1%-Stellung
//	if ( wwd[WW1].twwBed == 0 )
//		tvsbww = 0;								// Kein WW-Bedarf
//
//	if(tvsbww > 0)
//		tvsbww += pKes->TvPaWw;							// plus Kesselanhebung WW !!!AnFre

//Maximum HK1 oder TWE ?
	tvsbww = 0;								// Kein WW-Bedarf
	tvsbmax = tvsbhk;
//	if(tvsbww > tvsbhk)
//		tvsbmax = tvsbww;
	
// ---- Anforderung externe HAST1 oder/und	HAST2 ***AnFre 06.06.2012 ----------------------
// externe HAST1
	if ( pKes->ExtAnf1_Ja > 0 )
	{ 
		pKed->ExtAnf1Ke.messw = ExtAnf1.messw;
		pKed->ExtAnf1Ke.stat	= ExtAnf1.stat;

		if ( pKed->ExtAnf1Ke.stat == 0 )
		{ 
		 	if ( tvsbmax > 199 )
		 	{
				if ( pKed->ExtAnf1Ke.messw + pKes->TvAnhExt > tvsbmax )
					tvsbmax = pKed->ExtAnf1Ke.messw + pKes->TvAnhExt;
			}
			else
			{ // kein HK/WW-Sollwert
				if ( pKed->ExtAnf1Ke.messw > 199 )
					tvsbmax = pKed->ExtAnf1Ke.messw + pKes->TvAnhExt;
			}
		}
	}
	else
	{
		pKed->ExtAnf1Ke.messw	=	0;		
		pKed->ExtAnf1Ke.stat	=	0;		
	}
// externe HAST2
	if ( pKes->ExtAnf2_Ja )
	{ 
		pKed->ExtAnf2Ke.messw = ExtAnf2.messw;
		pKed->ExtAnf2Ke.stat	= ExtAnf2.stat;

		if ( pKed->ExtAnf2Ke.stat == 0 )
		{ 
		 	if ( tvsbmax > 200 )
		 	{
				if ( pKed->ExtAnf2Ke.messw + pKes->TvAnhExt > tvsbmax )
					tvsbmax = pKed->ExtAnf2Ke.messw + pKes->TvAnhExt;
			}
			else
			{ // kein HK/WW-Sollwert
				if ( pKed->ExtAnf2Ke.messw > 200 )
					tvsbmax = pKed->ExtAnf2Ke.messw + pKes->TvAnhExt;
			}
		}
	}
	else
	{
		pKed->ExtAnf2Ke.messw	=	0;		
		pKed->ExtAnf2Ke.stat	=	0;		
	}

// maximal geforderter Sollwert für Kessel-Sollwert eintragen
	tvsb = tvsbmax;


// Begrenzung Vorlauf MAX; MIN
	if(tvsb > 199)
	{
	// Rücklauftemperaturbegrenzung entfällt ***AnFre
//		tvsb = kessel_rueckl_begr(KE1,tvsb);		//  Fühler nicht vorhanden !!!

		if(tvsb > pKes->Tvma)
			tvsb = pKes->Tvma;
		if(tvsb < pKes->Tvmi)
			tvsb = pKes->Tvmi;
 // Bei Wartung erforderlich ( Notbetrieb )		 
		if ( NotBetr == 1 )
			tvsb = sos[SO1].NotTemp;
	}

// Sollwert für Parli *20: speichern	
	pKed->tvsb_kes = tvsb;

//-------- Sollwert schrittweise Steigerung/Minderung ------------------

 // TV-SollwertMax, wenn Kesseltemp niedrig (DT-Kaltstart)
	if((tvsb - TVKE[KE1]->messw > pKes->Ttaup) && (tvsb > 200) && (PUKEEA[KE1]->wert > 0) )
	{
		pKed->tvsb = pKes->Tvma;
	}
	else
	{
		if( tvsb < 200 || pKed->tvsb < 200 )
		{
			pKed->tvsb = tvsb;
		} 		
		else
		{
			if( pKed->tvsb < tvsb )						// Sollwert anheben
			{
				pKed->tvsb += pKes->TvsbSteig;
				if( pKed->tvsb > tvsb )
				 pKed->tvsb = tvsb;
			}
			if( pKed->tvsb > tvsb )						// Sollwert absenken
			{
				pKed->tvsb -= pKes->TvsbMinde;
				if( pKed->tvsb < tvsb )
				 pKed->tvsb = tvsb;
			}
		}
	}
	
//------------ Kessel Sollwert-Ausgabe -------------------------------------

		if( pKed->tvsb < 200 || pKed->kesea == 0 )				// wenn keine Anforderung 
		{	
			RVENTKE[KE1]->awert  = 0;							// Steuerspannung 0 Volt
			pKed->y_rel = 0;											// zur Anzeige
		}
		else																		// Freigabe einschalten
		{
			// Aus berechneter Temperatur(tvsb) die Steuerspannung(y) ermitteln
			// Eckpunkte X2 X1
			x2 = pKes->Tvsol10;			// SollTemperatur bei 10,0V
			x1 = pKes->Tvsol0;				// SollTemperatur bei  0,0V
			// y = mx + n			mit m = 10V-0V / X2-X1 		und n = (0V * X2 - 10V * X1) / X2-X1
			y = (int)(((long)1000 * (pKed->tvsb - x1)) / (x2 - x1));
			if(y > 1000)	// > 10,00 V ?
				y = 1000;
			if(y < 0)
				y = 0;	
			
			RVENTKE[KE1]->awert	 = y;						// Berechn.Temperatur als Steuerspannung ausgeben
			pKed->y_rel = y;									// zur Anzeige
		}

//------------ Kessel/Pumpe-Freigabe schalten -----------------------------

	if ( pKes->SchaltDiff > 0 && TVKE[KE1]->stat == 0 )
	{
		if ( (TVKE[KE1]->messw - pKed->tvsb) > pKes->SchaltDiff )
		{
			pKed->schaltdiff = 1;
		}
		else if ( TVKE[KE1]->messw < pKed->tvsb +10 )
		{
			pKed->schaltdiff = 0;
		}
	}
	else
	{
		pKed->schaltdiff = 0;
	}

	if( (pKed->tvsb < 200) || (pKed->schaltdiff > 0) )	
	{		// wenn keine Anforderung ODER oder 'VL-Kessel zu hoch'
		pKed->kesea  = 0;														// Kessel Freigabe AUS
	}
	else
	{
		pKed->kesea  = 1;														// Kessel Freigabe EIN
	}
	kespu_schalt(KE1);

	// Kessellaufzeiten
	klaufz(KE1);

	// Betriebsmeldungsüberwachung
	ke_Bm(KE1, PUKEEA[KE1]);

	#if ( BUS_PU_MAX > 0 )
	Pumpenbus_KE ( KE1 );											// nur im Automatik-Betrieb !!!
	#endif

	// -----------Pellet-Kesselsteuerung mit Weiche ----------------------------------

//-	if( pKed->tvsb > 200 && (TP_UNI[U1]->messw < pKes->LadBeg) )
//-	{ // Ladebeginn: Kessel-Freigebe Ein
//-		if(KE[KE1]->wert == 0 && KEAL[KE1]->bwert == 0 )
//-		{	pKed->kestarts++;								// Anzahl der Einschaltungen erhöhen
//-			bicbus(EEPADR, (char*)&pKed->kestarts, KESTARTS1_ADR+(KE1*2), 2, BICWR);
//-		}
//-		KE[KE1]->wert  = 1;										// Kessel Freigabe EIN
//-	}
//-	else																		// Freigabe einschalten
//-	{
//-		if(  TP_UNI[U1]->messw > pKes->LadEnd && TP_UNI[U2]->messw > pKes->LadEnd )
//-		{ // Ladeende: Kessel-Freigebe Aus
//-			KE[KE1]->wert  = 0;
//-		}
//-	}

}
/*---------------------- Task SteuerKe Ende ------------------------------------------------*/


// --------------------- Unterprogramme ----------------------------------- 

// Kessel/Pumpe schalten (Abfrage jede 30-Sekunde)
// IN:	ke......Kesselnummer
//			kesea...Merker für " Kessel/Pumpe Freigabe" ab 22.09.2011 in dyn. Kesselstructur
void kespu_schalt(char ke)
{
	if(ked[ke].kesea > 0)						// Kessel soll EIN ?
	{	
		if(ked[ke].puein_cnt > 0)		// Einschaltverzögerung wurde beim Kessel-Ausschalten gesetzt
			ked[ke].puein_cnt--;
		else
		{		
			if(PUKEEA[ke]->wert == 0)						// Abfrage Kessel Freigabe AUS ?
			{
				ked[ke].kestarts++;									// Anzahl der Einschaltungen erhöhen
				bicbus(EEPADR, (char*)&ked[ke].kestarts, KESTARTS1_ADR+(ke*2), 2, BICWR);
			}
			if ( DM_WPU_Ventil.stat != 0 )
			{
				PUKEEA[KE1]->wert = 1;			// Kesselpumpe EIN
				ked[ke].puna_cnt = kes[ke].Kepuna *2;		//  min Nachlauf für ZubringerPumpe
			}
			else if ( DM_WPU_Ventil.messw > kes[ke].WpVentEin )
			{
				PUKEEA[KE1]->wert = 1;			// Kesselpumpe EIN
				ked[ke].puna_cnt = kes[ke].Kepuna *2;		//  min Nachlauf für ZubringerPumpe
			}
			else if ( DM_WPU_Ventil.messw < kes[ke].WpVentAus )
			{
				PUKEEA[KE1]->wert = 0;			// Kesselpumpe AUS
				ked[ke].puein_cnt = kes[ke].Kepuverz *2;		//  min Einschaltverzögerung für ZubringerPumpe
			}
		}
	}
	else													// Kessel soll AUS ?
	{
		if(ked[ke].puna_cnt > 0)		// Nachlaufzähler wurde beim Kessel-Einschalten gesetzt
			ked[ke].puna_cnt--;
		else
		{
			PUKEEA[ke]->wert = 0;			// Kessel/Pumpe AUS
			ked[ke].puein_cnt = kes[ke].Kepuverz * 2;	// Kessel / Pumpen-Verzögerungzeit setzen
		}
	}
}				

// Kessellaufzeiten
void klaufz(char ke)
{
	static int kes_sec, kes2_sec;
	
	// Abfrage Relais Stufe 1: Kessel ist EIN   oder  Kessel Freigabe ist EIN
	if( PUKEEA[ke]->wert > 0 )
	{	if(++kes_sec > 120)					// ***AnFre 30-SekundenZähler hochzählen
		{	kes_sec = 0;
			ked[ke].kesh++;							// Laufzeit h erhöhen
			bicbus(EEPADR, (char*)&ked[ke].kesh, KESH1_ADR+(ke*2), 2, BICWR);
		}
	}
}	


// Kesselbetriebsmeldungüberwachung (Abfrage 30 Sekunden)
// IN:	ke......Kesselnummer
//			kesea...Schalt-Ausgang "KesselStufe1 oder Kesselfreigabe"
void ke_Bm(char ke, doutsp	*kesea)
{
	if(BMKES[ke]->bstat != NICHTV)		// Nur wenn Betriebsmelde-Kontakt vorhanden
	{	
		if(kesea->wert > 0)							// Kessel EIN ?
		{	
			if(BMKES[ke]->bwert == 1)			// Betriebsmeldung EIN ?		
			{
				ked[ke].keBm_sAl = 0;			// Softalarm löschen			
				ked[ke].keBm_cnt = 2;			// Zähler nachladen 1 Min für Betrieb-EIN Überwachung
			}
			else
			{	
				if(ked[ke].keBm_cnt > 0)		// Zähler gesetzt ?
				{	if(--ked[ke].keBm_cnt == 0)			
						ked[ke].keBm_sAl  = 1;	// Softalarm setzen
				}	
			}
		}
		else	// Kessel AUS	
		{
				ked[ke].keBm_sAl = 0;			// Softalarm löschen			
				ked[ke].keBm_cnt = 0;			// Zähler löschen
		}		
	}
}		

// ***AnFre 06.03.2008 Verzögerung Kessel-Alarm
void ke_Al(char ke)
{
	if(KEAL[ke]->bwert == 0)			// Keine Störung ?		
	{
		ked[ke].keAl_sAl = 0;			// Softalarm löschen			
		ked[ke].keAl_cnt = kes[ke].KeAl_verz * 2;			// Zähler nachladen (Min) für Alarm-EIN Überwachung
	}
	else
	{	
		if(ked[ke].keAl_cnt > 0)		// Zähler gesetzt ?
		{
			--ked[ke].keAl_cnt;			
		}
		else
		{
			ked[ke].keAl_sAl  = 1;	// Softalarm setzen
		}	
	}
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Rücklauftemperatur-abhaengige TVSOLL-Verschiebung						 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
// IN:	tvsb...berechnete  Vorlauftemperatur (Heizkennlinie + WW)
// OUT:	tvsb...verschobene Vorlauftemperatur

int kessel_rueckl_begr(char ke, int tvsb)
{
	float fl_tvsb, fl_absenk, fl_eir, fl_tr;
	
	fl_tvsb = fl_absenk = fl_eir = fl_tr = 0;

	ked[ke].rlbegr = 0;									// Statusanzeige Rücklauftemp.begrenzung löschen
	
		fl_tr = (float)TRKE[KE1]->messw / 10;
		
	// Abweichung RÜCKLAUF - RÜCKLAUF.MAX
	fl_eir = fl_tr - (float)kes[ke].Trma / 10;
	
	if(fl_eir >= 0)						// nur wenn MAX überschritten
	{	ked[ke].rlbegr = 1;		// Statusanzeige Rücklaufbegrenzung setzen
		
		// Errechne Absenkwert.
		fl_absenk = fl_eir * (float)kes[ke].Kpr / 100;
		
		// Lade berechnete  Vorlauftemperatur nach Heizkurve + WW
		fl_tvsb = (float)tvsb / 10;

		// Verschiebung berechnen
		if(fl_tvsb > fl_absenk + 1)				// Sicherstellen, dass das Ergebnis positiv ist
			fl_tvsb = fl_tvsb - fl_absenk;
		else
			fl_tvsb = 1;										// mindestens 1°C
		
		// float to integer
		tvsb = (int)(fl_tvsb * 10);
	}
	return (tvsb);
}	

void Pumpenbus_KE ( char ke )
{
#if BUS_PU_MAX > 0
	char pu = 0;
	
	if ( ke == KE1 )
		pu = PU_BUS_KE1;
	else if ( ke == KE2 )
		pu = PU_BUS_KE2;


	if ( pu > 0 && pu <= BUS_PU_MAX )
	{
		pu -= 1;
		
		if ( BusPuPara[pu].Funktion == 1 )
		{
			if ( 	ked[ke].tvsb > 0 )
			{
				BusPuPara[pu].Betrieb = 1;
// 				BusPuPara[pu].Sollwert = ??? falls nicht konstant
//				ked[ke].punaCtr = kes[ke].Kepuna * 6;	

			}
			else
			{
//				if ( ked[ke].punaCtr > 0 )
//					--ked[ke].punaCtr;
//				if ( ked[ke].punaCtr == 0 )
				if ( ked[ke].puna_cnt == 0 )
				{
					BusPuPara[pu].Betrieb = 0;
//					BusPuPara[pu].Sollwert = 0; falls nicht konstant
				}
			}
		}

#if GENI > 0	
		ked[ke].busPuEin = pu_control[pu].pu_operation;	
#endif

#if WILO > 0	
		ked[ke].busPuEin = modb_data[pu].output_value1_temp;	
#endif

	}
#endif
}
			
				