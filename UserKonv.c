/*	1.User-Konvertierungen für Task: Bedien, zur Ausgabe auf LCD-Anzeige
			Der Aufruf erfolgt aus Konvert.c
			
		2.Veränderung der Ersten Zeile: user_erste_zeile
			Aufruf erfolgt aus Bedien.c , Funktion ZweiZeilen

		3.User-Leittechnikkonvertierungen	zur Abarbeitung von Leittechnikkommnados
			Der Aufruf erfolgt aus LzKonv.c 


		reservierte Konvertierungsnummern: ab 200	(siehe uskonvform.h)

*/

#include <string.h>
#include "struct.h"
#include "ustruct.h"
#include "defins.h"
#include "sramext.h"
#include "uramext.h"
#include "konvform.h"
#include "uskonvform.h"
#include "constext.h"
#include "uconstext.h"

extern void ctoa(char *buf, char ch);
extern void ctoa3(char	*buf,	unsigned char	ch);
extern void asci1(char* Visual_Ptr, char wert, char komma, char signflag);
extern void asci2(char* Visual_Ptr, unsigned int wert, char komma, char signflag);
extern void asci4(char* Visual_Ptr, unsigned long wert, char komma);
extern UINT bin2(char* Visual_Ptr, char komma, char signflag);
//extern const Parli Pgruppe[];
void asci4s ( char* Visual_Ptr, long wert, char komma );
extern void set_vornul(char* Visual_Ptr, char komma, char vorz, char ziffern, char overfl);

// nun in UserConst.c
//-char ClearDisplay[] =
//-	{ "               \0"};

char UndefMessage[] =
	{ "Undef Meldung! \0"};


//----------- ModBus ------------
char undef_exeption[] =
	{ "UNDEF EXEPTION \0"};

char * exeption_set[]	=
	{	
		"KEINE          \0",		// 0
		"INVAL FUNCTION \0",		// 1
		"INVALID ADRESS \0",		// 2
		"INVALID DATA   \0",		// 3
		"EXECUTION ERR 1\0",		// 4
		"EXECUTION ERR 2\0",		// 5
		"EXECUTION ERR 3\0",		// 6
	};
//-------------------------------


//----------- Belimo-Antriebe ------------
//Kommando
char * beli_cmd_set[]	=
	{	
		"0 Kein         \0",
		"1 Adaption     \0",
		"2 Testlauf     \0",
		"3 Synchronisat.\0",
		"4 Reset Errors \0",
	};
//-------------------------------

//Zwangssteuerung
char * beli_force_set[]	=
	{	
		"0 Keine        \0",
		"1 Auf          \0",
		"2 Zu           \0",
		"3 Min          \0",
		"4 Mid          \0",
		"5 Max          \0",
		"6 Schnell_zu   \0",
	};
//-------------------------------

//Sensortyp
char * beli_sensor_set[] =
	{	
		"0 Keiner       \0",
		"1 Aktiv    mV  \0",
		"2 Passiv   1k  \0",
		"3 Passiv 1…20k \0",
		"4 Kontakt  0/1 \0",
	};
//-------------------------------

//Sensortyp
char * beli_typ_set[] =
	{	
		"Nicht Vorhanden\0",
		"Stellantr. L/W \0",
		"Ventilantr. lin\0",
		"Ventilantr. rot\0",
		"Brandschutzklap\0",
		"Volumenstromreg\0",
	};
//-------------------------------

//// Fehler/Störungen
//char * exeption_set[]	=
//	{	
//		"Ausnützung zu gross
//		"Stellweg vergrössert
//		"Mechanische Überlast
//		"-
//		"Sicherheitsrelevante Störung (nur Brandschutz)
//		"Fehler Klappengängigkeit (nur Brandschutz)
//		"Kanaltemperatur zu hoch (nur Brandschutz)
//		"Rauchmelder ausgelöst (nur Brandschutz)
//		"Keine Antwort vom MP Knoten
//		"Checksummen Fehler MP Knoten
//		"Fehlermeldung MP Knoten
//		"Wert ausserhalb Bereich
//		"Funktion nicht verfügbar für Antriebstyp
//		"Zeitüberschreitung (nur Brandschutz)
//	};
////-------------------------------

//--------- WILO-GateWay ---------
char target_exeption[] =
	{ "PUMPEN-TIMEOUT \0"};

char * bus_error_set[]	=
	{	
		"               \0",		// 0
		"OK             \0",		// 1
		"BUS-TIMEOUT    \0",		// 2
		"FUNKTION-ERROR \0",		// 3
		"EXCEPTION      \0",		// 4
		"LENGT-ERROR1   \0",		// 5
		"UNDEF-ERROR    \0",		// 6
		"NO WRITE-ECHO  \0",		// 7 
		"LENGT-ERROR2   \0",		// 8
	};

char * bus_stat_set[]	=
	{	"BUS-TIMEOUT    \0",		// 0
		"BUS OK         \0",		// 1
		"PUMPEN-TIMEOUT \0",		// 2
		"UNGUEL. ADRESSE\0",		// 3
	};
//--------------------------------

// nun in UserConst.c
//---------- Bus-Pumpen ---------
//-char * pu_operation_set[]	=
//-	{	
//-		"PUMPE STOP     \0",		// 0
//-		"PUMPE EIN      \0",		// 1
//-		"MIN-DREHZAHL   \0",		// 2
//-		"MAX-DREHZAHL   \0",		// 3
//-	};
//-
//-char * pu_control_set[]	=
//-	{	
//-		"KONSTANTDRUCK  \0",		// 0
//-		"PROPORT.DRUCK  \0",		// 1
//-		"KONSTANTFREQU. \0",		// 2
//-		"AUTOMATIK-MODE \0",		// 3
//-	};

//
//char * pu_func_set[]	=
//	{	
//		"DEAKTIVIERT!   \0",		// 0
//		"REMOTE MODE    \0",		// 1
//		"LOCAL MODE     \0",		// 2
//	};
//-------------------------------

//#if WILO
//char * wilo_servstat_set[]	=
//	{	
//		"Keine Meldung  \0",
//		"Servicebedarf  \0",				
//		"Lager wechseln \0",				
//		"Lager fetten   \0",				
//		"Dichtung tausch\0",				
//	};
//
//char * wilo_error_set[]	=
//	{	
//		"Keine Meldung  \0",
//		"Modul-Fehler   \0",				
//		"Motor-Fehler   \0",				
//		"Nicht belegt   \0",				
//		"Pumpen-Fehler  \0",				
//		"Versorg.Spannun\0",				
//	};
//
//char * wilo_errorstat_set[]	=
//	{	
//		"Keine Meldung  \0",
//		"Unterspannung  \0",		
//		"Ueberspannung  \0",
//		"Phase fehlt    \0",
//		"Trockenlauf    \0",
//		"Druck zu hoch  \0",
//		"Druck zu niedr.\0",
//		"Nicht belegt   \0",
//		"Nicht belegt   \0",
//		"UeberTemp Motor\0",
//		"Motorfehler    \0",
//		"Blockierung    \0",
//		"UeberTemp Modul\0",
//		"Modulfe.Warnung\0",
//		"Modulfehler    \0",
//		"Sensorfehler   \0",
//	};
//
//char * wilo_pustat_set[]	=
//	{	
//		"Keine Meldung  \0",
//		"Betriebsm.  EIN\0",	
//		"Drehricht.Links\0",	
//		"Soll-Ist A.>10%\0",
//		"Extern Aus     \0",	
//		"Doppelpumpe    \0",	
//		"Lokale Bedien. \0",	
//		"Q/H nicht ermit\0",	
//		"Extern Min.    \0",	
//		"Nicht belegt   \0",
//		"Nicht belegt   \0",
//		"Nicht belegt   \0",
//		"Nicht belegt   \0",
//		"Nicht belegt   \0",
//		"Wink/Service   \0",	
//	};
//
//char * wilo_statdiag_set[]	=
//	{	
//		"Keine Meldung  \0",
//		"Pu o.Modul-Fehl\0",
//		"Versorgung Eo.H\0",
//		"Nicht belegt   \0",
//		"Untere Regelgr.\0",
//		"Obere Regelgr. \0",
//		"Nicht belegt   \0",
//		"Sollwert-Fehler\0",
//		"Nicht belegt   \0",
//		"Lokale Bedien. \0",
//		"Nicht belegt   \0",
//		"Pumpe laeuft   \0",
//	};
////-------------------------------
//#endif	// WILO

//---------- KWB-Kessel ---------
char * icom_prog_set[]	=
	{	
		"ZEITPROGRAMM   \0",		// 0
		"TEMPERATUR     \0",		// 1
		"AUS            \0",		// 2
		"HANDBETRIEB    \0",		// 3
	};

char * icom_ea_set[]	=
	{	
		"AUS            \0",		// 0
		"EIN            \0",		// 1
	};
	
//##### KE-R-2000-BRNEU
// USV
//-char* usv_status[] =
//-	{
//-		"AUS            \0",		// 0
//-		"ZUENDEN WARTEN \0",		// 1
//-		"ZUENDEN        \0",		// 2
//-		"ZUENDNACHLAUF  \0",		// 3
//-		"HOCHFAHREN     \0",		// 4
//-		"EINMESSEN TL   \0",		// 5
//-		"EINMESSEN ML   \0",		// 6
//-		"EINMESSEN NL   \0",		// 7
//-		"EINMESSEN ZU   \0",		// 8
//-		"WARTUNG        \0",		// 9
//-		"BEREIT (+ANF)  \0",		// 10
//-		"TEILLAST       \0",		// 11
//-		"MITTELLAST     \0",		// 12
//-		"NENNLAST       \0",		// 13
//-		"AUFFRISCHUNG   \0",		// 14
//-		"BEREIT (-EXT1) \0",		// 15
//-		"BEREIT (-ANF)  \0",		// 16
//-		"KAMINKEHRER    \0",		// 17
//-	};

// TDS, USV06
char* usv_status[] =
	{
		"AUS            \0",		// 0
		"BEREITSCHAFT   \0",		// 1
		"ZUENDEN WARTEN \0",		// 2
		"ZUENDEN        \0",		// 3
		"BETRIEB        \0",		// 4
		"ABSTELLEN      \0",		// 5
		"STOERUNG       \0",		// 6
	};
//-------------------------------

//--------------------------------------------------------------------------------------
// 1.  User-Konvertierungen für Task: Bedien, zur Ausgabe auf LCD-Anzeige
//--------------------------------------------------------------------------------------
char User_Konv(char* Visual_Ptr, char *wert, char konv, char attr, char rw)
{
	#define KO_LONG		0x20		// ist auch in Konvert_Z definiert

	char i, ret = TRUE;		// Nur bei Konvertierungsfehlern auf FALSE gesetzt 
	MbusDynam *pZAEHLMB;
	MbusStandard *pMbusPara;
	zaehlsp *pZAEHLSP;
	char komma;
	UINT in;
	unsigned  long longwert;
	char ret_komma;
	char charwert;

#if BUS_PU_MAX > 0
	
	switch(konv)
	{

		case BUS_STAT_FORM:
			if(rw ==	KONVRD)
			{
				if ( *wert <= 3 )
					strcpy ( &Visual_Ptr[5],	bus_stat_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}
			break;				

		case CHARBUF_FORM:								// CharBuffer mit Index für Bufferanfang anzeigen und routieren
			if(rw ==	KONVRD)								// Wie HexBuf
			{
				htoa(&Visual_Ptr[1], hexbuf_idx);
				Visual_Ptr[3] = ']';
				
				for(i	=	0; i < 15; i++)		// mit Leerzeichen füllen
					Visual_Ptr[i+5]	=	' ';
				
				for(i = 0;	i	<	4;	i++)
					ctoa3(&Visual_Ptr[i*4 + 5],	wert[i + hexbuf_idx]);	// char to ASCII
			}
			break;
			
		case PU_CTR_FORM:
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 3 )
					strcpy ( &Visual_Ptr[5],	pu_control_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;	
			
		case PU_OPR_FORM:
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 3 )
					strcpy ( &Visual_Ptr[5],	pu_operation_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;		
			
		case PU_FUNC_FORM:
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 2 )
					strcpy ( &Visual_Ptr[5],	pu_func_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;		

	}

#endif


#if ( ((IMPLEMENT_S1 & MODBUS1_IMPL) == MODBUS1_IMPL) || ((IMPLEMENT_S2 & MODBUS1_IMPL) == MODBUS1_IMPL) || ((IMPLEMENT_S3 & MODBUS1_IMPL) == MODBUS1_IMPL) )

	switch ( konv )
	{
		case EXEPTION_FORM:
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 6 )
					strcpy ( &Visual_Ptr[5],	exeption_set[*wert]);
				else if( *wert == 11 )
					strcpy ( &Visual_Ptr[5],	target_exeption );
				else
					strcpy ( &Visual_Ptr[5],	undef_exeption );
			}		
			break;		


		case	BUS_ERROR_FORM:
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 8 )
					strcpy ( &Visual_Ptr[5],	bus_error_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;

		case ICOM_PPROG_FORM:
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 4 )
					strcpy ( &Visual_Ptr[5],	icom_prog_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;		

		case ICOM_EA_FORM:
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 1 )
					strcpy ( &Visual_Ptr[5],	icom_ea_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;

		case USV_STATUS_FORM:			
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 17 )
					strcpy ( &Visual_Ptr[5],	usv_status[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;

//----------- Wilo-Status-Meldungen ------------
#if ( WILO )
		case MODB_STATE_FORM:
			if(rw ==	KONVRD)
			{
			  ctoa(&Visual_Ptr[9], modb_slave_idx);
				Visual_Ptr[11] = '/';
			 	ctoa(&Visual_Ptr[12], modb_state_control);
			}
			break;			

//		case WILO_SERVSTAT_FORM:			
//			if ( rw ==	KONVRD )
//			{
//				if ( *wert <= 4 )
//					strcpy ( &Visual_Ptr[5],	wilo_servstat_set[*wert]);
//				else
//					strcpy ( &Visual_Ptr[5],	UndefMessage );
//			}		
//			break;
//
//		case WILO_ERROR_FORM:			
//			if ( rw ==	KONVRD )
//			{
//				if ( *wert <= 5 )
//					strcpy ( &Visual_Ptr[5],	wilo_error_set[*wert]);
//				else
//					strcpy ( &Visual_Ptr[5],	UndefMessage );
//			}		
//			break;
//
//		case WILO_ERRORSTAT_FORM:			
//			if ( rw ==	KONVRD )
//			{
//				if ( *wert <= 16 )
//					strcpy ( &Visual_Ptr[5],	wilo_errorstat_set[*wert]);
//				else
//					strcpy ( &Visual_Ptr[5],	UndefMessage );
//			}		
//			break;
//
//		case WILO_PUSTAT_FORM:			
//			if ( rw ==	KONVRD )
//			{
//				if ( *wert <= 15 )
//					strcpy ( &Visual_Ptr[5],	wilo_pustat_set[*wert]);
//				else
//					strcpy ( &Visual_Ptr[5],	UndefMessage );
//			}		
//			break;
//
//		case WILO_STATDIAG_FORM:			
//			if ( rw ==	KONVRD )
//			{
//				if ( *wert <= 12 )
//					strcpy ( &Visual_Ptr[5],	wilo_statdiag_set[*wert]);
//				else
//					strcpy ( &Visual_Ptr[5],	UndefMessage );
//			}		
//			break;
//----------------------------------------
#endif	// WILO

//----------- Belimo-Antriebe ------------
		case BELI_CMD_FORM:			
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 4 )
					strcpy ( &Visual_Ptr[5],	beli_cmd_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;

		case BELI_FORCE_FORM:			
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 6 )
					strcpy ( &Visual_Ptr[5],	beli_force_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;
			
		case BELI_SENSOR_FORM:			
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 4 )
					strcpy ( &Visual_Ptr[5],	beli_sensor_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;

		case BELI_TYP_FORM:			
			if ( rw ==	KONVRD )
			{
				if ( *wert <= 5 )
					strcpy ( &Visual_Ptr[5],	beli_typ_set[*wert]);
				else
					strcpy ( &Visual_Ptr[5],	ClearDisplay );
			}		
			break;
	}	
		
#endif


#if ( MBUSANZ > 0 )

	switch ( konv )
	{
		case MBUS_S_LONG:									// speziell für Anzeige mit einer Kommastelle
			if ( rw ==	KONVRD )						// lesen
			{	
				asci4s ( Visual_Ptr, *(long*)wert, attr );
			}
			break;
			
		case MBUS_TYP_FORM:
			pMbusPara = (MbusStandard*)((long*)wert);	
			if ( rw ==	KONVRD )
			{
				asci1 ( Visual_Ptr,	pMbusPara->Typ,	0, SIGNNO ); 
				switch ( pMbusPara->Typ )
				{
					case WRM:	
						strcpy ( &Visual_Ptr[14],	"WAERME");				
						break;									
					case ELT:	
						strcpy ( &Visual_Ptr[14],	"ELT   ");				
						break;									
					case KWS:	
						strcpy ( &Visual_Ptr[14],	"KWZ   ");				
						break;									
					case WWS:	
						strcpy ( &Visual_Ptr[14],	"WWZ   ");				
						break;									
					case KLM:	
						strcpy ( &Visual_Ptr[14],	"KLIMA ");				
						break;	
					case GMB:	
						strcpy ( &Visual_Ptr[14],	"GAS   ");				
						break;	
					default:	
						strcpy ( &Visual_Ptr[14],	"???   ");				
						break;									
				}
			}
			else
			{
				in	=	bin2 ( Visual_Ptr, attr, SIGNNO );
				if ( in > 255)
					ret	=	FALSE;
				else
					pMbusPara->Typ = (char)in;
				break;
			}
			break;
		
		case MBUS_WMENG_FORM:
			if ( rw ==	KONVRD )
			{
				pZAEHLMB = (MbusDynam*)((long*)wert);	
				komma = pZAEHLMB->komma.wmeng;
				if ( komma <= 3 )
				{
					if ( pZAEHLMB->wmeng < 10000000 || komma == 0 )
						asci4 ( Visual_Ptr,	pZAEHLMB->wmeng, komma );
					else																	// falls Komma vorhanden, wird es um eine Stelle verschoben
						asci4 ( Visual_Ptr,	pZAEHLMB->wmeng / 10, (char)( komma - 1 ) );
					memcpy ( &Visual_Ptr[14],	&ze_dim_namen[4].zdtext, 6 );	// Dimension MWh
				}
				else
				{
					komma -= 3;
					asci4 ( Visual_Ptr,	pZAEHLMB->wmeng, komma );
					memcpy ( &Visual_Ptr[14],	&ze_dim_namen[5].zdtext, 6 );	// Dimension kWh					
				}
				ret =  komma | KO_LONG | 0x80;	
			}
			break;
			
		case MBUS_VOL_FORM:
			if ( rw ==	KONVRD )
			{
				pZAEHLMB = (MbusDynam*)((long*)wert);	
				komma = pZAEHLMB->komma.volume;
				if ( pZAEHLMB->volume < 10000000 || komma == 0 )
					asci4 ( Visual_Ptr,	pZAEHLMB->volume, komma );
				else																	// falls Komma vorhanden, wird es um eine Stelle verschoben
					asci4 ( Visual_Ptr,	pZAEHLMB->volume / 10, (char)( komma - 1 ) );
				memcpy ( &Visual_Ptr[14],	&ze_dim_namen[6].zdtext, 6 );		// Dimension cbm
				ret =  komma | KO_LONG | 0x80;
			}	
			break;

		case MBUS_ZWERT_FORM:
			if ( rw ==	KONVRD )
			{
				pZAEHLMB = (MbusDynam*)((long*)wert);
				komma = pZAEHLMB->kommaLT;
				charwert = pZAEHLMB->dim_idx; 
				longwert = basiswert_to_displaywert ( komma, charwert, pZAEHLMB->zwertAkt, &ret_komma);
				asci4 ( Visual_Ptr,	longwert, ret_komma );		
				memcpy ( &Visual_Ptr[14],	&ze_dim_namen[charwert].zdtext, 6 );											
				ret =  komma | KO_LONG | 0x80;
			}
			break;			

		case MBUSMASTER_SYNCVJ_FORM:
			pZAEHLSP = (zaehlsp*)*(long*)wert;	// Extrahieren der Adresse,	auf	die	"wert" zeigt
			komma = pZAEHLSP->zdim.komma;
			if ( rw ==	KONVRD )
			{
				charwert = pZAEHLSP->zdim.dim_idx;														// Dimensionsnummer
				longwert = basiswert_to_displaywert ( komma, charwert, pZAEHLSP->syncw_vj, &ret_komma );
				asci4 ( Visual_Ptr,	longwert, ret_komma );
				memcpy ( &Visual_Ptr[14],	&ze_dim_namen[charwert].zdtext,6 );	// Dimensionstext
				ret =  komma | KO_LONG | 0x80;
			}
			else
			{
				longwert	=	bin4 ( Visual_Ptr, komma, SIGNNO );
				pZAEHLSP->syncw_vj = displaywert_to_basiswert ( longwert, komma, pZAEHLSP->zdim.dim_idx );
				pZAEHLSP->zwert_vj = pZAEHLSP->syncw_vj;
				ret =  komma | 0x80;
			}
			break;			

		case MBUSMASTER_SYNCMON_FORM:
			pZAEHLSP = (zaehlsp*)*(long*)wert;	// Extrahieren der Adresse,	auf	die	"wert" zeigt
			charwert = pZAEHLSP->sync_monat;
			komma = pZAEHLSP->zdim.komma;
			ret = komma;
			if ( rw ==	KONVRD )
			{
				if ( charwert > 0 && charwert < 25 )
				{
					if(charwert < 13)
						longwert = basiswert_to_displaywert(komma, pZAEHLSP->zdim.dim_idx, pZAEHLSP->verbr_akt[charwert-1], &ret_komma);
					else
						longwert = basiswert_to_displaywert(komma, pZAEHLSP->zdim.dim_idx, pZAEHLSP->verbr_vor[charwert-13], &ret_komma);

					asci4(Visual_Ptr,	longwert, ret_komma);
					memcpy(&Visual_Ptr[14],	&ze_dim_namen[pZAEHLSP->zdim.dim_idx].zdtext,6);		// Dimensionstext
					ret =  komma | KO_LONG | 0x80;
				}				
			}
			else
			{
				if ( charwert > 0 && charwert < 25 )
				{
					longwert	=	bin4(Visual_Ptr, komma, SIGNNO);
					if(charwert < 13)
						pZAEHLSP->verbr_akt[charwert-1]  = (unsigned long)displaywert_to_basiswert(longwert, komma, pZAEHLSP->zdim.dim_idx);
					else
						pZAEHLSP->verbr_vor[charwert-13] = (unsigned long)displaywert_to_basiswert(longwert, komma, pZAEHLSP->zdim.dim_idx);
					
					if ( Mon == charwert )
						pZAEHLSP->vwert	= (unsigned long)displaywert_to_basiswert(longwert, komma, pZAEHLSP->zdim.dim_idx);					
					
					ret =  komma | 0x80;
				}				
			}
			break;			
	
	}
	
#endif


	return ( ret );
}

//------------------------------------------------------------------------------------------------------------
// 2.  Veränderung der ersten Zeile: Aufruf aus Bedien.c , Funktion ZweiZeilen
//------------------------------------------------------------------------------------------------------------
unsigned char user_erste_zeile(char*	Visual_Ptr,	char gnum, char	pnum, unsigned char ucExp_num, char alfunc)
{
	unsigned char ucReturn = 0;
	
	ucMax_expander = 0;
	

// Beispiel Fußbodenheizung mit MM-Modul
//	if(strcmp(Pgruppe[gnum].grkz,"FBH:") == 0) {
//		if(pnum > 0) {
//			// in der Parametergruppe FBH soll das ausgewählte Gerät als Nummer im Gruppennamen erscheinen
//			if(ucExp_num == 0) {
//				ucExp_num = ucBedien_FBH_Nummer;
//			}
//			if(Visual_Ptr != NULL) {
//				Visual_Ptr[2] = ucExp_num + 0x30;
//			}
//		}
//		ucMax_expander = 6;
//		ucReturn = ucMax_expander;
//	}
//

// Beispiel Funkempfang
	#if ( ((IMPLEMENT_S1 & FUNK1_IMPL) == FUNK1_IMPL) || ((IMPLEMENT_S2 & FUNK1_IMPL) == FUNK1_IMPL) || ((IMPLEMENT_S3 & FUNK1_IMPL) == FUNK1_IMPL) )
	if(strcmp(Pgruppe[gnum].grkz,"RFS;") == 0)
	{
		if(alfunc == EXP_RFS)
		{	// in der Parametergruppe RFS soll das ausgewählte Gerät als Nummer im Gruppennamen erscheinen
			ctoa(&Visual_Ptr[1],bedien_rf_nummer);
			ucMax_expander = 12;
			ucReturn = ucMax_expander;
		}
	}
	#endif
	
// Änderung neue Genibus-Implementierung
#if USE_GENIBUS
	if ( strcmp ( Pgruppe[gnum].grkz,"GEN;") == 0 )
	{
		if ( alfunc == EXP_GEN )	
		{	// in der Parametergruppe GEN soll das ausgewählte Gerät als Nummer im Gruppennamen erscheinen
			if ( uc_genibus_device_nr == 0 )
				uc_genibus_device_nr = 1;
			else if ( uc_genibus_device_nr > GENI_DEVICE_MAX )
				uc_genibus_device_nr = GENI_DEVICE_MAX;
			ctoa ( &Visual_Ptr[1],uc_genibus_device_nr );
			ucMax_expander = GENI_DEVICE_MAX;
			ucReturn = ucMax_expander;
		}
	}
		
	if ( strcmp ( Pgruppe[gnum].grkz,"GBP:") == 0 )
	{
		if ( alfunc == EXP_GBP )
		{	// in der Parametergruppe GBP soll das ausgewählte Gerät als Nummer im Gruppennamen erscheinen
			if ( uc_genibus_device_nr == 0 )
				uc_genibus_device_nr = 1;
			else if ( uc_genibus_device_nr > BUS_PU_MAX )
				uc_genibus_device_nr = BUS_PU_MAX;
			ctoa ( &Visual_Ptr[1],uc_genibus_device_nr );
			ucMax_expander = BUS_PU_MAX;
			ucReturn = ucMax_expander;
		}
	}	
#endif	

	#if ( WILO )
	if ( strcmp ( Pgruppe[gnum].grkz,"WLP:") == 0 )
	{
		if( alfunc == EXP_WLP)
		{	// in der Parametergruppe WLP soll das ausgewählte Gerät als Nummer im Gruppennamen erscheinen
			ctoa ( &Visual_Ptr[1],SerialDeviceNr );
			if( SerialDeviceNr < 10 )
				Visual_Ptr[1] = 'L';			// verbesserte Darstellung
			ucMax_expander = BUS_PU_MAX;
			ucReturn = ucMax_expander;
		}
	}

	#endif

	#if ( ( IMPLEMENT_S3 & MBUS1_IMPL ) ==  MBUS1_IMPL )
		#if ( MBUSANZ > 0 )
			if(strcmp(Pgruppe[gnum].grkz,"ZMB:") == 0)
			{
				if(alfunc == EXP_ZMB)
				{	// in der Parametergruppe ZMB soll das ausgewählte Gerät als Nummer im Gruppennamen erscheinen
					ctoa(&Visual_Ptr[1], zaehlerNummer);
					ucMax_expander = MBUSANZ;
					ucReturn = ucMax_expander;
				}
			}
		#endif
	#endif													// ( IMPLEMENT_S3 & MBUS1_IMPL ==  MBUS1_IMPL )

	return(ucReturn);
	
}

	
//--------------------------------------------------------------------------------------
// 3.  User-Leittechnikkonvertierungen	zur Abarbeitung von Leittechnikkommnados
//--------------------------------------------------------------------------------------
// Übergabe-Parameter:	Dat_Ptr...Zeiger auf Sende- bzw.Empfangspuffer
//											dae_num...Datenelement-Nummer
//											rw........Lesen(KONVRD) oder Schreiben(KONVWR)
// Return-Parameter:		anz.......Anzahl der gelesenen oder geschriebenen Bytes
//																anz = 0 : Fehler
char UserLzKonv ( char* Dat_Ptr, char* pWert, char konv, char attr, char rw, char exp_num )
{
	
	char  anz = 0;		// Vorbelegung Fehler
	MbusDynam *pZAEHLMB;
	MbusStandard *pMbusPara;

	if ( rw ==	KONVRD )														// lesen
	{
		switch ( konv )
		{
			case MBUS_WMENG_FORM:
//				pZAEHLMB = (MbusDynam*)*((long*)pWert);	
				pZAEHLMB = (MbusDynam*)((long*)pWert);	
				memcpy ( Dat_Ptr, &pZAEHLMB->wmeng, 4 );
				anz = 4;
				break;
		
			case MBUS_VOL_FORM:
//				pZAEHLMB = (MbusDynam*)*((long*)pWert);	
				pZAEHLMB = (MbusDynam*)((long*)pWert);	
				memcpy ( Dat_Ptr, &pZAEHLMB->volume, 4 );
				anz = 4;
				break;

			case MBUS_TYP_FORM:
//				pMbusPara = (MbusStandard*)*((char*)pWert);	
				pMbusPara = (MbusStandard*)((char*)pWert);	
				memcpy ( Dat_Ptr, &pMbusPara->Typ, 1 );
				Dat_Ptr[1] = 0;						// Integer Erweiterung
				anz = 2;
				break;

			case MBUS_S_LONG:
//				pZAEHLMB = (MbusDynam*)((long*)pWert);	
				memcpy ( Dat_Ptr, pWert, 4 );
				anz = 4;
				break;
		}
	}
	
	return ( anz );				

}


//------------------------------------------------------------------------------
// vom System nach User ausgelagerte Konvertierungen
//------------------------------------------------------------------------------

/***** ulsch : Gradtagszahlen *****/
/* Konvertierung 1 Byte (< 100) zu 2 Byte ASCII in Dezimaldarstellung */
/* speziell fuer Expansion von Parametern */
void ctoa_exp ( char *buf, char ch )
{
	*buf++ = '<';	
	strcpy ( buf, &monName[ch][0] );
	*( buf + 3 ) = '>';
}

char HzgGrd_Konvert(char* Visual_Ptr, char *wert, char konv, char attr, char rw)
{
	char	ret	=	TRUE;

#if HEIZGRD == 1
	char charwert;
	UINT intwert;

	switch(konv)
	{
		case EXP_HGT:												// Monat, unsigned char
			if ( rw ==	KONVRD )							// lesen
			{	
				charwert = wert[0];
				asci1 ( Visual_Ptr,	charwert,	attr,	SIGNNO );
			}
			else															// schreiben
			{	
				intwert	=	bin2 ( Visual_Ptr, attr, SIGNNO );
				if ( intwert == 0 || intwert > 12 )
					ret = FALSE;
				else
				{
					wert[0] = (char)intwert;
					HgtAnzeigen ( (char)intwert );
				}					
			}
			break;	
			
		case EXP_HGT_CHAR:									// unsigned	char Format
			if ( rw ==	KONVRD )							// lesen
			{	
				asci1 ( Visual_Ptr,	*wert,	attr,	SIGNNO );
				ctoa_exp ( Visual_Ptr+14,	monHzGrdAnz );				
			}					
			break;
			

		case EXP_HGT_INT:										// unsigned	int	Format
			if ( rw ==	KONVRD )							// lesen
			{
				asci2 ( Visual_Ptr,	*(UINT *)wert, attr, SIGNNO );
				ctoa_exp ( Visual_Ptr+14,	monHzGrdAnz );
			}
			break;
	
	}
#endif	
	return(ret);
	
}

//--------------------------------------------------------------------------------------
// Veränderung der ersten Zeile Parametertext: Aufruf aus Bedien.c , Funktion ZweiZeilen
//--------------------------------------------------------------------------------------
void usertext_erste_zeile(char*	Visual_Ptr,	char gnum, char	pnum)
{
	char idx;
	char text[16] = {0};
	
	if(strcmp(Pgruppe[gnum].grkz,"ANL:") == 0)
	{
		if(strcmp(Pgruppe[gnum].Param[pnum].num, "*97:") == 0) // ***AnFre 30.05.2013 'EXTERNE SSM  '
		{
			strcpy(text, extAlarmText);
			if(text[0] != 0)
				strcpy(&Visual_Ptr[5],  text);
		}
	}		
	if(strcmp(Pgruppe[gnum].grkz,"QSM:") == 0)
	{
		if(pnum == 10) // ***AnFre 30.05.2013 'EXTERNE SSM  '
		{
			strcpy(text, extAlarmText);
			if(text[0] != 0)
				strcpy(&Visual_Ptr[5],  text);
		}
	}		
}


void asci4s ( char* Visual_Ptr, long wert, char komma )
{
	char *buf  = &Visual_Ptr[6];
	char ziffer, vorz, i;
	long subwert = 1000000;
	long maxwert = 10000000;
	char stellen = 6;
	char overfl = 0;


	vorz = ' ';
	/* Test auf negativen Wert */
	if ( wert & 0x80000000 )					// negativ
	{	
		wert = ( 0xFFFFFFFF - wert ) + 1;
		vorz = '-';
		if ( wert > 999999 )
		{
			wert = 999999;								// größte darstellbare neg. Zahl
			overfl = 1; 
		}
	}
	else
	{
		if ( wert > 9999999 )
		{
			wert = 9999999;								// größte darstellbare pos. Zahl
			overfl = 1; 
		}		
	}

	for ( i = 0;	i	<	stellen; i++ ) 
	{	
		ziffer =	0;
		while ( wert >= subwert )
		{	
			wert	-=	subwert;
			ziffer +=	1;
		};
		*buf++	=	(char)( ziffer + 0x30 );
		subwert = subwert / 10;
	}
	*buf++	=	(char)( (wert	%	10)	+	0x30 );

	set_vornul ( Visual_Ptr,	komma, vorz, 7, overfl );

}

		