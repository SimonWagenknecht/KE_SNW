/*            Strukturen und Funktionsprototypen im User-Bereich     */
/*            ustruct.h 				                                     */
#ifndef USTRUCT_H_INCLUDED
#define USTRUCT_H_INCLUDED

#include "projdef.h"
#include "userdef.h"

// Pumpenbus

typedef struct {
	char	Funktion; 			// deaktiviert (0), remote (1), local (2)
	char	Name[16];				// Beispiele: "PUMPE HK1       ", "ZIRK.-PUMPE     "
	char	Adresse;				// frei wählbar, Standard = Nummer
	char	Betrieb;				// Betriebsart bei Automatikbetrieb, Stop (0), Ein (1), Minimal-Kurve (2), Maximal-Kurve (3)
	char	Regelart;				// Konstantdruck (0), Proportionaldruck (1), Konstantfrequenz (2), Automatik (3)
	UINT	Sollwert;				// Standardwert, wenn Pumpe dauernd und konstant läuft, sonst anwendungsspezifisch überschreiben [%] / 10	
												// ( z.B. in Steuer.c für Heizungspumpe )			
	char	RMverz;					// Verzögerung Rückmeldung [min]
	char	Hand;						// Hand/Automatik, Hand = 1
	char	BetriebHand;		// Betriebsart bei Handbetrieb, Stop (0), Ein (1), Minimal-Kurve (2), Maximal-Kurve (3)
	UINT	SollHand;				// [%] / 10
	char	RegelartHand;		// Konstantdruck (0), Proportionaldruck (1), Konstantfrequenz (2), Automatik (3)
	char	AlarmQuit;
	char	HoldingReg300;	// Manual (5), siehe Modbus_RS485_spec_V_1_09_en.pdf Abschnitt 5.1.8 Table 5.4
} PuBusStandard;
#define BUS_PU_LENG sizeof(PuBusStandard)

typedef struct {
	char	puAlarm;				// Zustand der roten LED der Pumpe
	char	busSm;
	char	betriebSm;	
	UINT	betriebRmCtr;

	char betriebsArtSoll[16];	// ASCII	
	char betriebsArtIst[16];	// ASCII	
	char regelArtSoll[16];		// ASCII	
	char regelArtIst[16];			// ASCII	
	UINT sollwert;
} PuBusDynam;

// MBUS

typedef struct {
	char	Freigabe;
	char	Folge;					// Folge vordefinierter Telegramme bei jeder Abfrage
	char	Typ;						// Zähler-Typ
	char	Adr;						// MBus-Adresse
} MbusStandard;
#define MBUSLENG sizeof(MbusStandard)

typedef struct {
	char wrm;							// Wärmezähler
	char wrm_h2;					// Wärmezähler
	char was;							// Wasserzählerzähler
	char elt;							// Elektrozähler
	char klm;							// Klimazähler (Wärme/Kälte)
} Sichtbarkeit;


typedef struct {
	char	errors;
	ULONG	wmeng;
	ULONG	volume;
	ULONG	elektro;
	int		tVorlauf;
	int		tRuecklauf;
	int		tDifferenz;
	ULONG	volStrom;
	ULONG	leistung;
	ULONG	kaelte;
	struct {
			char wmeng 	: 4;
			char volume : 4;
		} komma;
	ULONG long zwertAkt;	// letzter erfasster Zählwert
	char	kommaLT;
	char	dim_idx;
} MbusDynam;

/*--------------------------------------------------------------------*/
// Modbus Allgemein - joja
/*--------------------------------------------------------------------*/
#if MODBUS_UNI > 0
	#include "Modbus/modbusstruct.h"
#endif	

//---------------------------------------------------------------------

// Struktur für Skalierung von Analogausgängen
typedef struct {
	int	NullP;						// bei   0%: in Volt [V] * 100  
	int HundP;						// bei 100%: in Volt [V] * 100
}aask;	

// Struktur für externes Erweiterungsmodul
typedef struct iob{
	char iodev;						// Gerätekennzeichen								(R37, R38, ...)
	char ionum;						// Gerätenummer											(1..4)
	char iobel;						// Nummer des IO-Belegungssatzes		(1..255)
}ioblock;

typedef struct {
	char min;
	char std;
} uhrzeit;

//#####ulsch: skalierbare Analogeingänge
typedef struct {
	int		Skal0;										// Skalierung des Spannungs-Messwerts  0 V	
	int		Skal10;										// Skalierung des Spannungs-Messwerts  10,00 V		
	UINT	ZkFilt;										// Glaettungszeitkonstante der AE [s]	( Sprungantwort: 66 % nach ZK )
	int		UGW;											// Grenzwert unten (Dimension wie Skalierung), Null: kein Grenzwert !
	int		OGW;											// Grenzwert oben (Dimension wie Skalierung), Null: kein Grenzwert !
	int		Hyst;											// Hysterese der Grenzwerte (Dimension wie Skalierung)
} sAnaInpPara;

typedef struct {
	UINT	mwFilt;										// geglaetteter Messwert							[%] * 10
	float	fl_mwFilt;								// geglaetteter Messwert		
	mwsp	mwSkal;										// skalierter und geglaetteter Messwert
	char ugwSM;											// SM UGW
	char ogwSM;											// SM OGW
} sAnaInp;

// ***AnFre Zeitsruktur
typedef struct zeit {
	char min;
	char std;
} zeit;

typedef struct datum {
	char jahr;
	char monat;
	char tag;
} datum;

/*--------------------------------------------------------------------*/
/* Struktur von RIEcon36 - Heizungsanlagen														*/
/*--------------------------------------------------------------------*/
typedef struct anl{			
	// Typ und Version
	char Typ[15+1];				// 15 ASCII-Zeichen und \0
	char Vers[3];					// Jahr Monat Tag (hex)
	// Kategoriebezeichnung für variable Projekte
	char KatVar[15+1];		// 15 ASCII-Zeichen und \0
	
	// Steuercode für Heizkreise 
	char hkcode[HKMAX];		// Nummer des Kaltstartparametersatzes für jeden Heizkreis			
	char abscode[HKMAX];	// Nummer des Absenkprofils für jeden Heizkreis	
	
	// Steuercode für Solare-Heizkreise 
	char hkSoLcode[HKSoLMAX];		// Nummer des Kaltstartparametersatzes für jeden Heizkreis			
	
	// Steuercode für Warmwasserkreise 
	char wwcode[WWMAX];		// Nummer des Kaltstartparametersatzes für jeden Warmwasserkreis	

	// Steuercode für Warmwasser-Temp-Profile 
	char prcode[PRMAX];		// Nummer des Kaltstartparametersatzes für jedes	Profil

	// Steuercode für Kesselkreise 
	char kecode[KEMAX];		// Nummer des Kaltstartparametersatzes für jeden Kesselkreis

	// Steuercode für Fernwärmevorregelkreis (Netztrennumg)
	char ntcode[NTMAX];		// Nummern der Kaltstartparametersätze für jeden Netztr.kreis [NT_PROFILE]

	// Steuercode für Solarregelung
	char socode[SOMAX];	// Nummern der Kaltstartparametersätze für jeden Solar-Kreis [SO_PROFILE]

	// Steuercode für MBUS
	char mbcode[MBUSANZ > 0 ? MBUSANZ : 1 ];	// Nummern der Kaltstartparametersätze für jeden MBUS-Zähler [MB_PROFILE]

	// Namen für Prozess Ein- und Ausgänge (Namensvereinbarung in "userdef.h")
	// Aus den Namen werden Steuercodes generiert
	unsigned int in1;			// 				Eingang IN1		Pt1000 / ----- / -------	
	unsigned int in2;			// 				Eingang IN2		Pt1000 / ----- / -------	
	unsigned int in3;			// Block  Eingang IN3		Pt1000 / 0-10V / Digital
	unsigned int in4;			// Block  Eingang IN4		Pt1000 / 0-10V / Digital
	unsigned int in5;			// Block  Eingang IN5		Pt1000 / 0-10V / Digital
	unsigned int in6;			// 				Eingang IN6		Pt1000 / 0-10V / Digital
	unsigned int in7;			// 				Eingang IN7		Pt1000 / 0-10V / Digital
	unsigned int in8;			// 				Eingang IN8		Pt1000 / 0-10V / Digital
	unsigned int in9;			// 				Eingang IN9		------ / 0-10V / Digital
	unsigned int in10;		// 				Eingang IN10	------ / 0-10V / Digital

	unsigned int aa1;			// Analog-Ausgang  AA1
	unsigned int aa2;			// Analog-Ausgang  AA2

	unsigned int da1;			// Digital-Ausgang DA1
	unsigned int da2;			// Digital-Ausgang DA2
	unsigned int da3;			// Digital-Ausgang DA3
	unsigned int da4;			// Digital-Ausgang DA4
	unsigned int da5;			// Digital-Ausgang DA5
	unsigned int da6;			// Digital-Ausgang DA6
	
	// Externe Erweiterungen mit R37, R38...
	ioblock	 iobl[4];			// maximal 4 Erweiterungsmodule
	
	// NAMUR-Kennzeichnung für Digitale Eingänge R36
	char namur;						// Bit=1...NAMUR-Eingang,	BIT0...7 = IN3...IN10

}Anlage;
	

/*--------------------------------------------------------------------*/
/* 				Struktur R37  IN/OUT-Modul 																	*/
/*--------------------------------------------------------------------*/
	// Namen für Prozess Ein- und Ausgänge (Namensvereinbarung in "userdef.h")
	// Aus den Namen werden Steuercodes generiert
typedef struct io37{			
	unsigned int xde1;		// Digital-Eingang DE1 
	unsigned int xde2;		// Digital-Eingang DE1 
	unsigned int xde3;		// Digital-Eingang DE1 
	unsigned int xde4;		// Digital-Eingang DE1 
	unsigned int xde5;		// Digital-Eingang DE1 
	unsigned int xde6;		// Digital-Eingang DE1
	
	unsigned int xaa1;		// Analog-Ausgang  AA1
	unsigned int xaa2;		// Analog-Ausgang  AA2
	unsigned int xaa3;		// Analog-Ausgang  AA3
	unsigned int xaa4;		// Analog-Ausgang  AA4

	unsigned int xda1;		// Digital-Ausgang DA1
	unsigned int xda2;		// Digital-Ausgang DA2
	unsigned int xda3;		// Digital-Ausgang DA3
}Anl37;
	
// Struktur des RAM-Bereiches für ein R37  IN/OUT-Modul
typedef struct ra37{
	// Datenübertragung
	char					bus;				// Busverbindung: 0=XBUS 1=GBUS
	char					adr;				// DeviceAdresse bzw. Moduladresse
	char					errcnt;			// Fehlerzähler Datenübertragung
	char					error;			// Anzeige des Datenübertragungsfehlers
	// Skalierungen
	aask					AASkal[4];	// Grundwerte (EEPROM gestützt, Voreinstellung in UserReply.c bei Kaltstart 66)	
		
	// Datue senden an R37
	char					Digout;			// 1 Digital-Ausgabebyte
	char					anout[4];		// 4 Analog-Ausgabebyte  0-255 Digit
	char					znum;				// aktueller Zähler
	char					zaufnum;		// Auftragsnummer für Zählerinformationen lesen
	
	// Datue empfangen vom R37
	char					Diginp;			// Digitaleingabe
	char 					sw_stat[3];	// Status für jeden Schalter 
	zaehlsp				zaehler[6];	// Zaehlerspeicher, wird stetig gefüllt
	
	// Arbeitsspeicher des Auftraggebers
	dinpsp				di_mewe[6];	// Digitaleingabe, extrahiert aus Diginp
	aasp					Aout[4];		// Analogausgabe:  0 - 100,0% vom Auftraggeber
	unsigned int	AoutSk[4];	// aktuelle skalierte Ausgabewerte; Umrechnung Aout[i]->awert nach AoutSk[i] in Output.c
	doutsp 				Dout[3];		// Digitalausgabe: (char)Ausgabe + (char)Funktion
	char					wpause[3];	// Wechselpause bei verriegelten Ausgängen
	char					dadat;			// vom R50 R3X_DADAT-Kommando
	char					aadat[4];		// vom R50 R3X_AADAT-Kommando
	char 		sw_stat_info[16];	// Info über Handschalter in ASCCI
	
}Ram37;	


/*--------------------------------------------------------------------*/
/* 				Struktur R38  PT1000-Modul 																	*/
/*--------------------------------------------------------------------*/
	// Namen für Prozess Ein- und Ausgänge (Namensvereinbarung in "userdef.h")
	// Aus den Namen werden Steuercodes generiert
typedef struct io38{			
	unsigned int xae1;		// Pt1000-Eingang AE1 
	unsigned int xae2;		// Pt1000-Eingang AE2 
	unsigned int xae3;		// Pt1000-Eingang AE3 
	unsigned int xae4;		// Pt1000-Eingang AE4 
	unsigned int xae5;		// Pt1000-Eingang AE5 
	unsigned int xae6;		// Pt1000-Eingang AE6 
	unsigned int xae7;		// Pt1000-Eingang AE7 
	unsigned int xae8;		// Pt1000-Eingang AE8 
	unsigned int xae9;		// Pt1000-Eingang AE9 
	unsigned int xae10;		// Pt1000-Eingang AE10 
	unsigned int xae11;		// Pt1000-Eingang AE11 
	unsigned int xae12;		// Pt1000-Eingang AE12 
	unsigned int xae13;		// Pt1000-Eingang AE13 
	unsigned int xae14;		// Pt1000-Eingang AE14 
}Anl38;

// Struktur des RAM-Bereiches für ein R38  PT1000-Modul
typedef struct ra38{
	// Datenübertragung
	char					bus;				// Busverbindung: 0=XBUS 1=GBUS
	char					adr;				// DeviceAdresse bzw. Moduladresse
	char					errcnt;			// Fehlerzähler Datenübertragung
	char					error;			// Anzeige des Datenübertragungsfehlers
	
	// Datue senden an R38
	unsigned int inpUsed;
	
	// Datue empfangen vom R38
	mwsp pt_mewe[14];					// (int)Messwert[0,1°C]	+	(char)Status 
	
}Ram38;

/*--------------------------------------------------------------------*/
/* 				Struktur R39  INP-Modul		 																	*/
/*--------------------------------------------------------------------*/
	// Namen für Prozess Ein- und Ausgänge (Namensvereinbarung in "userdef.h")
	// Aus den Namen werden Steuercodes generiert
typedef struct io39{			
	unsigned int xde1;		// Eingang DE1 
	unsigned int xde2;		// Eingang DE2 
	unsigned int xde3;		// Eingang DE3 
	unsigned int xde4;		// Eingang DE4 
	unsigned int xde5;		// Eingang DE5 
	unsigned int xde6;		// Eingang DE6 
	unsigned int xde7;		// Eingang DE7 
	unsigned int xde8;		// Eingang DE8 
	unsigned int xde9;		// Eingang DE9 
	unsigned int xde10;		// Eingang DE10 
	unsigned int xde11;		// Eingang DE11 
	unsigned int xde12;		// Eingang DE12
	// Signalart: 0=Schließer 1=Öffner  Bit0=DE1
	unsigned int art;

}Anl39;

// Struktur des RAM-Bereiches für ein R39  INP-Modul
typedef struct ra39{
	// Datenübertragung
	char					bus;				// Busverbindung: 0=XBUS 1=GBUS
	char					adr;				// DeviceAdresse bzw. Moduladresse
	char					errcnt;			// Fehlerzähler Datenübertragung
	char					error;			// Anzeige des Datenübertragungsfehlers
	
	// Datue senden an R39
	unsigned int Digart;			// Maske für Kontaktart: 0=Schließer 1=Öffner
	
	// Datue empfangen vom R39
	unsigned int Diginp2;			// Bit0=DE1....Bit11=DE12
	char				 Derror;			// Fehler Kontaktspannung UK24
	
	// Arbeitsspeicher des Auftraggebers
	dinpsp			 di_mewe[12];	// Digitaleingabe, extrahiert aus Diginp2

}Ram39;

/*--------------------------------------------------------------------*/
/* 				Struktur R33  OUT-Modul		 																	*/
/*--------------------------------------------------------------------*/
	// Namen für Prozess Ein- und Ausgänge (Namensvereinbarung in "userdef.h")
	// Aus den Namen werden Steuercodes generiert
typedef struct io33{			
	unsigned int xda1;		// Digital-Ausgang DA1
	unsigned int xda2;		// Digital-Ausgang DA2
	unsigned int xda3;		// Digital-Ausgang DA3
	unsigned int xda4;		// Digital-Ausgang DA4
	unsigned int xda5;		// Digital-Ausgang DA5
	unsigned int xda6;		// Digital-Ausgang DA6
}Anl33;
	
// Struktur des RAM-Bereiches für ein R33  OUT-Modul
typedef struct ra33{
	// Datenübertragung
	char					bus;				// Busverbindung: 0=XBUS 1=GBUS
	char					adr;				// DeviceAdresse bzw. Moduladresse
	char					errcnt;			// Fehlerzähler Datenübertragung
	char					error;			// Anzeige des Datenübertragungsfehlers
		
	// Datue senden an R33
	char					Digout;			// 1 Digital-Ausgabebyte
	
	// Datue empfangen vom R33
	char 					sw_stat[6];	// Status für jeden Schalter 
	
	// Arbeitsspeicher des Auftraggebers
	doutsp 				Dout[6];		// Digitalausgabe: (char)Ausgabe + (char)Funktion
	char					wpause[6];	// Wechselpause bei verriegelten Ausgängen
	char					dadat;			// vom R50 R3X_DADAT-Kommando
	char 		sw_stat_info[16];	// Info über Handschalter in ASCCI
}Ram33;	

/*--------------------------------------------------------------------*/
/*				Regelparameter 																							*/
/*--------------------------------------------------------------------*/
	
/*--------------------------------------------------------------------*/

/* Struktur der Regelparameter eines Heizkreises										*/
// Teil1: Statische Parameter (mit Standdardwerten)
typedef struct rs{
	int		Ts;								// Tastzeit (>= 1 s)						[s] * 10
	UINT	Tn;								// Nachstellzeit								[s] * 10
	int		Kp;								// P-Verstärkung							[%/K] * 100
	int		Kd;								// D-Verstärkung							[%/K] * 100
	int		Kpk;							// P-Verstärkung Kaskade			[%/K] * 100
	int		Kpr;							// P-Verstärkung TRS-Begrenzg.[%/K] * 100
	int		Anst;							// Steigung									 		[1] * 100	
	int		Tvpa;							// Niveau											 [°C] * 10		
	int		Tvma;							// max. Vorlauftemp.					 [°C] * 10		
	int		Tvmi;							// min. Vorlauftemp.					 [°C] * 10		
	int		Trma;							// max. Rücklauftemp.					 [°C] * 10		
	int		Taha;							// Temp. am Heizkurvenanfang	 [°C] * 10	
	int		Tiso;							// Raumsolltemp. für Kaskade	 [°C] * 10		
	int		Timi;							// Mindest-Raumtemp.					 [°C] * 10	
	int		Tsolu;						// Start-Sollwert Heizung			 [°C] * 10	
	int		Beabs;						// Bedarfs-Absenk.keine Nutzung [K] * 10	
	int		Fzk;							// Filterzk. f. tsol						[s] * 10
	char	Hub;							// Ventilhub									 [mm]
	char	Vst;							// Stellgeschwindigkeit		 [mm/min]
	char	Haut;							// HAND/AUTO-Umschaltung
	char	Puea;							// Hand: Pumpe	ein/aus
	int		PuDzHand;					// Hand: Pumpen-Drehzahl							[%] * 10		***AnFre 06.06.2012
	int		Vstell;						// Hand: Ventil stellen 0-10V Ventil	[%] * 10
	char	Puna;							// Pumpennachlaufzeit										[min]	
	char	Swhk;							// Schalter Sommer/Winter-Automatik 		ja/nein
	char	Wwvh;							// Schalter WW-Vorrang 									ja/nein
	char	Sauf;							// Schalter Schnellaufheizung 					ja/nein
	char	Psbegr;						// Schalter Primär/Sekundär Rückl.begr.	Prim=1 Sek=0
	int	  Kpww;							// P-Verstärkung für gleitenden Warmwasservorrang [%/K] * 100 (in Wwsteu behandelt)
	char	TabwMax;					// maximale Regelabweichung zur Bildung eines Alarms [K] * 10
	char	TabwZeit;					// Zeitdauer der maximalen Regelabweichung bis zum Alarm [min]
	char	FerienAutom;			// Schalter
	UINT	FerienAbsenk;			// Absenkwert
	UINT	SchalterAbsenk;		// Anlagenschalter-Absenk. keine Nutzung [K] * 10	
	UINT	Y_rel_min;				// Minimale Stellausgabe an 0-10V Ventil	[%] * 10
	UINT	Y_rel_beg;				// Öffnungsbeginn des 0-10V Ventils				[%] * 10

	int		TaFrostgrenze;		// Ta-FrostGrenze bei Absenkung				 	[°C] * 10		
	int		TaHeizgrenze;			// Ta-HeizGrenze beim Heizbetrieb			 	[°C] * 10		
	int		AbschZeitKonst;		// Zur Berechnung der Abschaltdauer in der Absenkzeit		[h/K] * 10
	char 	SoWiExtra;				// Schalter Auswahl Sommer/Winter Extrawerte ja/nein 
	int		Tae;							// Temp. Winter ein						 					[°C] * 10	
	int		Taa;							// Temp. Sommer aus						 					[°C] * 10	

	char	MessBeginn;				// Beginn der Adaption, Stunde
	char	AdaptWert;				// [K] * 10
	int		EiMaxAdapt;				// max. Regelabweichung, bei der Adaptionswert berechnet wird [K] * 10

	char 	FuehrPu;					// Führunspumpe
	UINT 	PuWechseln;				// laufzeitabh. Umschalten der Führungspumpe  [h]
	char 	FuehrPuFest;			// Führungspumpe fest vorgegeben, wenn ungestoert	(1,2); 3=beide Pu laufen gleichzeitig, 0=laufzeitabh.
	char	KaskMode;					// Modus der Raum-Kaskade bei gleichzeitiger Adaption : 0 -> nur anhebend und bei Unterversorgung, 1 -> immer
	int		TvpAnh;						// Anhebung der Anforderung an den Wärmeerzeuger		[K] * 10		
// ***AnFre 04.12.2008
	char	ExtAnf1_Ja;			// Externe Anforderung für diesen HK gültig
	char	ExtAnf2_Ja;			// Externe Anforderung für diesen HK gültig
	int		TvAnhExt;				// Anhebung externe Anforderung
	// Estrich-Trocknung
	datum	StartAufheiz;
	char	EstrichTemp[ESTR_STUFEN];	
	int		TimiHyst;					// Hysterese Raum-Mindesttemp. [K] * 10
	UINT	XpLeistBegr;			// [K/% * 100]
	int		KaskBegr;					// Begrenzung des Raum-Sollwerts für Kaskade	[°C] * 10
//#####090119													
	char	PuMin;						// Hand: Pumpe auf Mindestdrehzahl 													
													// 119 Byte Parameter !!!
// Pumpenbus
	UINT	BusPuSollwert;					//HKN: Sollwert NetzPumpe		0...100,0 %
	UINT	BusPuSollwertAbsenk;		//HKN: Sollwert NetzPumpe		0...100,0 %

// Heizkreis abschalten mangels Bedarf 
// Bedingungen: Funktion freigegeben, Raum ext Ist > Soll sowie (wenn parametriert) offene Ventile <= VentOffenMax
	char	AbschRaumanf;		// Funktion freigegeben ?
	char	VentilAuswert;	// Ventilstellung berücksichtigen ? (sonst nur Raumtemperatur)
	char	VentOffenMax;		// HK abschalten, wenn offene Ventile (%) kleiner VentOffenMax
// ***AnFre 16.11.2010
	int  Tvsol10;		// Solltemperatur bei 10V					[°C] * 10
	int  Tvsol0;		// Solltemperatur bei 0V					[°C] * 10
	UINT Sollstell;	// HAND Sollwert stellen	Volt *100
	int 	VorlaufMaxSommer;	// wenn VL > VorlaufMaxSommer, wird ucHeat = 1 trotz Sommer (R50 öffnet dann nicht die Heizungs-Ventile)
	char	VorrangZeitabsenkung;
// ***AnFre 06.06.2012 Legionelle für NahwärmeNetz von KER1s02_SEZ01
	char Legjn;							// Legionellentod HK1-Temp.-Anhebung
	char Legtag;						// Legionellentage
	char Legzeit[2];				// Legionellen Einschaltzeit			[min][std]
	char Legdau;						// Legionellen Einschaltdauer			[std]
	int  Twwleg;						// Legionellen Temperatur
// ***AnFre 06.06.2012 solare Anhebung NahwärmeNetz
	int	SolAnhebung;		// Anhebung Netz-Sollwert [ K ] *10
	int	SolAnhebTH3;		// solare Anhebung wenn TH3 > [°C ] *10
	int	SolAnhebTS5;		// 						 und wenn TS5 > [°C ] *10
// ***AnFre 06.06.2012 Speicher-Ladung
	char WwVorrang;			// TS4 statt TS3 im Speicher für Einschalten		
	int DTLadEin;			// dT Ladung einschalten

	int	PuDzMax;			//HK1:	Ladepumpen-Drehzahl	Max
	int	PuDzMin;			//HK1:	Ladepumpen-Drehzahl	Min
//	int	PuDzTaMin;		//HK1:	Ladepumpen-Drehzahl TaMin	[°C ] *10			Funktion entfernt 06.01.2017
//	int	PuDzTaMax;		//HK1:	Ladepumpen-Drehzahl TaMax [°C ] *10			Funktion entfernt 06.01.2017
	int	DTPuDzMin;		// Umschalten auf Drehzahl min wenn Tist (TH1) + DTPuDzMin < Tsoll
	int	PuDzSteig;		// Drehzahl Steigerung Bzw. Absenkung alle 30s
	int DTVorEin;			// dT WW-Vorrang einschalten (in ANL:)
	int VorZeitMax;		// extern Vorrang-Zeit maximal		  (in ANL:)
	int	RLKorrMin;		//HKN: RL-Korrektur zur Berechnung dynamische Ventilstellung Min 
	UINT VentMaxSolar;//HKL:	RV-Begrenzung bei Solares Heizen (hkSolJa = Ja)
// ***AnFre 03.03.2015 Absenkung HKL bei "Solares Heizen Ein"
//	UINT AbsenkSol;		//HKL:	Absenkung bei Solares Heizen (hkSolJa = Ja)		Funktion entfernt 06.01.2017
//	UINT AbsenkFaktor;//HKL: Absenk-Faktor (TS3 - SollHKL) * Faktor [ 0,00]	Funktion entfernt 06.01.2017
}HkStandard;
#define HKSLENG sizeof(struct rs)

/* Struktur der Regelparameter eines Heizkreises										*/
// Teil2: Dynamische Parameter	
typedef struct rd{
	int		zts;					// Zaehler fuer Tastzeit
	char	regstart;			// Reglerstart nach Reset
	int		ei;						// Regelabweichung							[K] * 10
	float	fl_ei1;				// Regelabweichung zum Tastzeitpunkt i-1
	float	fl_ei2;				// Regelabweichung zum Tastzeitpunkt i-2
	int		tsol;					// Vorlaufsolltemp. , nach der geregelt wird
	float fl_tsol;			//		"								(für Filterberechnung)
	int		tvsb;					// berechnete Vorlaufsolltemp. total
	int		tvsb_hkl;			// berechnete Vorlaufsolltemp. nach Heizkennlinie (für debug)							
	int		stellzeit;		// berechnete Zeit für Stellausgabe
	int		stellsum;			// Summenzähler für Stellausgabe
	int 	fahren;				// Fahrzeit Stellausgabe in ganzen Sekunden
	int		y_rel;				// Stellausgabe an 0-10V Ventil	[%] * 10
	float	fl_y_rel;			//		"		
	int		dy_rel;				// für debug
	UINT	zpu;					// Zähler für Pumpenachlaufzeit
	char	vzu;					// Hand Ventil zufahren			[s]
	char	vauf;					// Hand Ventil auffahren		[s]
	char	sowi;					// Ermittelter Zustand: 1 = Sommerbetrieb
	char	somme;				// Statusanzeige Sommerbetrieb
	char	heiz_grenz;		// Merker für Außentemperatur-Heizgrenze
	char	vorra;				// Statusanzeige Warmwasservorrang
	char	rlbegr;				// Statusanzeige Rücklaufbegrenzung
	char	vorra_bed;		// Zustand  1=Bedarfsabsenkung hat Vorrang vor Zeitabsenkung	
	char	bedarf;				// Statusanzeige Heizungsbedarfsanforderung (vom RIEcon50) 1=JA 2=NEIN 0=UNBEKANNT
	char	vorheiz_bed;	// Heizungsbedarf nur durch Räume in Vorheizung
	int		tisobed;			// RaumSolltemp. Bedarf 			 (vom RIEcon50)
	int		tibed;				// RaumIsttemp.  Bedarf 			 (vom RIEcon50)
	char	ticopy;				// kopiert TI nach tibed, wenn tibed = 0 (Dummy-Raum)
	char 	CountDown;		// Kommandoüberwachung	
	UINT  abwCtr;				// (Sekunden)-Counter der Regelabweichung
	int		kaskaKorr;		// aktueller temperaturabh. Kaskade-Korrekturwert zur berechn.Vorlaufsolltemp. nach Heizkennlinie tvsb_hkl	
	int		adaptKorr;		// aktueller temperaturabh. Adaptions-Korrekturwert zur berechn.Vorlaufsolltemp. nach Heizkennlinie tvsb_hkl	
	int		rlKorr;				// aktueller temperaturabh. Rücklauf-Korrekturwert zur berechn.Vorlaufsolltemp. nach Heizkennlinie tvsb_hkl	
	int	  wwvorKorr;		// Absenkwert für gleitenden Warmwasser-Vorrang
	
	UINT puLz;					// Pumpen-Laufzeit [h]	
	UINT pudLz;					// Laufzeit [h] 2.Pumpe	bei Doppelpumpe
	char pu1min;				// Minutenzähler Laufzeit
	char pu2min;
	
	char raumname[16];

	// Ferienabsenkung
	UINT	tvabFerien;			// Absenkwert bei Ferien, immer 0 wenn keine Ferien	
	char	absenFerien;		// Zustand  Absenkphase 				
	char	abschFerien;		// Zustand 	Abschaltphase
	char	stuetzFerien;		// Zustand 	Stützbetrieb 
	char	frostFerien;		// Zustand	Frostschutzbetrieb 
	// Schalterabsenkung
	UINT	tvabSchalter;		// Absenkwert bei Schalter, immer 0 wenn keine Schalter	
	char	absenSchalter;	// Zustand  Absenkphase 				
	char	abschSchalter;	// Zustand 	Abschaltphase
	char	stuetzSchalter;	// Zustand 	Stützbetrieb 
	char	frostSchalter;	// Zustand	Frostschutzbetrieb 

	// Gemeinsame Anzeige von Parametern aus verschiedenen Quellen:
	// Schalterabsenkung oder Bedarfsabsenkung oder Ferienabsenkung oder Zeitabsenkung 
	char	regie;					// Absenk-Regie aktuell
	char	regie_txt[16];	// Textanzeige Absenk-Regie
	char	absen;					// Zustand  Absenkphase
	char	aufhe;					// Zustand  Aufheizphase
	char	absch;					// Zustand	Abschaltphase
	char	hunt;						// Zustand  Heizunterbrechung
	char	stuetz;					// Zustand 	Stützbetrieb
	char	frost;					// Zustand	Frostschutzbetrieb
	
	// Softalarme
	char	abwAlarm;			// Regelabweichung ueberschreitet fuer eine definierte Zeit einen vorgegebenen Betrag
	char  puBm;					// Softalarm für 'Betriebsmeldung Heizungspumpe' kommt nicht
	char  puBmVerz;			// Verzögerung Softalarm für 'Betriebsmeldung Heizungspumpe' [30s]

	// Estrich-Trocknung
	char	estrichProg;
	int		estrichTemp;
	char	estrichTag;	

// Heizkreis abschalten mangels Bedarf 
// Bedingungen: Funktion freigegeben, Raum ext Ist > Soll sowie (wenn parametriert) offene Ventile <= VentOffenMax
	UINT	ventOffen;		// Anzahl offener Ventile (von R50)
	UINT	ventGesamt;		// Gesamt-Anzahl Ventile (von R50)
	char 	ventOffenBer;	// Prozentsatz offener Ventile (gestörte Räume nicht berücksichtigt)
	char 	raumAbsch;		// Ergebnis der Berechnungen

	char	busPuSm;			// Zusammenfassung der über den Pumpenbus (WILO, GENI) signalisierten und der vom Regler gebildeten Störungen
	char	busPuEin;

	char	ucCool;					// Meldung Kühlen
	char 	ucHeat;					// Meldung Heizen

	UINT	leistBegrAbsenk;	// [K] * 10	
//***AnFre 08.07.2010 Legionelle für NahwärmeNetz von KER1s02_SEZ01
	char legio;			// Legionellenbetrieb
	int  zleg;			// Zähler für Legionellenbetrieb

//***AnFre 06.06.2012
	mwsp ExtAnf1Hk;	// von Unterstation 1 für HKN und HK1 gültig
	mwsp ExtAnf2Hk;	// von Unterstation 2 für HKN und HK1 gültig
	char extAnfAktiv;	// Externe Anforderung ist Aktiv ?
	char solAnheb;	// Anhebung Netz-Sollwert ist?
	char bedLadung;	// Bedarfs-Ladung
	char ww_Vorrang;	// WW-Vorrang ist Ein (ANL: und Ausgabe auf R37_1,DA2)
	UINT zVorZeitMax;	// Zähler extern Vorrang-Zeit maximal		  (in ANL:)
	int	 puSoll;			// Pumpen-Sollwert bei BedarfsLadung
	int	 puSollBer;		// Pumpen-Sollwert bei BedarfsLadung berechnet nach Ta (für Anzeige)
	int	 ventMinBer;	// HKN: Ventilstellung Min berechnet "dynamische Ventilstellung Min"		[ % ] *10
// 12.12.2012 Anzeigen für Pumpen 
	char	puEinAnz;		// Anzeige in parli Pumpe eingeschaltet	(von Wilo oder 'normal')
	ULONG	puLzAnz;		// Anzeige in parli Pumpe Laufzeit			(von Wilo oder 'normal')
	char	puBmAnz;		// Anzeige in Parli Pumpe Rückmeldung		(von Wilo oder 'normal')
	char	puSmAnz;		// Anzeige in Parli Pumpe Störung				(von Wilo oder 'normal')
// 06.01.2017	int	absSolBer;		// Absenkung berechnet bei Solares Heizen Ein	 ***AnFre 03.03.2015
	char	puDzSteiEin;// Lade-Pumpe Drehzahlsteigerung ist Ein
}HkDynam;	
 

/* Struktur der Absenktabelle für jeden Wochentag	eines Heizkreises	*/
typedef struct abs_wo{
	char begin;				// Beginn Min
	char begin_h;			// Beginn Std
	char dauer;				// Dauer  [h]
	int  abwert;			// Absenktemperaturwert [K] * 10
}absenktab;
#define ABSTABLENG	sizeof(struct abs_wo)


/* Struktur des Absenk-Arbeitsbereiches für einen Heizkreis	
	 wird im UserNoinit-Ram verwendet mit 2 Instanzen: Zeitabsenkung und Bedarfsabsenkung
*/
typedef struct abs_ti{ 
	UINT	ramex;			// RAM-Überwachung
	UINT	tvab;				// Absenkwert
	UINT	zasd;				// Zähler		Absenkzeit
	UINT	zahd;				// Zähler 	Aufheizzeit
	UINT	zAbsch;			// Zähler		Abschaltzeit
	char	absen;			// Zustand  Absenkphase
	char	aufhe;			// Zustand  Aufheizphase
	char	absch;			// Zustand	Abschaltphase
	char	hunt;				// Zustand  Heizunterbrechung
	char	stuetz;			// Zustand 	Stützbetrieb
	char	frost;			// Zustand	Frostschutzbetrieb
}absenkram;
#define ABSRAMLENG	sizeof(struct abs_ti)

/* Struktur der Regelparameter eines Solar-Heizkreises	***AnFre 14.04.2009	*/
// Teil1: Statische Parameter (mit Standdardwerten)
typedef struct rsSoL{
	int		Ts;								// Tastzeit (>= 1 s)						[s] * 10
	UINT	Tn;								// Nachstellzeit								[s] * 10
	int		Kp;								// P-Verstärkung							[%/K] * 100
	int		Kd;								// D-Verstärkung							[%/K] * 100
	int		DTSoLad;					// dT-Heiz. TS3-TH5						 [K] * 10		
	int		TvAbsenk;					// Vorl.-Absenkung HK1:				  [K] * 10		
	int		Fzk;							// Filterzk. f. tsol						[s] * 10
	char	Hub;							// Ventilhub									 [mm]
	char	Vst;							// Stellgeschwindigkeit		 [mm/min]
	char	Haut;							// HAND/AUTO-Umschaltung
	int		Vstell;						// Hand: Ventil stellen 0-10V Ventil			[%] * 10
	UINT	Y_rel_min;				// Minimale Stellausgabe an 0-10V Ventil	[%] * 10
	UINT	Y_rel_beg;				// Öffnungsbeginn des 0-10V Ventils				[%] * 10
	UINT	RVSoMax;					// RegelVentil maximale Öffnung						[%] * 10
	char	RVzu;			// Dauer RV HK1 ist ZU --> 	kein Heiz-Bedarf 	für Kessel KES:		[min]
	char	RVauf;		// Dauer RV HK1 ist AUF --> wieder HEIZ-Bedarf	für Kessel KES:		[min]
// 30.05.2013 Munzert	int		TvPaKes;		// Absenkung Vorl.Soll HK1:, wenn Kessel AUS
// ***AnFre 06.06.2012	RegelParameter für PumpenRegelung
	int		Pu_Ts;								// Tastzeit (>= 1 s)						[s] * 10
	UINT	Pu_Tn;								// Nachstellzeit								[s] * 10
	int		Pu_Kp;								// P-Verstärkung							[%/K] * 100
	int		Pu_Kd;								// D-Verstärkung							[%/K] * 100
	int		Pu_Fzk;							// Filterzk. f. tsol						[s] * 10
//	int		Vstell;						// Hand: Pumpe stellen 0-10V Ventil			[%] * 10 ist in HK1:
	UINT	PuSoMax;				// Maximale Stellausgabe an Pumpe 0-10V 		[%] * 10
	UINT	PuSoMin;				// Minimale Stellausgabe an Pumpe 0-10V 		[%] * 10
// ***AnFre 06.06.2012
	char	SoLadMin;		// Mindestzeit solare Ladung 		[min]
	char	SoLadSperr;	// Sperrzeit solare Ladung 			[min]
	int		TH3Max;			// Solare Ladung abbrechen, wenn TH3 > TH3Max
	int		TvAnheb;		// Vorl.-Anhebung HKS:	bei solarer Heizunterstützung			  [K] * 10		06.01.2017
}HkSoLStandard;
#define HKSSoLLENG sizeof(struct rsSoL)

/* Struktur der Regelparameter eines Solar-Heizkreises										*/
// Teil2: Dynamische Parameter	
typedef struct rdSoL{
	int		zts;					// Zaehler fuer Tastzeit
	char	regstart;			// Reglerstart nach Reset
	int		ei;						// Regelabweichung							[K] * 10
	float	fl_ei1;				// Regelabweichung zum Tastzeitpunkt i-1
	float	fl_ei2;				// Regelabweichung zum Tastzeitpunkt i-2
	int		tsol;					// Vorlaufsolltemp. , nach der geregelt wird
	float fl_tsol;			//		"								(für Filterberechnung)
	int		tvsb;					// berechnete Vorlaufsolltemp. total
	int		tvsb_hkl;			// berechnete Vorlaufsolltemp. nach Heizkennlinie (für debug)							
	int		stellzeit;		// berechnete Zeit für Stellausgabe
	int		stellsum;			// Summenzähler für Stellausgabe
	int 	fahren;				// Fahrzeit Stellausgabe in ganzen Sekunden
	int		y_rel;				// Stellausgabe an 0-10V Ventil	[%] * 10
	float	fl_y_rel;			//		"		
	int		dy_rel;				// für debug
	UINT	zpu;					// Zähler für Pumpenachlaufzeit
	char	vzu;					// Hand Ventil zufahren			[s]
	char	vauf;					// Hand Ventil auffahren		[s]
	char	sowi;					// Ermittelter Zustand: 1 = Sommerbetrieb
	char	somme;				// Statusanzeige Sommerbetrieb
	char	hkSolJa;	// Solare Heizunterstützung ja = 1
	UINT	zHkSolJa;	// Abschalt-Zähler
	UINT	zRVzu;		// Zähler [s] HK1-RegelVentil ZU --> kein HeizBedarf
	UINT	zRVauf;		// Zähler [s] HK1-RegelVentil AUF --> wieder HeizBedarf
	char	heizBed;  // Heiz-Bedarf für Kessel melden
// ***AnFre 06.06.2012	RegelParameter für PumpenRegelung
	int		pu_zts;					// Zaehler fuer Tastzeit
	char	pu_regstart;			// Reglerstart nach Reset
	int		pu_ei;						// Regelabweichung							[K] * 10
	float	pu_fl_ei1;				// Regelabweichung zum Tastzeitpunkt i-1
	float	pu_fl_ei2;				// Regelabweichung zum Tastzeitpunkt i-2
	int		pu_tsol;					// Sollwert PumpenDrehzahl , nach der geregelt wird
	float pu_fl_tsol;			//		"								(für Filterberechnung)
	int		pu_tvsb;					// berechneter Sollwert PumpenDrehzahl total
	int		pu_y_rel;				// Stellausgabe an Pumpe 0-10V 	[%] * 10
	float	pu_fl_y_rel;			//		"		
	int		pu_dy_rel;				// für debug

// ***AnFre 06.06.2012
	char solLadung;	// Solare-Ladung
	char solLadung_war;  // für Pumpennachlauf
	UINT	zSoLadMin;		// Zähler Mindestzeit solare Ladung 		[30s]
	UINT	zSoLadSperr;	// Zähler Sperrzeit solare Ladung 			[30s]
	char	th3Max;				// Anzeige TH3 Speicher oben Temp. überschritten	
}HkSoLDynam;			// ***AnFre Ende	
 
	
//-------------------------------------------------------------
/* Struktur der Regelparameter eines Warmwasserkreises				*/
// Teil1: Statische Parameter (mit Standdardwerten)
typedef struct ws{
	int	 Twwsol;		// WW-Solltemp
	int  DTTwwlad;	// Speicher-Ladetemperatur = akt.WW-Solltemp. + DTTwwlad	[ K ] *10
	int  TwwladLegio;	// Speicher-Ladetemperatur bei therm.Desinf. Nicht benutzt !!!
	int  DTTwwlbeg;		// delta Speicherlade-Beginn Für Kessel
	int  DTTwwlbegS;	//delta Speicherlade-Beginn für Solar
	int  DTTwwlend;		// deltaSpeicherlade-Ende-Temperatur
	char Puwlna;			// Ladepumpe Nachlaufzeit
//#####ulsch	
	char Puwtna;		// Tauscherpumpe Nachlaufzeit
	char Puwzdau;		// Zirkulationspumpe Abschaltdauer [std]
	char Puwzab[2];	// Zirkulationspumpe Abschaltzeit [min][std]
	char PuwwEin;		// Zirkulationspumpe Einschaltdauer [min]
	char Legjn;			// Legionellentod
	char Legtag;		// Legionellentage
	char Legzeit[2];// Legionellen Einschaltzeit			[min][std]
	char Legdau;		// Legionellen Einschaltdauer			[std]
	char LegSperr;	// Legionellen sperren						[std]
	int  Twwleg;		// Legionellen Temperatur
	char Vorrad;		// Vorrangdauer	max								[min]
// ***AnFre
	int  DTTwwVor;	// WW-Regel-Vorrang Temperatur= akt.WW-Solltemp. + DTTwwlad	[ K ] *10
	int	 DTRegSol;	// DeltaTemp	Regel-Sollwert Solar		[K] * 10
	int	 DTTwwUnter;// WW-Unterwert-Temp = akt.WW-Solltemp. + DTTwwUnter			[K] * 10
	int	 TwwHkAbsenk; // HK1-Absenk-Temp			[K] * 10

	int		Ts;				// Tastzeit (>= 1 s)						[s] * 10
	UINT	Tn;				// Nachstellzeit								[s] * 10
	int		Kp;				// P-Verstärkung							[%/K] * 100
	int		Kd;				// D-Verstärkung							[%/K] * 100
	char	Hub;			// Ventilhub									 [mm]
	char	Vst;			// Stellgeschwindigkeit		 [mm/min]
	char 	Haut;			// Hand/Automatik
	int  	Vstell;		// Hand Ventil stellen			0-10V Ventil	[%] * 10
	char 	Lpea;			// Hand Ladepumpe ein/aus 
	char 	Zpea;			// Hand Zirkpumpe ein/aus
	char 	Tpea;			// Hand Tauscherpumpe ein/aus	
	int		Fzk;			// Filterzk. f. tsol						[s] * 10	
	char	TabwMax;	// maximale Regelabweichung zur Bildung eines Alarms [K] * 10
	char	TabwZeit;	// Zeitdauer der maximalen Regelabweichung bis zum Alarm [min]	
	int		TvorrDiff;// Regelabweichung zur Aktivierung des WW-Vorrangs [K] * 10	
	char	TvorrZeit;// Mindestzeit zur Aktivierung des WW-Vorrangs [min]	
	char	ZpuAusLad;// ZirkPu bei Ladung ausschalten
	char	FrLadBedarf;	// Freigabe für Schalten des Ladebetriebs nach Bedarfsanford. ( über Kommando WR_KASKADE )
	char	FrZirkBedarf;	// Freigabe für Schalten der Zirk.pumpe   nach Bedarfsanford. ( über Kommando WR_KASKADE )	
	char	AuswahlFuehler;	// Speicherladung: Auswahl Fühler ( 1 = oben, 2 = unten, 0 = beide )																															
	int		TvpAnh;		// Anhebung der Anforderung an den Wärmeerzeuger		[K] * 10		
	int		TempWWmin;			// minimale Gebäude-Vorlauftemp. zur Bildung eines Alarms [°C] * 10
	char	TempWWminZeit;	// Zeitdauer der minimalen Gebäude-Vorlauftemp. bis zum Alarm [min]	
	int 	LadungMax;			// [h]		
// ***AnFre 14.04.2009
	UINT	VentStell;// WW-Ventilstellung bei Wiederanlauf der Regelung	
	char	RVzu;			// Dauer RV ist ZU --> 	kein TWW-Bedarf 	für Kessel KES:		[min]
	char	RVauf;		// Dauer RV ist AUF --> wieder TWW-Bedarf	für Kessel KES:		[min]
	UINT	RegHyst;	// DT TW2 > Twwsol ---> Regelventil ZU
	UINT	VentGrenz;// WW-Ventilstellung max. bei Kesseltemp. zu niedrig	 [°C] * 10
	char AbsTag;		// Absenk-Tage
	int  TwwAbs;		// Absenk Soll-Temperatur
// ***AnFre 14.08.2013
	int		BusTauPuSoll;		// Sollwert Tauscherpumpe, wenn WILO-Modbus
	int		BusZirPuSoll;		// Sollwert Zirk.pumpe, wenn WILO-Modbus
	int		BusZirPuSollLad;// Sollwert bei Ladebetrieb Zirk.pumpe, wenn WILO-Modbus
}WwStandard;			
#define WWSLENG sizeof(struct ws)

/* Struktur der Regelparameter eines Warmwasserkreises				*/
// Teil2: Dynamische Parameter	
typedef struct wd{
	char vzu;				// Hand Ventil zufahren			[s]
	char vauf;			// Hand Ventil auffahren		[s]
	int	 zts;				// Zaehler fuer Tastzeit
	char regstart;	// Reglerstart nach Reset
	char wwvor;			// Warmwasservorrang
	char wwlad;			// Ladebetrieb
	char wwLadS;			// Ladebetrieb für Solar //***AnFre
	char solWwLad;	// Solarer-Ladebetrieb möglich //***AnFre
	char solWwVW;		// Solarer-Vorwärmung  //***AnFre
	char solWwLdg;	// Solarer-Ladung  		//***AnFre
	char solWwZir;	// Solarer-Zirkulationsausgleich  		//***AnFre
	int  zpuwlna;		// Zähler für Ladepumpennachlaufzeit
// ##### ulsch #####	
	UINT zpuwtna;		// Zähler für Tauscherpumpennachlaufzeit	
	UINT puwlsta;		// Anzahl der Ladepumpenstarts
	UINT puwlh;			// Ladepumpenlaufzeit							[h]
	UINT puwth;			// Tauscherpumpenlaufzeit					[h]	// 30.05.2013
	UINT puwzh;			// Zirk.pumpenlaufzeit						[h]	// 30.05.2013
	UINT zpuwz;			// Zähler für ZirkPu Abschaltdauer
	UINT zpuwwEin;	// Zähler für WW-Pu Einschaltdauer [30s]
	ULONG zwertAlt;	// Zählerwert KW alt
	char legio;			// Legionellenbetrieb	
	char konvLegio;	// Konventioneller Legionellenbetrieb	//***AnFre
	char solarLegio;	// Solar- Legionellenbetrieb				//***AnFre
	UINT zleg;			// Zähler für Legionellenbetrieb
	UINT zvorrad;		// Zähler für Vorrangdauer
	int  ei_ww;			// Regelabweichung für gleitenden Vorrang, zur Auswertung in RegelHk

	int		ei;				// Regelabweichung							[K] * 10
	float	fl_ei1;		// Regelabweichung zum Tastzeitpunkt i-1
	float	fl_ei2;		// Regelabweichung zum Tastzeitpunkt i-2
	int		tsol;			// Solltemp. , nach der geregelt wird
	float fl_tsol;	//		"								(für Filterberechnung)
	int		stellzeit;// berechnete Zeit für Stellausgabe
	int		stellsum;	// Summenzähler für Stellausgabe
	int 	fahren;		// Fahrzeit Stellausgabe in ganzen Sekunden
	int		y_rel;		// Stellausgabe an 0-10V Ventil	[%] * 10
	float	fl_y_rel;	//		"		
	int		dy_rel;		// für debug

// #####ulsch
	int		twwsb;		// berechneter Sollwert ( Zirkulations-, Ladungs-, Legionellentemp. )
	UINT 	abwCtr;		// (Sekunden)-Counter der Regelabweichung
	int	vorrVerz;	// Zähler für Vorrang-Ein-Verzoegerung
// ***AnFre 14.04.2009
	UINT	zRVzu;		// Zähler RegelVentil ZU --> 	kein TWW-Bedarf
	UINT	zRVauf;		// Zähler RegelVentil AUF --> wieder TWW-Bedarf
	char	twwBed; 	// TWW-Bedarf für Kessel melden
	char	regHystEin;	// TW2 > Twwsol ---> Regelventil ZU ist EIN
	char	kesMinEin;	// TVKES < twsol ---> Regelventil Min-Stellung ist EIN

	char wwladBedarf;			// Ladebetrieb nach Bedarfsanford. ( über Kommando WR_KASKADE )
	char wwladCountDown;
	char wwzirkBedarf;		// Zirkulationspumpe ein nach Bedarfsanford. ( über Kommando WR_KASKADE )
	char wwzirkCountDown;

	UINT 	wwMinCtr;	// (Sekunden)-Counter für SM Gebäude-Vorlauf-Temp.

	// Softalarme
	char tempAlarm;	// Gebäude-Vorlauftemp. TWWV überschreitet fuer eine definierte Zeit einen vorgegebenen Parameter
	char abwAlarm;	// Regelabweichung ueberschreitet fuer eine definierte Zeit einen vorgegebenen Betrag
	char puwlBm;		// Softalarm für 'Betriebsmeldung Ladepumpe' kommt nicht
	char puwzBm;		// Softalarm für 'Betriebsmeldung Zirk.pumpe' kommt nicht
	char puwtBm;		// Softalarm für 'Betriebsmeldung Tauscherpumpe' kommt nicht
	char legioAl;
	UINT wwLadCtr;	// min		
	char wwLadAlarm;		

	char zirkPu;		// EIN = TRUE	
	char tauschPu;	// EIN = TRUE	***AnFre für Wilo-ModBus
	
	
	char busZPuSm;	  // Zusammenfassung der über den Pumpenbus (WILO, GENI) signalisierten und der vom Regler gebildeten Störungen
	char busZPuEin;
	char busTPuSm;	  // Zusammenfassung der über den Pumpenbus (WILO, GENI) signalisierten und der vom Regler gebildeten Störungen
	char busTPuEin;
	char busLPuSm;	  // Zusammenfassung der über den Pumpenbus (WILO, GENI) signalisierten und der vom Regler gebildeten Störungen
	char busLPuEin;
// 12.12.2012 Anzeigen für Pumpen 
	char	puwtEinAnz;	// Anzeige in Parli.h TauscherPumpe eingeschaltet	(von Wilo oder 'normal')
	ULONG	puwtLzAnz;		// Anzeige in parli.h	TauscherPumpe Laufzeit			(von Wilo oder 'normal')
	char	puwtBmAnz;		// Anzeige in Parli.h TauscherPumpe Rückmeldung		(von Wilo oder 'normal')
	char	puwtSmAnz;		// Anzeige in Parli.h TauscherPumpe Störung				(von Wilo oder 'normal')
	char	puwzEinAnz;	// Anzeige in Parli.h	Zirk.Pumpe eingeschaltet
	ULONG	puwzLzAnz;		// Anzeige in parli h	Zirk.Pumpe Laufzeit					(von Wilo oder 'normal')
	char	puwzBmAnz;		// Anzeige in Parli.h Zirk.Pumpe Rückmeldung
	char	puwzSmAnz;		// Anzeige in Parli.h Zirk.Pumpe Störung	

}WwDynam;	

//--------*** AnFre ***-----------------------------------------------------
/* Struktur der Profilparameter eines Warmwasserkreises				*/
// Teil1: Statische Parameter (mit Standdardwerten)
// Achtung: Für jede Structur maximal 1 Page =64Byte,
//					sonst besondere Behandlung bei Übertragung in den EEPROM 
typedef struct profil{
	char Zeit;				// Beginn [Min]
	char ZeitStd;			// Beginn [Std]
	char Dauer;				// Dauer  [h]
	int	 Twwsol;			// WW-Solltemp 
}profil;	
typedef struct profils{
	profil profil[4];		// 4 Zeiten pro Profil
}PrStandard;	
#define PRSLENG sizeof(struct profils)								// Struktur  Byte	

typedef struct profild{
	char	profEin[4];
	int		zProfEin[4];
	int		deltaTwwsol;
	int 	deltaTwwAbs;	// Differenz Absenk-Soll-Temperatur - Soll-Temp.// 07.01.2010
	int		aktTwwsol;
	int		aktTwwlad;
	int		aktTwwVor;
	int		aktTwwUnter;
	int		aktTwwlbeg;
	int		aktTwwlbegS;
	int		aktTwwlend;
}PrDynam;	

//----------------------------------------------------

/* Struktur der Parameter einer Kesselsteuerung			*/
// Teil1: Statische Parameter (mit Standdardwerten)
typedef struct ks{
	int  Tvma;			// max. Kesseltemp.					 			[°C] * 10
	int  Tvmi;			// min. Kesseltemp.					 			[°C] * 10
	int	 Trma;			// max. Rücklauftemp.						  [°C] * 10		
	int	 Kpr;		  	// P-Verstärkung TRS-Begrenzg.    [%/K] * 100
	int  Ttaup;			// Kessel-Taupunkttemp.			 			[°C] * 10
	int  Tvpa;			// Kesselanhebung	HK1			 				[°C] * 10
	int  TvPaWw;		// Kesselanhebung	WW			 				[°C] * 10	//***AnFre
	int  Tvsol10;		// Solltemperatur bei 10V					[°C] * 10
	int  Tvsol0;		// Solltemperatur bei 0V					[°C] * 10
	char Haut;			// Hand/Automatik
	char Kesea;			// Hand Kesselfreigabe ein/aus		
	char Kestufe1;	// Hand Kesselstufe 1  ein/aus		
	char Kestufe2;	// Hand Kesselstufe 2  ein/aus
	int  Kestell;		// Hand Führungswert 0-10,0 Volt 		
	char Kespuea;		// Hand Kesselpumpe ein/aus
	char Kesklaz;		// Hand Kesselabsperrklappe auf/zu
	char Kepuna;		// Kesselpumpen- bzw. Absperrklappen-Nachlauf [min]
	char Kepuverz;	// Kesselpumpen- bzw. Absperrklappen-Verzögerung [min]
	// Rücklauf-Begrenzung (Anhebung)	
	char Haut_begr;	// Hand/Automatik Ventil Rücklauf-Anhebung
	UINT Vstell;		// Hand Ventil Führungswert 0-10,0
	UINT Ts_begr;		// Tastzeit (>= 1 s)							 [s] * 10
	UINT Tn_begr;		// Nachstellzeit									 [s] * 10
	UINT Kp_begr;		// P-Verstärkung								 [%/K] * 100
	UINT Kd_begr;		// D-Verstärkung								 [%/K] * 100		

	char KeBm_verz;	// Verzögerung Kesselbetriebsmeldung  [min]
	char KeAl_verz;	// Verzögerung KesselStörung  [min]
// ***AnFre 10.10.2007
	int	DTLadWw;		// Delta Temp. WWB Pufferladung ein 
	int	DTLadHyst;	// DElta Temp. Schalt-Hysterese
	char	PufMvea;	// Hand MotorVentil Kessel/Puffer	
// ***AnFre 31.01.2008
	char 	UvHkea;		// Hand Umschaltventil Puffer/Kessel auf/zu in SteuerKe.c
	int HkAbsenk;		// Absenkung HK, wenn PufferLadung
	int	DTHeizSol;	// DElta Temp. Solare Heizung über Puffer
// ***AnFre 08.04.2008
	int	DTHeizKon;	// DElta Temp. Konventionelle Heizung über Puffer
	char DTHeizStu;	// DElta Temp. Stufung von dT-Konventionell nach dT-Solar 
// ***AnFre 08.04.2008
	int	DTHeizFBH;	// Delta Temp. FB-Heizung Kessel/Puffer-Umschaltung
// ***AnFre 20.10.2008
	UINT PuffLadStell;// Hand Puffer-Ladung 0-100%
	UINT PuffLadMin;	// Puffer-Ladung Minimum %
	UINT PuffLadMax;	// Puffer-Ladung Maximum %
	UINT PuffLadHkV;	// Puffer-Ladung HK-Ventil geöffnet %
	char ExtAnf1_Ja;	// Externe Anforderung für Kessel gültig
	char ExtAnf2_Ja;	// Externe Anforderung für Kessel gültig
	int		TvAnhExt;		// Anhebung externe Anforderung	30.04.2014
	UINT TvsbSteig;		// langsame Sollwert-Steigerung     	[K/30s] * 10
	UINT TvsbMinde;		// langsame Sollwert-Minderung     		[K/30s] * 10
// Modulation nicht mehr im Programm ab	15.08.2013
//	char Modulation;// Brenner-Modulation einschalten statt Sollwert-Ausgabe
//	int		Ts;				// Tastzeit (>= 1 s)						[s] * 10
//	UINT	Tn;				// Nachstellzeit								[s] * 10
//	int		Kp;				// P-Verstärkung							[%/K] * 100
//	int		Kd;				// D-Verstärkung							[%/K] * 100
//	int	ModMin;			// minimale Modulation 				[ % ] *10
	int	SchaltDiff;		// maximale Temp.-Abweichung abschalten	[ % ] *10
// 24.10.2012 Kessel-Freigabe entsprechend der Ventilstellung der Wärmepumpe: DM_WPU_Ventil
	int	WpVentEin;	// WP-Ventilstellung Kessel Ein	[ % ] *10
	int	WpVentAus;	// WP-Ventilstellung Kessel Aus	[ % ] *10
}KeStandard;	
#define KESLENG sizeof(struct ks)

/* Struktur der Parameter einer Kesselsteuerung			*/
// Teil2: Dynamische Parameter	
typedef struct kd{
	int	 tvsb;			// berechnete KesselVorlaufsolltemp.
	int	 tvsb_kes;	// berechnete KesselVorlaufsolltemp. Anzeige Parli *19:
	int  y_rel;			// Stellausgabe Führungswert
	char zkmin_m;		// Zähler f. Kessel Mindestlaufzeit [min]
	char zkmin_s;		// Zähler f. Kessel Mindestlaufzeit [sek]
	char zkanf_m;		// Zähler f. Kessel Anfahrzeit [min]
	char zkanf_s;		// Zähler f. Kessel Anfahrzeit [sek]
	int  abw;				// Abweichung Kessel_Ist - Kessel_Soll
	int  abw_1;			// alte Abweichung
	int  abwsumme;	// Summe der Abweichungen total (Abtastung alle 10 Sekunden)
	int  abw_sum;		// Abweichsumme normiert auf K*min
	UINT zts;				// Zähler f. Tastzeit der Abweichuntersuchung
	UINT zts_begr;	// Zähler f. Tastzeit der Rücklauf-Begrenzung(Anhebung)	
	UINT puna_cnt;	// Zähler für Pumpen- und Absperrklappen-Nachlauf
	UINT puein_cnt;	// Zähler für Pumpen- und Absperrklappen-Einschaltverzögerung
	char rlbegr;		// Statusanzeige Rücklaufbegrenzung 						***AnFre
// ***AnFre 31.01.2008
	char HkAbse;		// Schalter für HK-Absenken bei Puffer-Ladung

	// Regelparameter für Rücklauf-Begrenzung(Anhebung)
	int y_rel_begr;			// Stellausgabe Führungswert
	int ei_begr;				// Regelabweichung							[K] * 10		
	float	fl_ei1_begr;	// Regelabweichung zum Tastzeitpunkt i-1
	float	fl_ei2_begr;	// Regelabweichung zum Tastzeitpunkt i-2
	float fl_y_rel_begr;
	// Regelparameter für Brenner-Modulation
//	int		ei;				// Regelabweichung							[K] * 10
//	float	fl_ei1;		// Regelabweichung zum Tastzeitpunkt i-1
//	float	fl_ei2;		// Regelabweichung zum Tastzeitpunkt i-2
//	float	fl_y_rel;	//		"		
	char schaltdiff;	// Abschaltdifferenz für Modulation=0 (statt min)

	// eventl. noinit-ram
	UINT kestarts;	// Anzahl der Kesselstarts
	UINT kesh;			// Kessellaufzeit
	UINT kes2h;			// Laufzeit Stufe 2 
	int	dTHeiz;			// DElta Temp. IST-Wert

	// Softalarme
	char keBm_sAl;	// Softalarm Fehlende Betriebsmeldung Kessel 
	UINT keBm_cnt;	// Wartezeit [s]
	char keAl_sAl;	// Softalarm Sammelstörung Kessel verzögert ***AnFre
	UINT keAl_cnt;	// Wartezeit [s]

	char pukeBm;		// Softalarm für 'Betriebsmeldung Zubringer-Pumpe' kommt nicht
	char kesea;				// Merker für Kessel ein/aus 				***AnFre 07.03.2012
// ***AnFre 06.06.2012
	mwsp ExtAnf1Ke;	// von Unterstation 1 für KES gültig
	mwsp ExtAnf2Ke;	// von Unterstation 2 für KES gültig

	char	busPuSm;			// Zusammenfassung der über den Pumpenbus (WILO, GENI) signalisierten und der vom Regler gebildeten Störungen
	char	busPuEin;
	
}KeDynam;	
//---------------------------------------------------------------------


/* Struktur der Regelparameter eines Vorregelkreises (Netztrennung)*/
// Teil1: Statische Parameter (mit Standdardwerten)
typedef struct ns{
	char	FuehrWt;	// Führungs-Tauscher

	int		Ts;				// Tastzeit (>= 1 s)						[s] * 10
	UINT	Tn;				// Nachstellzeit								[s] * 10
	int		Kp;				// P-Verstärkung							[%/K] * 100
	int		Kd;				// D-Verstärkung							[%/K] * 100
	int		Kpr;			// P-Verstärkung TRS-Begrenzg.[%/K] * 100
	int		Tvma;			// max. Vorlauftemp.					 [°C] * 10		
	int		Tvmi;			// min. Vorlauftemp.					 [°C] * 10		
	int		Trma;			// max. Rücklauftemp.					 [°C] * 10		
	int  Tvpa;			// NT-Anhebung	HK1			 				[°C] * 10
	int  TvPaWw;		// NT-Anhebung	WW			 				[°C] * 10	//***AnFre
	int		Fzk;			// Filterzk. f. tsol						[s] * 10
	char	Hub;			// Ventilhub									 [mm]
	char	Vst;			// Stellgeschwindigkeit		 [mm/min]
	char	Haut;			// HAND/AUTO-Umschaltung
	int 	Vstell;		// Hand Ventil stellen			0-10V Ventil	[%] * 10
	char 	Puea;			// Hand Solarpumpe ein/aus
	char	TabwMax;	// maximale Regelabweichung zur Bildung eines Alarms [K] * 10
	char	TabwZeit;	// Zeitdauer der maximalen Regelabweichung bis zum Alarm [min]
	char	Puna;			// Pumpennachlaufzeit										[min]	

// Parameter für 2 Tauscher (Tauscher-Folge)
	char	FuehrWtFest;	// Führungs-Tauscher fest vorgegeben, wenn ungestoert	
	int 	WtWechseln;		// laufzeitabh. Umschalten des Führungs-Tauschers  [h]
	char 	FolgeWtAus;		// Abschalten des Folge-Tauschers, wenn VL - RL des Primär-Tauschers zu klein [K] * 10 	
	UINT 	WtSchaltDiff;	// Schaltdifferenz zum Abschalten des Führungs-Tauschers [K] * 10 
	int	 	Einkrit;			// Einschaltkriterium Folge-Tauscher	[K*min] * 10
	int	 	Auskrit;			// Ausschaltkriterium Folge-Tauscher	[K*min] * 10
	char	KlAZ;					// Klappenstellung bei Hand	
	char	KlappeVerz;		// Verzögerung des Schließens der Klappe [min]
	
	// Sollwert-Anhebung
	UINT 	Ts_anh;				// Tastzeit (>= 1 s)							 [s] * 10
	UINT 	Tn_anh;				// Nachstellzeit									 [s] * 10
	UINT 	Kp_anh;				// P-Verstärkung								 [%/K] * 100
	UINT 	Kd_anh;				// D-Verstärkung								 [%/K] * 100		

}NtStandard;
#define NTSLENG sizeof(struct ns)

/* Struktur der Regelparameter eines Vorregelkreises									*/
// Teil2: Dynamische Parameter	
typedef struct nd{
	int		zts;			// Zaehler fuer Tastzeit
	char	regstart;	// Reglerstart nach Reset
	int		ei;				// Regelabweichung							[K] * 10
	float	fl_ei1;		// Regelabweichung zum Tastzeitpunkt i-1
	float	fl_ei2;		// Regelabweichung zum Tastzeitpunkt i-2
	int		tsol;			// Vorlaufsolltemp. , nach der geregelt wird
	float fl_tsol;	//		"								(für Filterberechnung)
	int		tvsb;			// berechnete Vorlaufsolltemp. total
	int		stellzeit;// berechnete Zeit für Stellausgabe
	int		stellsum;	// Summenzähler für Stellausgabe
	int 	fahren;		// Fahrzeit Stellausgabe in ganzen Sekunden
	int		y_rel;		// Stellausgabe an 0-10V Ventil	[%] * 10
	float	fl_y_rel;	//		"		
	char	zpu;			// Zähler für Pumpenachlaufzeit
	char	vzu;			// Hand Ventil zufahren			[s]
	char	vauf;			// Hand Ventil auffahren		[s]
	char	rlbegr;		// Statusanzeige Rücklaufbegrenzung
	UINT 	puLz;			// Pumpen-Laufzeit [h]	
	UINT  abwCtr;		// (Sekunden)-Counter der Regelabweichung

// Parameter für 2 Tauscher (Tauscher-Folge)
	int	 	tvsb_anl;	// Anlage Vorlauf Solltemp. (max. Anforderung )
	char	wtEin;
	// Sollwert-Anhebung
	int 	y_rel_anh;								
	float	fl_ei1_anh;		// Regelabweichung zum Tastzeitpunkt i-1
	float	fl_ei2_anh;		// Regelabweichung zum Tastzeitpunkt i-2
	float fl_y_rel_anh;		
	UINT 	zts_anh;			// Zähler f. Tastzeit der Anhebung
	
	char 	gestoert;		
	UINT 	lzStd;				// Laufzeit Stunden
	char 	lzMin; 				// Laufzeit Minuten/2
	int 	abw;					// aktuelle Istwert-Sollwert-Differenz, Anlagenfuehler
	int  	abwAlt;				// alte Istwert-Sollwert-Differenz
	int  	abwsumme;			// Summe der Regel-Abweichungen

	UINT	klappeCtr;		// Verzögerung des Schließens der Klappe [sec]	

	//Soft-Alarme
	char abwAlarm;	// Regelabweichung ueberschreitet fuer eine definierte Zeit einen vorgegebenen Betrag	
	char puntBm;		// Softalarm für 'Betriebsmeldung Netzpumpe' kommt nicht
	int		rlKorr;				// aktueller temperaturabh. Rücklauf-Korrekturwert zur berechn.Vorlaufsolltemp. nach Heizkennlinie tvsb_hkl	

	char pu;
	char busPuSm;				// Zusammenfassung der über den Pumpenbus (WILO, GENI) signalisierten und der vom Regler gebildeten Störungen
	char busPuEin;

	UINT blockSec;			// Hauptpumpe: Verzögerung Blockierschutz, wenn Pumpe aus [sec]
	char blockStd;			// Hauptpumpe: Verzögerung Blockierschutz, wenn Pumpe aus [std]
	char blockPuEin;		// Hauptpumpe EIN zwecks Blockierschutz [sec]

}NtDynam;	
 
//---------------------------------------------------------------------


/* Struktur der Regelparameter einer Solar-Regelung */
// Statische Parameter (mit Standdardwerten)

typedef struct ss {

	int		TSolBeg;	// erforderliche Mindesttemp. am Kollektor-Fühler ( Solar-Lad. ein ) [°C] * 10
	int 	TSolEnd;	// Min. Temp. am Kollektor-Fühler ( Solar-Lad. aus ) [°C] * 10
	char	SolAbDau;		// Solar-Abschaltdauer [std]
	char	SolAb[2];	// Solar-Abschaltzeit [min][std]
	int		TS6Max;		// Max. Temp. am oberen Puffer-Fühler ( Solar-Lad. aus ) [°C] * 10
	int		TW3Max;		// Max.Temp. WW-Speicher 
	int		TW2Max;		// Max.Temp. VL-Zirkulation 
	ULONG		V2Min;	// minimaler Kollektor-Vol.Strom 
	int 	TDiffEnd;	// Differenz Kollektor - unterer Puffer-Fühler ( Solar-Lad. aus )	[K] * 10
//	int		DTSoHk;		// DeltaTemp SolarHeizung    TS3-TH2
	int		DTSoWwZ;	// DeltaTemp Solar-WW-Zirk.  TS2-TW2
	int		DTSoWwL;	// DeltaTemp Solar-WW-Ladung TS2-TW4
	int		DTKoll;		// DeltaTemp KollektorBetrieb 	(TS1 > TH5 + DTKoll) oder (TS1 > TS7)
	int		DTPuff;		// DeltaTemp PufferBetrieb 
	int		DTPufLad;	// DeltaTemp Puffer-Ladung TS4>TS7+DTPufLad
	int		TS1Leg;	 	// Legionellen-Temp. am Kollektor ( Solar-Legio ein ) [°C] * 10
	int		TS6Leg; 	// Legionellen-Temp. am oberen Puffer-Fühler ( Solar-Legio ein ) [°C] * 10
	char	LegPause;	// Legionellen-Pause [std]
	int		RVSoMax;	// RegelVentil-Solar max [%]
	char	Puna;			// Kollektorpumpen-Nachlaufzeit	[min]
	char	Haut;			// HAND/AUTO-Umschaltung
	char 	SolPuea;	// Hand Solarpumpe ein/aus
	char 	PufPuea;	// Hand Pufferpumpe ein/aus
	char	KolMvea;	// Hand MotorVentil Kollektor auf/zu
	char	PufMvea;	// Hand MotorVentil Puffer	
	char	SolMvea;	// Hand MotorVentil TWW-Solar auf/zu
	int		KwMvst;		// Hand MotorVentil KaltWasser stellen
	int		NotCode;	//Code und Eingabefeld für Notbetrieb
	int		NotTemp;	//Temperatur bie NotBetrieb
	char	zNotBetr;	//Zähler 20 Sekunden
	char	VerzKollEin;//Verzögerung Einschalten KollektorBetrieb [min]
	char	VerzPuffEin;//Verzögerung Einschalten PufferBetrieb			[min]
	int		SS1_Skal0;	// Skalierung Solar-Sensor Watt/m^2 bei 0 Volt
	int		SS1_Skal10;	// Skalierung Solar-Sensor Watt/m^2 bei 10 Volt
// 10.04.2008	int		SS1_Off;		// Sensor-Offset
	char 	SS1_Filter;	// Anzeige gedämpft z.B. =5 => 1/5 des Neuwertes zum Altwert
	char	vts23Al;		// Warte-Zähler für T3>T2-Alarm
	int		TS5Max;			// Max.Temp. Solar-Pumpe abschalten 
	int		TS3Frost;		// Frostgefahr an der Solar-Pumpe						 [°C] * 10
	int		KwVol0;			// Kaltwasser-Volumenstr. für KW-Ventil 0,0% offen 						[m^3 ]
	int		KwVol100;		// Kaltwasser-Volumenstr. für KW-Ventil 100,0% offen 					[m^3 ]
	int		KwVentMin;	// Kaltwasser-Ventilstellung min.	Vorwärmerweg								[ % ]
	int		KwSolVolMax;// Kaltwasser-Volumenstr. Max., wenn Solar  KW-Ventil 				[m^3 ]
	int		KwSolVentMin;	// Kaltwasser-Ventilstellung Mindestöffnung	Vorwärmerweg		[ % ]
	UINT	DzSoPuKol;	// Drehzahl SolarPumpe Kollektorbetrieb	auch Sollwert bei WILO-Modbus	[ % ] *10
	UINT	DzSoPuPuf;	// Drehzahl SolarPumpe Pufferbetrieb		auch Sollwert bei WILO-Modbus	[ % ] *10
	UINT	DzSoPuHand;	// Drehzahl SolarPumpe Handbetrieb												[ % ] *10
// 03.08.2011 Prüfbetrieb (Schnupper-Schaltung)
	int		PruefSS1;		// prüfen ab SS1 > 300 W/m*2
	int		PruefTS1;		// oder prüfen ab TS1 > 20,0 °C
	int		PruefDauer;	// Pumpe ein für 3 Minuten
	int		PruefSperre;// Prüfen sperren für 60 Minuten
// ***AnFre 06.06.2012
	int		RvHksStart;	// RV-HKS Stellung für Pufferbeladung starten  [ % ] *10
	int		RvHksEnde;	// RV-HKS Stellung für Pufferbeladung beenden  [ % ] *10
	int		PumStart;		// L-Pumpen-Drehzahl für Pufferbeladung starten, wenn Solare Ladung Ein   [ % ] *10
	int		PumEnde;		// L-Pumpen-Drehzahl für Pufferbeladung beenden, wenn Solare Ladung Ein   [ % ] *10
// ***AnFre 14.08.2013
	int		BusPufPuSoll;		// Sollwert Pufferpumpe, wenn WILO-Modbus

} SoStandard;								// Struktur  Byte

#define SOSLENG sizeof (struct ss)

/* Struktur der Regelparameter einer Solar-Regelung	*/
// Dynamische Parameter	
typedef struct sd {
	char	pufLad;		// aktueller Ladezustand ( 0/1 )
	char	speiLad;	// aktueller Speicher-Ladezustand ( 1 = Speicherladung )
	char	zpu;			// Zähler für Pumpenachlaufzeit
	UINT  lzKolPuH;	// Kollektorpumpenlaufzeit [h]
	UINT  lzKolPuMin;	// Kollektorpumpenlaufzeit, Zwischenzaehler [min]
// ***AnFre
	UINT  lzPufPuH;	// Kollektorpumpenlaufzeit [h]
	UINT  lzPufPuMin;	// Kollektorpumpenlaufzeit, Zwischenzaehler [min]
	char	kollEin;			//Kollektor-Betrieb
	UINT	zKollEin;			//Zähler Kollektor-Betrieb
	char	puffEin;			//Puffer-Betrieb
	UINT	zPuffEin;			//Zähler Puffer-Betrieb
	char	solAus;				//Solar-Abschaltung
	UINT	zSolAus;			//Zähler Solar-Abschaltung
	UINT	zLegSperr;		//Zähler konv.Legio sperren
	UINT	zLegPause;		//Zähler Solar-Legio Pause
	UINT	zSLeg;				//Zähler Solar-Legionelle ein 
	char	jahrAlt;			// Jahreswechsel
	char	monAlt;				// Monatswechsel
	float fl_ss_Wh_g;		//Gesamtwert
	float fl_ss_Wh_j;		//Jahreswert
	float fl_ss_Wh_m;		//Monatswert
	float fl_ss_Wh_d;		//Tageswert
	float fl_ss_Wh_h;		//Stundenwert
	UINT	SS1_kWh_vj;		// SolarSensor Messung  Jahr	[ kWh/m^2 ]
	UINT	SS1_kWh_vm;		// SolarSensor Messung  Monat	[ kWh/m^2 ]
	UINT	SS1_Wh_vd;		// SolarSensor Messung  Tag	[ kWh/m^2 ]
	UINT	SS1_Wh_vh;		// SolarSensor Messung  Stunde	[ Wh/m^2 ]
	ULONG	SS1_kWh_g;		// SolarSensor Messung gesamt [ kWh/m^2 ]
	UINT	SS1_kWh_j;		// SolarSensor Messung  Jahr	[ kWh/m^2 ]
	UINT	SS1_kWh_m;		// SolarSensor Messung  Monat	[ kWh/m^2 ]
	UINT	SS1_Wh_d;		// SolarSensor Messung  Tag	[ Wh/m^2 ]
	UINT	SS1_Wh_h;			// SolarSensor Messung  Stunde	[ Wh/m^2 ]

	UINT	SS1_Monat[12];		// SolarSensor Messung  Monate	[ kWh/m^2 ]
	UINT	SS1_VJ_Monat[12];		// SolarSensor Messung  Monate Vorjahr	[ kWh/m^2 ]
	char	SS1_Sync;			//Synchronisieren ALLER SS-Zähler (Ges.,Monate...)
	// Softalarme
	char pusoBm;		// Softalarm für 'Betriebsmeldung Solarpumpe' kommt nicht
	char pusoBmVerz;		// Verzögerung Softalarm für 'Betriebsmeldung Solarpumpe' [30s]
	char pupuBm;		// Softalarm für 'Betriebsmeldung Pufferpumpe' kommt nicht
	char pupuBmVerz;		// Verzögerung Softalarm für 'Betriebsmeldung Pufferpumpe' [30s]
	mwsp	SS1_Anz;
	char	ts23Alarm;	// T3>T2-Alarm
	char	zts23Al;		// Warte-Zähler für T3>T2-Alarm
	UINT	ts23AlDauer;		// Zähler für Dauer T3>T2-Alarm
	char	pufVoll;		// Puffer ist voll beladen ==> KesselSpeicher laden
//im RAM	char	pusoTempAl;			// Merker Temp.-Alarm: Solar-Pumpe AUS
// 03.08.2011 Prüfbetrieb (Schnupper-Schaltung)
	UINT	zPruefDauer; 	// Zähler Pumpe ein Dauer [30s]
	UINT	zPruefSperre; // Zähler Prüfung sperren [30s]
	char	pruefBetrieb;	// Anzeige Prüfbetrieb

	char	busSolPuSm;			// Zusammenfassung der über den Pumpenbus (WILO, GENI) signalisierten und der vom Regler gebildeten Störungen
	char	busPufPuSm;			// Zusammenfassung der über den Pumpenbus (WILO, GENI) signalisierten und der vom Regler gebildeten Störungen
	char	busPuEin;
	char	solarPu;			// Merker Pumpe Solarkreis	ein/aus für Wilo-PumpenBus
	char	pufferPu;			// Merker Pumpe Pufferkreis ein/aus für Wilo-PumpenBus
	char	busSolPuEin;	// 
	char	busPufPuEin;	// 
// 12.12.2012 Anzeigen für Pumpen 
	char	solPuEinAnz;	// Anzeige in Parli.h SolarPumpe eingeschaltet	(von Wilo oder 'normal')
	ULONG	solPuLzAnz;		// Anzeige in parli Pumpe Laufzeit							(von Wilo oder 'normal')
	char	solPuBmAnz;		// Anzeige in Parli.h SolarPumpe Rückmeldung		(von Wilo oder 'normal')
	char	solPuSmAnz;		// Anzeige in Parli.h SolarPumpe Störung				(von Wilo oder 'normal')
	char	pufPuEinAnz;	// Anzeige in Parli.h	PufferPumpe eingeschaltet
	ULONG	pufPuLzAnz;		// Anzeige in parli Pumpe Laufzeit							(von Wilo oder 'normal')
	char	pufPuBmAnz;		// Anzeige in Parli.h PufferPumpe Rückmeldung
	char	pufPuSmAnz;		// Anzeige in Parli.h PufferPumpe Störung	
	
} SoDynam;	
 
//---------------------------------------------------------------------

// #####ulsch : Ferienprogramm #####
typedef struct ferienZeit {
	datum ferienBeg;
	datum ferienEnd;
} ferienZeit;


// #####ulsch : Heizgradtage #####
typedef struct {
	UINT hzGrdTage;
	char hzTage;
	char hzTageBew;	
	UINT hzGrdTageVj;
	char hzTageVj;
	char hzTageBewVj;		
} monHzGrd;												// size 8 Byte, wichtig fuer Bicbus-Zugriffe ( 64/8 ist restlos teilbar ! )

typedef struct {
	mwsp copyData;
	char copyFirst;
	UINT copyRxTimer; 
} copydata;



/* Struktur eines Wärmezählers */
typedef struct {
		ULONG wmeng;						// Zählwert absolut	( kWh )
		ULONG verbr[12];				// Monatsverbräuche Aktuell
		ULONG verbr_vj[12];			// Monatsverbräuche Vorjahr		
		ULONG flow_max;
		UINT wmLeistg_max;
		char resMax;						// Maximal-Werte in Struktur zaehlspWmeng loeschen ( flow_max, wmLeistg_max, auch wmLeistgMittelMax )
		char syncFlag;
		ULONG flow;							// Dim. 0,0 m3/h 
		UINT wmLeistg;					// kW
		zaehlsp *pZaehlsp;
		mwsp *pVl;
		mwsp *pRl;
		ULONG lastWmeng;				// für Leistungsberechnung
		ULONG wmeng_Jsum;
		ULONG wmeng_J;					// in Joule 
		char monInVj;
		ULONG long lastWert;		// aktueller Zaehlerstand

		UINT volumen;
		UINT zeitdiff;	
		UINT zeitdiffZuletzt;
		float masse;
		char monSichern;
		char monReset;
			
} zaehlspWmeng;		

// ***AnFre: Kaltwasserzaehler TWW Volumenstrom-Berechnung
/* Struktur eines Volumenstrom-Rechners 19 Byte */ 
//	neu: 03.09.2009 mit Abspeicherung der Max-Werte
typedef struct {
		char resVMax;						// Maximal-Werte in Struktur zaehlspVolstr loeschen ( flow_max )
		ULONG flow;							// Dim. 0,000 m3/h 
		ULONG flow_max;
		datum VmaxDatum;				// Datum bei V1-Maximum
		zeit	VmaxZeit;					// Zeit  bei V1-Maximum
		mwsp	VmaxTW1;					// WW-Regel-Temp TW1 bei V1-Maximum
		mwsp	VmaxTW2;					// WW-Speicher-Temp TW2 bei V1-Maximum
} zaehlspVolstr;		

// ***AnFre: Waermezaehler-Maximumwerte
/* Struktur WZ-Maxima 15 Byte */
typedef struct {
	datum PDatum;
	zeit PZeit;
	mwsp Pta1m;
	ULONG PVolStr;
	mwsp PsolSens;		//WZ2-Solar
} wzMax;

/* Struktur zur Berechnung und Anzeige der maximalen Durchschnittsleistung (Standard: Stunde) */
typedef struct {
		char MittelnLeistg;			// im EEPROM gespeichert [min] (zulässig : 5, 10, 15, 20, 30, 60)
		UINT mittelMax;					// immer zuerst in dieser Struktur !!!	
		UINT mittel;						// letzter berechneter Wert
		datum tag;
		uhrzeit zeit;
		mwsp vl;								// Primär-Vorlauf	
		mwsp rl;								// Primär-Rücklauf
		mwsp ta;								// Außentemperatur
		ULONG flow;							// Volumenstrom	
		char resMax;						// alles rücksetzen
} maxLeistung;

/* Struktur zur Berechnung und Anzeige Grenz-Leistung */
typedef struct {
		UINT GrenzLeistg;				// Grenz-Leistung im EEPROM gespeichert [kW] mit Standard-Wert
		UINT LeistgHyst;				// Hysterese für Grenz-Leistung im EEPROM gespeichert [kW] mit Standard-Wert
		char BegrAkt;						// 0 : Begrenzung der mittl.Leistung (5 min) 1: Begr.der akt.Leistung
		UINT Daempfung;					// leistBegr wird gedämpft, wenn die Momentanleistung zur Begrenzung benutzt wird
} grenzLeistungStat;

typedef struct {
		char begrenzung;				// Meldung Begrenzung Ein ?
		UINT msec;							// Messung der Millisekunden ab letztem Zählimpuls
		UINT altMsec;						// Millisekunden alter Impulsabstand
		ULONG long lastWert;		// letzter Zähler-Wert
		UINT kwhImp;						// KWh pro Impuls
		UINT leistung;					// Leistung in kW
		UINT leistBegr;
} grenzLeistungDyn;


/* Diagnose */
typedef struct {
	int	temp;
	datum tag;
	mwsp ta;		
} extrTemp;												// Min- oder Max-Temperatur mit zugehoerigem Tag und Aussentemperatur

typedef struct {
	ULONG flow_max;
	UINT wmLeistg_max;
	UINT flowBereich_h[4];
	UINT leistgBereich_h[4];
//	UINT wwlad_anz;
//	UINT wwlad_time;
//	UINT wwvor_anz;
//	UINT wwvor_time;
	extrTemp tvpMax;
	extrTemp tvpMin;			
// 24.09.2014 für 2.Wärmezähler ACHTUNG bei BICRD feste Bytezahl 38
	ULONG flow_max2;
	UINT wmLeistg_max2;
	UINT flowBereich_h2[4];
	UINT leistgBereich_h2[4];
	extrTemp tvpMax2;
	extrTemp tvpMin2;			
} diagnose;

typedef struct {
	datum begin;
	datum end;
	ULONG	flowBereich[4];
	UINT leistgBereich[4];
// 24.09.2014 für 2.Wärmezähler
	ULONG	flowBereich2[4];
	UINT leistgBereich2[4];
} diagParam;

/* AnFre 03.09.2009 Diagnose KWZ Volumenbereiche */
typedef struct {
	UINT flowBereich_h[4];	// 4 Bereiche  in h
	UINT	gesZeit_h;					//  Gesamtzeit in h
} diagnoseKWZ;

typedef struct {
	ULONG	FlowBereich[4];		// Parameter für 4 Bereiche
} diagParamKWZ;

/*---------------------------------------------------------------*/


// für U_Test_RS485
typedef struct {
	char Mode;							// Master (1) oder Slave (0) Mode
	char status;						// Status der Datenübertragung im Task 'SlaveS1'
	char rxtest;						// Testanzeige
	char Leng;							// Telegrammlänge im Rx-Interrupt   
	char TLeng;							// Telegrammlänge im Task 'SlaveS1'
	char Bcount;						// Sende- / Empfangspuffer Index
	char Cond;							// Empfangszustand im Rx-Interrupt 
	char bcc;								// Block Check Code
	char TelCnt;						// Teiler für Telegramm-Timeout
	char stx;								// Merker für STX oder STX8
	char ordnum;						// letzer Auftrag
	char tsk_cnt;						// Antwortverzögerung
	UINT Baudrate;
	UINT match_fall;
	UINT match_rise;
	int  MatchCount;
	int  BaudQuali;
	char tmr_count;				// Zähler für TMR_BIT Überwachung
	UINT toutl;
	UINT touth;
	char tout_ext;					// bei FLASHCONTROL-Kommandos TimeOut-Verlängerung in Rx_Init();
	DatBits cstat;
	char merkAdr;
} serial485;	

// für U_Funkempf
typedef struct rf{
	unsigned char ucRf_status;						// Status des Sensors und der Schalterstellung am Sensor
	unsigned char ucRf_praesens;
	signed int siRf_temp;
	signed char scRf_temp_shift;
	unsigned char ucLive_check_timer;
	unsigned char ucSetup;								// muß in den EEPROM
	unsigned long ulRf_modul_id;					// muß in den EEPROM
	unsigned char ucfBedienZeit[6];				// [0] = Tag, [1] = Monat, [2] = Jahr, [3] = Stunde, [4] = Minute, [5] = Sekunde
	unsigned char ucfEmpfangsZeit[6];			// [0] = Tag, [1] = Monat, [2] = Jahr, [3] = Stunde, [4] = Minute, [5] = Sekunde
	unsigned char ucRf_teach_enable;
//	char rf_switch;
}rfsens;

// ***** josch: Datenmanager ***************************************************************
// Gerät im DM_Modus (Data Master),	Datenbearbeitung in der Funktion DManager(), Organisation der Datenübertragung im Task DTimer() 
typedef struct {
	char aktiv;									// Datenübertragung anregen im DTimer
	char ready;									// Datenübertragung ausgeführt
	char SlaveAdr;							// Moduladresse des Slavegerätes
	char TxLeng;								// an den Slave zu sendende Datenanzahl
	char TxBuf[DM_TXBUF_MAX];		// an den Slave zu sendende Daten
	char RxTout;								// Empfangs-Timeout in Minuten
	char RxStat;								// Status der Empfangsdaten
	char RxBuf[DM_RXBUF_MAX];		// vom Slave empfangene Daten
	mwsp rdta;									// Außentemperatur vom Slave empfangen 
	char rdta_bc;								// Außentemperaturquelle (bc_flag)
	char rdta_flag;							// Flag für erfolgte Datenübertragung					
}dm_param;	
	
/******** GENIbus-Strukturen ********/
// Control
typedef struct gc{
	char pu_adr;
	char pu_func;				// HZG,  WW, oder deaktiviert
	char pu_func_shad;
	char prod_name[16];
	char pu_operation;
	char pu_control;
	char setpoint;
	char pu_al_reset;
	char comand_reply;	// Für Auswertung Kommandos Outp: 	0 = deaktiviert, 0x80 = OK, sonst Error-Codes
	char value_reply;		// Für Auswertung Daten Inp:    		0 = deaktiviert, 0x80 = OK, sonst Error-Codes
	char error_count;
	char error_flag;
}control_value;
#define GCLENG sizeof(struct gc)
//---------------------------------------------------------------

// Measure Values
typedef struct gm{
	int pu_t_w;					// (Medientemperatur)
	int pu_p;						// (akt. Leistung)
	int pu_speed;				// (Drehzahl)
	int pu_h;						// (akt. Förderhöhe)
	int pu_q;						// (akt. Förderstrom)
	int pu_led_contr;
	int pu_act_mode1;
	int pu_act_mode2;
	int pu_act_mode3;
	int pu_al_code;
//	char pu_f_act;				// (akt. Frequenz)
// werden getrennt behandelt
	UINT energy;				// (Energieverbrauch)
	UINT t2hour;				// (Betriebsstunden)
	char pu_unit_family;
	char pu_unit_type;
	char pu_unit_version;
	char pu_operation_inp;
	char pu_control_inp;
}measure_value;
//#define GMLENG sizeof(struct gm)
//---------------------------------------------------------------


/******** Modbus-Const-Strukturen für Wilo, Belimo,********/
// Const
typedef struct reg_rq{
	UINT reg_adr;					// 
	UINT reg_cnt;					// 
	char obj_typ;					//
	const char *conv_table;
}reg_request;	

/******** Modbus-Control-Strukturen für Wilo ********/
// Control
typedef struct mbcs{
	char slave_adr;
	char exeption_code;
	char bus_timeout;
	char pu_timeout;
	char clear_flag;
	char Pu_function;
	char Pu_function_shad;
	char tio_count;
	char error_count;
	char rx_errorflag;
	char tx_errorflag;
}modb_control_value;
#define MBCSLENG sizeof(struct mbcs)

/******** Modbus-Daten-Struktur für Wilo ********/
// Data
typedef struct mbds{
	UINT input_reg0;  	  		// Actual Differential Pressure		0.1 m WS1          
	UINT input_reg1;  	  		// Flow Rate											0.1 m³/h           
	UINT input_reg2;  	  		// Power Consumption							1 kWh              
	UINT input_reg3;  	  		// Power Rating										1 W                
	UINT input_reg4;      		// Mains Current									0.1 A                   
	UINT input_reg5;  	  		// Speed													1 rpm                   
	UINT input_reg6;  	  		// Medium Temperature							0.1 K                   
	UINT input_reg7;  	  		// Current Operation Mode					See Table 5.3.          
	UINT input_reg8;  	  		// Pump Module										See Table 5.5.          
	UINT input_reg9;  	  		// Pump Type											See Table 5.6.          
	UINT input_reg10; 	  		// Max Speed											1 rpm                   
	UINT input_reg11; 	  		// Min Speed											1 rpm                   
	UINT input_reg12; 	  		// Supported Errors								See Table 5.9.            
	UINT input_reg13; 	  		// Supported Service Messages			See Table 5.7.            
	UINT input_reg14; 	  		// Max Power Rating								1 W 
	UINT input_reg15; 	  		// Service Message								See Table 5.7.      
	UINT input_reg16; 	  		// Error Type											See Table 5.8.      
	UINT input_reg17; 	  		// Error Message									See Table 5.9.      
	UINT input_reg18; 	  		// Pump Status										See Table 5.10.   
	UINT input_reg19; 	  		// State Diagnostics							See Table 5.11.   
	UINT input_reg20; 	  		// Reserve  
	UINT SetPoint_Input;			// Sollwert 	  Ist, auf Geni-Konvention zurück dekodiert		
	UINT Operation_Input;			// Betriebsart Ist, auf Geni-Konvention zurück dekodiert	
	UINT Control_Input;				// Regelart Ist, auf Geni-Konvention zurück dekodiert	

//ulsch
//-	UINT output_value_shad0;  // Sollwert, 		VergleichsRegister, 		
//-	UINT output_value_shad1;  // Betriebsart, VergleichsRegister, 		
//-	UINT output_value_shad2;  // Regelart, 		VergleichsRegister, 
//-	UINT holding_reg0;  			// Sollwert, 		Kopie des HoldingRegister 1  der Pu, 		
//-	UINT holding_reg1;  			// Betriebsart, Kopie des HoldingRegister 40 der Pu 		
//-	UINT holding_reg2;  			// Regelart,		Kopie des HoldingRegister 42 der Pu		
	UINT output_value_shad[HOLDING_REG_CNT];	// VergleichsRegister
													  // Sollwert		
														// Betriebsart
														// Regelart
														// Bus Command Timer
	UINT holding_reg[HOLDING_REG_CNT];
														// Sollwert, 		Kopie des HoldingRegister 1  der Pu,
														// Betriebsart, Kopie des HoldingRegister 40 der Pu 
														// Regelart,		Kopie des HoldingRegister 42 der Pu
														// Bus Command Timer, Kopie des HoldingRegister 300 der Pu
															// 0 = unknown
															// 1 = OFF
															// 2 = SET (IF-Module wechselt zu ACTIVE und startet Timer 5 min
															// 3 = ACTIVE
															// 4 = RESET
															// 5 = MANUAL (set value ist gültig, aber lokales Pumpen-Menü bleibt offen. "last write wins"
	char message0;						// Global-Flag Doppelbelegung z. Zt. noch keine Anzeige mehrerer Zustände 
	char message1;            // Service Message,   aus dem Bitmuster von Input_reg15 dekodierter Charwert für UserKonv.c		
	char message2;            // Error Type,			  aus dem Bitmuster von Input_reg16 "						
	char message3;            // Error Message,		  aus dem Bitmuster von Input_reg17 "				
	char message4;            // Pump Status,			  aus dem Bitmuster von Input_reg18 "					
	char message5;            // State Diagnostics, aus dem Bitmuster von Input_reg19 "		
	char message1_text[16];   // Service Message,  Text-Anzeige in parli_Wilo.h
	char message2_text[16];   // Error Type,			 Text-Anzeige in parli_Wilo.h
	char message3_text[16];   // Error Message,		 Text-Anzeige in parli_Wilo.h
	char message4_text[16];   // Pump Status,			 Text-Anzeige in parli_Wilo.h				
	char message5_text[16];   // State Diagnostics,Text-Anzeige in parli_Wilo.h
	ULONG op_hours;   				// Operation Hours	10 h   
	ULONG op_hours_dp;				// z. Zt. nicht relevant
	int input_reg6_celsius;  	// Medium Temperature							0.1 °C                   

	char busSmWar;

// ab hier nicht löschen in Funktion ClearValues()
//ulsch
	UINT output_value[HOLDING_REG_CNT];
														// Sollwert, 		OutputRegister, auf Wilo-Konvention dekodiert
														// Betriebsart, OutputRegister, auf Wilo-Konvention dekodiert
														// Regelart, 		OutputRegister, auf Wilo-Konvention dekodiert
														// Bus Command Timer	
//-	UINT output_value0;				// Sollwert, 		OutputRegister, auf Wilo-Konvention dekodiert
//-	UINT output_value1;				// Betriebsart, OutputRegister, auf Wilo-Konvention dekodiert
//-	UINT output_value2;				// Regelart, 		OutputRegister, auf Wilo-Konvention dekodiert
	UINT output_value0_temp;	// Sollwert Soll		in 10-tel %
	UINT output_value1_temp;	// Betriebsart Soll, 	OutputRegister, auf Wilo-Konvention dekodiert
	UINT output_value2_temp;	// Regelart Soll, 		OutputRegister, auf Wilo-Konvention dekodiert
}modb_data_value;
#define MBDSLENG sizeof(struct mbds)

// ***** Josch-SDM : Archiv SD-Card-Memory ********************************************************* 
// Struktur für Ferro-Ram: Infobereich für den Schnellzugriff bei der Auswahl der Pages auf der SD-Karte
typedef struct fe_info {
	UINT	akt_DP;		// aktuelle Datenpage-Nr.
	char	num_DP;		// Anzahl der Einträge in der Datenpage
	char	off_DP;		// Offset zum nächsten Eintrag (+4 Byte Zeitstempel)
	char	akt_VP;		// aktuelle Verweispage-Nr.
	char	num_VP;		// Anzahl der Einträge in der Verweispage
	char	num_HP;		// Anzahl der Einträge in der Hauptpage
	char	ovr_HP;		// Anzahl der Datenpage-Überläufe (Ringpuffer)
}ferro_info;

//---------------------------------------------------------------

/* Funktionsprototypen */
//float e_filter(float alt, int neu, int ts, int zk);		
float g_filter(float alt, int neu, int ts, int zk);		
void Check_CountDown(void);
void set_bcta1(void);
void Typelist_copy(void);
void Load_Userdef(void);
void T_E_S_T___________P_U_N_K_T(void);
void PumpenAus(void);
void hk_abschalten(char hk, char stemp);
char* get_feldadresse(char *wert, unsigned char ucExp_num, unsigned char ucAttr, unsigned char alfunc); 
void Fehler(char code, signed char idx);		
void fill_dummy(void);
char TempMitt(mwsp* pT1, mwsp* pT2, mwsp* pTmitt);
char get_alspeicher_idx(char *p_wert);
char get_alspeicher(char *p_wert);
void set_alspeicher(char *p_wert, char ea);

/***** ulsch : Archivspeicher *****/
void WritePageToArchiv ( char *TxBuf, char *RxBuf, char prot );		
void ReadPageFromArchiv ( char *TxBuf, char *RxBuf, char prot );		
float Dy_rel ( int  kp, int kd, int ts, UINT tn, float ei, float ei1, float ei2 );
int Gerade_YvonX ( int x, int x1, int y1, int x2, int y2 );

/***** ulsch : Heizgradtage *****/
void HgtAnzeigen ( char mon );
void Init_Hgt_User(void);
void Init_Leittab(void);

// josch: in CopyCheck.c
char CopyInit(unsigned int dae_num, char *pDae);
void clear_abs_ram(char hk);
void clear_abs_bed(char hk);

UINT DayOfYear ( datum date );

// josch: Datenmanager
void DReply(void);
void DS_CountDown(void);
void DManager(void);
void DM_CountDown(void);
void RW_dmanager_reply(char *RsTxBuf, char *RsRxBuf, char prot);

// Archiv SD-Card
void ArchReply(char *RsTxBuf, char *RsRxBuf, char prot);


// Josch-SDM : Archiv SD-Card-Memory
void ArchSDMReply(char *RsTxBuf, char *RsRxBuf, char prot);

#endif	// USTRUCT_H_INCLUDED
