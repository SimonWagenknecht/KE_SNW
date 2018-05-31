// Task: AnalogEA
// Skalierung der 0-10V Eingänge, Grenzwertüberwachung

#include "struct.h"
#include "ustruct.h"
#include "defins.h"
#include "sramext.h"
#include "uramext.h"
#include "uconstext.h"
#include "unoinitext.h"


// --------------- Task Input -------------------------------
// Taskaufruf: 1 s

void AnalogEA ( void )
{

#if ANALOG_AE > 0

	static char anlauf = 1;
	static char i = 8;

	int mw;																			// aktueller Messwert
	char stat;																	// aktueller Messwert-Status
	int mwFilt;																	// gedämpfter Messwert
	float	fl_mwFilt = 0;												// gedämpfter Messwert
	int mwSkal;																	// skalierter Messwert

	if ( ++i >= 8 )
		i = 0;
	if ( i >= ANALOG_AE )
		return;

//-// Anforderung ausgeben : -40°C...+60°C => 0...10,0 V
//-	if ( i == 0 )
//-	{
//-		if ( maxAnford <= hks[HK1].AnaOutp[0].Skal0 )
//-			AA_UNI[U1]->awert = 0;
//-		else if ( maxAnford >= hks[HK1].AnaOutp[0].Skal10 )
//-			AA_UNI[U1]->awert = 1000;			
//-		else
//-			AA_UNI[U1]->awert = Gerade_YvonX ( maxAnford, hks[HK1].AnaOutp[0].Skal0, 0, hks[HK1].AnaOutp[0].Skal10, 1000 );
//-	}

//-// Kessel-Anforderung ausgeben : 0..100,0°C => 0..10,00 V
//-		if ( maxAnford <= 0 )
//-			AA_UNI[U1]->awert = 0;
//-		else if ( maxAnford >= 1000 )
//-			AA_UNI[U1]->awert = 1000;			
//-		else
//-			AA_UNI[U1]->awert = maxAnford;
//-	
// kanalspez.Vorbereitung		
	mw = AE_UNI[i]->messw;
	stat = AE_UNI[i]->stat;		
	fl_mwFilt	= anaInp[i].fl_mwFilt;
	mwFilt = anaInp[i].mwFilt;

// allg. Berechnung des gedämpften und des skalierten Messwerts													
	if ( anlauf )
	{
		mwFilt = mw;
		fl_mwFilt = (float)mwFilt / 10 ;		
	}	
	else
	{
		fl_mwFilt = g_filter ( fl_mwFilt, mw, 1, AnaInpPara[i].ZkFilt );	// Sprungantwort 66 % nach ZkFilt	
		mwFilt = (int)( fl_mwFilt * 10 );
	}	

	mwSkal = Gerade_YvonX ( mwFilt, 0, AnaInpPara[i].Skal0, 1000, AnaInpPara[i].Skal10 );

// speichern der berechneten Daten		
	anaInp[i].mwFilt = mwFilt;
	anaInp[i].fl_mwFilt = fl_mwFilt;
	anaInp[i].mwSkal.messw = mwSkal;
	anaInp[i].mwSkal.stat = stat;
	
	if ( anlauf && i == ( ANALOG_AE - 1 ) )
		anlauf = 0;																// Anlauf ist füe alle Eingänge erledigt

// Grenzwertüberwachung
	if ( anaInp[i].mwSkal.stat == 0 )
	{
		if ( AnaInpPara[i].UGW != 0 )
		{
			if ( anaInp[i].mwSkal.messw <= AnaInpPara[i].UGW )
				anaInp[i].ugwSM = TRUE;
			else if ( anaInp[i].mwSkal.messw >= AnaInpPara[i].UGW + AnaInpPara[i].Hyst )
				anaInp[i].ugwSM = FALSE;	
		}
		else
		{																					// kein Grenzwert
			anaInp[i].ugwSM = FALSE;					
//		anaInp[i].ugwSM_sp = FALSE;	
		}				
	}
	else
		anaInp[i].ugwSM = FALSE;

	if ( anaInp[i].mwSkal.stat == 0 )
	{
		if ( AnaInpPara[i].OGW != 0 )
		{
			if ( anaInp[i].mwSkal.messw >= AnaInpPara[i].OGW )
				anaInp[i].ogwSM = TRUE;
			else if ( anaInp[i].mwSkal.messw <= AnaInpPara[i].OGW - AnaInpPara[i].Hyst )
				anaInp[i].ogwSM = FALSE;	
		}
		else
		{																					// kein Grenzwert
			anaInp[i].ogwSM = FALSE;					
//		anaInp[i].ogwSM_sp = FALSE;	
		}					
	}
	else
		anaInp[i].ogwSM = FALSE;
	
#endif
}				

