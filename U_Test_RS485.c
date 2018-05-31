// ----------- Beispiel für eine Test-RS485 an verschiedenen Schnittstellen -----------------------
//	Aufruf erfolgt vom HandlerS1 bis HandlerS3 organisiert vom UserHandler

#include "sfr32C87.h"
#include <string.h>
#include "sio.h"
#include "timer.h"
#include "struct.h"
#include "defins.h"
#include "uramext.h"
#include "sramext.h"
#include "kommando.h"
#include "pubramext.h"

#define BDCHK_LOSS			4				// MatchCount-Verlust durch ISR BaudCheck 

extern char ModulAdr;
extern char StationsAdr;

void UP_Master(char port);
void UP_Slave(char port);
void UP_baudset(char port);
void UP_Init_BaudCheck(char port);
void UP_CheckUp_Baudrate(char port);
void UP_Quali(char port, int MatchCount, int MatchSoll);
void UP_Rx_Init(char port);
void UP_Rx_End(char port, char	ec);
void change_RTS(char port, char set);	
void enable_send(char port);	

#if ( ((IMPLEMENT_S1 & GBUS1_IMPL) == GBUS1_IMPL) || ((IMPLEMENT_S2 & GBUS1_IMPL) == GBUS1_IMPL) || ((IMPLEMENT_S3 & GBUS1_IMPL) == GBUS1_IMPL) )


//--------------------------------------------------------------------------------------
// Taskweiterleitung: Einsprung vom UserHandler
//--------------------------------------------------------------------------------------
char U_Test_RS485(char port, char funktion)
{
	serial485 *pSerial;

	char fu_ret = funktion;		// Ok Vorbelegung
	
	pSerial = NULL;
	
	// für das Zusammenspiel mit verschiedenen Auftraggebern wird in diesem Beispiel die Masterfunktion bekannt gemacht
	switch(port)
	{ 
		case S1:
			pSerial = &serial[S1];
			Mode_S1 = pSerial->Mode;
			break;
		case S2:
			pSerial = &serial[S2];
			Mode_S2 = pSerial->Mode;
			break;
		case S3:
			pSerial = &serial[S3];
			Mode_S3 = pSerial->Mode;
			break;
	}		
				
	if(pSerial->Mode ==	MASTER)
	{	if( (pSerial->status == BD_CHECK) || (pSerial->status == BD_CHECKSTART) )
		{	// _Sx-Baudratenerkennung disabled
			switch(port)
			{ 
				case S1:
					TME_BIT_S1 	= 0;		// Interrupt Enable  Bit löschen		
					break;
				case S2:
					TME_BIT_S2 	= 0;		// Interrupt Enable  Bit löschen		
					break;
				case S3:
					TME_BIT_S3 	= 0;		// Interrupt Enable  Bit löschen		
					break;
			}		

			pSerial->status 	  = 0;
			pSerial->BaudQuali = 0;
		}	
		UP_Master(port);
	}
	else	// SLAVE - MODE
		UP_Slave(port);
	

	return(fu_ret);			// Rückkehr
	
}		

/*--------------------------------------------------------*/
/*----------  Test-RS485	als	Master ---------------------*/
/*--------------------------------------------------------*/
void UP_Master(char port)
{
	char i,	j, rcb, leng;
	signed char k;
	char *pRxBuf;
	char *pTxBuf;
	serial485 *pSerial;
	
	static char send_verz = 0;
	char master = 0;
	
	pSerial = NULL;
	pRxBuf = NULL;
	pTxBuf = NULL;
	switch(port)
	{
		case S1:
			master  = MASTER_S1;
			pRxBuf  = RxBuf_S1;
			pTxBuf  = TxBuf_S1;
			pSerial = &serial[S1];
			break;
			
		case S2:
			master  = MASTER_S2;
			pRxBuf  = RxBuf_S2;
			pTxBuf  = TxBuf_S2;
			pSerial = &serial[S2];
			break;
			
		case S3:
			master  = MASTER_S3;
			pRxBuf  = RxBuf_S3;
			pTxBuf  = TxBuf_S3;
			pSerial = &serial[S3];
			break;
	}
			

	// Abfrage,	ob ein Auftrag noch	in Arbeit	ist
	i	=	0;
	for(k	=	0; k < VEC_ANZAHL; k++)
	{
		if(vec[k].work ==	1)
		{
			if(vec[k].master == master)			// bin ich gemeint ?
			{ // ja
				i	=	(char)(k+1);
				break;
			}	
		}				
	}
	
	if(i > 0)	// Auftrag unterwegs
	{	i	-= 1;
		// Abfrage der Auftragsfertigstellung
		if(pSerial->cstat.rs485run ==	FALSE)	// Ergebnis	verfügbar
		{
			send_verz = 1;								// eine Taskverzögerung für Sendeauftrag bei 50ms-Task

			vec[i].work = 0;							// Work	löschen
			rcb = 0x80;										// Returncode	vorbelegen

			if(pSerial->cstat.rs485tout ==	TRUE)	
				rcb	=	0x40;									// Time	Out
			else
			{	if(	(pSerial->cstat.overflow ==	TRUE)	|| (pSerial->bcc !=	0) )
						rcb	=	0x20;							// Telegrammfehler
			}
			
			if(rcb	!= 0x80)					// bei Fehler
			{
				if((vec[i].request & 0x7F) > 0)	//MD	if(vec[i].request > 0)	// sind	noch Wiederholungen	möglich	? (BroadCast-Kennzeichen 0x80 ausblenden)
					vec[i].request--;			// Wiederholungen	decrementieren
			}
			else											// ohne Fehler, Antwort formal	ok
			{	vec[i].request = 0;			// Anforderung löschen
				vec[i].wait = 0;
			}

			if((vec[i].request & 0x7F) > 0)	//MD	if(vec[i].request > 0)	// sind	noch Wiederholungen	möglich	? (BroadCast-Kennzeichen 0x80 ausblenden)
			{
				if( rcb == 0x40 )				// bei Time	Out
					vec[i].wait	=	1;			// minimale Wartezeit laden und Ende						
				else
					vec[i].wait	=	10;			// Wartezeit laden und Ende						
				
				send_verz = 0;					// keine Sendeverzögerung
			}
			else	
			{
				if(rcb ==	0x80)
				{
					if(cstat_S1.broadcast == FALSE)
					{
						// Antwortpuffer des Auftraggebers laden
						leng = (char)(RxBuf_S1[0]+3);	// Länge der Daten +Leng +Adr	+rcode
						if(leng	<	ANSWER_SIZE)
						{
							for(j	=	0; j < leng; j++)
								vec[i].buffer[j]	=	RxBuf_S1[j];
						}
						else
							rcb	=	0x20;
					}
					else
					{
						vec[i].buffer[1] = vec[i].buffer[0];	// gesendete Adresse, in der Regel BROADC
						vec[i].buffer[0] = 0;									// Länge
						vec[i].buffer[2] = RC_OK;
					}	
				}
				vec[i].rcode = rcb;			// Returncode	für	den	Auftraggeber
			}
		}
	}
	else	// Testen	auf	neuen	Auftrag
	{
		j = 0;											// Zähler für VEC_ANZAHL
		i = 0;											// flag für request
		k = pSerial->ordnum;				// letzer Auftrag
		if(vec[k].wait > 0)					// Wartezeit gesetzt ?
		{	vec[k].wait--;						// ja
			if(vec[k].wait == 0)			// bei Ablauf,
				k--;										// Wiederholung mit letztem Auftrag, keine Weiterschaltung  
			else	
				return;									// Task-Ende, Wartezeit noch nicht abgelaufen
		}

		if(	send_verz > 0)					// eine Taskverzögerung für Sendeauftrag bei 50ms-Task
		{	send_verz--;
			return;
		}	

		while( j < VEC_ANZAHL)
		{
			k++;											// nächster Auftrag		
			if( k >= VEC_ANZAHL )
				k = 0;
			
			if( vec[k].request > 0)	
			{
				if(vec[k].master == master)	// bin ich gemeint ?
				{ // ja
					i = (char)(k+1);			// flag für request
					pSerial->ordnum = k;				// Auftragnummer merken
					break;
				}
			}
			j++;
		}			

		if(i	>	0)	// ein Auftrag liegt an
		{	
			i	-= 1;
#if SYSTEM_CPL == RIECON50_CPL
			if((vec[i].request >= 0x80) && (vec[i].buffer[2] == BCUHR)) {	//MD
				// Es soll BCUHR übertragen werden													//MD
				Fuelle_BCUHR(vec[i].buffer);																//MD
			}																															//MD
#endif
			//	Auftrag	ausführen
			leng	=	vec[i].buffer[1];	// Länge(Kommando	+	Daten)
			if(leng < S3_TXSIZE)
			{
				if(vec[i].buffer[2] == FLASHCONTROL)	// Primärkommando mit erwarteter größerer Antwortverzögerung
					pSerial->tout_ext = 10;							// TimeOut-Verlängerung in Rx_Init();
				else
					pSerial->tout_ext = 1;

				change_RTS(port, 1);									// Sendekanal	Freigabe
				delay(5);															// 500µs Zwangspause für eine stabile Startbit-Breite
				vec[i].work 	= 1;										// Work	setzen
				
				//	Sendepuffer	füllen
				pTxBuf[0] 		= PSYNC;
				pTxBuf[1] 		= STX;
				for(j = 0;	j	<	leng+2;	j++)
					pTxBuf[j+2]	=	vec[i].buffer[j];
			
				//	BlockCheckCode errechnen
				rcb = 0xA8;
				for (j	=	2; j < leng+4; j++)	//4	=	leng + 2 + j / 2 = SlaveAdr	+	Kommando
				{	
					if(pTxBuf[j]	!= 0xAA)
						rcb ^=	pTxBuf[j];
				}
				pTxBuf[j++] = rcb;
				pTxBuf[j++] = PSYNC;
				pTxBuf[j]	 	= ETX;
				pTxBuf[j+1] = 0xFF;			// Abschlußbyte, damit ETX sauber	gesendet wird
																	//(RTS kommt zu	früh beim	letzten	Sendeinterrupt)

				pSerial->Leng		=	(char)(pTxBuf[3] +	8);	// Telegrammlänge	für	Sendeinterrupt
				//	+8 = PSYNC,	STX, SlaveAdr, Leng, Kommando, ... , BCC,	PSYNC, ETX,	Zusatzbyte
				/* SIO - Initialisieren	und	Sendeinterrupt vorbereiten		 */
				pSerial->cstat.psyncflag	=	0;		// Merker	für	PSYNC-Verdopplung
				pSerial->cstat.rs485run		=	TRUE;
				pSerial->cstat.rs485tout	=	FALSE;
				pSerial->cstat.overflow		=	FALSE;
				if(vec[i].request >= 0x80) {		//MD
					pSerial->cstat.broadcast = TRUE;		//MD
					vec[i].request -= 0x80;				//MD
				}																//MD
				else {													//MD
					pSerial->cstat.broadcast = FALSE;		//MD
				}																//MD
				pSerial->Bcount						=	0;		// Sendepuffer Index 
				UP_baudset(port);
				enable_send(port);								// Senden erlauben				
			}
			else // Pufferüberschreitung
			{
				vec[i].request = 0;
				vec[i].rcode = 0x20;			// Returncode	für	den	Auftraggeber
			}	
		}
	}
	
	
}
// Ende Test-RS485	als	Master
//--------------------------------------------------------------------------------------------


//-------------------------------------
// Sendekanal	Freigabe / Sperren
//-------------------------------------
void change_RTS(char port, char set)	
{
	switch(port)
	{
		case S1:
			RTS_S1 = set;
			break;
		case S2:
			RTS_S2 = set;
			break;
		case S3:
			RTS_S3 = set;
			break;
	}
}			
			
//-------------------------------------
// Senden erlauben
//-------------------------------------
void enable_send(char port)
{	
	switch(port)
	{
		case S1:
			TE_BIT_S1 = 1;					// Senden erlauben
			DISABLE_IRQ			
			TIC_S1 = 0x04;					// Enable Transmit Interrupt Prio: 4			
			ENABLE_IRQ	
			TB_S1	 = 0xFF;					// Startbyte ausgeben
			break;
		case S2:
			TE_BIT_S2 = 1;					// Senden erlauben
			TIRLT_BIT_S2 = 1;		// Interrupt Request Select Bit im Interrupt Enable  Register (IIO1IE) setzen  auf used for interrupt
			TIR_BIT_S2	 = 0;		// Interrupt Request Bit 				im Interrupt Request Register (IIO1IR) löschen
			TIE_BIT_S2	 = 1;		// Interrupt Enable  Bit 				im Interrupt Enable  Register (IIO1IE) setzen
			DISABLE_IRQ			
			TIC_S2 = 0x04;					// Enable Transmit Interrupt Prio: 4			
			ENABLE_IRQ	
			TB_S2	 =	0xFF;					// Startbyte ausgeben
			break;
		case S3:
			TE_BIT_S3 = 1;					// Senden erlauben
			DISABLE_IRQ			
			TIC_S3 = 0x04;					// Enable Transmit Interrupt Prio: 4			
			ENABLE_IRQ	
			TB_S3	 = 0xFF;					// Startbyte ausgeben
			break;
	}
}

			
//-------------------------------------
// Master Baudrate einstellen
//-------------------------------------
void UP_baudset(char port)
{
	serial485 *pSerial;
	
	#if (IMPLEMENT_S1 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S1)
	{
		pSerial = &serial[S1];

		// Transmit Receive Mode Register
		MR_S1 = 0x05;				// 0 0 0 0 0 101	
												// | | | | | |||
												// | | | | | UART Mode 8 Bit
												// | | | | Internal Clock             
												// | | | 1 Stop Bit	
												// | | x	
												// | no parity
												// Rx Tx Polarity not inversed
	
		// Transmit Receive Control Register 0											
		if(pSerial->Baudrate >= 9600)
		{
		//C0_S1 = 0x10;				// 0 0 0 1 0 0 00
			C0_S1 = 0x30;				// 0 0 1 1 0 0 00
													// | | | | | | ||
													// | | | | | | f1 Takt
													// | | | | | x CTS oder RTS Funktion siehe Bit4
													// | | | | x Transmit Register Empty Flag TXEPT, Read Only, 
													// | | | Disable CTS/RTS Funktion											
													// | | TxD: 0 = CMOS Output, 1 = N-channel Open Drain Output
													// | CLK Polarität
													// LSB first
		}
		else
			C0_S1 = 0x31;				// f8 Takt								 
	
	
	
		// Für	Telegramm-Timeout	Überwachung: TimerB0 (Vorteiler) + TimerB1 (Teiler) 
		// SourceClock: f2n = 16 (29,491200 MHz / 16 = 1,843200 MHz = 0,54253472 µs)
		pSerial->toutl 		= 64;		// Vorteiler: 64
		P_TOUTS_S1	= 0;		// Stop	 Telegramm-Timeout	Timer Prescaler
		C_TOUTS_S1	= 0;		// Stop	 Telegramm-Timeout	Timer Counter
	
		switch(pSerial->Baudrate)
		{
			case 38400:
				BRG_S1 = (unsigned char)( ( (f1_CLK_SPEED/16) / 38400) -1);
				pSerial->touth = 1200; 	// Telegramm-Timeout Überwachung: 80Bytes*10Bits * 26,04166µs = 20833,33µs	/	0,5425µs(f2n)	=	38400 / 64 = 600 
				break;
				
			case 19200:
				BRG_S1 = (unsigned char)( ( (f1_CLK_SPEED/16) / 19200) -1);
				pSerial->touth = 2400;
				break;
			
			case 9600:			
				BRG_S1 = (unsigned char)( ( (f1_CLK_SPEED/16) / 9600) -1);
				pSerial->touth = 4800;
				break;
				
			case 4800:
				BRG_S1 = (unsigned char)( ( (f8_CLK_SPEED/16) / 4800) -1);
				pSerial->touth = 9600;
				break;
				
			case 2400:
				BRG_S1 = (unsigned char)( ( (f8_CLK_SPEED/16) / 2400) -1);
				pSerial->touth = 19200;
				break;
				
			default:								// 1200
				BRG_S1 = (unsigned char)( ( (f8_CLK_SPEED/16) / 1200) -1);
				pSerial->Baudrate	=	1200;
				pSerial->touth = 38400;
				break;
		}
	}	
	#endif
	
	//----------------------------------------------------------------------------
	
	#if (IMPLEMENT_S2 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S2)
	{
		pSerial = &serial[S2];

		// Transmit Receive Mode Register
		MR_S2 = 0x05;				// 0 0 0 0 0 101	
												// | | | | | |||
												// | | | | | UART Mode 8 Bit
												// | | | | Internal Clock             
												// | | | 1 Stop Bit	
												// | | x	
												// | no parity
												// Rx Tx Polarity not inversed
	
		// Transmit Receive Control Register 0											
		if(pSerial->Baudrate >= 9600)
		{
			C0_S2 = 0x10;				// 0 0 0 1 0 0 00
													// | | | | | | ||
													// | | | | | | f1 Takt
													// | | | | | x CTS oder RTS Funktion siehe Bit4
													// | | | | x Transmit Register Empty Flag TXEPT, Read Only, 
													// | | | Disable CTS/RTS Funktion											
													// | | TxD CMOS Output
													// | CLK Polarität
													// LSB first
		}
		else
			C0_S2 = 0x11;				// f8 Takt								 
	
	
	
		// Für	Telegramm-Timeout	Überwachung: TimerB0 (Vorteiler) + TimerB1 (Teiler) 
		// SourceClock: f2n = 16 (29,491200 MHz / 16 = 1,843200 MHz = 0,54253472 µs)
		pSerial->toutl 		= 64;		// Vorteiler: 64
		P_TOUTS_S2	= 0;		// Stop	 Telegramm-Timeout	Timer Prescaler
		C_TOUTS_S2	= 0;		// Stop	 Telegramm-Timeout	Timer Counter
	
		switch(pSerial->Baudrate)
		{
			case 38400:
				BRG_S2 = (unsigned char)( ( (f1_CLK_SPEED/16) / 38400) -1);
				pSerial->touth = 1200; 	// Telegramm-Timeout Überwachung: 80Bytes*10Bits * 26,04166µs = 20833,33µs	/	0,5425µs(f2n)	=	38400 / 64 = 600 
				break;
				
			case 19200:
				BRG_S2 = (unsigned char)( ( (f1_CLK_SPEED/16) / 19200) -1);
				pSerial->touth = 2400;
				break;
			
			case 9600:			
				BRG_S2 = (unsigned char)( ( (f1_CLK_SPEED/16) / 9600) -1);
				pSerial->touth = 4800;
				break;
				
			case 4800:
				BRG_S2 = (unsigned char)( ( (f8_CLK_SPEED/16) / 4800) -1);
				pSerial->touth = 9600;
				break;
				
			case 2400:
				BRG_S2 = (unsigned char)( ( (f8_CLK_SPEED/16) / 2400) -1);
				pSerial->touth = 19200;
				break;
				
			default:								// 1200
				BRG_S2 = (unsigned char)( ( (f8_CLK_SPEED/16) / 1200) -1);
				pSerial->Baudrate	=	1200;
				pSerial->touth    = 38400;
				break;
		}
	}	
	#endif
	
	//----------------------------------------------------------------------------
	
	#if (IMPLEMENT_S3 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S3)
	{
		pSerial = &serial[S3];

		// Transmit Receive Mode Register
		MR_S3 = 0x05;				// 0 0 0 0 0 101	
												// | | | | | |||
												// | | | | | UART Mode 8 Bit
												// | | | | Internal Clock             
												// | | | 1 Stop Bit	
												// | | x	
												// | no parity
												// Rx Tx Polarity not inversed
	
		// Transmit Receive Control Register 0											
		if(pSerial->Baudrate >= 9600)
		{
			C0_S3 = 0x10;				// 0 0 0 1 0 0 00
													// | | | | | | ||
													// | | | | | | f1 Takt
													// | | | | | x CTS oder RTS Funktion siehe Bit4
													// | | | | x Transmit Register Empty Flag TXEPT, Read Only, 
													// | | | Disable CTS/RTS Funktion											
													// | | TxD CMOS Output
													// | CLK Polarität
													// LSB first
		}
		else
			C0_S3 = 0x11;				// f8 Takt								 
	
	
	
		// Für	Telegramm-Timeout	Überwachung: TimerB0 (Vorteiler) + TimerB1 (Teiler) 
		// SourceClock: f2n = 16 (29,491200 MHz / 16 = 1,843200 MHz = 0,54253472 µs)
		pSerial->toutl 		= 64;		// Vorteiler: 64
		P_TOUTS_S3	= 0;		// Stop	 Telegramm-Timeout	Timer Prescaler
		C_TOUTS_S3	= 0;		// Stop	 Telegramm-Timeout	Timer Counter
	
		switch(pSerial->Baudrate)
		{
			case 38400:
				BRG_S3 = (unsigned char)( ( (f1_CLK_SPEED/16) / 38400) -1);
				pSerial->touth = 1200; 	// Telegramm-Timeout Überwachung: 80Bytes*10Bits * 26,04166µs = 20833,33µs	/	0,5425µs(f2n)	=	38400 / 64 = 600 
				break;
				
			case 19200:
				BRG_S3 = (unsigned char)( ( (f1_CLK_SPEED/16) / 19200) -1);
				pSerial->touth = 2400;
				break;
			
			case 9600:			
				BRG_S3 = (unsigned char)( ( (f1_CLK_SPEED/16) / 9600) -1);
				pSerial->touth = 4800;
				break;
				
			case 4800:
				BRG_S3 = (unsigned char)( ( (f8_CLK_SPEED/16) / 4800) -1);
				pSerial->touth = 9600;
				break;
				
			case 2400:
				BRG_S3 = (unsigned char)( ( (f8_CLK_SPEED/16) / 2400) -1);
				pSerial->touth = 19200;
				break;
				
			default:								// 1200
				BRG_S3 = (unsigned char)( ( (f8_CLK_SPEED/16) / 1200) -1);
				pSerial->Baudrate	=	1200;
				pSerial->touth = 38400;
				break;
		}
	}
	#endif	

}



/*--------------------------------------------------------*/
/*----------  Test-RS485	als	Slave	----------------------*/
/*--------------------------------------------------------*/
void UP_Slave(char port)
{
	char *pRxBuf;
	char *pTxBuf;
	serial485 *pSerial;

	char	auswerten;

	pSerial = NULL;
	pRxBuf = NULL;
	pTxBuf = NULL;
	switch(port)
	{
		case S1:
			pRxBuf  = RxBuf_S1;
			pTxBuf  = TxBuf_S1;
			pSerial = &serial[S1];
			break;
			
		case S2:
			pRxBuf  = RxBuf_S2;
			pTxBuf  = TxBuf_S2;
			pSerial = &serial[S2];
			break;
			
		case S3:
			pRxBuf  = RxBuf_S3;
			pTxBuf  = TxBuf_S3;
			pSerial = &serial[S3];
			break;
	}

	/*	-------------- wenn	Empfang	noch nicht beendet -----------*/
	if(pSerial->status	== RX_RUN)
	{	pSerial->rxtest	=	1;
	}
	else
	{ /*	-----------------	wenn Empfang beendet ----------------	*/
		if(pSerial->status	== RX_END)
		{
			/*	Testen auf Time	Out		*/
			if(pSerial->cstat.rs485tout ==	TRUE)
			{	pSerial->status	=	RX_TOUT;
				pSerial->rxtest	=	2;
			}
			else
			{ /*	Testen auf Empfangsüberlauf	*/
				if(pSerial->cstat.overflow	== TRUE)
				{	pSerial->status	=	RX_ERROR;
					pSerial->rxtest	=	3;
				}
				else
				{ /*	Anfangskennung untersuchen */
					if(pRxBuf[0]	!= STX)
					{	pSerial->status	=	RX_ERROR;
						pSerial->rxtest	=	4;
					}
					else
					{ /*	Endekennung	untersuchen	*/
						if(pRxBuf[pSerial->TLeng	+	2] !=	ETX)													 
						{	pSerial->status	=	RX_ERROR;
							pSerial->rxtest	=	5;
						}
						else
						{ /*	Adressen entspr. Kommando selektieren */
							pub_merkAdr = pRxBuf[1];
							auswerten = FALSE;

							if(pRxBuf[3] == DATUE_KOM)
							{	if(pub_merkAdr == StationsAdr || StationsAdr == 255)
									auswerten = TRUE;
							}	
							else
							{	if(pub_merkAdr == BROADC)
									auswerten = TRUE;
								if(pub_merkAdr == ModulAdr || ModulAdr == 255)
									auswerten = TRUE;
							}
						
							if(auswerten == FALSE)
							{	 pSerial->status = RX_ERROR;
								 pSerial->rxtest = 6;
							}	 
							else
							{ /*	BCC-Check	 */
								if(pSerial->bcc !=	0)
								{	pSerial->status	=	RX_ERROR;
									pSerial->rxtest	=	7;
								}	 
								else
								{									 
									if(pSerial->tsk_cnt == 0)	// Antwort um eine Taskzeit verzögern
									{	pSerial->tsk_cnt = 1;
										return;
									}
									pSerial->tsk_cnt = 0;

									/*	Kommandoausführung,			Kommando	in [3]	*/
									SlaveReply(&pTxBuf[0],	&pRxBuf[3],	PROT485);	 	// Sendepuffer	wird komplett	gefüllt
									
									/*	Antwort	nicht	senden */
									//merkAdr = BROADC; //!!!!!!!!!!!!	Test
									if(pub_merkAdr	== BROADC)
									{	pSerial->status	=	RX_ERROR;
										pSerial->rxtest	=	8;
									}	 
									else	 /*	Antwort	senden */
									{
										change_RTS(port, 1);									// Sendekanal	Freigabe
										// Die Zeit zwischen RTS und UART-Ausgabe des Startbytes schwankt zwischen 10 und 100µs

										delay(5);							// 500µs Zwangspause für eine stabile Startbit-Breite

										pSerial->status	=	TX_RUN;
										pSerial->rxtest	=	9;
										pSerial->Leng = (char)(pTxBuf[3] + 10);	// Telegrammlänge	für	Sendeinterrupt
										// +10 = PSYNC,	STX8,	MasterAdr, Leng, ModulAdr, Rcode,	BCC, PSYNC,	ETX, Zusatzbyte
										/*	SIO	-	Initialisieren und Sendeinterrupt	vorbereiten			*/
										pSerial->cstat.psyncflag = 0;	// Merker	für	PSYNC-Verdopplung
										pSerial->Bcount	= 0;			 		// Sendepuffer	Index	
										enable_send(port);						// Senden erlauben				
										
									}// keine Antwort bei Broadcast
								} //falscher	BCC		 
							}	// falsche Adresse
						}		// falsche Endekennung
					}			// falsche Anfangskennung
				}				// Empfangsüberlauf		
			}					// Timeout erkannt 
		}						// Empfang nicht im	RX_END modus
	
		/*----------------	wenn "Sendung	beendet" oder	noch keine Aktion	--------------*/ 
		if(pSerial->status	== TX_END	|| pSerial->status ==	0	|| pSerial->status ==	RX_TOUT	|| pSerial->status ==	RX_ERROR)
		{
			/* ----	 Empfang vorbereiten ----- */
			UP_Init_BaudCheck(port);
			
				/* weiter	im Hintergrund mit
					 - interrupt Baud_Check_Int
						 ...startet
					 - interrupt Rs_Rx_Int
						 ...liest	alle Zeichen ein bis zum Endekennzeichen oder	max. Puffer	
				*/
		}
		else
		{	// Überwachung auf Interrupt Request BaudCheck (ISR wurde nicht ausgeführt)
			auswerten = 0;
			switch(port)
			{
				case S1:
					if(TMR_BIT_S1 == 1)
						auswerten = 1;
					break;
				case S2:
					if(TMR_BIT_S2 == 1)
						auswerten = 1;
					break;
				case S3:
					if(TMR_BIT_S3 == 1)
						auswerten = 1;
					break;
			}			
			
			if(auswerten == 1)
			{	if(++pSerial->tmr_count > 9)			// Zähler für TMR Bit Überwachung
				{	pSerial->tmr_count = 0;
					UP_Init_BaudCheck(port);
				}
			}
			else
				pSerial->tmr_count = 0;
		}			
						
	} //	Empfang	noch nicht beendet
	
}		// Ende Test-RS485	als	Slave
//---------------------------------------------------------------------------------------------------------		

//-----------------------------------------------
// Slave Baudrate testen und einstellen
//-----------------------------------------------
void UP_CheckUp_Baudrate(char port)
{
	 
	serial485 *pSerial;
	
	#if (IMPLEMENT_S1 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S1)
	{
		pSerial = &serial[S1];

		// Für	Telegramm-Timeout	Überwachung: TimerB0 (Vorteiler) + TimerB1 (Teiler) 
		// SourceClock: f2n = 16 (29,491200 MHz / 16 = 1,843200 MHz = 0,54253472 µs)
		P_TOUTR_S1 = 64;							// Vorteiler: 64
		
		// gemessene Werte mit RIEcon91.  Beim RIEcon90 durchschnittlich 20 Counts mehr
		if(pSerial->MatchCount	<	415)			// Soll: 384, gemessen: 384
		{	
			BRG_S1 = (unsigned char)( ( (f1_CLK_SPEED/16) / 38400) -1);
			pSerial->Baudrate	=	38400;
			UP_Quali(port, pSerial->MatchCount, 384);
			// Telegramm-Timeout Überwachung: 80Bytes*10Bits * 26,04166µs = 20833,33µs	/	0,5425µs(f2n)	=	38400 / 64 = 600 
			C_TOUTR_S1 = 1200;
		}
		else if(pSerial->MatchCount	<	800)	// Soll: 768, gemessen: 766
		{	
			BRG_S1 = (unsigned char)( ( (f1_CLK_SPEED/16) / 19200) -1);
			pSerial->Baudrate	=	19200;
			UP_Quali(port, pSerial->MatchCount, 768);
			// Telegramm-Timeout Überwachung
			C_TOUTR_S1 = 2400;
		}	 
		else if(pSerial->MatchCount	<	1600)	// Soll: 1536, gemessen: 1532
		{	
			BRG_S1 = (unsigned char)( ( (f1_CLK_SPEED/16) / 9600) -1);
			pSerial->Baudrate	=	9600;
			UP_Quali(port, pSerial->MatchCount, 1536);
			// Telegramm-Timeout Überwachung
			C_TOUTR_S1 = 4800;
		}
		else if(pSerial->MatchCount < 3100)	// Soll: 3072, gemessen: 3063	
		{	
			BRG_S1 = (unsigned char)( ( (f8_CLK_SPEED/16) / 4800) -1);
			pSerial->Baudrate = 4800;
			UP_Quali(port, pSerial->MatchCount, 3072);
			//	Telegramm-Timeout	Überwachung
			C_TOUTR_S1 = 9600;
		}
		else if(pSerial->MatchCount	<	6175)	// Soll: 6144, gemessen: 6125
		{	
			BRG_S1 = (unsigned char)( ( (f8_CLK_SPEED/16) / 2400) -1);
			pSerial->Baudrate	=	2400;
			UP_Quali(port, pSerial->MatchCount, 6144);
			// Telegramm-Timeout Überwachung
			C_TOUTR_S1 = 19200;
		}
		else											// Soll: 12288, gemessen: 12250
		{	
			BRG_S1 = (unsigned char)( ( (f8_CLK_SPEED/16) / 1200) -1);
			pSerial->Baudrate	=	1200;
			UP_Quali(port, pSerial->MatchCount, 12288);
			//	Telegramm-Timeout	Überwachung
			C_TOUTR_S1 = 38400;
		}
			
		//	Telegramm-Timeout	initialisieren
		P_TOUTS_S1	= 1;					// Start Telegramm-Timeout Timer Prescaler
		C_TOUTS_S1	= 1;					// Start Telegramm-Timeout Timer Counter
	
		//	Empfangsinit
		pSerial->status = RX_RUN;
		UP_Rx_Init(port);
	}
	#endif
	
	//----------------------------------------------------------------------------
	
	#if (IMPLEMENT_S2 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S2)
	{
		pSerial = &serial[S2];
		
		// Für	Telegramm-Timeout	Überwachung: TimerA3 (Vorteiler) + TimerA4 (Teiler) 
		// SourceClock: f2n = 16 (29,491200 MHz / 16 = 1,843200 MHz = 0,54253472 µs)
		P_TOUTR_S2 = 64;							// Vorteiler: 64
		
		// gemessene Werte mit RIEcon91.  Beim RIEcon90 durchschnittlich 20 Counts mehr
		if(pSerial->MatchCount	<	415)			// Soll: 384, gemessen: 384
		{	
			BRG_S2 = (unsigned char)( ( (f1_CLK_SPEED/16) / 38400) -1);
			pSerial->Baudrate	=	38400;
			UP_Quali(port, pSerial->MatchCount, 384);
			// Telegramm-Timeout Überwachung: 80Bytes*10Bits * 26,04166µs = 20833,33µs	/	0,5425µs(f2n)	=	38400 / 64 = 600 
			C_TOUTR_S2 = 1200;
		}
		else if(pSerial->MatchCount	<	800)	// Soll: 768, gemessen: 766
		{	
			BRG_S2 = (unsigned char)( ( (f1_CLK_SPEED/16) / 19200) -1);
			pSerial->Baudrate	=	19200;
			UP_Quali(port, pSerial->MatchCount, 768);
			// Telegramm-Timeout Überwachung
			C_TOUTR_S2 = 2400;
		}	 
		else if(pSerial->MatchCount	<	1600)	// Soll: 1536, gemessen: 1532
		{	
			BRG_S2 = (unsigned char)( ( (f1_CLK_SPEED/16) / 9600) -1);
			pSerial->Baudrate	=	9600;
			UP_Quali(port, pSerial->MatchCount, 1536);
			// Telegramm-Timeout Überwachung
			C_TOUTR_S2 = 4800;
		}
		else if(pSerial->MatchCount < 3100)	// Soll: 3072, gemessen: 3063	
		{	
			BRG_S2 = (unsigned char)( ( (f8_CLK_SPEED/16) / 4800) -1);
			pSerial->Baudrate = 4800;
			UP_Quali(port, pSerial->MatchCount, 3072);
			//	Telegramm-Timeout	Überwachung
			C_TOUTR_S2 = 9600;
		}
		else if(pSerial->MatchCount	<	6175)	// Soll: 6144, gemessen: 6125
		{	
			BRG_S2 = (unsigned char)( ( (f8_CLK_SPEED/16) / 2400) -1);
			pSerial->Baudrate	=	2400;
			UP_Quali(port, pSerial->MatchCount, 6144);
			// Telegramm-Timeout Überwachung
			C_TOUTR_S2 = 19200;
		}
		else											// Soll: 12288, gemessen: 12250
		{	
			BRG_S2 = (unsigned char)( ( (f8_CLK_SPEED/16) / 1200) -1);
			pSerial->Baudrate	=	1200;
			UP_Quali(port, pSerial->MatchCount, 12288);
			//	Telegramm-Timeout	Überwachung
			C_TOUTR_S2 = 38400;
		}
			
		//	Telegramm-Timeout	initialisieren
		P_TOUTS_S2	= 1;					// Start Telegramm-Timeout Timer Prescaler
		C_TOUTS_S2	= 1;					// Start Telegramm-Timeout Timer Counter
	
		//	Empfangsinit
		pSerial->status = RX_RUN;
		UP_Rx_Init(port);
	}	
	#endif
	
	//----------------------------------------------------------------------------
	
	#if (IMPLEMENT_S3 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S3)
	{
		pSerial = &serial[S3];

		// Für	Telegramm-Timeout	Überwachung: TimerB3 (Vorteiler) + TimerB4 (Teiler) 
		// SourceClock: f2n = 16 (29,491200 MHz / 16 = 1,843200 MHz = 0,54253472 µs)
		P_TOUTR_S3 = 64;							// Vorteiler: 64
		
		// gemessene Werte mit RIEcon91.  Beim RIEcon90 durchschnittlich 20 Counts mehr
		if(pSerial->MatchCount	<	415)			// Soll: 384, gemessen: 384
		{	
			BRG_S3 = (unsigned char)( ( (f1_CLK_SPEED/16) / 38400) -1);
			pSerial->Baudrate	=	38400;
			UP_Quali(port, pSerial->MatchCount, 384);
			// Telegramm-Timeout Überwachung: 80Bytes*10Bits * 26,04166µs = 20833,33µs	/	0,5425µs(f2n)	=	38400 / 64 = 600 
			C_TOUTR_S3 = 1200;
		}
		else if(pSerial->MatchCount	<	800)	// Soll: 768, gemessen: 766
		{	
			BRG_S3 = (unsigned char)( ( (f1_CLK_SPEED/16) / 19200) -1);
			pSerial->Baudrate	=	19200;
			UP_Quali(port, pSerial->MatchCount, 768);
			// Telegramm-Timeout Überwachung
			C_TOUTR_S3 = 2400;
		}	 
		else if(pSerial->MatchCount	<	1600)	// Soll: 1536, gemessen: 1532
		{	
			BRG_S3 = (unsigned char)( ( (f1_CLK_SPEED/16) / 9600) -1);
			pSerial->Baudrate	=	9600;
			UP_Quali(port, pSerial->MatchCount, 1536);
			// Telegramm-Timeout Überwachung
			C_TOUTR_S3 = 4800;
		}
		else if(pSerial->MatchCount < 3100)	// Soll: 3072, gemessen: 3063	
		{	
			BRG_S3 = (unsigned char)( ( (f8_CLK_SPEED/16) / 4800) -1);
			pSerial->Baudrate = 4800;
			UP_Quali(port, pSerial->MatchCount, 3072);
			//	Telegramm-Timeout	Überwachung
			C_TOUTR_S3 = 9600;
		}
		else if(pSerial->MatchCount	<	6175)	// Soll: 6144, gemessen: 6125
		{	
			BRG_S3 = (unsigned char)( ( (f8_CLK_SPEED/16) / 2400) -1);
			pSerial->Baudrate	=	2400;
			UP_Quali(port, pSerial->MatchCount, 6144);
			// Telegramm-Timeout Überwachung
			C_TOUTR_S3 = 19200;
		}
		else											// Soll: 12288, gemessen: 12250
		{	
			BRG_S3 = (unsigned char)( ( (f8_CLK_SPEED/16) / 1200) -1);
			pSerial->Baudrate	=	1200;
			UP_Quali(port, pSerial->MatchCount, 12288);
			//	Telegramm-Timeout	Überwachung
			C_TOUTR_S3 = 38400;
		}
			
		//	Telegramm-Timeout	initialisieren
		P_TOUTS_S3	= 1;					// Start Telegramm-Timeout Timer Prescaler
		C_TOUTS_S3	= 1;					// Start Telegramm-Timeout Timer Counter
	
		//	Empfangsinit
		pSerial->status = RX_RUN;
		UP_Rx_Init(port);
	}
	#endif	

}

//------------------------------------------------------------------------------
// Beurteilung der Qualität der Datenübertragungsleitung
// Der Startimpuls wird kürzer bei langen Leitungen.
//------------------------------------------------------------------------------
void UP_Quali(char port, int MatchCount, int MatchSoll)
{
	int NullProz;
	
	NullProz = MatchSoll / 2;
	
	serial[port].bq_temp = (int)( (long)(MatchCount - NullProz) * 100 / (MatchSoll - BDCHK_LOSS - NullProz) );
	// Kopie auf BaudQuali erfolgt in SioSx.c beim Erkennen eines Mastertelegramms
}

//------------------------------------------------------------------------------
// Initialisierung Baudcheck
//------------------------------------------------------------------------------

void UP_Init_BaudCheck(char port)
{
	serial485 *pSerial;
	
	#if (IMPLEMENT_S1 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S1)
	{
		pSerial = &serial[S1];

		pSerial->status						= BD_CHECK;
		pSerial->cstat.rs485tout	= FALSE;
		pSerial->cstat.overflow		= FALSE;
		RTS_S1							=	0;			// Empfangskanal freigeben RTS=Low


		// Baudratencheck	erfolgt durch Pulsbreitenmessung des Startbits in der Time Measurement Function (Group 1  Intelligent I/O )
		// Portanschluss:
		// INPC17 : Group1 Channel 7 (Port71)	!! gleiches Port wie RxD
		//				: zugehöriges Interrupt Request Register IIO4IR
		//				: zugehöriges Interrupt Enable  Register IIO4IE
		
		TMCR_S1 = 0x02;		// Group 1 Channel 7 Time Measurement Control Register
											// 0000 00 10
											// |||| || ||
											// |||| || Trigger Select 10: Falling edges
											// |||| no digital filter
											//  not used
											
		FSC_BIT_S1  = 1;	// Group 1 Channel 7 Time Measurement/Waveform Generation Function Select Bit = Time Measurement Function
		IFE_BIT_S1  = 1;	// Group 1 Channel 7 Function Enable Bit 
		IRLT_BIT_S1 = 1;	// Interrupt Request Select Bit im Interrupt Enable  Register setzen  auf used for interrupt
		TMR_BIT_S1  = 0;	// Interrupt Request Bit 				im Interrupt Request Register löschen
		TME_BIT_S1  = 1;	// Interrupt Enable  Bit 				im Interrupt Enable  Register setzen
		DISABLE_IRQ			
		BAUDIC_S1 = 0x05;	// Interrupt Control Register (Intelligent I/O Interrupt 4) Prio: 5
		ENABLE_IRQ
	}
	#endif
	//------------------------------------------------		
	#if (IMPLEMENT_S2 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S2)
	{
		pSerial = &serial[S2];

		pSerial->status						= BD_CHECK;
		pSerial->cstat.rs485tout	= FALSE;
		pSerial->cstat.overflow		= FALSE;
		RTS_S2							=	0;			// Empfangskanal freigeben RTS=Low


		// Baudratencheck	erfolgt durch Pulsbreitenmessung des Startbits in der Time Measurement Function (Group 1  Intelligent I/O )
		// Portanschluss:
		// INPC12 : Group1 Channel 2 (Port75)
		//				: zugehöriges Interrupt Request Register IIO2IR
		//				: zugehöriges Interrupt Enable  Register IIO2IE
		
		TMCR_S2 = 0x02;		// Group 1 Channel 2 Time Measurement Control Register
											// 0000 00 10
											// |||| || ||
											// |||| || Trigger Select 10: Falling edges
											// |||| no digital filter
											//  not used
											
		FSC_BIT_S2  = 1;	// Group 1 Channel 2 Time Measurement/Waveform Generation Function Select Bit = Time Measurement Function
		IFE_BIT_S2  = 1;	// Group 1 Channel 2 Function Enable Bit 
		IRLT_BIT_S2 = 1;	// Interrupt Request Select Bit im Interrupt Enable  Register setzen  auf used for interrupt
		TMR_BIT_S2  = 0;	// Interrupt Request Bit 				im Interrupt Request Register löschen
		TME_BIT_S2  = 1;	// Interrupt Enable  Bit 				im Interrupt Enable  Register setzen
		DISABLE_IRQ			
		BAUDIC_S2 = 0x05;	// Interrupt Control Register (Intelligent I/O Interrupt ) Prio: 5
		ENABLE_IRQ
	}	
	#endif
	//------------------------------------------------		
	#if (IMPLEMENT_S3 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S3)
	{
		pSerial = &serial[S3];

		pSerial->status						= BD_CHECK;
		pSerial->cstat.rs485tout	= FALSE;
		pSerial->cstat.overflow		= FALSE;
		RTS_S3							=	0;			// Empfangskanal freigeben RTS=Low


		// Baudratencheck	erfolgt durch Pulsbreitenmessung des Startbits in der Time Measurement Function (Group 1  Intelligent I/O )
		// Portanschluss:
		// INPC15 : Group1 Channel 5 (Port81)
		//				: zugehöriges Interrupt Request Register IIO9IR
		//				: zugehöriges Interrupt Enable  Register IIO9IE
		
		TMCR_S3 = 0x02;		// Group 1 Channel 5 Time Measurement Control Register
											// 0000 00 10
											// |||| || ||
											// |||| || Trigger Select 10: Falling edges
											// |||| no digital filter
											//  not used
											
		FSC_BIT_S3  = 1;	// Group 1 Channel 5 Time Measurement/Waveform Generation Function Select Bit = Time Measurement Function
		IFE_BIT_S3  = 1;	// Group 1 Channel 5 Function Enable Bit 
		IRLT_BIT_S3 = 1;	// Interrupt Request Select Bit im Interrupt Enable  Register setzen  auf used for interrupt
		TMR_BIT_S3  = 0;	// Interrupt Request Bit 				im Interrupt Request Register löschen
		TME_BIT_S3  = 1;	// Interrupt Enable  Bit 				im Interrupt Enable  Register setzen
		DISABLE_IRQ			
		BAUDIC_S3 = 0x05;	// Interrupt Control Register (Intelligent I/O Interrupt ) Prio: 5
		ENABLE_IRQ	
	}	
	#endif
}
	
//-----------------------------------------------------------------------------
// Empfangskanal initialisieren
//-----------------------------------------------------------------------------
void UP_Rx_Init(char port)
{
	serial485 *pSerial;
	
	#if (IMPLEMENT_S1 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S1)
	{
		pSerial = &serial[S1];
	
		pSerial->Bcount		=	0;		// Sende-	/	Empfangspuffer Index
		RTS_S1						=	0;		// Empfangskanal freigeben RTS=Low
		pSerial->cstat.psyncflag = 0;
	
		// Transmit Receive Mode Register
		MR_S1 = 0x05;				// 0 0 0 0 0 101	
												// | | | | | |||
												// | | | | | UART Mode 8 Bit
												// | | | | Internal Clock             
												// | | | 1 Stop Bit	
												// | | x	
												// | no parity
												// Rx Tx Polarity not inversed
	
		// Transmit Receive Control Register 0											
		if(pSerial->Baudrate >= 9600)
		{
		//C0_S1 = 0x10;				// 0 0 0 1 0 0 00
			C0_S1 = 0x30;				// 0 0 1 1 0 0 00
													// | | | | | | ||
													// | | | | | | f1 Takt
													// | | | | | x CTS oder RTS Funktion siehe Bit4
													// | | | | x Transmit Register Empty Flag TXEPT, Read Only, 
													// | | | Disable CTS/RTS Funktion											
													// | | TxD: 0 = CMOS Output, 1 = N-channel Open Drain Output
													// | CLK Polarität
													// LSB first
		}
		else
			C0_S1 = 0x31;				// f8 Takt								 
	
																						
	
		pSerial->Cond = 0;	// Empfangszustand im	Rx_S1-Interrupt	 
		TB_S1 = RB_S1;			// SIO entleeren	(Kopieren auf Sendepuffer)
		TB_S1 = 0;					// Sendepuffer löschen
	
		DISABLE_IRQ			
		RIC_S1 = 0x04;			// Enable Receive Interrupt Prio: 4			
		ENABLE_IRQ	
	
		// Transmit Receive Control Register 1											
		// Fehlerstatus im Empfangspuffer löschen durch Rücksetzen des Receive Enable Bit
		C1_S1 = 0x00;				// 0 0 0 0 0 0 0 0
												// | | | | | | | |
												// | | | | | | | Transmit Enable Bit TE
												// | | | | | | Transmit Buffer Empty Flag TI (nach Reset = 1), Read Only
												// | | | | | Receive Enable Bit RE
												// | | | | x Receive Complete Flag, Read Only
												// | | | Auswahl Transmit Interrupt: 0= no Data in TB (TI = 1), 1= Transmission complete (TXEPT = 1)
												// | | Continious Receive Mode Enable Bit: 0= Disable to be to be entered
												// | Data Logic: 0= not inversed
												// nur für Spezialmode 3 und 5
	
		RE_BIT_S1 = 1;		// Empfang erlauben 																						
	
		if(pSerial->Mode ==	MASTER)	// Klasse	IFU
		{										 
			// Timeout Überwachung bis zum Empfang des ersten Zeichens	ca. 666ms								 
			P_TOUTS_S1	= 0;								// Stop	 Telegramm-Timeout	Timer Prescaler
			C_TOUTS_S1	= 0;								// Stop	 Telegramm-Timeout	Timer Counter
			P_TOUTR_S1	=	64 * tout_ext_S1;	// Load  Prescaler
			C_TOUTR_S1	=	19200;						// Load  Counter
			P_TOUTS_S1	= 1;								// Start Telegramm-Timeout Timer Prescaler
			C_TOUTS_S1	= 1;								// Start Telegramm-Timeout Timer Counter
		}
	}
	#endif
	//------------------------------------------------		
	#if (IMPLEMENT_S2 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S2)
	{
		pSerial = &serial[S2];

		pSerial->Bcount	=	0;		// Sende-	/	Empfangspuffer Index
		RTS_S2					=	0;		// Empfangskanal freigeben RTS=Low
		pSerial->cstat.psyncflag = 0;
	
	
		// Transmit Receive Mode Register
		MR_S2 = 0x05;				// 0 0 0 0 0 101	
												// | | | | | |||
												// | | | | | UART Mode 8 Bit
												// | | | | Internal Clock             
												// | | | 1 Stop Bit	
												// | | x	
												// | no parity
												// Rx Tx Polarity not inversed
	
		// Transmit Receive Control Register 0											
		if(pSerial->Baudrate >= 9600)
		{
			C0_S2 = 0x10;			// 0 0 0 1 0 0 00
												// | | | | | | ||
												// | | | | | | f1 Takt
												// | | | | | x CTS oder RTS Funktion siehe Bit4
												// | | | | x Transmit Register Empty Flag TXEPT, Read Only, 
												// | | | Disable CTS/RTS Funktion											
												// | | TxD: 0 = CMOS Output, 1 = N-channel Open Drain Output
												// | CLK Polarität
												// LSB first
		}
		else
			C0_S2 = 0x11;			// f8 Takt								 
	
																						
		RIRLT_BIT_S2 = 1;		// Interrupt Request Select Bit im Interrupt Enable  Register setzen  auf used for interrupt
		RIR_BIT_S2   = 0;		// Interrupt Request Bit 				im Interrupt Request Register löschen
		RIE_BIT_S2   = 1;		// Interrupt Enable  Bit 				im Interrupt Enable  Register setzen
	
		pSerial->Cond = 0;				// Empfangszustand im	Rx_S2-Interrupt	 
		TB_S2 = RB_S2;			// SIO entleeren	(Kopieren auf Sendepuffer)
		TB_S2 = 0;					// Sendepuffer löschen
	
		DISABLE_IRQ			
		RIC_S2 = 0x04;			// Enable Receive Interrupt Prio: 4			
		ENABLE_IRQ	
	
		// Transmit Receive Control Register 1											
		// Fehlerstatus im Empfangspuffer löschen durch Rücksetzen des Receive Enable Bit
		C1_S2 = 0x00;				// 0 0 0 0 0 0 0 0
												// | | | | | | | |
												// | | | | | | | Transmit Enable Bit TE
												// | | | | | | Transmit Buffer Empty Flag TI (nach Reset = 1), Read Only
												// | | | | | Receive Enable Bit RE
												// | | | | x Receive Complete Flag, Read Only
												// | | | Auswahl Transmit Interrupt: 0= no Data in TB (TI = 1), 1= Transmission complete (TXEPT = 1)
												// | | Continious Receive Mode Enable Bit: 0= Disable to be to be entered
												// | Data Logic: 0= not inversed
												// nur für Spezialmode 3 und 5
	
		RE_BIT_S2 = 1;			// Empfang erlauben 																						
	
		if(pSerial->Mode ==	MASTER)	// Klasse	IFU
		{										 
			// Timeout Überwachung bis zum Empfang des ersten Zeichens	ca. 666ms								 
			P_TOUTS_S2	= 0;								// Stop	 Telegramm-Timeout	Timer Prescaler
			C_TOUTS_S2	= 0;								// Stop	 Telegramm-Timeout	Timer Counter
			P_TOUTR_S2	=	64 * tout_ext_S2;	// Load  Prescaler
			C_TOUTR_S2	=	19200;						// Load  Counter
			P_TOUTS_S2	= 1;								// Start Telegramm-Timeout Timer Prescaler
			C_TOUTS_S2	= 1;								// Start Telegramm-Timeout Timer Counter
		}
	}	
	#endif
	//------------------------------------------------		
	#if (IMPLEMENT_S3 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S3)
	{
		pSerial = &serial[S3];
		
		pSerial->Bcount	=	0;		// Sende-	/	Empfangspuffer Index
		RTS_S3					=	0;		// Empfangskanal freigeben RTS=Low
		pSerial->cstat.psyncflag = 0;
	
	
		// Transmit Receive Mode Register
		MR_S3 = 0x05;				// 0 0 0 0 0 101	
												// | | | | | |||
												// | | | | | UART Mode 8 Bit
												// | | | | Internal Clock             
												// | | | 1 Stop Bit	
												// | | x	
												// | no parity
												// Rx Tx Polarity not inversed
	
		// Transmit Receive Control Register 0											
		if(pSerial->Baudrate >= 9600)
		{
			C0_S3 = 0x10;				// 0 0 0 1 0 0 00
													// | | | | | | ||
													// | | | | | | f1 Takt
													// | | | | | x CTS oder RTS Funktion siehe Bit4
													// | | | | x Transmit Register Empty Flag TXEPT, Read Only, 
													// | | | Disable CTS/RTS Funktion											
													// | | TxD: 0 = CMOS Output, 1 = N-channel Open Drain Output
													// | CLK Polarität
													// LSB first
		}
		else
			C0_S3 = 0x11;				// f8 Takt								 
	
																						
		pSerial->Cond = 0;				// Empfangszustand im	Rx_S3-Interrupt	 
		TB_S3 = RB_S3;			// SIO entleeren	(Kopieren auf Sendepuffer)
		TB_S3 = 0;					// Sendepuffer löschen
	
		DISABLE_IRQ			
		RIC_S3 = 0x04;			// Enable Receive Interrupt Prio: 4			
		ENABLE_IRQ	
	
		// Transmit Receive Control Register 1											
		// Fehlerstatus im Empfangspuffer löschen durch Rücksetzen des Receive Enable Bit
		C1_S3 = 0x00;				// 0 0 0 0 0 0 0 0
												// | | | | | | | |
												// | | | | | | | Transmit Enable Bit TE
												// | | | | | | Transmit Buffer Empty Flag TI (nach Reset = 1), Read Only
												// | | | | | Receive Enable Bit RE
												// | | | | x Receive Complete Flag, Read Only
												// | | | Auswahl Transmit Interrupt: 0= no Data in TB (TI = 1), 1= Transmission complete (TXEPT = 1)
												// | | Continious Receive Mode Enable Bit: 0= Disable to be to be entered
												// | Data Logic: 0= not inversed
												// nur für Spezialmode 3 und 5
	
		RE_BIT_S3 = 1;		// Empfang erlauben 																						
	
		if(pSerial->Mode ==	MASTER)	// Klasse	IFU
		{										 
			// Timeout Überwachung bis zum Empfang des ersten Zeichens	ca. 666ms								 
			P_TOUTS_S3	= 0;								// Stop	 Telegramm-Timeout	Timer Prescaler
			C_TOUTS_S3	= 0;								// Stop	 Telegramm-Timeout	Timer Counter
			P_TOUTR_S3	=	64 * tout_ext_S3;	// Load  Prescaler
			C_TOUTR_S3	=	19200;						// Load  Counter
			P_TOUTS_S3	= 1;								// Start Telegramm-Timeout Timer Prescaler
			C_TOUTS_S3	= 1;								// Start Telegramm-Timeout Timer Counter
		}
	}
	#endif	
}

//-----------------------------------------------------------------------------
// Empfangskanal sperren und Timer entschärfen
//-----------------------------------------------------------------------------
void UP_Rx_DeInit(char port)
{
	#if (IMPLEMENT_S1 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S1)
	{
		RE_BIT_S1 = 0;						// Empfang sperren, löschen der Fehlerbits
		DISABLE_IRQ			
		RIC_S1 = 0x00;						// Disable Receive Interrupt 
		ENABLE_IRQ	
	
		P_TOUTS_S1	= 0;					// Stop	 Telegramm-Timeout	Timer Prescaler
		C_TOUTS_S1	= 0;					// Stop	 Telegramm-Timeout	Timer Counter

		serial[port].cstat.rs485run = FALSE;	// Empfang beendet
	}
	#endif

	#if (IMPLEMENT_S2 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S2)
	{
		RE_BIT_S2 	= 0;					// Empfang sperren, löschen der Fehlerbits
		RIE_BIT_S2  = 0;					// Interrupt Enable  Bit 	löschen
		
		DISABLE_IRQ			
		RIC_S2 = 0x00;						// Disable Receive Interrupt 
		ENABLE_IRQ	
	
		P_TOUTS_S2	= 0;					// Stop	 Telegramm-Timeout	Timer Prescaler
		C_TOUTS_S2	= 0;					// Stop	 Telegramm-Timeout	Timer Counter
	
		serial[port].cstat.rs485run = FALSE;	// Empfang beendet
	}
	#endif

	#if (IMPLEMENT_S3 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S3)
	{
		RE_BIT_S3 = 0;						// Empfang sperren, löschen der Fehlerbits
		DISABLE_IRQ			
		RIC_S3 = 0x00;						// Disable Receive Interrupt 
		ENABLE_IRQ	
	
		P_TOUTS_S3	= 0;					// Stop	 Telegramm-Timeout	Timer Prescaler
		C_TOUTS_S3	= 0;					// Stop	 Telegramm-Timeout	Timer Counter
	
		serial[port].cstat.rs485run = FALSE;	// Empfang beendet
	}
	#endif	
		
}
	
//-----------------------------------------------------------------------------
// Ende	der	Empfangsprozedur 
// Endecode	ec:	0	=	Ok,	 1 = Intervall Time	Out	 2 = Clock Time	Out		3 = Error
//-----------------------------------------------------------------------------
void UP_Rx_End(char port, char	ec)
{
	serial485 *pSerial = NULL;
	
	switch(port)
	{ 
		case S1:
			pSerial = &serial[S1];
			break;
		case S2:
			pSerial = &serial[S2];
			break;
		case S3:
			pSerial = &serial[S3];
			break;
	}		
				
	
	if(pSerial->Mode	== MASTER)						// Klasse	IFU
	{
		pSerial->status	=	0;
		UP_Rx_DeInit(port);
	}
	else															// Klasse	UFU
	{	switch(ec)
		{
			case	0:											// Normal	Ende	
				UP_Rx_DeInit(port); 
				pSerial->status = RX_END;		 
				break;
			case	1:											// Intervall Interrupt (15,6 ms)	
				pSerial->Bcount	= 0;							// Empfangspuffer Index zurücksetzen	
				pSerial->Cond		= 0;							// Empfangsstatus zurücksetzen	
				pSerial->cstat.rs485tout	=	FALSE;// Time	Out	löschen
				break;
			case	2:											// Clock Interrupt (0,5	s)	
				UP_Rx_DeInit(port);
				pSerial->status = RX_END;		 
				break;
			case	3:											// Abbruch wegen Fehlererkennung
				UP_Rx_DeInit(port);
				pSerial->status = RX_ERROR;		 
				break;
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------
// Interrupt Baudratenmessung
// Einsprung von UserHandlerS1, UserHandlerS2 oder UserHandlerS3
//---------------------------------------------------------------
void U_Test_ISR_BaudCheck(char port)
{
	serial485 *pSerial;
	
	#if (IMPLEMENT_S1 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S1)
	{
		pSerial = &serial[S1];

		TMR_BIT_S1 = 0;																			// Interrupt Request Bit löschen
	                            													
		if(CTS1_BIT_S1 == 1)																// Falling Edge ?
		{                         													
			pSerial->match_fall = TMR_S1;														// Time Measurement Register (Zählwert speichern)
			TMCR_S1 = 0x01;																		// Trigger Select 01: Rising edges
		}                         													
		else																								// Rising Edge
		{	                        													
			TME_BIT_S1 = 0;																		// Interrupt Enable Bit löschen
			pSerial->match_rise = TMR_S1;														// Time Measurement Register (Zählwert speichern)
			IFE_BIT_S1 = 0;																		// Function Enable Bit löschen
			if(pSerial->match_fall <= pSerial->match_rise)
				pSerial->MatchCount = pSerial->match_rise - pSerial->match_fall;
			else
				pSerial->MatchCount = 65535 - pSerial->match_fall + pSerial->match_rise;
		
			UP_CheckUp_Baudrate(port);												// Baudrate	einstellen und Empfang initialisieren	
		}
	}
	#endif
			
	#if (IMPLEMENT_S2 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S2)
	{
		pSerial = &serial[S2];

		TMR_BIT_S2 = 0;																			// Interrupt Request Bit löschen
	                            													
		if(CTS1_BIT_S2 == 1)																// Falling Edge ?
		{                         													
			pSerial->match_fall = TMR_S2;														// Time Measurement Register (Zählwert speichern)
			TMCR_S2 = 0x01;																		// Trigger Select 01: Rising edges
		}                         													
		else																								// Rising Edge
		{	                        													
			TME_BIT_S2 = 0;																		// Interrupt Enable Bit löschen
			pSerial->match_rise = TMR_S2;														// Time Measurement Register (Zählwert speichern)
			IFE_BIT_S2 = 0;																		// Function Enable Bit löschen
			if(pSerial->match_fall <= pSerial->match_rise)
				pSerial->MatchCount = pSerial->match_rise - pSerial->match_fall;
			else
				pSerial->MatchCount = 65535 - pSerial->match_fall + pSerial->match_rise;
		
			UP_CheckUp_Baudrate(port);												// Baudrate	einstellen und Empfang initialisieren	
		}
	}
	#endif
			
	#if (IMPLEMENT_S3 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S3)
	{
		pSerial = &serial[S3];

		// Funktion_S3: 1  GBUS
		TMR_BIT_S3 = 0;																			// Interrupt Request Bit löschen
                            													
		if(CTS1_BIT_S3 == 1)																// Falling Edge ?
		{                         													
			pSerial->match_fall = TMR_S3;														// Time Measurement Register (Zählwert speichern)
			TMCR_S3 = 0x01;																		// Trigger Select 01: Rising edges
		}                         													
		else																								// Rising Edge
		{	                        													
			TME_BIT_S3 = 0;																		// Interrupt Enable Bit löschen
			pSerial->match_rise = TMR_S3;														// Time Measurement Register (Zählwert speichern)
			IFE_BIT_S3 = 0;																		// Function Enable Bit löschen
			if(pSerial->match_fall <= pSerial->match_rise)
				pSerial->MatchCount = pSerial->match_rise - pSerial->match_fall;
			else
				pSerial->MatchCount = 65535 - pSerial->match_fall + pSerial->match_rise;
	
			UP_CheckUp_Baudrate(port);												// Baudrate	einstellen und Empfang initialisieren	
		}
	}	
	#endif
}

//----------------------------------------------------------
// Timer Interrupt für Telegramm-Timeout
// Einsprung von UserHandler
//----------------------------------------------------------
void U_Test_ISR_Tel_Tout(char port)
{
	serial[port].cstat.rs485tout	=	TRUE;
	UP_Rx_End(port, 2);
}
	
//----------------------------------------------------------
// Sende Interrupt
// Einsprung von UserHandler
//----------------------------------------------------------
void U_Test_ISR_Tx_Int(char port)
{
	char bsend;
	serial485 *pSerial;
	
	#if (IMPLEMENT_S1 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S1)
	{
		pSerial = &serial[S1];

		if(pSerial->Bcount < pSerial->Leng)
		{
			if( (pSerial->Bcount	>	1) &&	(pSerial->Bcount < pSerial->Leng - 3)	)	
			{	// Datenbereich
				if(pSerial->cstat.psyncflag	== TRUE)
				{
					TB_S1								=	PSYNC;		 //	PSYNC-Verdopplung	senden
					pSerial->cstat.psyncflag	=	FALSE;
				}
				else
				{
					bsend = TxBuf_S1[pSerial->Bcount++];
					if(bsend	== PSYNC)
						pSerial->cstat.psyncflag = TRUE;
					
					TB_S1	=	bsend;						 			//	Byte senden	
				}
			}
			else
			{	 //	Rahmenbereich
				TB_S1	=	TxBuf_S1[pSerial->Bcount++];		// Byte	senden
			}
		}
		else	// Ende	 
		{
			TE_BIT_S1		= 0;							// Sendeerlaubnis sperren
			TIC_S1 			= 0x00;						// Disable	Interrupt	Request
			pSerial->status	=	TX_END;			// Status der Datenübertragung	im Task	'SlaveS1'
			RTS_S1			=	0;							// Sendekanal sperren
			
			if(pSerial->Mode	== MASTER)			//	Klasse IFU
			{
				// Empfang	initialisieren
				if(pSerial->cstat.broadcast == FALSE) {
					pSerial->status = RX_RUN;
					UP_Rx_Init(port);
				}
				else {
					UP_Rx_DeInit(port);
				}						
			}
		}
	}
	#endif
			
	#if (IMPLEMENT_S2 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S2)
	{
		pSerial = &serial[S2];

		if(TIR_BIT_S2 == 1)											// Sendeinterrupt ?
		{
			TIR_BIT_S2	= 0;											// Interrupt Request Bit löschen
			
			if(pSerial->Bcount < pSerial->Leng)
			{
				if( (pSerial->Bcount	>	1) &&	(pSerial->Bcount < pSerial->Leng - 3)	)	
				{	// Datenbereich
					if(pSerial->cstat.psyncflag	== TRUE)
					{
						TB_S2								=	PSYNC;		//	PSYNC-Verdopplung	senden
						pSerial->cstat.psyncflag	=	FALSE;
					}
					else
					{
						bsend = TxBuf_S2[pSerial->Bcount++];
						if(bsend	== PSYNC)
							pSerial->cstat.psyncflag = TRUE;
						
						TB_S2	=	bsend;						 			//	Byte senden	
					}
				}
				else
				{	 //	Rahmenbereich
					TB_S2	=	TxBuf_S2[pSerial->Bcount++];		// Byte	senden
				}
			}
			else	// Ende	 
			{
				TE_BIT_S2		= 0;							// Sendeerlaubnis sperren
				TIE_BIT_S2	= 0;							// Interrupt Enable  Bit löschen
				TIC_S2 			= 0x00;						// Disable	Interrupt	Request
				pSerial->status	=	TX_END;			// Status der Datenübertragung	im Task	'SlaveS2'
				RTS_S2			=	0;							// Sendekanal sperren
				
				if(pSerial->Mode	== MASTER)			//	Klasse IFU
				{
					// Empfang	initialisieren
					if(pSerial->cstat.broadcast == FALSE) {
						pSerial->status = RX_RUN;
						UP_Rx_Init(port);
					}
					else {
						UP_Rx_DeInit(port);
					}						
				}
			}
		}
		else
		{
			if(TMR_BIT_S2 == 1)							// BaudCheck-Interrupt ?
				U_Test_ISR_BaudCheck(port);		// wird als Unterprogramm aufgerufen
		}		
	}
	#endif
			
	#if (IMPLEMENT_S3 & GBUS1_IMPL) == GBUS1_IMPL
	if(port == S3)
	{
		pSerial = &serial[S3];

		if(pSerial->Bcount < pSerial->Leng)
		{
			if( (pSerial->Bcount	>	1) &&	(pSerial->Bcount < pSerial->Leng - 3)	)	
			{	// Datenbereich
				if(pSerial->cstat.psyncflag	== TRUE)
				{
					TB_S3								=	PSYNC;		 //	PSYNC-Verdopplung	senden
					pSerial->cstat.psyncflag	=	FALSE;
				}
				else
				{
					bsend = TxBuf_S3[pSerial->Bcount++];
					if(bsend	== PSYNC)
						pSerial->cstat.psyncflag = TRUE;
				
					TB_S3	=	bsend;						 			//	Byte senden	
				}
			}
			else
			{	 //	Rahmenbereich
				TB_S3	=	TxBuf_S3[pSerial->Bcount++];		// Byte	senden
			}
		}
		else	// Ende	 
		{
			TE_BIT_S3		= 0;							// Sendeerlaubnis sperren
			TIC_S3 			= 0x00;						// Disable	Interrupt	Request
			pSerial->status	=	TX_END;			// Status der Datenübertragung	im Task	'SlaveS2'
			RTS_S3			=	0;							// Sendekanal sperren
		
			if(pSerial->Mode	== MASTER)			//	Klasse IFU
			{
				// Empfang	initialisieren
				if(pSerial->cstat.broadcast == FALSE) {
					pSerial->status = RX_RUN;
					UP_Rx_Init(port);
				}
				else {
					UP_Rx_DeInit(port);
				}						
			}
		}
	}	
	#endif
}

//--------------------------------------------------------------
// Empfangs Interrupt
// Einsprung von UserHandler
//--------------------------------------------------------------
void U_Test_ISR_Rx_Int(char port)
{
	
	char rbyte, abort;
	char *pRxBuf;
	serial485 *pSerial;

	rbyte = 0;
	abort = 0;
	pSerial = NULL;
	pRxBuf = NULL;
	switch(port)
	{
		case S1:
			pRxBuf  = RxBuf_S1;
			pSerial = &serial[S1];
			rbyte 	= RB_S1;										// Empfangsbyte	abholen
			if(SUM_BIT_S1 == 1)									// Summenfehlerbit gesetzt ?
			{	if(pSerial->Mode ==	MASTER)
				{	RE_BIT_S1 	= 0;								// Empfang sperren, löschen der Fehlerbits
					RE_BIT_S1 	= 1;								// Empfang freigeben
				}
				else
				{	UP_Rx_End(port, 3);							// Ende	der	Empfangsprozedur
					abort = 1;
				}	
			}		
			break;
			
		case S2:
			pRxBuf  = RxBuf_S2;
			pSerial = &serial[S2];
			rbyte 	= RB_S2;										// Empfangsbyte	abholen
			RIR_BIT_S2   = 0;										// Interrupt Request Bit löschen
			if(SUM_BIT_S2 == 1)									// Summenfehlerbit gesetzt ?
			{	if(pSerial->Mode ==	MASTER)
				{	RE_BIT_S2 	= 0;								// Empfang sperren, löschen der Fehlerbits
					RE_BIT_S2 	= 1;								// Empfang freigeben
				}
				else
				{	UP_Rx_End(port, 3);							// Ende	der	Empfangsprozedur
					abort = 1;
				}	
			}		
			break;
			
		case S3:
			pRxBuf  = RxBuf_S3;
			pSerial = &serial[S3];
			rbyte 	= RB_S3;										// Empfangsbyte	abholen
			if(SUM_BIT_S3 == 1)									// Summenfehlerbit gesetzt ?
			{	if(pSerial->Mode ==	MASTER)
				{	RE_BIT_S3 	= 0;								// Empfang sperren, löschen der Fehlerbits
					RE_BIT_S3 	= 1;								// Empfang freigeben
				}
				else
				{	UP_Rx_End(port, 3);							// Ende	der	Empfangsprozedur
					abort = 1;
				}	
			}		
			break;
	}
	
	if(abort == 0)
	{
		pRxBuf[pSerial->Bcount++] = rbyte;		// Empfangspuffer	laden
		switch(pSerial->Cond)
		{
			case	0:												// Warte auf PSYNC
				//	PSYNC	wird mit dem nächsten	empfangenen	Zeichen	überschrieben
				pSerial->Bcount	=	0;								// Zeiger	auf	Empfangspuffer rückstellen
				if(rbyte	== PSYNC)
				{	pSerial->Cond	+= 1;
					if(pSerial->Mode ==	MASTER)
					{	
						switch(port)
						{
							case S1:
								P_TOUTS_S1	= 0;								// Stop	 Telegramm-Timeout	Timer Prescaler
								C_TOUTS_S1	= 0;								// Stop	 Telegramm-Timeout	Timer Counter
								P_TOUTR_S1	=	pSerial->toutl;		// Load  Prescaler
								C_TOUTR_S1	=	pSerial->touth;		// Load  Counter
								P_TOUTS_S1	= 1;								// Start Telegramm-Timeout Timer Prescaler
								C_TOUTS_S1	= 1;								// Start Telegramm-Timeout Timer Counter
								break;
							case S2:
								P_TOUTS_S2	= 0;							
								C_TOUTS_S2	= 0;							
								P_TOUTR_S2	=	pSerial->toutl;	
								C_TOUTR_S2	=	pSerial->touth;	
								P_TOUTS_S2	= 1;							
								C_TOUTS_S2	= 1;
								break;
							case S3:
								P_TOUTS_S3	= 0;				
								C_TOUTS_S3	= 0;				
								P_TOUTR_S3	=	pSerial->toutl;	
								C_TOUTR_S3	=	pSerial->touth;	
								P_TOUTS_S3	= 1;				
								C_TOUTS_S3	= 1;
								break;
						}						
					}
				}
				else	// kein PSYNC als 1.Zeichen
				{	if( (pSerial->Mode ==	SLAVE) && (rbyte != 0xFF) )		// Ausnahme 0xFF (WRE100 ETR100)
						UP_Rx_End(port, 3);				// Ende	der	Empfangsprozedur wegen fehlendem PSYNC
				}
				break;
	
			case	1:												//	Erwarte	STX	oder STX8
				if(pSerial->Mode	== MASTER)	//	Klasse IFU
				{
					// PSYNC und STX8	werden mit dem nächsten	empfangenen	Zeichen	überschrieben	
					pSerial->Bcount	 = 0;				//	Zeiger auf Empfangspuffer	rückstellen	
					if(rbyte ==	STX8)
						pSerial->Cond +=	1;
					else
						pSerial->Cond = 0;				// Nochmal von vorne
				}
				else													//	Klasse UFU
				{	// Erstes	Zeichen, welches nicht mit dem nächsten	empfangenen	Zeichen	überschrieben	wird	
					if((rbyte	== STX)	|| (rbyte	== STX8))	// Jedes	Telegramm(Master oder	Slave)
					{																			// wird	komplett eingezogen		
						pSerial->stx = rbyte;											// Merker für Telegrammlängenberechnung 
						pSerial->Cond +=	1;											
						pSerial->bcc	=	(char)(PSYNC ^ rbyte);		// PSYNC ^ rbyte
						// Baudqualität anzeigen beim Erkennen eines Mastertelegramms
						if(rbyte	== STX)
							pSerial->BaudQuali = pSerial->bq_temp;
					}
					else
						UP_Rx_End(port, 3);				// Ende	der	Empfangsprozedur wegen falscher Startkennung
				}
				break;
				
			case	2:												//	Erwarte	Leitadresse	"0"	oder Slaveadresse
				if(pSerial->Mode	== MASTER)	//	Klasse IFU
				{
					pSerial->Bcount =	0;
					if(rbyte	== 0)								
					{	pSerial->Cond	+= 1;
						pSerial->bcc	=	0x28;						// PSYNC ^ STX8	^	00			 
					}	 
					else
						pSerial->Cond	=	0;							// Nochmal von vorne
				}
				else													//	Klasse UFU
				{
					// Slaveadresse	wird erst	im Task	SlaveEtr ausgewertet
					pSerial->bcc	 ^=	rbyte;
					pSerial->Cond	+= 1;
				}
				break;
	
			case	3:														// Längenbyte: Anzahl	der	Daten
				pSerial->Cond +=	1;
				pSerial->bcc	^= rbyte;
				if(pSerial->Mode	== MASTER)						// Klasse	IFU
					pSerial->Leng	=	(char)(rbyte + 5);		// (Nur	Daten) + Slaveadr	+Rcode +BCC	+PSYNC +ETX
				else
				{																	// Klasse	UFU
					if(pSerial->stx == STX)
						pSerial->Leng	=	(char)(rbyte + 3);	// Mastertelegramm empfangen (Kommando+Daten)	+	BCC	+PSYNC +ETX
					else
						pSerial->Leng	=	(char)(rbyte + 5);	// Fremdes Slavetelegr. (Nur	Daten) + Slaveadr	+Rcode +BCC	+PSYNC +ETX
	
					pSerial->TLeng = pSerial->Leng;							// Merken	für	Task SlaveEtr
				}
				break;
	
			default:														// restliche Bytes
				pSerial->Cond +=	1;
				if(pSerial->Leng	>	2)
				{
					pSerial->bcc ^=	rbyte;								// Check	über alle	Bytes	(inclusive Empfangs-BCC)
																					// bcc wird	0, wenn	alles	ok
					if(pSerial->Leng !=	3)								// wenn nicht Empfangs-BCC,
					{	if(rbyte	== PSYNC)						// PSYNC-Verdopplung behandeln
						{	if(pSerial->cstat.psyncflag	== 0)
							{	pSerial->cstat.psyncflag	=	1;
								pSerial->Bcount -=	1;					// Zeiger	auf	Puffer rückstellen
							}
							else
							{	pSerial->cstat.psyncflag	=	0;
								pSerial->Leng	 -=	1;
							}
						}
						else
							pSerial->Leng	-= 1;
					}
					else
						pSerial->Leng -=	1;
				}
				else
					pSerial->Leng	-= 1;									//	hier wird	nur	noch PSYNC,	ETX	erwartet
	
				if(pSerial->Leng	== 0)
					UP_Rx_End(port, 0);									// Ende	der	Empfangsprozedur
				else
				{
					if(pSerial->Bcount >=	S1_RXSIZE - 1)
					{	pSerial->Bcount	 =	S1_RXSIZE - 2;	//	bei	Überlänge	Pointer	rücksetzen
						pSerial->cstat.overflow = 1;				//	Überlauf merken
					}
				}
				break;
		}
	}	
}

#endif // #if ( ((IMPLEMENT_S1 & GBUS1_IMPL) == GBUS1_IMPL) || ((IMPLEMENT_S2 & GBUS1_IMPL) == GBUS1_IMPL) || ((IMPLEMENT_S3 & GBUS1_IMPL) == GBUS1_IMPL) )
