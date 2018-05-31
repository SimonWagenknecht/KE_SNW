/////////////////////////////////////////////////////////////////////////////////
//                       ---- WILO-MODBUS Handler ----                         //
//	Aufruf erfolgt vom HandlerS1 und HandlerS3 organisiert vom UserHandler     //
//  Verwendet den Interupt-Service von U_ModBusSio !  Grundtakt = 50ms         //
/////////////////////////////////////////////////////////////////////////////////
 
#include "sfr32C87.h"
#include <string.h>
#include "sio.h"
#include "timer.h"
#include "struct.h"
#include "ustruct.h"
#include "defins.h"
#include "sramext.h"
#include "uramext.h"
#include "projdef.h"
#include "uconstext.h"
#include "standdef.h"
 
#if ( WILO )

// Status	der	Datenübertragung MODBUS 												 
#define EXEPTION				0x05
#define	FUNKTION_ERROR	0x06
#define	LENGT_ERROR			0x08
#define	RX_TIMEOUT			0x10
#define	RX_BUF_READY		0x80

// MODBUS-Functioncodes
// reads
#define READ_COIL_STATE      		1    // Einzel-Bit-Werte aus Ausgangsregistern Lesen
#define READ_DISCRETE_INP    		2    // Einzel-Bit-Werte aus Discreten Inputregistern lesen (read inputs)
#define READ_HOLDING_REGS    		3		 // 16Bit-Werte aus I/O-Registern lesen
#define READ_INPUT_REGS      		4    // 16Bit-Werte aus Input-Registern lesen (Meßwerte,nicht beschreibbar) 

// writes
#define WRITE_SINGLE_COIL    		5    // Einzel-Bit-Werte in Ausgangsregister schreiben (wie Relais-Ausgänge, Einzelne Werte) 
#define WRITE_SINGLE_HOLDING 		6    // 16Bit-Werte in I/O-Register schreiben (Einzelne Werte)
#define WRITE_MULTIPLE_COILS 		15	 // Einzel-Bit-Werte in Ausgangsregister schreiben (wie Relais-Ausgänge, mehrere Bits hintereinander)
#define WRITE_MULTIPLE_HOLDINGS 16   // 16Bit-Werte in I/O-Register schreiben (Mehrere Werte hintereinander)

// Extended registers
#define READ_EXTENDED        		0x14
#define WRITE_EXTENDED       		0x15

// Diagnostics
#define READ_DIAGNOSTIC      		8    // Mainfunktion
#define READ_SLAVE_ID						17
#define READ_DEVICE_ID					43

//Diagnostic-Sub-Funktions
#define GET_LOOP				0			// Loopback
#define CLEAR_DIAG_REGS	10		// Clear Counters and Diagnostic Register
#define GET_BMC					11		// Return Bus Message Count
#define GET_BCEC				12		// Return Bus Communication Error Count
#define GET_BEEC				13		// Return Bus Exception Error Count
#define GET_SMC					14		// Return Slave Message Count
#define GET_SNRC				11		// Return Slave No Response Count
#define GET_SNC					11		// Return Slave NAK Count

// modbus error codes
#define EXCEPTION_ILLEGALFUNC   	0x01
#define EXCEPTION_ILLEGALADDR   	0x02
#define EXCEPTION_ILLEGALVALUE  	0x03
#define EXCEPTION_UNDEF_STATE   	0x04
#define EXCEPTION_BUSY          	0x06
#define EXCEPTION_GATEWAY_TARGET  0x0B

#define EXCEPTION_MASK          0x80

#define	SM_DELAY  ( 40 / MODB_SLAVE_MAX )

//////////////////////////////////////////////////////
//                                                  //
// 		    WILO-ModBus-Projekt-Definitionen          // 
//		    ================================          // 
//	                                                // 
//////////////////////////////////////////////////////
 

// Konvertierungs-Typen
#define SKIP			0	   // Skip, nicht verwendete Int-Werte überspringen 
#define LOWBYTE 	1    // Byte-Werte in LowByte
#define HIGHBYTE	2    // Byte-Werte in HighByte
#define REG1			3 	 // Modbus-Register nomal, Int-Werte Auflösung 1
#define DISCRET		4 	 // Bit-Werte 1:1 aus RxBuffer ablegen (Discrete Inputs)
#define REG01			32	 // Modbus-Register nomal, Int-Werte Auflösung 0,1
#define REG10			33	 // Modbus-Register nomal, Int-Werte Auflösung 10

// Datenpunkt-Adresstabelle Holding Registers			
//ulsch
//const int hold_adr_table[] = { 1, 40, 42, 44, 45, 46, 47 };
const int hold_adr_table[] = { 1, 40, 42, 300, 44, 45, 46, 47 };
// #define HOLD_ADR_TABLENG 7 // Wird nur für PU-interne Regelung benötigt
//#define HOLDING_REG_CNT 3
// Definition nach userdef.h verschoben
//#define HOLDING_REG_CNT 4

/////////////// Input registers (Single Pump)///////////////
const char Convert1_table[] =	{  	 
// 					address		Description										Unit
REG01,		//	1				Actual Differential Pressure	0.1 m WS
REG01,		//	2				Flow Rate											0.1 m³/h
REG1,			//	3				Power Consumption							1 kWh
REG1,			//	4				Power Rating									1 W
REG10,		//	5				Operation Hours								10 h
REG01,		//	6				Mains Current									0.1 A
REG1,			//	7				Speed													1 rpm
REG01,		//	8				Medium Temperature						0.1 K
SKIP,			//	9				Operating Hours DP						10 h	(nur bei Doppelpumpe)
LOWBYTE,	//	10			Current Operation Mode				See Table 5.3.
};
#define RQ1_FIRST_OFFSET1 	1
#define RQ1_COUNTER1	10

const char Convert2_table[] =	{
LOWBYTE,	//	16			Pump Module										See Table 5.5.
LOWBYTE,	//	17			Pump Type											See Table 5.6.
REG1,			//	18			Max Speed											1 rpm
REG1,			//	19			Min Speed											1 rpm
};
#define RQ2_FIRST_OFFSET1 	16
#define RQ2_COUNTER1 	4
// Diese erzeugen, wenn nichtvohanden, eine Exception, daher nicht verwendbar!
//I,	//	24					Max Flow Rate									0.1 m³/h
//I,	//	25					Min Flow Rate									0.1 m³/h

const char Convert3_table[] =	{
REG1,	//	26					Supported Errors							See Table 5.9.
REG1,	//	27					Supported Service Messages		See Table 5.7.
REG1,	//	28					Max Power Rating							1 W
// S,  //  29 erzeugen error 2
};
#define RQ3_FIRST_OFFSET1 	26
#define RQ3_COUNTER1 	3

const char Convert4_table[] =	{
REG1,	//	35					Service Message								See Table 5.7.
REG1,	//	36					Error Type										See Table 5.8.
REG1,	//	37					Error Message									See Table 5.9.
REG1,	//	38					Pump Status										See Table 5.10.
REG1,	//	39					State Diagnostics							See Table 5.11.
};
#define RQ4_FIRST_OFFSET1 	35
#define RQ4_COUNTER1 		5

// Register-Converttabelle WILO			
const reg_request regs_read_table[] = {         
//	reg_adr             reg_cnt        obj_typ          *conv_table
 	{ RQ1_FIRST_OFFSET1,  RQ1_COUNTER1,  READ_INPUT_REGS, Convert1_table },                           
	{ RQ2_FIRST_OFFSET1,  RQ2_COUNTER1,  READ_INPUT_REGS, Convert2_table },	                         
	{ RQ3_FIRST_OFFSET1,  RQ3_COUNTER1,  READ_INPUT_REGS, Convert3_table },
	{ RQ4_FIRST_OFFSET1,  RQ4_COUNTER1,  READ_INPUT_REGS, Convert4_table },                
};                                             
#define RQ_TABLENG  4            

//////////////////////////////////////////////////////////////////////////////////////////////////

extern char check_modb_rx_buffer(char *pRxBuf, char port);
extern void modb_sio_init(char port);
extern void modbus_request(char *pTxBuf, char port);

void ClearValues (char);
void Get_Messages (char);
void Get_Values (char);
void Put_Values (char); 
unsigned int Get_Op_Decoder(unsigned int);
unsigned int Put_Op_Decoder(unsigned int);
unsigned int Get_Con_Decoder(unsigned int);
unsigned int Put_Con_Decoder(unsigned int);

//////////////////////////////////////////////////////////////////////////////
// Funktion U_MODBUS		Grundtakt = 50ms																		//
//////////////////////////////////////////////////////////////////////////////
char U_Modbus(char port, char funktion)
{
	char fu_ret = funktion;		// Ok Vorbelegung
	char data_idx;
	unsigned char i, t, offs = 0, leng = 0;
	static int	temp;
	static int	rx_control_timer = 0;
	static char modb_slav_max = MODB_SLAVE_MAX;
	static char TaskTimer;
	static char hold_read_idx;
	static char hold_wrt_idx;
	static char cnv_type;
	static const char *conv_mode;
	static char start_up;
//	static char start_up1;
	static char error_flag;
	static char tio_count;
	static UINT *p_holding_reg;
	static UINT *p_output_reg;
	static UINT *p_output_reg_shad;
	static UINT *p_input_reg;

	char *pRxBuf;
	char *pTxBuf;
	
	pRxBuf = NULL;
	pTxBuf = NULL;

	if(TaskTimer > 0)
	{	TaskTimer--;
		return(fu_ret);
	}
	TaskTimer = TaskTimer1;

	switch(port)
	{
		case S1:
			pRxBuf  = RxBuf_S1;
			pTxBuf  = TxBuf_S1;
			break;
			
		case S2:
			pRxBuf  = RxBuf_S2;
			pTxBuf  = TxBuf_S2;
			break;
			
		case S3:
			pRxBuf  = RxBuf_S3;
			pTxBuf  = TxBuf_S3;
			break;
	}

	if(Bus_restart)			// Bus-Neustart
	{	Bus_restart = 0;
		modb_state_control = 0;
// ***AnFre Pumpen-Parameter A: Achtung!: Baudrate IMMER 5 = 9600
		ModbusBaudWilo = c_ModbusBaudWilo;
// ***AnFre eventuell neu eingetragenen Pumpen-Parameter C retten
		bicbus(EEPADR,	(char *)&ModbusSioWilo,	MODBUSSIOWILO_ADR,		1, BICWR);
	}
	if(port != modb_curr_port)
		modb_state_control = 0;											// Bus-Neustart bei Portwechsel

/********************** Spezielle Testfunktionen ***********************/
//	service_code = 137;

	switch(modb_state_control)
	{
//////////////////////////////////////////////////////////////////////////////
//  Modb_State-0 	  Bus-Neustart 0 Variablen-Init 1   
//////////////////////////////////////////////////////////////////////////////
		case 0:
			modb_curr_port = port;
			modb_bus_status = 0;							//	Fag für Anzeige
			rx_control_timer = 0;
			modb_slave_idx = 0;								//	Index für die verschied. Slaves
			cycle_count = 0;	
//			TaskTimer1 = 1;
			cycle_timer = cycle_timer1 = 5;
			modb_rx_int_state = 0;						//  Rx-ISR in Leerlauf
			input_reg_idx = holding_reg_idx = 0;

			// Strukturpointer zuweisen
			p_holding_reg = &modb_data[modb_slave_idx].holding_reg[0];
			p_output_reg = &modb_data[modb_slave_idx].output_value[0];
			p_output_reg_shad = &modb_data[modb_slave_idx].output_value_shad[0];
			p_input_reg = &modb_data[modb_slave_idx].input_reg0;

//##### ulsch die erste Abfrage nach Programmstart erfolgt immer mit Adresse 0, deshalb
			modb_control[modb_slave_idx].slave_adr = BusPuPara[modb_slave_idx].Adresse;

			for ( i = 0; i < MODB_SLAVE_MAX; i++ )
			{
				modb_data[i].Operation_Input = 4;
				modb_data[i].Control_Input = 4;
				modb_data[i].holding_reg[1] = 4;
				modb_data[i].holding_reg[2] = 4;
			}							
// Test only
//			TestHandMode = 1;
// Adressvorbelegung muß noch in die Anwendung verlegt werden.

//			for(i=0; i<modb_slav_max; i++)
//			{	
////				modb_control[i].slave_adr = BusPuPara[i].Adresse;		// werden nur bei Busneustart übernommen
////				modb_control[i].slave_adr = i+1; 	//!!! 3 für gateway-grenzen-test adr 3-6
////				modb_control[i].Pu_function = 1;		//  Test only
////				modb_control[i].Pu_function_shad = 0;
////				modb_control[i].exeption_code = 0;
//			}
			switch(port)
			{
				case S1:
					Baudrate_S1	=	9600;
					Mode_S1 = MODB_MASTER_SLAVE_MODE;
					modb_sio_init(port);
//***AnFre
					// IF-Modul Modbus Stratos (2097808): Pumpen-Parameter C: 
					// parli.h WILO-Anhang: 2, 3, 6 oder 10
					//	MR_S1 = 0x65	 0 1 1 0 0 101			Beispiel: 6
					//								 | | | | | |||
					//								 | | | | | UART Mode 8 Bit
					//								 | | | | Internal Clock             
					//								 | | | 1 Stop Bit	
					//								 | | parity even	
					//								 | parity
					//								 Rx Tx Polarity not inversed
					if ( ModbusSioWilo == 3 )
					MR_S1 |= 0x10;
					if ( ModbusSioWilo == 6 )
					MR_S1 |= 0x60;
					if ( ModbusSioWilo == 10 )
					MR_S1 |= 0x40;
					break;
					
				case S2:
					Baudrate_S2	=	9600;
					Mode_S2 = MODB_MASTER_SLAVE_MODE;
					modb_sio_init(port);		
//***AnFre
					if ( ModbusSioWilo == 3 )
					MR_S2 |= 0x10;
					if ( ModbusSioWilo == 6 )
					MR_S2 |= 0x60;
					if ( ModbusSioWilo == 10 )
					MR_S2 |= 0x40;
					break;
					
				case S3:
					Baudrate_S3	=	9600;
					Mode_S3 = MODB_MASTER_SLAVE_MODE;
					modb_sio_init(port);
//***AnFre
					if ( ModbusSioWilo == 3 )
					MR_S3 |= 0x10;
					if ( ModbusSioWilo == 6 )
					MR_S3 |= 0x60;
					if ( ModbusSioWilo == 10 )
					MR_S3 |= 0x40;
					break;
			}
			modb_state_control = 1;
			break;
//////////////////////////////////////////////////////////////////////////////
//  Modb_State-1 	 Bus-Neustart 2 Variablen-Init 2 
//////////////////////////////////////////////////////////////////////////////
		case 1:
			if(	start_up < 5 )
			{
				start_up++;
				break;								
			}
			modb_rx_count		=	0;			//  Sende-	/	Empfangspuffer Index
			modb_rx_buffer_ready = 0;
			rq_idx = 0;
			hold_read_idx = 0;
			holding_reg_idx = 0;
//##### ulsch: da das Schreiben nun aich bei unverändertem Ausgabe-Wert erfolgt und die Holding-Register 
// danach immer gelesen werden, entfallen Status 3, 4, 5
//			modb_state_control = 3;
			modb_state_control = 10;
			break;

//////////////////////////////////////////////////////////////////////////////
// Modb_State-2   Ausfiltern nicht aktivierter Pumpen 
//////////////////////////////////////////////////////////////////////////////
		case 2:
			Put_Values (modb_slave_idx);
			if(modb_control[modb_slave_idx].Pu_function)								// P. aktiv
			{
				if(modb_control[modb_slave_idx].Pu_function_shad == 0)		// ist aktiv geworden, auch bei Neustart			
				{
					modb_control[modb_slave_idx].Pu_function_shad = 1;
				}
//##### ulsch: da das Schreiben nun aich bei unverändertem Ausgabe-Wert erfolgt und die Holding-Register 
// danach immer gelesen werden, entfallen Status 3, 4, 5
//				modb_state_control = 3;
				modb_state_control = 10;

			}
			else			// P. nicht aktiv
			{
				if(modb_control[modb_slave_idx].Pu_function_shad)				// ist deaktiviert worden			
				{	modb_control[modb_slave_idx].Pu_function_shad = 0;
					modb_control[modb_slave_idx].tx_errorflag = 1;	// OK
					modb_control[modb_slave_idx].rx_errorflag = 1; // OK  dieses Flag ist nur für die Anzeige ( UserKonv.c: char * bus_error_set[])
					modb_control[modb_slave_idx].error_count = 0;
					modb_control[modb_slave_idx].bus_timeout = 0;
					modb_control[modb_slave_idx].exeption_code = 0;
					ClearValues (modb_slave_idx);
				}
				modb_state_control = 21;		//3;
			}
			break;

//##### ulsch: da das Schreiben nun aich bei unverändertem Ausgabe-Wert erfolgt und die Holding-Register 
// danach immer gelesen werden, entfallen Status 3, 4, 5

//-//////////////////////////////////////////////////////////////////////////////
//-//  Modb_State-3  	MODBUS-Holding-Regs Read-Request
//-//////////////////////////////////////////////////////////////////////////////
//-		case 3: 	
//-			first_reg = hold_adr_table[hold_read_idx];
//-			cnt_regs  = 1;																	// Register einzeln lesen
//-			func_code = READ_HOLDING_REGS;
//-			modb_curr_adr = modb_control[modb_slave_idx].slave_adr;
//-			modbus_request(pTxBuf, port);
//-			tio_count = 0;
//-			modb_rx_status = 0;
//-			modb_state_control = 4;
//-			break;
//-
//-//////////////////////////////////////////////////////////////////////////////
//-//  Modb_State-4  	MODBUS-Holding-Regs Read
//-//////////////////////////////////////////////////////////////////////////////
//-		case 4:
//-			modb_rx_status = check_modb_rx_buffer(pRxBuf, port);
//-			if(modb_rx_status == 0) 
//-			{
//-				if(++tio_count > 8)					//  keine Antwort in 800 ms
//-				{
//-					tio_count = 0;
//-					modb_control[modb_slave_idx].error_count++;			// wenn dauerhafter Fehler > prohylakt. Neuinit
//-					modb_control[modb_slave_idx].rx_errorflag = 2;	// bus_timeout;
//-					if( ! BusPuData[modb_slave_idx].busSm )					// Überlauf verhindern
//-						modb_control[modb_slave_idx].bus_timeout++;		// Störungsmeldung verzögert weiterleiten
//-					modb_state_control = 21;
//-				}
//-			}
//-			else
//-			{
//-				bus_tio_count = 0;
//-				switch (modb_rx_status) 
//-				{
//-					case RX_BUF_READY:					
//-					offs =	1;			// Auf Funktion stellen		
//-// Function-Code vergeichen
//-					if(pRxBuf[offs] != func_code)		// Test, ist func_code identisch
//-					{ 
//-						if(pRxBuf[offs] != (func_code + EXCEPTION_MASK))		
//-						{	
//-							modb_control[modb_slave_idx].exeption_code = 0;
//-							modb_control[modb_slave_idx].rx_errorflag = 3;	// FUNKTION_ERROR,  Undef. Error
//-						}
//-						else
//-						{
//-							modb_control[modb_slave_idx].rx_errorflag = 4;	// EXCEPTION;
//-							offs =	2;
//-							modb_control[modb_slave_idx].exeption_code = pRxBuf[offs];			// Exeption-Code speichern
//-						}
//-						modb_control[modb_slave_idx].error_count++;
//-						modb_state_control = 21;			// next adress
//-					}
//-					else	// Rx-Buffer analysieren
//-					{
//-						modb_control[modb_slave_idx].exeption_code = 0;
//-						offs =	2;	
//-						leng = pRxBuf[offs];
//-						if(leng == 2)	// Länge testen
//-						{	
//-							modb_control[modb_slave_idx].rx_errorflag = 1;
//-							modb_control[modb_slave_idx].error_count = 0;
//-							offs = 3;	
//-							p_holding_reg[hold_read_idx] = pRxBuf[offs++] * 256 + pRxBuf[offs++];
//-						}
//-						else
//-						{							
//-							modb_control[modb_slave_idx].error_count++;
//-							modb_control[modb_slave_idx].rx_errorflag = 5;	// LENGT_ERROR1;
//-						}
//-						modb_state_control = 5;			// next device
//-					}							
//-					break;
//-
//-					default:					 
//-						modb_control[modb_slave_idx].error_count++;
//-						modb_control[modb_slave_idx].rx_errorflag = 6; // UNDEF-ERROR
//-						modb_state_control = 21;			// next adress
//-						break;
//-				}
//-			}
//-			break;
//-
//-//////////////////////////////////////////////////////////////////////////////
//-//  Modb_State-5  	MODBUS-HoldingReg-Sequenzer
//-//////////////////////////////////////////////////////////////////////////////
//-		case 5:
//-			if(hold_read_idx < HOLDING_REG_CNT - 1)
//-			{	hold_read_idx++;
//-				modb_state_control = 3;		// next request
//-			}
//-			else
//-			{
//-				hold_read_idx = 0;
//-// 			Beim Sysstem-Neustart die Pumpenwerte übernehmen, bis die Regelung die Parameter vorgibt
//-//ulsch
//-//				for(i=0; i<3; i++)
//-				for(i=0; i<HOLDING_REG_CNT-1; i++)					// nicht für HoldingReg300
//-				{
//-					p_output_reg[i] = p_holding_reg[i];
//-				}
//-				modb_state_control = 10; // Read Input-Regs
//-			}
//-			break;
//-
//-//////////////////////////////////////////////////////////////////////////////
//-//  Modb_State-6  	MODBUS-Holding-Adr-Sequenzer
//-//////////////////////////////////////////////////////////////////////////////
//-//ulsch: kann nicht erkennen, wie case 6 aktiviert werden könnte
//-		case 6:
//-			if ( modb_slav_max > 1 && modb_slave_idx < modb_slav_max-1 )
//-			{	modb_slave_idx++;		
//-				// Strukturpointer zuweisen
//-//ulsch
//-//-				p_holding_reg = &modb_data[modb_slave_idx].holding_reg;
//-//-				p_output_reg = &modb_data[modb_slave_idx].output_value;
//-//-				p_output_reg_shad = &modb_data[modb_slave_idx].output_value_shad;
//-			}
//-			else
//-			{	modb_slave_idx = 0;
//-				// Strukturpointer zuweisen
//-//ulsch
//-//-				p_holding_reg = &modb_data[modb_slave_idx].holding_reg;
//-//-				p_output_reg = &modb_data[modb_slave_idx].output_value;
//-//-				p_output_reg_shad = &modb_data[modb_slave_idx].output_value_shad;
//-			}
//-
//-//ulsch
//-			// Strukturpointer zuweisen
//-			p_holding_reg = &modb_data[modb_slave_idx].holding_reg[0];
//-			p_output_reg = &modb_data[modb_slave_idx].output_value[0];
//-			p_output_reg_shad = &modb_data[modb_slave_idx].output_value_shad[0];
//-
//-			modb_state_control = 2;
//-			break;
//-
//////////////////////////////////////////////////////////////////////////////
//  Modb_State-10  	MODBUS-Input-Regs-Request
//////////////////////////////////////////////////////////////////////////////
		case 10:
			first_reg = regs_read_table[rq_idx].reg_adr;
			cnt_regs  = regs_read_table[rq_idx].reg_cnt;
			func_code = regs_read_table[rq_idx].obj_typ;
// 		Strukturpointer zuweisen
			p_input_reg = &modb_data[modb_slave_idx].input_reg0;
			modb_curr_adr = modb_control[modb_slave_idx].slave_adr;
			modbus_request(pTxBuf, port);
			tio_count = 0;
			modb_rx_status = 0;
			modb_state_control = 11;
			break;

//////////////////////////////////////////////////////////////////////////////
//  Modb_State-11  	MODBUS-Reader
//////////////////////////////////////////////////////////////////////////////
		case 11:
			modb_rx_status = check_modb_rx_buffer(pRxBuf, port);
			if(modb_rx_status == 0) 
			{
				if(++tio_count > 8)					//  keine Antwort in 500 ms
				{
					tio_count = 0;
					modb_control[modb_slave_idx].error_count++;			// wenn dauerhafter Fehler -> prohylakt. Neuinit
					modb_control[modb_slave_idx].rx_errorflag = 2;	// bus_timeout;
					if( ! BusPuData[modb_slave_idx].busSm )					// Überlauf verhindern
						modb_control[modb_slave_idx].bus_timeout++;		// Störungsmeldung verzögert weiterleiten
					modb_state_control = 21;
				}
			}
			else
			{
				switch (modb_rx_status) 
				{
					case RX_BUF_READY:					
					offs =	1;			// Auf Funktion stellen		
// Function-Code vergeichen
					if(pRxBuf[offs] != func_code)		// Test, ist func_code identisch
					{ 
						if(pRxBuf[offs] != (func_code + EXCEPTION_MASK))		
						{	
							modb_control[modb_slave_idx].exeption_code = 0;
							modb_control[modb_slave_idx].rx_errorflag = 3;	// FUNKTION_ERROR, Undef. Error
						}
						else
						{
							modb_control[modb_slave_idx].rx_errorflag = 4;	// EXEPTION;
							offs =	2;
							modb_control[modb_slave_idx].exeption_code = pRxBuf[offs];	// Exeption-Code speichern
							modb_control[modb_slave_idx].pu_timeout = 1;
						}
						modb_control[modb_slave_idx].error_count++;
						modb_state_control = 21;
					}
					else	// Rx-Buffer analysieren
					{
						switch (func_code)
						{
							case READ_INPUT_REGS:
								offs =	2;	
								leng = pRxBuf[offs];
								if(leng == cnt_regs * 2)	//  Länge testen
								{	modb_control[modb_slave_idx].rx_errorflag = 1;	// OK
									modb_control[modb_slave_idx].error_count = 0;
									modb_control[modb_slave_idx].bus_timeout = 0;
									modb_control[modb_slave_idx].pu_timeout = 0;
									modb_control[modb_slave_idx].clear_flag = 0;
									modb_control[modb_slave_idx].exeption_code = 0;
									modb_state_control = 12;	// Daten ablegen
								}
								else
								{	modb_control[modb_slave_idx].error_count++;
									modb_control[modb_slave_idx].rx_errorflag = 8;	// LENGT_ERROR2;
									modb_state_control = 21;
								}
								break;
						}						
					}							
					break;

				default:					 
					modb_control[modb_slave_idx].error_count++;
					modb_control[modb_slave_idx].rx_errorflag = 6;
					modb_state_control = 21;
					break;
				}
			}
			break;

//////////////////////////////////////////////////////////////////////////////
//  Modb_State-12  	MODBUS-Daten ablegen
//////////////////////////////////////////////////////////////////////////////
		case 12:
			BusPuData[modb_slave_idx].busSm = 0;

			offs = 3;
			conv_mode	  = regs_read_table[rq_idx].conv_table; // Konvertierung festlegen

			switch(func_code)
			{
				case READ_INPUT_REGS:
					t = 0;
					for(i=0; i<cnt_regs; i++)
					{	
						cnv_type = conv_mode[i];

						switch(cnv_type)
						{
							case SKIP:			// Wert ausblenden	
								offs += 2;	
								break;

								case REG1:						// Zu Int konvertieren
								case REG01:						// Kommastelle wird in Parli erzeugt
								case LOWBYTE:
								p_input_reg[input_reg_idx++] = pRxBuf[offs++] * 256 + pRxBuf[offs++];
								break;

							case REG10:						// Zu Int konvertieren Auflösung * 10  		result = (int)(fl_result * 10);
								temp =	pRxBuf[offs++] * 256 + pRxBuf[offs++];
								if(i == 4)						// vorläufig, noch verbessern!
									modb_data[modb_slave_idx].op_hours = (unsigned long)(temp * 10);
								break;
						}
					}
					break;
			}
			modb_state_control = 13;
			break;

//////////////////////////////////////////////////////////////////////////////
//  Modb_State-13  --- Sequenzer  ---	
//////////////////////////////////////////////////////////////////////////////
		case 13:	
			rq_idx++;
			if(rq_idx < RQ_TABLENG)
				modb_state_control = 10;		// next request
			else
			{
				Get_Values (modb_slave_idx);
				rq_idx = 0;
				holding_reg_idx = 0;
				input_reg_idx = 0;
				hold_wrt_idx = 0;
				modb_state_control = 14;
			}
			break;

//////////////////////////////////////////////////////////////////////////////
//   Modb_State-14  	  Reserve
//////////////////////////////////////////////////////////////////////////////
		case 14:
			Put_Values (modb_slave_idx);
			modb_state_control = 15;
			break;

//////////////////////////////////////////////////////////////////////////////
//  Modb_State-15  	Test auf Änderungen
//////////////////////////////////////////////////////////////////////////////

//##### ulsch: Ausgaben erfolgen nun immer, also auch, wenn Output- und Holding-Register übereinstimmen
// Grund ist die Vermutung, dass nach einem Handbetrieb der Pumpe die Holding-Register nicht den Zustand der Pumpe zeigen 
//-		case 15: 	
//-			if(p_output_reg[hold_wrt_idx] != p_holding_reg[hold_wrt_idx])
//-				modb_state_control = 16;
//-			else
//-			{	Get_Messages (modb_slave_idx);
//-				modb_state_control = 20;				
//-			}
//-			break;

		case 15: 	
			if ( hold_wrt_idx == 0 )
				Get_Messages ( modb_slave_idx );		// das erfolgt nun nur einmal
			modb_state_control = 16;
			break;

//////////////////////////////////////////////////////////////////////////////
//  Modb_State-16  	Write single Holdings
//////////////////////////////////////////////////////////////////////////////
		case 16: 	
			first_reg = hold_adr_table[hold_wrt_idx];
			func_code = WRITE_SINGLE_HOLDING;
			reg_value = p_output_reg[hold_wrt_idx];
			modb_curr_adr = modb_control[modb_slave_idx].slave_adr;
			modbus_request(pTxBuf, port);
			modb_rx_status = 0;
			tio_count = 0;
			modb_state_control = 17;
			break;

//////////////////////////////////////////////////////////////////////////////
//  Modb_State-17  	Wait for Single Holdings Response
//////////////////////////////////////////////////////////////////////////////
		case 17: 	
			modb_rx_status = check_modb_rx_buffer(pRxBuf, port);
//			if(modb_rx_status == 0) 
// ulsch: für alle Fehler muss es irgendwie weitergehen !
			if ( modb_rx_status != RX_BUF_READY ) 
			{
				if(++tio_count > 8)					//  keine Antwort in 500 ms
				{
					tio_count = 0;
					modb_control[modb_slave_idx].tx_errorflag = 2;	 // CMD WR-TIMEOUT; //umbenennenRX_TIMEOUT_WR;
					modb_state_control = 21;
				}
			}
			else
			{
//				if(modb_rx_status == RX_BUF_READY) 
//				{
				offs =	1;			// Auf funktion stellen		
				error_flag = 0;

				for(i=0; i<6; i++) 	// String vergeichen nur bei diesem obj. type !
				{
					if(pRxBuf[i] != pTxBuf[i])		// Test, ist func_code identisch
						error_flag = 1;						
				}
				if(error_flag)
				{ 
					if(pRxBuf[offs] != (func_code + EXCEPTION_MASK))		
					{	
						modb_control[modb_slave_idx].exeption_code = 0;
						modb_control[modb_slave_idx].tx_errorflag = 4;							// FUNKTION_ERROR;   // Undef. Error
					}
					else
					{
						modb_control[modb_slave_idx].tx_errorflag = 5;							// EXCEPTION;
						offs =	2;
						modb_control[modb_slave_idx].exeption_code = pRxBuf[offs];				// Exeption-Code speichern
					}
					modb_state_control = 21;
				}
				else
				{
					modb_control[modb_slave_idx].exeption_code = 0;
					modb_control[modb_slave_idx].tx_errorflag = 1; // OK
					modb_state_control = 18;  // Holding rücklesen
				}							
//				}
			}
			break;

//////////////////////////////////////////////////////////////////////////////
//  Modb_State-18  	Request MODBUS-Holding-Regs zurück lesen
//////////////////////////////////////////////////////////////////////////////
		case 18: 	
			first_reg = hold_adr_table[hold_wrt_idx];
			cnt_regs  = 1;																	// Register einzeln lesen
			func_code = READ_HOLDING_REGS;
			modb_curr_adr = modb_control[modb_slave_idx].slave_adr;
			modbus_request(pTxBuf, port);
			tio_count = 0;
			modb_rx_status = 0;
			modb_state_control = 19;
			break;

//////////////////////////////////////////////////////////////////////////////
//  Modb_State-19  	Wait for Single Holdings Response
//////////////////////////////////////////////////////////////////////////////
		case 19:
			modb_rx_status = check_modb_rx_buffer(pRxBuf, port);
			if(modb_rx_status == 0) 
			{
				if(++tio_count > 8)					//  keine Antwort in 800 ms
				{
					tio_count = 0;
					modb_control[modb_slave_idx].tx_errorflag = 2;	// RX_TIMEOUT;
					modb_state_control = 15;
				}
			}
			else
			{
				switch (modb_rx_status) 
				{
					case RX_BUF_READY:					
					offs =	1;			// Auf Funktion stellen		
// Function-Code vergeichen
					if(pRxBuf[offs] != func_code)		// Test, ist func_code identisch
					{ 

						if(pRxBuf[offs] != (func_code + EXCEPTION_MASK))		
						{	
							modb_control[modb_slave_idx].exeption_code = 0;
							modb_control[modb_slave_idx].tx_errorflag = 4;	// FUNKTION_ERROR,  Undef. Error
						}
						else
						{
							modb_control[modb_slave_idx].tx_errorflag = 5;	// EXCEPTION;
							offs =	2;
							modb_control[modb_slave_idx].exeption_code = pRxBuf[offs];			// Exeption-Code speichern
						}
					}
					else	// Rx-Buffer analysieren
					{
						modb_control[modb_slave_idx].exeption_code = 0;
						offs =	2;	
						leng = pRxBuf[offs];
						if(leng == 2)	// Länge testen
						{	
							offs = 3;	
							p_holding_reg[hold_wrt_idx] = pRxBuf[offs++] * 256 + pRxBuf[offs++];
							if(p_output_reg[hold_wrt_idx] == p_holding_reg[hold_wrt_idx])
							{	modb_control[modb_slave_idx].tx_errorflag = 1;
							}
							else
								modb_control[modb_slave_idx].tx_errorflag = 8;  // Write Error
						}
						else
							modb_control[modb_slave_idx].tx_errorflag = 6;	// LENGT_ERROR;
					}							
					break;

					default:					 
						modb_control[modb_slave_idx].tx_errorflag = 7; // UNDEF-ERROR
						break;
				}
				modb_state_control = 20;
			}
			break;

//////////////////////////////////////////////////////////////////////////////
//  Modb_State-20  	Sequenzer Wait for Single Holdings Response
//////////////////////////////////////////////////////////////////////////////
		case 20:
			if(hold_wrt_idx < HOLDING_REG_CNT-1)
			{	hold_wrt_idx++;
				modb_state_control = 15;
			}
			else
			{	
				modb_state_control = 21;				
			}
			break;

//////////////////////////////////////////////////////////////////////////////
//   Modb_State-21  	  CycleTimer +  AdressIncrement
//////////////////////////////////////////////////////////////////////////////
		case 21:
			if(modb_control[modb_slave_idx].error_count > 100) // wenn dauerhafter Fehler > prohylakt. Neuinit
			{	modb_control[modb_slave_idx].error_count = 0;
				modb_state_control = 0;						// Bus-Neustart
			}
			if ( --cycle_timer > 0)	
				break;
			else
			{	
				cycle_timer = cycle_timer1;	
				if ( BusPuPara[modb_slave_idx].Funktion > 0 )
				{
					BusPuData[modb_slave_idx].busSm = (( modb_control[modb_slave_idx].pu_timeout) || ( modb_control[modb_slave_idx].bus_timeout > SM_DELAY )) ? 1 : 0;
//				BusPuData[pu].busSm = modb_control[pu].pu_timeout || modb_control[pu].bus_timeout;	// ohne Delay
				}
				if ( modb_slav_max > 1 && modb_slave_idx < modb_slav_max-1 )
				{	modb_slave_idx++;		
					// Strukturpointer zuweisen
					p_holding_reg = &modb_data[modb_slave_idx].holding_reg[0];
					p_output_reg = &modb_data[modb_slave_idx].output_value[0];
					p_output_reg_shad = &modb_data[modb_slave_idx].output_value_shad[0];
					p_input_reg = &modb_data[modb_slave_idx].input_reg0;
					modb_state_control = 2;
				}
				else
				{	
					modb_slave_idx = 0;
					// Strukturpointer zuweisen
					p_holding_reg = &modb_data[modb_slave_idx].holding_reg[0];
					p_output_reg = &modb_data[modb_slave_idx].output_value[0];
					p_output_reg_shad = &modb_data[modb_slave_idx].output_value_shad[0];
					p_input_reg = &modb_data[modb_slave_idx].input_reg0;
					bus_timeout_com = 0;				// Common-Error-Generierung Gateway-Timeout
					for(i=0; i<MODB_SLAVE_MAX; i++)
					{	if(modb_control[i].bus_timeout > SM_DELAY)
						{	bus_timeout_com = 1;
							if(!modb_control[i].clear_flag)
							{	modb_control[i].clear_flag = 1;
								ClearValues (i);
							}
						}
					}
					pu_timeout_com = 0;							// Common-Error-Generierung Pumpen-Timeout
					for(i=0; i<MODB_SLAVE_MAX; i++)	// Pumpen-Timeout ist eine Gateway-Meldung, und wird bei Direkanschluß nicht generiert!
					{	if(modb_control[i].pu_timeout)
						{	pu_timeout_com = 1;					
							if(!modb_control[modb_slave_idx].clear_flag)
							{	modb_control[i].clear_flag = 1;
								ClearValues (i);
							}
						}
					}
					if(bus_timeout_com)								// Bus-Timeout aktiv
					{
						if(bus_timeout_com_shad == 0)		// Bus-Timeout ist aktiv geworden		
						{
							bus_timeout_com_shad = 1;
							modb_state_control = 0;						// Prophylaktischer Busneustart
						}
					}
					else			// P. nicht aktiv
					{
						if(bus_timeout_com_shad)				// ist deaktiviert worden			
						{	bus_timeout_com_shad = 0;
							bus_tio_count = 0;
						}
					}
					if(++cycle_count < 150)
						modb_state_control = 2;
					else
						modb_state_control = 0;					// Prophylaktischer Busneustart
				}
			
				modb_data[modb_slave_idx].input_reg6_celsius = modb_data[modb_slave_idx].input_reg6 - 2732;
				
				if ( modb_data[modb_slave_idx].output_value1_temp <= 4 )
					memcpy ( &BusPuData[modb_slave_idx].betriebsArtSoll, pu_operation_set[modb_data[modb_slave_idx].output_value1_temp], 15 );
				else
					memcpy ( &BusPuData[modb_slave_idx].betriebsArtSoll, ClearDisplay, 15 );
				if ( modb_data[modb_slave_idx].Operation_Input <= 4 )
					memcpy ( &BusPuData[modb_slave_idx].betriebsArtIst, pu_operation_set[modb_data[modb_slave_idx].Operation_Input], 15 );
				else
					memcpy ( &BusPuData[modb_slave_idx].betriebsArtIst, ClearDisplay, 15 );
				if ( modb_data[modb_slave_idx].output_value2_temp <= 4 )
					memcpy ( &BusPuData[modb_slave_idx].regelArtSoll, pu_control_set[modb_data[modb_slave_idx].output_value2_temp], 15 );
				else
					memcpy ( &BusPuData[modb_slave_idx].regelArtSoll, ClearDisplay, 15 );
				if ( modb_data[modb_slave_idx].Control_Input <= 4 )
					memcpy ( &BusPuData[modb_slave_idx].regelArtIst, pu_control_set[modb_data[modb_slave_idx].Control_Input], 15 );
				else
					memcpy ( &BusPuData[modb_slave_idx].regelArtIst, ClearDisplay, 15 );
			
			}
			break;

		default:					 
			modb_state_control = 0;		// ***AnFre Bus-Neustart 0 Variablen-Init 1 
			break;

	}		 
	
	return(fu_ret);
}
//////////////////////////////////////////////////////////////////////////////
// Ende WILO-MODBUS-Task
//////////////////////////////////////////////////////////////////////////////

//----------------------------------- Upros ----------------------------------
void ClearValues (char pu)
{
	char i;
	char *p_clear_reg;
	UINT clearLength = MBDSLENG-(6+2*HOLDING_REG_CNT);

	p_clear_reg = (char *)(&modb_data[pu]);
	for ( i = 0; i < clearLength; i++ )
		*(p_clear_reg+i) = 0;
		
	modb_data[pu].Operation_Input = 4;
	modb_data[pu].Control_Input	= 4;
	modb_data[pu].holding_reg[1] = 4;
	modb_data[pu].holding_reg[2] = 4;
		
}

// Commands Opmode: Aus / Ein / Min / Max 	
unsigned int Get_Op_Decoder(unsigned int GetOp)
{
	switch(GetOp)
	{
		case 8:		return 0;
		case 9:		return 1;
		case 10:	return 2;
		case 12:	return 3;
		default:	return 4;
	}
}

unsigned int Put_Op_Decoder(unsigned int PutOp)
{
	switch(PutOp)
	{
		case 0:		return 8;
		case 1:		return 9;
		case 2:		return 10;
		case 3:		return 12;
		default:	return 9;
	}
}

// Commands Control Mode: Konstantdruck, Proportionaldruck, Konstantfrequenz, Automatik
unsigned int Get_Con_Decoder(unsigned int GetCon)
{	
	switch(GetCon)
	{	
		case 1:		return 2;
		case 3:		return 0;
		case 4:		return 1;
		case 6:		return 3;
		default:	return 4;	// ungültiger Wert
	}
}

unsigned int Put_Con_Decoder(unsigned int PutCon)
{
	switch(PutCon)
	{
		case 0:		return 3;
		case 1:		return 4;
		case 2:		return 1;
		case 3:		return 6;
		default:	return 3;
	}
}
//---------------------------------- Ende Upros --------------------------------/


//////////////////////////////////////////////////////////////////////////////
//                             Parameter-Übergabe
//////////////////////////////////////////////////////////////////////////////

void Get_Values (char pu)
{
	unsigned int t;
// SetPoint
	t = modb_data[pu].holding_reg[0];
	modb_data[pu].SetPoint_Input = t * 5;		// 10-tel %

// Commands Opmode: Aus / Ein / Min / Max 	
	modb_data[pu].Operation_Input = Get_Op_Decoder(modb_data[pu].holding_reg[1]);

// Commands Control Mode: Konstantdruck, Proportionaldruck, Konstantfrequenz, Automatik
	modb_data[pu].Control_Input = Get_Con_Decoder(modb_data[pu].input_reg7);
}
//-----------------------------------------------------------------------------

void Put_Values (char pu)
{
	char t;
	
	modb_control[pu].slave_adr = BusPuPara[pu].Adresse;
	if ( BusPuPara[pu].Funktion > 1 ) 
		BusPuPara[pu].Funktion = 0;
	modb_control[pu].Pu_function = BusPuPara[pu].Funktion;	

	if ( modb_control[pu].slave_adr > 0 )
	{			
		if ( BusPuPara[pu].Hand == FALSE )
		{
			if ( BusPuPara[pu].Sollwert > 1000 )
				modb_data[pu].output_value[0] = 200; 
			else if ( BusPuPara[pu].Sollwert > 0 )
				modb_data[pu].output_value[0] = BusPuPara[pu].Sollwert / 5 ;
			else
				modb_data[pu].output_value[0] = 0; 

//		Sollwert Soll 0 bis 100 %
			modb_data[pu].output_value0_temp = BusPuPara[pu].Sollwert;		// mit Komma !

// 		Commands Opmode: Aus / Ein / Min / Max 	
			modb_data[pu].output_value1_temp = BusPuPara[pu].Betrieb;

// 		Commands Control Mode: Konstantdruck, Proportionaldruck, Konstantfrequenz, Automatik
			modb_data[pu].output_value2_temp = BusPuPara[pu].Regelart;
// 		
		}
		else
		{
			BusPuPara[pu].Hand = TRUE;
			if ( BusPuPara[pu].SollHand > 1000 )
				modb_data[pu].output_value[0] = 200; 
			else if ( BusPuPara[pu].SollHand > 0 )
				modb_data[pu].output_value[0] = BusPuPara[pu].SollHand / 5 ;
			else
				modb_data[pu].output_value[0] = 0; 

//		Sollwert Soll 0 bis 100 %
			modb_data[pu].output_value0_temp = BusPuPara[pu].SollHand;		// mit Komma !

// 		Commands: Aus / Ein / Min / Max
			modb_data[pu].output_value1_temp = BusPuPara[pu].BetriebHand;

// 		Konstantdruck, Proportionaldruck, Konstantfrequenz, Automatik
			modb_data[pu].output_value2_temp = BusPuPara[pu].RegelartHand;
		}
	}

	if ( BusPuData[pu].busSm )
	{
		modb_data[pu].output_value[3] = 1;				// HoldingReg300 = OFF
		modb_data[pu].busSmWar = TRUE;
	}
	else
	{
		if ( modb_data[pu].busSmWar == TRUE )
		{
			if ( modb_data[pu].holding_reg[3] == 1 )
			{
				modb_data[pu].busSmWar = FALSE;		// OFF ist angekommen
				modb_data[pu].output_value[3] = BusPuPara[pu].HoldingReg300;				
			}
			else
				modb_data[pu].output_value[3] = 1;			 			
		}
		else	
			modb_data[pu].output_value[3] = BusPuPara[pu].HoldingReg300;		// alles ok
	}

// die Variablen _temp codiert Betriebs-/Regelart als 0...3, wird hier umgesetzt entsprechend WILO-Konvention
	modb_data[pu].output_value[1] = Put_Op_Decoder ( modb_data[pu].output_value1_temp );
	modb_data[pu].output_value[2] = Put_Con_Decoder ( modb_data[pu].output_value2_temp );
	
}
//-----------------------------------------------------------------------------

void Get_Messages (char pu)
{
	char i;
	unsigned int	msg, mask;

	mask = 0x0001;

	modb_data[pu].message0 = 0;
	BusPuData[pu].puAlarm = 0;

// Service Message
	msg	= ( modb_data[pu].input_reg15 & 0x000F ); // inrelevante Bit maskieren
	if(msg)
	{
		for(i=1; i<4; i++)
		{
			if(msg	&	mask)		
				break;
			mask <<= 1;
		}
		modb_data[pu].message1 = i;		
		mask ^= 0xFFFF;	// Für Test auf Doppelbelegung, Vorläufig
		if(msg	&	mask)		
			modb_data[pu].message0 = 1;	// Vorläufig global! event. Doppelbelegungen markieren
	}		
	else
		modb_data[pu].message1 = 0;		

// Error Type
	msg	= ( modb_data[pu].input_reg16 & 0x001B ); // inrelevante Bit maskieren
	if(msg)
	{
		BusPuData[pu].puAlarm = 1;		// Störungsmeld. der der Pumpe 
		mask = 0x0001;
		for(i=1; i<5; i++)
		{
			if(msg	&	mask)		
				break;
			mask <<= 1;
		}
		modb_data[pu].message2 = i;		
		mask ^= 0xFFFF;
		if(msg	&	mask)		
			modb_data[pu].message0 = 1;
	}		
	else
		modb_data[pu].message2 = 0;		

// Error Message
	msg	= ( modb_data[pu].input_reg17 & 0x7F3F ); // inrelevante Bit maskieren
	if(msg)
	{
		mask = 0x0001;
		for(i=1; i<15; i++)
		{
			if(msg	&	mask)		
				break;
			mask <<= 1;
		}
		modb_data[pu].message3 = i;		
		mask ^= 0xFFFF;
		if(msg	&	mask)		
			modb_data[pu].message0 = 1;
	}		
	else
		modb_data[pu].message3 = 0;

// 	Pump Status	
// Betriebsmeldung separieren d.h. Bit 0 einzeln testen dann maskieren. Nur wenn sinnvoll, noch prüfen
	msg	= ( modb_data[pu].input_reg18 & 0x20FF ); // inrelevante Bit maskieren
	if(msg)
	{
		mask = 0x0001;
		for(i=1; i<14; i++)
		{
			if(msg	&	mask)		
				break;
			mask <<= 1;
		}
		modb_data[pu].message4 = i;		
		mask ^= 0xFFFF;
		if(msg	&	mask)		
			modb_data[pu].message0 = 1;
	}		
	else
		modb_data[pu].message4 = 0;		

// 	State Diagnose	
	msg	= ( modb_data[pu].input_reg19 & 0x055B ); // inrelevante Bit maskieren
	if(msg)
	{
		if (modb_data[pu].input_reg19 & 0x03)
		{
			BusPuData[pu].puAlarm = 1;		// Störungsmeld. der der Pumpe 
		}
		mask = 0x0001;
		for(i=1; i<11; i++)
		{
			if(msg	&	mask)		
				break;
			mask <<= 1;
		}
		modb_data[pu].message5 = i;		
		mask ^= 0xFFFF;
		if(msg	&	mask)		
			modb_data[pu].message0 = 1;	// Vorläufig! event. Doppelbelegungen global markieren
	}		
	else
		modb_data[pu].message5 = 0;		
}
//////////////////////////////////////////////////////////////////////////////
//                          Ende Parameter-Übergabe
//////////////////////////////////////////////////////////////////////////////
#endif		// Wilo-Handler
