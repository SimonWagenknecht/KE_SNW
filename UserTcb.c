
#include "struct.h"
#include "usertask.h"

extern void U1_TASK(void);
extern void U2_TASK(void);
extern void U3_TASK(void);
extern void U4_TASK(void);
extern void U5_TASK(void);
extern void U6_TASK(void);
extern void U7_TASK(void);
extern void U8_TASK(void);
extern void U9_TASK(void);
extern void U10_TASK(void);
extern void U11_TASK(void);
extern void U12_TASK(void);
extern void U13_TASK(void);
extern void U14_TASK(void);
extern void U15_TASK(void);
extern void U16_TASK(void);
extern void U17_TASK(void);
extern void U18_TASK(void);
extern void U19_TASK(void);
extern void U20_TASK(void);
extern void U21_TASK(void);
extern void U22_TASK(void);
extern void U23_TASK(void);
extern void U24_TASK(void);
extern void U25_TASK(void);

/* Aufbau des Task-Control-Blocks (variabler Teil): Status, Zähler */
tcb_var tcblist_var_user[] = {
		{ U1_STAT,	  U1_ZAEHL},
		{ U2_STAT,	  U2_ZAEHL},
		{ U3_STAT,	  U3_ZAEHL},
		{ U4_STAT,	  U4_ZAEHL},
		{ U5_STAT,	  U5_ZAEHL},
		{ U6_STAT,	  U6_ZAEHL},
		{ U7_STAT,	  U7_ZAEHL},
		{ U8_STAT,	  U8_ZAEHL},
		{ U9_STAT,	  U9_ZAEHL},
		{U10_STAT,	 U10_ZAEHL},
		{U11_STAT,	 U11_ZAEHL},
		{U12_STAT,	 U12_ZAEHL},
		{U13_STAT,	 U13_ZAEHL},
		{U14_STAT,	 U14_ZAEHL},
		{U15_STAT,	 U15_ZAEHL},
		{U16_STAT,	 U16_ZAEHL},
		{U17_STAT,	 U17_ZAEHL},
		{U18_STAT,	 U18_ZAEHL},
		{U19_STAT,	 U19_ZAEHL},
		{U20_STAT,	 U20_ZAEHL},
		{U21_STAT,	 U21_ZAEHL},
		{U22_STAT,	 U22_ZAEHL},
		{U23_STAT,	 U23_ZAEHL},
		{U24_STAT,	 U24_ZAEHL},
		{U25_STAT,	 U25_ZAEHL},
};

// Aufbau des Task-Control-Blocks (konstanter Teil): 
//	Bicbus-Benutzung,  Zeitgeber, Taskname 
const tcb_con tcblist_con_user[] = {
		{ U1_BICB,	 U1_ZEITG,	(void *) U1_TASK},
		{ U2_BICB,	 U2_ZEITG,	(void *) U2_TASK},
		{ U3_BICB,	 U3_ZEITG,	(void *) U3_TASK},
		{ U4_BICB,	 U4_ZEITG,	(void *) U4_TASK},
		{ U5_BICB,	 U5_ZEITG,	(void *) U5_TASK},
		{ U6_BICB,	 U6_ZEITG,	(void *) U6_TASK},
		{ U7_BICB,	 U7_ZEITG,	(void *) U7_TASK},
		{ U8_BICB,	 U8_ZEITG,	(void *) U8_TASK},
		{ U9_BICB,	 U9_ZEITG,	(void *) U9_TASK},
		{U10_BICB,	U10_ZEITG,	(void *)U10_TASK},
		{U11_BICB,	U11_ZEITG,	(void *)U11_TASK},
		{U12_BICB,	U12_ZEITG,	(void *)U12_TASK},
		{U13_BICB,	U13_ZEITG,	(void *)U13_TASK},
		{U14_BICB,	U14_ZEITG,	(void *)U14_TASK},
		{U15_BICB,	U15_ZEITG,	(void *)U15_TASK},
		{U16_BICB,	U16_ZEITG,	(void *)U16_TASK},
		{U17_BICB,	U17_ZEITG,	(void *)U17_TASK},
		{U18_BICB,	U18_ZEITG,	(void *)U18_TASK},
		{U19_BICB,	U19_ZEITG,	(void *)U19_TASK},
		{U20_BICB,	U20_ZEITG,	(void *)U20_TASK},
		{U21_BICB,	U21_ZEITG,	(void *)U21_TASK},
		{U22_BICB,	U22_ZEITG,	(void *)U22_TASK},
		{U23_BICB,	U23_ZEITG,	(void *)U23_TASK},
		{U24_BICB,	U24_ZEITG,	(void *)U24_TASK},
		{U25_BICB,	U25_ZEITG,	(void *)U25_TASK},
};
const char USER_TASKZ = sizeof(tcblist_con_user) / sizeof(tcb_con);

