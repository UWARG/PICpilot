//#define Fcy  14754600
#define Fcy  40000000

void Delay( unsigned int delay_count );
void Delay_Us( unsigned int delayUs_count );

//Used with Delay_Us()
#define Delay1uS_count  (Fcy * 0.0001) / 1080

//Used with Delay()
#define Delay_1mS_Cnt	  (Fcy * 0.001) / 2950
#define Delay_2mS_Cnt	  (Fcy * 0.002) / 2950
#define Delay_5mS_Cnt	  (Fcy * 0.005) / 2950
#define Delay_15mS_Cnt 	  (Fcy * 0.015) / 2950
#define Delay_1S_Cnt	  (Fcy * 1) / 2950 

