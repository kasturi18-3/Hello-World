typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned long  DWORD;
typedef unsigned int   BOOL; 

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

#define MODE_MEASUREMENT 	0xA0
#define MODE_SIMULATION10 	0xA1
#define MODE_SIMULATION50 	0xA2
#define MODE_SIMULATION100 	0xA3
#define MODE_FLOWCALIB0 	0xA4

