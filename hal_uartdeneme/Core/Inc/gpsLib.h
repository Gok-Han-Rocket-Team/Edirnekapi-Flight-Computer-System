#ifndef GPS_HEADER
#define GPS_HEADER

#include "stdint.h"

//GPS hata kodlari:
#define GPS_ERR_ENL 	0b000000001
#define GPS_ERR_BYL 	0b000000010
#define GPS_ERR_ENLYON 	0b000000100
#define GPS_ERR_BYLYON 	0b000001000
#define GPS_ERR_IRTFA 	0b000010000
#define GPS_ERR_UYDUS 	0b000100000
#define GPS_ERR_ZAMAN 	0b001000000
#define GPS_ERR 		0b010000000

typedef uint16_t 		GPS_RTRN;

typedef struct Gps{
	uint8_t saat;
	uint8_t dakika;
	uint8_t saniye;
	uint8_t enlemYon;
	uint8_t boylamYon;
	uint8_t uyduS;
	float enlem;
	float boylam;
	float irtifa;
}gps;

GPS_RTRN gpsAyikla(gps*);
void gpsBaslat(gps*, uint8_t*);
void gpsYazdir(gps*);
void gpsHataYazdir(GPS_RTRN);

#endif
