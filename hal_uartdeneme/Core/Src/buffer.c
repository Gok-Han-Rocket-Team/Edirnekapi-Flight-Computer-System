#include "buffer.h"
#include "gpsLib.h"

extern uint8_t dizi[80];
extern gps myGPS;
uint8_t GP[] = "$GPGGA,";
int sayac = 0;
int diziSayac = 0;
int durum = 0;

void bufferEkle(uint8_t veri){

	if(sayac == 7){
		if(veri == '\r'){
			diziSayac = 0;
			sayac = 0;
			durum = 1;
		}
		else{
			dizi[diziSayac] = veri;
			diziSayac++;
		}
	}
	else{
		if(veri == GP[sayac])
			sayac++;
		else
			sayac = 0;
	}

}
int bufferDurum()
{
	if(durum)
	{
		durum = 0;
		return 1;
	}
	else
		return 0;
}
