//standart kutuphaneler
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
//kisisel kutuphaneler
#include "main.h"
#include "gpsLib.h"

//Debug modunu acarak gelen ham verileri ekraana yazdirabilirsin.
//#define uartDebug

extern UART_HandleTypeDef huart2;

uint8_t 	gpsBuf[80];
uint8_t* 	tampon;

/**
* @brief saatAyikla
* 						Bu fonksiyon zaman degerini ayiklar ve verilen adresteki gps yapisina yazar.
* @param 	yeni: 		Gps bilgilerini tutan yapinin adresi.
* 			zaman:		Zaman verisinin tutuldugu dizi.
* @retval 	GPS_RTRN: 	Ayiklama hata kodu.
*/
static GPS_RTRN zamanAyikla(gps* yeni, uint8_t* zaman)
{
	int uzunluk = strlen((char*) zaman);
	if(uzunluk != 9){
		return GPS_ERR_ZAMAN;
	}

	uint8_t saat[3] = {'\0'};
	uint8_t dakika[3] = {'\0'};
	uint8_t saniye[3] = {'\0'};

	int isaretci;
	for(isaretci = uzunluk; zaman[isaretci] != '.' && isaretci != 0; isaretci--);
	if(isaretci == 0)
		return GPS_ERR_ZAMAN;

	isaretci--;

	for(int i = isaretci; isaretci - i != 6; i--){
		if(i > isaretci - 2)
			saniye[i - isaretci + 1] = zaman[i];
		else if(i > isaretci - 4)
			dakika[i - isaretci + 3] = zaman[i];
		else if(i > isaretci - 6)
			saat[i - isaretci + 5] = zaman[i];
	}
	yeni->saat = atoi((char*) saat);
	yeni->dakika = atoi((char*) dakika);
	yeni->saniye = atoi((char*) saniye);

	return 0;
}

/**
* @brief 	konumAyikla
* 						Bu fonksiyon konum bilgilerini ayiklar ve verilen adresteki gps yapisina yazar.
* @param 	yeni: 		Gps bilgilerini tutan yapinin adresi.
* 			konum:		Konum verisinin tutuldugu dizi.
* @retval 	GPS_RTRN: 	Ayiklama hata kodu.
*/
static GPS_RTRN konumAyikla(gps* yeni, uint8_t* konum)
{
	//enlem hata ayikla
	//boylam hata duzenle.

	GPS_RTRN donut = 0;

	//enlem yon
	if(strlen((char*) konum + 11 * 2) != 1 || (*(konum + 11 * 2) != 'N' && *(konum + 11 * 2) != 'S')){
		donut |= GPS_ERR_ENLYON;
	}
	else{
		yeni->enlemYon = *(konum + 11 * 2);
	}

	//boylam yon
	if(strlen((char*) konum + 11 * 4) != 1 || (*(konum + 11 * 4) != 'E' && *(konum + 11 * 4) != 'W')){
			donut |= GPS_ERR_BYLYON;
	}
	else{
		yeni->boylamYon = *(konum + 11 * 4);
	}

	int i = 0;

	//enlem

	uint8_t enlemBayrak = 1;
	if(strlen((char*) konum + 11) != 9){
		donut |= GPS_ERR_ENL;
		enlemBayrak = 0;
	}
	else{
		for(i = 0; *(konum + 11 + i) != '.'; i++){
			if(i > 6){
				donut |= GPS_ERR_ENL;
				enlemBayrak = 0;
				break;
			}
		}
	}

	if(enlemBayrak){
		uint8_t dereceEnl[4] = {0};
		dereceEnl[4] = '\0';
		float enlem = 0.0;

		for(int j = i - 3; j >= 0; j--)
			dereceEnl[j - i + 4] = *(konum + 11 + j);

		enlem = atof((char*) dereceEnl);
		enlem += atof((char*) (konum + 11 + i - 2)) / 60.0;

		yeni->enlem = enlem;
	}

	//boylam
	uint8_t boylamBayrak = 1;
	if(strlen((char*) konum + 33) != 10){
		donut |= GPS_ERR_BYL;
		boylamBayrak = 0;
	}
	else{
		for(i = 0; *(konum + 33 + i) != '.'; i++){
			if(i > 6){
				donut |= GPS_ERR_BYL;
				boylamBayrak = 0;
				break;
			}
		}
	}

	if(boylamBayrak){
		uint8_t dereceByl[4] = {0};

		dereceByl[4] = '\0';
		float boylam = 0.0;

		for(int j = i - 3; j >= 0; j--)
			dereceByl[j - i + 5] = *(konum + 33 + j);

		boylam = atof((char*) dereceByl);
		boylam += atof((char*) (konum + 33 + i - 2)) / 60.0;

		yeni->boylam = boylam;
	}

	return donut;
}

/**
* @brief 	irtifaAyikla
* 						Bu fonksiyon irtifa ve uydu sayisi bilgilerini ayiklar ve verilen adresteki gps yapisina yazar.
* @param 	yeni: 		Gps bilgilerini tutan yapinin adresi.
* 			irtifa:		Irtifa verisinin tutuldugu dizi.
* 			uyduS:		Uydu sayisi verisinin tutuldugu dizi.
* @retval 	GPS_RTRN: 	Ayiklama hata kodu.
*/
static GPS_RTRN irtifa_uyduSAyikla(gps* yeni, uint8_t* irtifa, uint8_t* uyduS)
{
	uint8_t irtifaUzunluk = strlen((char*) irtifa);
	uint8_t uyduSUzunluk = strlen((char*) uyduS);
	uint16_t donut = 0;

	if(irtifaUzunluk > 0 && irtifaUzunluk < 9)
		yeni->irtifa = atof((char*) irtifa);
	else{
		donut |= GPS_ERR_IRTFA;
	}

	if(uyduSUzunluk > 0 && uyduSUzunluk < 3)
			yeni->irtifa = atof((char*) irtifa);
	else{
			return (donut | GPS_ERR_UYDUS);
	}
		yeni->uyduS = (uint8_t) atoi((char*) uyduS);

	return donut;
}

/**
* @brief 	gpsBaslat
* 						Bu fonksiyon gps kutuphanesini baslatir.
* @param 	yeni: 		Gps bilgilerini tutan yapinin adresi.
* 			gpsTambon:	GPGGA verilerinin bulundugu tamponun adresi.
* @retval 	GPS_RTRN: 	Ayiklama hata kodu.
*/
void gpsBaslat(gps* yeni, uint8_t* gpsTampon){
	tampon = gpsTampon;
	yeni->boylam = 0.0;
	yeni->enlem = 0.0;
	yeni->boylamYon = 'x';
	yeni->enlemYon = 'x';
	yeni->saat = 0;
	yeni->dakika = 0;
	yeni->saniye = 0;
	yeni->irtifa = 0;
	yeni->uyduS = 0;
}

/**
* @brief 	gpsAyikla
* 						Bu fonksiyon gps verilerini ayiklar.
* @param 	yeni: 		Gps bilgilerini tutan yapinin adresi.
* @retval 	none
*/
GPS_RTRN gpsAyikla(gps* yeni){
	//veriler[]:
	//0-zaman
	//1-enlem
	//2-enlem kutup
	//3-boylam
	//4-boylam yon
	//5-kalite
	//6-uydu sayisi
	//7-yatay dogruluk
	//8-irtifa
	//9-irtifa birim
	//10-*******
	//11-birim
	//12-konum yasi (s)
	//13-DGPS ID

	strcpy((char*)gpsBuf, (char*)tampon);
	uint8_t veriler [14][11] = {'\0'};

	uint8_t* adres;
	adres = gpsBuf;

	int k = 0;
	for(int sayac = 0; sayac < 100 && adres[sayac] != '*'; sayac++){
		if(adres[sayac] == ',')
			k++;
	}

	if(k == 13){
		for(k = 0; *adres != '*'; k++){
			adres = adres + (k > 0);
			int j = 0;
			for(j = 0; *adres != ',' && *adres != '*'; j++){
				veriler[k][j] = *adres;
				adres = adres + 1;
			}
			veriler[k][j] = '\0';
		}
		GPS_RTRN ZAMAN = zamanAyikla(yeni, veriler[0]);
		GPS_RTRN KONUM = konumAyikla(yeni, (uint8_t*) &veriler[0][0]);
		GPS_RTRN IRTIFA = irtifa_uyduSAyikla(yeni, veriler[8], veriler[6]);

		return ZAMAN | KONUM | IRTIFA;
	}
	else{
		uint8_t yazi[22];
		sprintf((char*)yazi, "HATA veri sayisi: %d", k);
//		HAL_UART_Transmit(&huart2, yazi, strlen(yazi), 20);
//		HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, 20);
		return GPS_ERR;
	}

#ifdef uartDebug
	for(int i = 0; i<k; i++){
		HAL_UART_Transmit(&huart2, veriler[i], strlen((char*) veriler[i]), 10);
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 20);
	}
//println_int((int) checksum);
#endif

}

/**
* @brief 	gpsYazdir
* 						Bu fonksiyon gps verilerini yazdirir.
* @param 	yeni: 		gps bilgilerini tutan yapinin adresi.
* @retval 	GPS_RTRN: 	ayiklama hata kodu.
*/
void gpsYazdir(gps* yeni)
{
	uint8_t gecici[30];
	printer((uint8_t*)"\r\n\n*************************************\r\n\n");
	sprintf((char*)gecici, "enlem = %f %c\r\n", yeni->enlem, yeni->enlemYon);
	printer(gecici);
	sprintf((char*)gecici, "boylam = %f %c\r\n", yeni->boylam, yeni->boylamYon);
	printer(gecici);
	sprintf((char*)gecici, "%u.%u.%u\r\n", yeni->saat, yeni->dakika, yeni->saniye);
	printer(gecici);
	sprintf((char*)gecici, "uydu sayisi = %u\r\n", yeni->uyduS);
	printer(gecici);
	sprintf((char*)gecici, "irtifa = %.2f\r\n", yeni->irtifa);
	printer(gecici);
}

/**
* @brief 	gpsHataYazdir
* 						Bu fonksiyon gps'te olusan hata verilerini yazdirir.
* @param 	hata: 		gps hata kodu.
* @retval 	none
*/
void gpsHataYazdir(GPS_RTRN hata)
{
	uint8_t hataKodlari[8][6] = {"ENL", "BYL", "ENLYN", "BYLYN", "IRTIF", "UYDUS", "ZMN", "GPS"};
	if(hata > 0){
		HAL_UART_Transmit(&huart2, (uint8_t*) "HATA: ", 6, 10);
		for(int i = 0; i<8; i++)
		{
			if(((int)pow((float)2, (float)i) & hata) == pow(2, i)){
				HAL_UART_Transmit(&huart2, hataKodlari[i], strlen((char*) hataKodlari[i]), 20);
				HAL_UART_Transmit(&huart2, (uint8_t*) " ", 1, 10);
			}
		}
	}
}
