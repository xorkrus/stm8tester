#include "stm8s.h"
#include "stm8s_adc1.h"
#include "stm8s_clk.h"
#include "delay.h"
#include "HD44780.h"

//pins C1-C6 - digital probes
//pins B0, B1, B2 - analog testpoints
#define TP1 0
//ADC1_CHANNEL_0
#define TP2 1
//ADC1_CHANNEL_1
#define TP3 2
//ADC1_CHANNEL_2

/* Settings for capacitance measurement (for ATMega8 interesting)
The test of whether there is a capacitor takes a relatively long time, with more than 50 ms per test procedure is expected to
In all six possible test events is an extension of the test period by about 0.3 s to 0.5 s.
With CAP_TEST_MODE used to set the tests.

Meanings of the bits (7 = MSB):
7:6 not used

5-4 Test Mode
00: capacitor measurement disabled
01: capacitor measurement for an adjustable pin combination (both ways), extended testing time by about 200ms 120th ..
10: capacitor measurement for every 6-pin combinations, extended test times by about 300 .. 500ms

3:2 The first pin of the pin-selected combination (0 .. 2), only decisive when bits 5:4 = 01

1-0 second pin of the pin-selected combination (0 .. 2), only important when bits 5:4 = 01
*/
uint8_t ctmode = 0b00100010; // measure for all 6-pin combinations

/*
Exact values of the resistors used in ohms.
The nominal value for R_L is 680 ohms, for 470kOhm R_H
The program for deviations from these values (eg due to component tolerances)
To calibrate, enter the resistor values in ohms in the following defines:
*/
const	unsigned int rlval = 672;			//R_L; Normwert 680 Ohm
const	unsigned int rhval = 4690;		//R_H; Normwert 470000 Ohm, durch 100 dividiert angeben

/*
Factors for Kapatitatsmessung with capacitors
These factors depend on manufacturing tolerances from the AVR and thus have to be adjusted, if necessary
H_CAPACITY_FACTOR for the test with 470k resistor (low capacity)
L_CAPACITY_FACTOR for the measurement with 680-ohm resistor (high capacity)
The entire range is about 0.2 nF to 7300?F.
*/
const	unsigned int H_CAPACITY_FACTOR = 394;
const	unsigned int L_CAPACITY_FACTOR = 283;

const	unsigned char TestRunning[]  = "Testing ...";
const	unsigned char Bat[]  = "Battery ";
const	unsigned char BatWeak[]  = "weak";
const	unsigned char BatEmpty[]  = "empty!";
const	unsigned char TestFailed1[]  = "No, unknown, or";
const	unsigned char TestFailed2[]  = "damaged ";
const	unsigned char Bauteil[]  = "part";
const	unsigned char Unknown[]  = " unknown";
const	unsigned char Diode[]  = "Diode: ";
const	unsigned char DualDiode[]  = "Double diode ";
const	unsigned char TwoDiodes[]  = "2 diodes";
const	unsigned char Antiparallel[]  = "anti-parallel";
const	unsigned char InSeries[]  = "serial A=";
const	unsigned char K1[]  = ";C1=";
const	unsigned char K2[]  = ";C2=";
const	unsigned char GAK[]  = "GAC=";
const	unsigned char NextK[]  = ";C=";
const	unsigned char K[]  = "C=";
const	unsigned char Triac[]  = "Triac";
const	unsigned char Thyristor[]  = "Thyristor";
const unsigned char OrBroken[]  = "or damaged ";
const	unsigned char Resistor[]  = "Resistor: ";
const	unsigned char Capacitor[]  = "Capacitor: ";
const	unsigned char mosfet[]  = "-MOS";
const	unsigned char emode[]  = "-E";
const	unsigned char dmode[]  = "-D";
const	unsigned char jfet[]  = "-JFET";
const	unsigned char A1[]  = ";A1=";
const	unsigned char A2[]  = ";A2=";
const	unsigned char NullDot[]  = "0,";
const	unsigned char GateCap[]  = " C=";
const	unsigned char hfestr[]  ="hFE=";
const	unsigned char NPN[]  = "NPN";
const	unsigned char PNP[]  = "PNP";
const	unsigned char bstr[]  = " B=";
const	unsigned char cstr[]  = ";C=";
const	unsigned char estr[]  = ";E=";
const	unsigned char gds[]  = "GDS=";
const	unsigned char Uf[]  = "Uf=";
const	unsigned char vt[]  = "Vt=";
const	unsigned char mV[]  = "mV";
const	unsigned char Anode[]  = "A=";
const	unsigned char Gate[]  = "G=";
const	unsigned char CA[]  = "CA";
const	unsigned char CC[]  = "CC";
const	unsigned char TestTimedOut[]  = "Timeout!";

const	unsigned char DiodeIcon[]  = {4,31,31,14,14,4,31,4,0};	//Dioden-Icon

/*If the define "WDT_enabled" removed, the watchdog on startup
  no longer active. This is useful for testing and debugging purposes.
  For normal use of the tester, the watchdog should also be activated without fail!
*/
//#define WDT_enabled

uint16_t ReadADC(uint8_t tp)
{
	uint8_t i;
	uint16_t value = 0;
	uint8_t oldCr = GPIOB->CR1;
	uint8_t oldDdr = GPIOB->DDR;
	
	GPIOB->DDR &= (uint8_t)(~(1 << tp));
	GPIOB->CR1 &= (uint8_t)(~(1 << tp));
	GPIOB->CR2 &= (uint8_t)(~(1 << tp));
	
	ADC1_DeInit();
	ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, tp, ADC1_PRESSEL_FCPU_D12,
	ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, tp, DISABLE);

	for( i = 0; i < 16; i++)
	{
		ADC1_StartConversion();
		while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC))
			;
		value += ADC1_GetConversionValue(); // read ADC conversion data, the first low, then high
	}
	
	ADC1_DeInit();

	GPIOB->DDR = oldDdr;
	GPIOB->CR1 = oldCr;

	return (value >> 4);
}

struct Diode {
	uint8_t Anode;
	uint8_t Cathode;
	int Voltage;
};

void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin);
void DischargePin(uint8_t PinToDischarge, uint8_t DischargeDirection);
void lcd_show_format_cap(char outval[], uint8_t strlength, uint8_t CommaPos);
void ReadCapacity(uint8_t HighPin, uint8_t LowPin);		//Kapazitatsmessung nur auf Mega8 verfugbar

void itoa(uint32_t v, char *buf)
{
	char tmp[17];
	int8_t i = 0;
	
	while(v != 0)
	{
		tmp[i++] = '0' + v % 10;
		v /= 10;
	}
	
	if (i == 0)
	{
		*buf++ = '0';
	}
	else
	{
		for (; i > 0; i--)
		{
			*buf++ = tmp[i-1];
		}
	}
	
	*buf = 0;
}

#define PART_NONE 0
#define PART_DIODE 1
#define PART_TRANSISTOR 2
#define PART_FET 3
#define PART_TRIAC 4
#define PART_THYRISTOR 5
#define PART_RESISTOR 6
#define PART_CAPACITOR 7

#define PART_MODE_N_E_MOS 1
#define PART_MODE_P_E_MOS 2
#define PART_MODE_N_D_MOS 3
#define PART_MODE_P_D_MOS 4
#define PART_MODE_N_JFET 5
#define PART_MODE_P_JFET 6

#define PART_MODE_NPN 1
#define PART_MODE_PNP 2

struct Diode diodes[6];
uint8_t NumOfDiodes;

uint8_t b,c,e;			//Anschlusse des Transistors
unsigned long lhfe;		//Verstarkungsfaktor
uint8_t PartReady;		//Bauteil fertig erkannt
unsigned int hfe[2];		//Verstarkungsfaktoren
unsigned int uBE[2];	//B-E-Spannung fur Transistoren
uint8_t PartMode;
uint8_t tmpval, tmpval2;

uint8_t ra, rb;				//Widerstands-Pins
unsigned int rv[2];			//Spannungsabfall am Widerstand
unsigned int radcmax[2];	//Maximal erreichbarer ADC-Wert (geringer als 1023, weil Spannung am Low-Pin bei Widerstandsmessung uber Null liegt)
uint8_t ca, cb;				//Kondensator-Pins
uint8_t cp1, cp2;			//Zu testende Kondensator-Pins, wenn Messung fur einzelne Pins gewahlt

unsigned long cv;

uint8_t PartFound, tmpPartFound;	//das gefundene Bauteil
char outval[8];
unsigned int adcv[4];
unsigned int gthvoltage;	//Gate-Schwellspannung
uint8_t tmpval, tmpval2;

char outval2[6];

int main(void) 
{
	uint16_t value;
	char tmpBuf[17];

	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);

	//InitClocks();
	
	InitLcd(GPIOD, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_HNIB);
////////////////////////////////////
	//TODO ADC Prescaler = 8
	cp1 = (ctmode & 12) >> 2;
	cp2 = ctmode & 3;
	ctmode = (ctmode & 48) >> 4;
  //TODO watchdog 2s
	//!!SendCommand(0x40);//custom character
	//!!Out((char *)DiodeIcon);
	//!!SendData(0);
	PartFound = PART_NONE;
	tmpPartFound = PART_NONE;
	NumOfDiodes = 0;
	PartReady = 0;
	PartMode = 0;
	ca = 0;
	cb = 0;
	ClearLcd(0);
	Outline(0, TestRunning);
	CheckPins(TP1, TP2, TP3);
	CheckPins(TP1, TP3, TP2);
	CheckPins(TP2, TP1, TP3);
	CheckPins(TP2, TP3, TP1);
	CheckPins(TP3, TP2, TP1);
	CheckPins(TP3, TP1, TP2);

/*	if(((PartFound == PART_NONE) || (PartFound == PART_RESISTOR) || (PartFound == PART_DIODE)) && (ctmode > 0)) {
		//Kondensator entladen; sonst ist evtl. keine Messung moglich
			R_PORT = 0;
			R_DDR = (1<<(TP1 * 2)) | (1<<(TP2 * 2)) | (1<<(TP3 * 2));
			_delay_ms(10);
			R_DDR = 0;
		//Kapazitat in allen 6 Pin-Kombinationen messen
		if(ctmode == 1) {
			ReadCapacity(cp1, cp2);
			ReadCapacity(cp2, cp1);
		} else {
			ReadCapacity(TP3, TP1);
			ReadCapacity(TP3, TP2);
			ReadCapacity(TP2, TP3);
			ReadCapacity(TP2, TP1);
			ReadCapacity(TP1, TP3);
			ReadCapacity(TP1, TP2);
		}
	}*/

	ClearLcd(0);
	if(PartFound == PART_DIODE) {
		if(NumOfDiodes == 1) {
			//Standard-Diode
			Out(Diode);	//"Diode: "
			Out(Anode);
			SendData(diodes[0].Anode + 49);
			Out(NextK);//";K="
			SendData(diodes[0].Cathode + 49);
			SetLine(1);	//2. Zeile
			Out(Uf);	//"Uf = "
			itoa(diodes[0].Voltage, tmpBuf);
			Out(tmpBuf);
			//lcd_string(itoa(diodes[0].Voltage, outval, 10));
			Out(mV);
			goto end;
		} else if(NumOfDiodes == 2) {
		//Doppeldiode
			if(diodes[0].Anode == diodes[1].Anode) {
				//Common Anode
				Out(DualDiode);	//Doppeldiode
				Out(CA);	//"CA"	
				SetLine(1); //2. Zeile
				Out(Anode);
				SendData(diodes[0].Anode + 49);
				Out(K1);	//";K1="
				SendData(diodes[0].Cathode + 49);
				Out(K2);	//";K2="
				SendData(diodes[1].Cathode + 49);
				goto end;
			} else if(diodes[0].Cathode == diodes[1].Cathode) {
				//Common Cathode
				Out(DualDiode);	//Doppeldiode
				Out(CC);	//"CC"
				SetLine(1); //2. Zeile
				Out(K);	//"K="
				SendData(diodes[0].Cathode + 49);
				Out(A1);		//";A1="
				SendData(diodes[0].Anode + 49);
				Out(A2);		//";A2="
				SendData(diodes[1].Anode + 49);
				goto end;
			} else if ((diodes[0].Cathode == diodes[1].Anode) && (diodes[1].Cathode == diodes[0].Anode)) {
				//Antiparallel
				Out(TwoDiodes);	//2 Dioden
				SetLine(1); //2. Zeile
				Out(Antiparallel);	//Antiparallel
				goto end;
			}
		} else if(NumOfDiodes == 3) {
			//Serienschaltung aus 2 Dioden; wird als 3 Dioden erkannt
			b = 3;
			c = 3;
			/* Uberprufen auf eine fur eine Serienschaltung von 2 Dioden mogliche Konstellation
				Dafur mussen 2 der Kathoden und 2 der Anoden ubereinstimmen.
				Das kommmt daher, dass die Dioden als 2 Einzeldioden und ZUSATZLICH als eine "gro?e" Diode erkannt werden.
			*/
			if((diodes[0].Anode == diodes[1].Anode) || (diodes[0].Anode == diodes[2].Anode)) b = diodes[0].Anode;
			if(diodes[1].Anode == diodes[2].Anode) b = diodes[1].Anode;

			if((diodes[0].Cathode == diodes[1].Cathode) || (diodes[0].Cathode == diodes[2].Cathode)) c = diodes[0].Cathode;
			if(diodes[1].Cathode == diodes[2].Cathode) c = diodes[1].Cathode;
			if((b<3) && (c<3)) {
				Out(TwoDiodes);//2 Dioden
				SetLine(1); //2. Zeile
				Out(InSeries); //"in Serie A="
				SendData(b + 49);
				Out(NextK);
				SendData(c + 49);
				goto end;
			}
		}
	} else if (PartFound == PART_TRANSISTOR) {
		if(PartReady == 0) {	//Wenn 2. Prufung nie gemacht, z.B. bei Transistor mit Schutzdiode
			hfe[1] = hfe[0];
			uBE[1] = uBE[0];
		}
		if((hfe[0]>hfe[1])) {	//Wenn der Verstarkungsfaktor beim ersten Test hoher war: C und E vertauschen!
			uint8_t tmp;
			hfe[1] = hfe[0];
			uBE[1] = uBE[0];
			tmp = c;
			c = e;
			e = tmp;
		}

		if(PartMode == PART_MODE_NPN) {
			Out(NPN);
		} else {
			Out(PNP);
		}
		Out(bstr);	//B=
		SendData(b + 49);
		Out(cstr);	//;C=
		SendData(c + 49);
		Out(estr);	//;E=
		SendData(e + 49);
		SetLine(1); //2. Zeile
		//Verstarkungsfaktor berechnen
		//hFE = Emitterstrom / Basisstrom
		lhfe = hfe[1];

//TODO		#ifdef UseM8
			lhfe *= (((unsigned long)rhval * 100) / (unsigned long)rlval);	//Verhaltnis von High- zu Low-Widerstand
//		#else
//			lhfe *= M48_RH_RL_RATIO;
//		#endif
		if(uBE[1]<11) uBE[1] = 11;
		lhfe /= uBE[1];
		hfe[1] = (unsigned int) lhfe;
		Out(hfestr);	//"hFE="
		itoa(hfe[1], tmpBuf);
		Out(tmpBuf);
		//lcd_string(utoa(hfe[1], outval, 10));
		SetCursor(2,7);			//Cursor auf Zeile 2, Zeichen 7
		if(NumOfDiodes > 2) {	//Transistor mit Schutzdiode
			//TODO lcd_data(LCD_CHAR_DIODE);	//Diode anzeigen
			SendData('D');
		} else {
//			#ifdef UseM8
				SendData(' ');
//			#endif
		}
//		#ifdef UseM8
			for(c=0;c<NumOfDiodes;c++) {
				if(((diodes[c].Cathode == e) && (diodes[c].Anode == b) && (PartMode == PART_MODE_NPN)) || ((diodes[c].Anode == e) && (diodes[c].Cathode == b) && (PartMode == PART_MODE_PNP))) {
					Out(Uf);	//"Uf="
					itoa(diodes[c].Voltage, tmpBuf);
					Out(tmpBuf);
					SendData('m');
					goto end;
				}
			}
//		#endif
		goto end;
	} else if (PartFound == PART_FET) {	//JFET oder MOSFET
		if(PartMode&1) {	//N-Kanal
			SendData('N');
		} else {
			SendData('P');	//P-Kanal
		}
		if((PartMode==PART_MODE_N_D_MOS) || (PartMode==PART_MODE_P_D_MOS)) {
			Out(dmode);	//"-D"
			Out(mosfet);	//"-MOS"
		} else {
			if((PartMode==PART_MODE_N_JFET) || (PartMode==PART_MODE_P_JFET)) {
				Out(jfet);	//"-JFET"
			} else {
				Out(emode);	//"-E"
				Out(mosfet);	//"-MOS"
			}
		}
/*TODO		#ifdef UseM8	//Gatekapazitat
			if(PartMode < 3) {	//Anreicherungs-MOSFET
				lcd_eep_string(GateCap);	//" C="
				ReadCapacity(b,e);	//Messung
				hfe[0] = (unsigned int)cv;
				if(hfe[0]>2) hfe[0] -= 3;
				utoa(hfe[0], outval2, 10);

				tmpval = strlen(outval2);
				tmpval2 = tmpval;
				if(tmpval>4) tmpval = 4;	//bei Kapazitat >100nF letze Nachkommastelle nicht mehr angeben (passt sonst nicht auf das LCD)
				lcd_show_format_cap(outval2, tmpval, tmpval2);
				lcd_data('n');
			}
		#endif*/
		SetLine(1); //2. Zeile
		Out(gds);	//"GDS="
		SendData(b + 49);
		SendData(c + 49);
		SendData(e + 49);
		if((NumOfDiodes > 0) && (PartMode < 3)) {	//MOSFET mit Schutzdiode; gibt es nur bei Anreicherungs-FETs
			//TODO lcd_data(LCD_CHAR_DIODE);	//Diode anzeigen
			SendData('D');
		} else {
			SendData(' ');	//Leerzeichen
		}
		if(PartMode < 3) {	//Anreicherungs-MOSFET
			gthvoltage=(gthvoltage/8);
			Out(vt);
			itoa(gthvoltage, tmpBuf);
			Out(tmpBuf);	//Gate-Schwellspannung, wurde zuvor ermittelt
			SendData('m');
		}
		goto end;
	} else if (PartFound == PART_THYRISTOR) {
		Out(Thyristor);	//"Thyristor"
		SetLine(1); //2. Zeile
		Out(GAK);	//"GAK="
		SendData(b + 49);
		SendData(c + 49);
		SendData(e + 49);
		goto end;
	} else if (PartFound == PART_TRIAC) {
		Out(Triac);	//"Triac"
		SetLine(1); //2. Zeile
		Out(Gate);
		SendData(b + 49);
		Out(A1);		//";A1="
		SendData(e + 49);
		Out(A2);		//";A2="
		SendData(c + 49);
		goto end;
		} else if(PartFound == PART_RESISTOR) {
			Out(Resistor); //"Widerstand: "
			SendData(ra + 49);	//Pin-Angaben
			SendData('-');
			SendData(rb + 49);
			SetLine(1); //2. Zeile
			if(rv[0]>512) {		//Uberprufen, wie weit die an den Testwiderstanden anliegenden Spannungen von 512 abweichen
				hfe[0] = (rv[0] - 512);
			} else {
				hfe[0] = (512 - rv[0]);
			}
			if(rv[1]>512) {
				hfe[1] = (rv[1] - 512);
			} else {
				hfe[1] = (512 - rv[1]);
			}
			if(hfe[0] > hfe[1])  {
				radcmax[0] = radcmax[1];
				rv[0] = rv[1];	//Ergebnis verwenden, welches naher an 512 liegt (bessere Genauigkeit)
				rv[1] = rhval;	//470k-Testwiderstand	
			} else {
				rv[1] = rlval;	//680R-Testwiderstand
			}
			if(rv[0]==0) rv[0] = 1;
			lhfe = (unsigned long)((unsigned long)((unsigned long)rv[1] * (unsigned long)rv[0]) / (unsigned long)((unsigned long)radcmax[0] - (unsigned long)rv[0]));	//Widerstand berechnen
			itoa(lhfe,outval);

			if(rv[1]==rhval) {	//470k-Widerstand?
				char *tmpS = outval;
				ra = 0;
				while(*tmpS++)
					ra++;
					
				//ra = strlen(outval);	//Notig, um Komma anzuzeigen
				for(rb=0;rb<ra;rb++) {
					SendData(outval[rb]);
					if(rb==(ra-2)) SendData('.');	//Komma
				}
				SendData ('k'); //Kilo-Ohm, falls 470k-Widerstand verwendet
			} else {
				Out(outval);
			}
			Out("Ohm");
			//lcd_data(LCD_CHAR_OMEGA);	//Omega fur Ohm 
			goto end;
/*TODO
		} else if(PartFound == PART_CAPACITOR) {	//Kapazitatsmessung auch nur auf Mega8 verfugbar
			lcd_eep_string(Capacitor);
			lcd_data(ca + 49);	//Pin-Angaben
			lcd_data('-');
			lcd_data(cb + 49);
			Line2(); //2. Zeile
			tmpval2 = 'n';
			if(cv > 99999) {	//ab 1µF
				cv /= 1000;
				tmpval2 = LCD_CHAR_U;
			}
			ultoa(cv, outval, 10);
			tmpval = strlen(outval);
			lcd_show_format_cap(outval, tmpval, tmpval);
			lcd_data(tmpval2);
			lcd_data('F');
			goto end;
	#endif*/
	}
//	#ifdef UseM8	//Unterscheidung, ob Dioden gefunden wurden oder nicht nur auf Mega8
		if(NumOfDiodes == 0) {
			//Keine Dioden gefunden
			Out(TestFailed1); //"Kein,unbek. oder"
			SetLine(1); //2. Zeile
			Out(TestFailed2); //"defektes "
			Out(Bauteil);
		} else {
			Out(Bauteil);
			Out(Unknown); //" unbek."
			SetLine(1); //2. Zeile
			Out(OrBroken); //"oder defekt"
			SendData(NumOfDiodes + 48);
			SendData('D');//lcd_data(LCD_CHAR_DIODE);
		}
//	#endif
	end:

////////////////////////////////////
	while(1)
	{
//		value = ReadAdc(7);
//		itoa(value, v);
//		Outline(0, "                ");
//		Outline(0, v);
//		delay(MS(50));
		;
	}
}

void DischargePin(uint8_t PinToDischarge, uint8_t DischargeDirection) 
{
	/*
Connecting a component short (10 ms) set to a particular potential
This function is provided for discharging of MOSFET gate to protect diodes u.ä. MOSFETs to recognize können
Parameters:
Pinto discharge: to be unloaded pin
Discharge direction: 0 = to ground (N-channel FET), 1 = to positive (P-channel FET)
*/
	uint8_t tmpval;
	tmpval = (PinToDischarge * 2 + 1);		//nötig wegen der Anordnung der Widerstände

	GPIOC->DDR |= (1<<tmpval);			//Pin auf Ausgang und über R_L auf Masse

	if(DischargeDirection)
	{
		GPIOC->ODR |= (1 << tmpval);			//R_L aus
	}
		
	delay(MS(10));
	GPIOC->DDR &= ~(1<<tmpval);			//Pin wieder auf Eingang
	if(DischargeDirection) 
		GPIOC->ODR &= ~(1<<tmpval);			//R_L aus
}
/*
Function to test the properties of the component at the specified pin assignment
Parameters:
HighPin: pin, which is initially set to positive potential
LowPin: pin, which is initially set at a negative potential
TristatePin: pin, which is initially left open

During testing TristatePin is switched course, also have a positive or negative.
*/
/*
HighPin is placed firmly on Vcc
LowPin is placed over R_L to GND
TristatePin is switched to highZ	
*/

void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin) {
	unsigned int adcv[6];
	uint8_t tmpval, tmpval2;
	//TODO wdt_reset();
	//Pins setzen
	tmpval = (LowPin * 2 + 1);
	GPIOC->DDR = (1 << tmpval);//Low-pin to output and to ground via R_L
	GPIOC->CR1 = (1 << tmpval);//!!! all others - HiZ
	GPIOC->ODR = 0;
	GPIOB->DDR = (1 << HighPin);
	GPIOB->CR1 = (1 << HighPin);//!!! all others - HiZ
	GPIOB->ODR = (1 << HighPin);////High-pin to output and Vcc
	delay(MS(5));
	//Some MOSFETs must be the gate (TristatePin) first discharge
	//N-Kanal:
	DischargePin(TristatePin,0);
	//voltage at Low-pin determined
	adcv[0] = ReadADC(LowPin);
	if(adcv[0] < 200) goto next;	//Locks the device now?
	//else: Unload for P-channel (gate to plus)
	DischargePin(TristatePin,1);
	//voltage at Low-pin determined
	adcv[0] = ReadADC(LowPin);

	next:

	if(adcv[0] > 19) {//Bauteil leitet ohne Steuerstrom etwas
		//Test on N-JFET, or even conducting N-MOSFET
		GPIOC->DDR |= (2<<(TristatePin*2 + 1));//Tristate Pin (suspected Gate) via R_H to ground
		GPIOC->CR1 |= (2<<(TristatePin*2 + 1));//!!!
		delay(MS(20));
		adcv[1] = ReadADC(LowPin);		//Measure voltage at the suspected source
		GPIOC->ODR |= (2<<(TristatePin*2 + 1)); //Tristate Pin (suspected Gate) via R_H to Plus
		delay(MS(20));
		adcv[2] = ReadADC(LowPin);		//Measure voltage at the suspected source
		//If it is a normally on MOSFET or JFET has adcv[1]> adcv[0]
		if(adcv[2]>(adcv[1]+100)) {
			//Measure voltage at the gate, to distinguish between the MOSFET and JFET
			GPIOB->ODR = 0;
			GPIOB->DDR = (1 << LowPin);//Low-Pin sets to ground
			GPIOB->CR1 = (1 << LowPin);//!!! all others to HiZ
			tmpval = (HighPin * 2 + 1);
			GPIOC->DDR |= (1 << tmpval);
			GPIOC->CR1 |= (1 << tmpval);//!!!
			GPIOC->ODR |= (1 << tmpval);//High-Pin output with R_L to Vcc
			delay(MS(20));
			adcv[2] = ReadADC(TristatePin);		//Measure voltage at the suspected gate
			if(adcv[2]>800) {	//MOSFET
				PartFound = PART_FET;			//N-Kanal-MOSFET
				PartMode = PART_MODE_N_D_MOS;	//Verarmungs-MOSFET
			} else {	//JFET (pn-Ubergang zwischen G und S leitet)
				PartFound = PART_FET;			//N-Kanal-JFET
				PartMode = PART_MODE_N_JFET;
			}
			b = TristatePin;
			c = HighPin;
			e = LowPin;
		}
		
		//Test for P-JFET, or even conducting P-MOSFET
		//Low-Pin (suspected drain) firmly on earth, tri-pin (suspected Gate) is still about to R_H Plus
		GPIOB->ODR = 0;
		GPIOB->CR1 = (1 << LowPin);//!!! all others to HiZ
		GPIOB->DDR = (1 << LowPin);
		tmpval = (HighPin * 2 + 1);
		GPIOC->DDR |= (1 << tmpval);
		GPIOC->CR1 |= (1 << tmpval);//!!!
		GPIOC->ODR |= (1 << tmpval); //High-pin to Vcc via R_L
		delay(MS(20));
		adcv[1] = ReadADC(HighPin);		//Measure voltage at the suspected source
		GPIOC->ODR = (1 << tmpval);  //Tristate Pin (suspected Gate) via R_H to ground
		delay(MS(20));
		adcv[2] = ReadADC(HighPin);		//Measure voltage at the suspected source
		//If it is a normally on MOSFET P-or P-JFET, had adcv [0]> adcv [1] are
		if(adcv[1]>(adcv[2]+100)) {
			//Measure voltage at the gate, to distinguish between the MOSFET and JFET
			GPIOB->ODR = (1 << HighPin);
			GPIOB->CR1 = (1 << HighPin);//!!! all others to HiZ
			GPIOB->DDR = (1 << HighPin);//High-pin firmly Plus
			delay(MS(20));
			adcv[2] = ReadADC(TristatePin);		//Voltage at the gate suspected measure
			if(adcv[2]<200) {	//MOSFET
				PartFound = PART_FET;			//P-Kanal-MOSFET
				PartMode = PART_MODE_P_D_MOS;	//Verarmungs-MOSFET
			} else {	//JFET (pn-Ubergang zwischen G und S leitet)
				PartFound = PART_FET;			//P-Kanal-JFET
				PartMode = PART_MODE_P_JFET;
			}
			b = TristatePin;
			c = LowPin;
			e = HighPin;
		}
	}
	//Pins erneut setzen
	tmpval = (LowPin * 2 + 1);
	GPIOC->DDR = (1 << tmpval);
	GPIOC->CR1 = (1 << tmpval); //!!! all others to HiZ
	GPIOC->ODR = 0;//Low-Pin auf Ausgang und uber R_L auf Masse
	GPIOB->DDR = (1 << HighPin);
	GPIOB->CR1 = (1 << HighPin);// !!!
	GPIOB->ODR = (1 << HighPin);//High-Pin fest auf Vcc
	delay(MS(5));
	
	if(adcv[0] < 200) {	//If the component is no continuity between HighPin and has LowPin
		//Test auf pnp
		tmpval2 = (TristatePin * 2 + 1);
		GPIOC->DDR |= (1 << tmpval2);//Tristate-Pin uber R_L auf Masse, zum Test auf pnp
		GPIOC->CR1 |= (1 << tmpval2);//!!!
		delay(MS(2));
		adcv[1] = ReadADC(LowPin);		//Spannung messen
		if(adcv[1] > 700) {
			//Bauteil leitet => pnp-Transistor o.a.
			//Gain factor measured in both directions
			GPIOC->DDR = (1 << tmpval);
			GPIOC->CR1 = (1 << tmpval);
			tmpval2++;
			GPIOC->DDR |= (1 << tmpval2);
			GPIOC->CR1 |= (1 << tmpval2);
			delay(MS(10));
			adcv[1] = ReadADC(LowPin);		//Low voltage on the pin (assumed collector) measure
			adcv[2] = ReadADC(TristatePin);	//Base voltage measure
			//Prooven if test already run times
			if((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) PartReady = 1;
			hfe[PartReady] = adcv[1];
			uBE[PartReady] = adcv[2];

			if(PartFound != PART_THYRISTOR) {
				if(adcv[2] > 200) {
					PartFound = PART_TRANSISTOR;	//PNP transistor found (base is "up" solid)
					PartMode = PART_MODE_PNP;
				} else {
					if(adcv[0] < 20) {	//Forward voltage in the off state is low enough? (otherwise D-mode FETs are mistakenly identified as E-mode)
					 	PartFound = PART_FET;			//P-channel MOSFET found (base / gate is not pulled "up")
						PartMode = PART_MODE_P_E_MOS;
						//Measurement of the gate threshold voltage
						gthvoltage = 0;
/*TODO!!!		tmpval = (1<<LowPin);
						tmpval2 = R_DDR;
						ADMUX = TristatePin | (1<<REFS0);
						for(b=0;b<13;b++) {
							wdt_reset();
							DischargePin(TristatePin,1);
							while (!(ADC_PIN&tmpval));  // Warten, bis der MOSFET schaltet und Drain auf high geht
							R_DDR = 0;
							ADCSRA |= (1<<ADSC);
							while (ADCSRA&(1<<ADSC));
							gthvoltage += (1023 - ADCW);
							R_DDR = tmpval2;
						}
						gthvoltage *= 3;	//Umrechnung in mV, zusammen mit der Division durch 8 (bei der LCD-Anzeige)*/
					}
				}
				b = TristatePin;
				c = LowPin;
				e = HighPin;
			}
		}

		//Tristate (assumed basis) Plus, for testing on an npn
		GPIOB->ODR = 0;  //Low-Pin fest auf Masse
		tmpval = (TristatePin * 2 + 1);
		tmpval2 = (HighPin * 2 + 1);
		GPIOC->DDR = (1 << tmpval) | (1 << tmpval2);
		GPIOC->CR1 = (1 << tmpval) | (1 << tmpval2);
		GPIOC->ODR = (1 << tmpval) | (1 << tmpval2);//High-Pin und Tristate-Pin uber R_L auf Vcc
		GPIOB->DDR = (1 << LowPin);
		GPIOB->CR1 = (1 << LowPin);
		delay(MS(10));
		adcv[1] = ReadADC(HighPin);		//Spannung am High-Pin messen
		if(adcv[1] < 500) {
			if(PartReady==1) goto testend;
			//Bauteil leitet => npn-Transistor o.a.

			//Test auf Thyristor:
			//Gate entladen
			
			GPIOC->ODR = (1 << tmpval2);			//Tristate-Pin (Gate) uber R_L auf Masse
			GPIOC->CR1 = (1 << tmpval2);
			delay(MS(10));
			GPIOC->DDR = (1 << tmpval2);			//Tristate-Pin (Gate) hochohmig
			//Test auf Thyristor
			delay(MS(5));
			adcv[3] = ReadADC(HighPin);		//Spannung am High-Pin (vermutete Anode) erneut messen
			
			GPIOC->ODR = 0;						//High-Pin (vermutete Anode) auf Masse
			delay(MS(5));
			GPIOC->ODR = (1 << tmpval2);			//High-Pin (vermutete Anode) wieder auf Plus
			delay(MS(5));
			adcv[2] = ReadADC(HighPin);		//Spannung am High-Pin (vermutete Anode) erneut messen
			if((adcv[3] < 500) && (adcv[2] > 900)) {	//Nach Abschalten des Haltestroms muss der Thyristor sperren
				//war vor Abschaltung des Triggerstroms geschaltet und ist immer noch geschaltet obwohl Gate aus => Thyristor
				uint16_t tmpAdc;
				PartFound = PART_THYRISTOR;
				//Test auf Triac
				GPIOC->DDR = 0;
				GPIOC->CR1 = 0;
				GPIOC->ODR = 0;
				GPIOB->ODR = (1 << LowPin);	//Low-Pin fest auf Plus
				GPIOB->CR1 = (1 << LowPin);
				delay(MS(5));
				GPIOC->DDR = (1 << tmpval2);	//HighPin uber R_L auf Masse
				GPIOC->CR1 = (1 << tmpval2);
				delay(MS(5));
				tmpAdc = ReadADC(HighPin);
				if(tmpAdc > 50) goto savenresult;	//Spannung am High-Pin (vermuteter A2) messen; falls zu hoch: Bauteil leitet jetzt => kein Triac
				GPIOC->DDR |= (1 << tmpval);	//Gate auch uber R_L auf Masse => Triac musste zunden
				GPIOC->CR1 |= (1 << tmpval);
				delay(MS(5));
				tmpAdc = ReadADC(TristatePin);
				if(tmpAdc < 200) goto savenresult; //Spannung am Tristate-Pin (vermutetes Gate) messen; Abbruch falls Spannung zu gering
				tmpAdc = ReadADC(HighPin);
				if(tmpAdc < 150) goto savenresult; //Bauteil leitet jetzt nicht => kein Triac => Abbruch
				GPIOC->DDR = (1 << tmpval2);	//TristatePin (Gate) wieder hochohmig
				GPIOC->CR1 = (1 << tmpval2);
				delay(MS(5));
				tmpAdc = ReadADC(HighPin);
				if(tmpAdc < 150) goto savenresult; //Bauteil leitet nach Abschalten des Gatestroms nicht mehr=> kein Triac => Abbruch
				GPIOC->ODR = (1 << tmpval2);	//HighPin uber R_L auf Plus => Haltestrom aus
				delay(MS(5));
				GPIOC->ODR = 0;				//HighPin R_L over again on earth; Triac now had to block
				delay(MS(5));
				tmpAdc = ReadADC(HighPin);
				if(tmpAdc > 50) goto savenresult;	//Spannung am High-Pin (vermuteter A2) messen; falls zu hoch: Bauteil leitet jetzt => kein Triac
				PartFound = PART_TRIAC;
				PartReady = 1;
				goto savenresult;
			}
			//Test auf Transistor oder MOSFET
			tmpval++;
			GPIOC->DDR |= (1 << tmpval);		//Tristate-Pin (Basis) auf Ausgang
			GPIOC->CR1 |= (1 << tmpval);
			GPIOC->ODR |= (1 << tmpval);		//Tristate-Pin (Basis) uber R_H auf Plus
			delay(MS(50));
			adcv[1] = ReadADC(HighPin);		//Spannung am High-Pin (vermuteter Kollektor) messen
			adcv[2] = ReadADC(TristatePin);	//Basisspannung messen

			if((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) PartReady = 1;	//prufen, ob Test schon mal gelaufen
			hfe[PartReady] = 1023 - adcv[1];
			uBE[PartReady] = 1023 - adcv[2];
			if(adcv[2] < 500) {
				PartFound = PART_TRANSISTOR;	//NPN-Transistor gefunden (Basis wird "nach unten" gezogen)
				PartMode = PART_MODE_NPN;
			} else {
				if(adcv[0] < 20) {	//Durchlassspannung im gesperrten Zustand gering genug? (sonst werden D-Mode-FETs falschlicherweise als E-Mode erkannt)
					PartFound = PART_FET;			//N-Kanal-MOSFET gefunden (Basis/Gate wird NICHT "nach unten" gezogen)
					PartMode = PART_MODE_N_E_MOS;
					//Gate-Schwellspannung messen
					gthvoltage = 0;
/* TODO		tmpval2 = GPIOC->DDR;
					tmpval=(1<<HighPin);
					ADMUX = TristatePin | (1<<REFS0);
					for(b=0;b<13;b++) {
						wdt_reset();
						DischargePin(TristatePin,0);
						while ((ADC_PIN&tmpval));  // Warten, bis der MOSFET schaltet und Drain auf low geht
						R_DDR = 0;
						R_PORT = 0;
						ADCSRA |= (1<<ADSC);
						while (ADCSRA&(1<<ADSC));
						gthvoltage += ADCW;
						R_DDR = tmpval2;
						R_PORT = tmpval2;
					}
					gthvoltage *= 3;	//Umrechnung in mV, zusammen mit der Division durch 8 (bei der LCD-Anzeige)
					*/
				}
			}
			savenresult:
			b = TristatePin;
			c = HighPin;
			e = LowPin;
		}
		GPIOB->DDR = 0;
		GPIOB->CR1 = 0;
		GPIOB->ODR = 0;
		//Fertig
	} else {	//Durchgang
		//Test auf Diode
		tmpval2 = (2<<(2*HighPin + 1));	//R_H
		tmpval = (1<<(2*HighPin + 1));	//R_L
		GPIOB->ODR = 0;
		GPIOB->CR1 = (1 << LowPin);
		GPIOB->DDR = (1 << LowPin);	//Low-Pin fest auf Masse, High-Pin ist noch uber R_L auf Vcc
		DischargePin(TristatePin,1);	//Entladen fur P-Kanal-MOSFET
		delay(MS(5));
		adcv[0] = ReadADC(HighPin);// - ReadADC(LowPin);
		GPIOC->DDR = tmpval2;	//High-Pin uber R_H auf Plus
		GPIOC->CR1 = tmpval2;
		GPIOC->ODR = tmpval2;
		delay(MS(5));
		adcv[2] = ReadADC(HighPin);// - ReadADC(LowPin);
		GPIOC->DDR = tmpval;	//High-Pin uber R_L auf Plus
		GPIOC->CR1 = tmpval;
		GPIOC->ODR = tmpval;
		DischargePin(TristatePin,0);	//Entladen fur N-Kanal-MOSFET
		delay(MS(5));
		adcv[1] = ReadADC(HighPin);// - ReadADC(LowPin);
		GPIOC->DDR = tmpval2;	//High-Pin uber R_H  auf Plus
		GPIOC->CR1 = tmpval2;
		GPIOC->ODR = tmpval2;
		delay(MS(5));
		adcv[3] = ReadADC(HighPin);// - ReadADC(LowPin);
		/*Without unloading can cause false detections, because the gate of a MOSFET can still be charged.
The additional measurement with the "big" resistance R_H is carried out to anti-parallel diode of
Resistors to be able to distinguish.
A diode has a forward current of relatively independent Durchlassspg.
If the resistance is the voltage drop changes significantly (linear) with the flow.
		*/
		if(adcv[0] > adcv[1]) {
			adcv[1] = adcv[0];	//the higher value wins
			adcv[3] = adcv[2];
		}

		if((adcv[1] > 30) && (adcv[1] < 950)) { //Spannung liegt uber 0,15V und unter 4,64V => Ok
			uint8_t i,j;
			if((PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) PartFound = PART_DIODE;	//Diode nur angeben, wenn noch kein anderes Bauteil gefunden wurde. Sonst gabe es Probleme bei Transistoren mit Schutzdiode
			diodes[NumOfDiodes].Anode = HighPin;
			diodes[NumOfDiodes].Cathode = LowPin;
			diodes[NumOfDiodes].Voltage = (adcv[1]*54/11);	// ca. mit 4,9 multiplizieren, um aus dem ADC-Wert die Spannung in Millivolt zu erhalten
			NumOfDiodes++;
			for(i=0;i<NumOfDiodes;i++) {
				if((diodes[i].Anode == LowPin) && (diodes[i].Cathode == HighPin)) {	//zwei antiparallele Dioden: Defekt oder Duo-LED
					if((adcv[3]*64) < (adcv[1] / 5)) {	//Durchlassspannung fallt bei geringerem Teststrom stark ab => Defekt
						if(i<NumOfDiodes) {
							for(j=i;j<(NumOfDiodes-1);j++) {
								diodes[j].Anode = diodes[j+1].Anode;
								diodes[j].Cathode = diodes[j+1].Cathode;
								diodes[j].Voltage = diodes[j+1].Voltage;
							}
						}
						NumOfDiodes -= 2;
					}
				}
			}
		}
	}

	#if 0
	//def UseM8	//Widerstandsmessung nur auf dem Mega8 verfugbar
		//Test auf Widerstand
		tmpval2 = (2<<(2*HighPin));	//R_H
		tmpval = (1<<(2*HighPin));	//R_L
		ADC_PORT = TXD_VAL;
		ADC_DDR = (1<<LowPin) | (1<<TxD);	//Low-Pin fest auf Masse
		R_DDR = tmpval;	//High-Pin uber R_L auf Plus
		R_PORT = tmpval;
		adcv[2] = ReadADC(LowPin);
		adcv[0] = ReadADC(HighPin) - adcv[2];
		R_DDR = tmpval2;	//High-Pin uber R_H auf Plus
		R_PORT = tmpval2;
		adcv[3] = ReadADC(LowPin);
		adcv[1] = ReadADC(HighPin) - adcv[3];

		//Messung der Spannungsdifferenz zwischen dem Pluspol von R_L und R_H und Vcc
		tmpval2 = (2<<(2*LowPin));	//R_H
		tmpval = (1<<(2*LowPin));	//R_L
		ADC_DDR = (1<<HighPin) | (1<<TxD);		//High-Pin auf Ausgang
		ADC_PORT = (1<<HighPin) | TXD_VAL;	//High-Pin fest auf Plus
		R_PORT = 0;
		R_DDR = tmpval;				//Low-Pin uber R_L auf Masse
		adcv[2] += (1023 - ReadADC(HighPin));
		R_DDR = tmpval2;				//Low-Pin uber R_H auf Masse
		adcv[3] += (1023 - ReadADC(HighPin));
		
		if(((adcv[0] - adcv[2]) < 900) && ((adcv[1] - adcv[3]) > 20)) goto testend; 	//Spannung fallt bei geringem Teststrom nicht weit genug ab
		if(((adcv[1] * 32) / 31) < adcv[0]) {	//Abfallende Spannung fallt bei geringerem Teststrom stark ab und es besteht kein "Beinahe-Kurzschluss" => Widerstand
			if((PartFound == PART_DIODE) || (PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) {
				if((tmpPartFound == PART_RESISTOR) && (ra == LowPin) && (rb == HighPin)) {*/
					/* Das Bauteil wurde schon einmal mit umgekehrter Polaritat getestet.
					Jetzt beide Ergebnisse miteinander vergleichen. Wenn sie recht ahnlich sind,
					handelt es sich (hochstwahrscheinlich) um einen Widerstand. */
					if(!((((adcv[0] + 100) * 6) >= ((rv[0] + 100) * 5)) && (((rv[0] + 100) * 6) >= ((adcv[0] + 100) * 5)) && (((adcv[1] + 100) * 6) >= ((rv[1] + 100) * 5)) && (((rv[1] + 100) * 6) >= ((adcv[1] + 100) * 5)))) {
						//min. 20% Abweichung => kein Widerstand
						tmpPartFound = PART_NONE;
						goto testend;
					}
					PartFound = PART_RESISTOR;
				}
				rv[0] = adcv[0];
				rv[1] = adcv[1];

				radcmax[0] = 1023 - adcv[2];	//Spannung am Low-Pin ist nicht ganz Null, sondern rund 0,1V (wird aber gemessen). Der dadurch entstehende Fehler wird hier kompenisert
				radcmax[1] = 1023 - adcv[3];
				ra = HighPin;
				rb = LowPin;
				tmpPartFound = PART_RESISTOR;
			}
		}
	#endif
	testend:
	GPIOB->DDR = 0;
	GPIOB->CR1 = 0;
	GPIOB->ODR = 0;
	GPIOC->DDR = 0;
	GPIOC->CR1 = 0;
	GPIOC->ODR = 0;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
  }
}
#endif
