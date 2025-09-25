/*
   APRS transmitter using STM32F103C8T6 microcontroller
   Tranceiver DRA818U : 400~470MHz
   APRS packet format:
   Flag (1 byte 0x7E)
   Destination callsign(6 bytes), Desitnation SSID (1 byte) , SSID : Secondary Station Identifier
   Source Callsign (6 bytes),Src SSID (1 byte)
   Digipeater Callsign (6 bytes), Digipeater SSID (1 byte) .... upto 8 digipeaters
   Control Field (1 byte 0x03)
   Protocol ID (1 byte 0xF0) 
   Message (upto 256 characters)
   CRC (2 bytes)
   Flag (1 byte 0x7E)
*/

#include "STM32TimerInterrupt.h"  // Library for Timer interrrupt in STM32

#define PTT PA5  // PTT pin in DRA818U transcievr
#define PD PA6  // PTT pin in DRA818U transcievr


HardwareSerial Serial2(USART2);  // Second UART enabling in STM32

STM32Timer ITimer0(TIM1); // Init STM32 timer TIM1
/*
   function prototypes

*/

void APRS_TX();
void AFSK(unsigned char Byte_in, bool flag);
void playtone(uint16_t tonefreq , double tonedur );
void TimerHandler0();


//variables
unsigned short int dacBuffer[64] = {0};
volatile unsigned char head = 0;  // head and tail for circular buffer implementation
volatile unsigned char tail = 0;
volatile byte startTX = 0;
unsigned short crc = 0xffff;   // CRC(16bit) initail value
char src[6] = {0};  // SRC callsign
unsigned char srcSSID = 0;
char dest[6] = {0}; // desitination callsign
unsigned char destSSID = 0;
char digi[8][6] = {0}; // digipeater callsign
char digiSSID[8] = {0};
char msg[256] = {0};  //msg to be transmitted
unsigned char msgLen = 0;
unsigned char digiCnt = 0;
unsigned char CF = 0;
unsigned char PID = 0;
unsigned char Ndigi = 0 ;

/*
   char is same as signed char (signed 8-bit integer )
   unsigned char is unsigned 8-bit value
*/

int toneid = 0;
unsigned char countOnes = 0;
/*
   Look-up table for one cycle of sinewave values
*/
unsigned short int sineLUT[] = {483, 512, 513, 514, 515, 545, 546, 547, 576, 577, 578, 579, 608, 609, 610, 611, 641, 642, 643, 672, 673, 674, 675, 704, 705, 706, 707, 736, 737, 738, 739, 768, 769, 770, 771, 800, 800, 801, 802, 803, 832, 833, 834, 834, 835, 864, 865, 866, 866, 867, 896, 896, 897, 898, 898, 899, 928, 928, 929, 930, 930, 931, 931, 960, 960, 961, 961, 962, 962, 962, 963, 963, 963, 992, 992, 992, 993, 993, 993, 993, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 994, 993, 993, 993, 993, 993, 992, 992, 992, 963, 963, 963, 962, 962, 961, 961, 960, 960, 931, 931, 930, 930, 929, 929, 928, 899, 899, 898, 897, 897, 896, 867, 867, 866, 865, 864, 864, 835, 834, 833, 832, 803, 803, 802, 801, 800, 771, 770, 769, 768, 739, 738, 737, 736, 707, 706, 705, 704, 675, 674, 673, 672, 643, 642, 641, 640, 611, 610, 609, 608, 579, 578, 576, 547, 546, 545, 544, 515, 514, 513, 512, 482, 481, 480, 451, 450, 449, 448, 419, 418, 416, 387, 386, 385, 384, 355, 354, 353, 352, 323, 322, 321, 320, 291, 290, 289, 288, 259, 258, 257, 256, 227, 226, 225, 224, 195, 194, 193, 192, 163, 163, 162, 161, 160, 131, 130, 130, 129, 128, 99, 99, 98, 97, 97, 96, 67, 67, 66, 65, 65, 64, 64, 35, 35, 34, 34, 33, 33, 32, 32, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 32, 32, 32, 33, 33, 34, 34, 35, 35, 64, 64, 65, 66, 66, 67, 96, 96, 97, 98, 98, 99, 128, 128, 129, 130, 131, 160, 160, 161, 162, 163, 192, 193, 194, 194, 195, 224, 225, 226, 227, 256, 257, 258, 259, 288, 289, 290, 291, 320, 321, 322, 323, 352, 353, 355, 384, 385, 386, 387, 416, 417, 418, 419, 448, 449, 451, 480, 481, 482, 483};
double gtheta = 0;
double deltaTheta;
unsigned short deg;
double g_fudge;
/*
   Sample rate to accomodate tw ofrequenciues 1200 HZ and 2200 HZ
*/
double sampRate = 55000;//5263 * 10; // 9600;//38400.0;
double twoPibysampRate  = 2 * PI / sampRate;
double SampleTimeInuS = 1000000.0 / (sampRate);

/*
   Playtone for the generating the signal for given frequencu and given duration
*/
void playtone(uint16_t tonefreq , double tonedur )
{
  uint16_t tonesamples, i ;
  double   deltatheta ;
  tonedur += g_fudge ;
  tonesamples = tonedur / (SampleTimeInuS) + 0.5;

  deltatheta = twoPibysampRate * tonefreq ;

  for (i = 1; i <= tonesamples; i++)
  {
    gtheta += deltatheta;
    if (gtheta > 2 * PI)
      gtheta -= 2 * PI;
    double gthetatmp = gtheta * 57.29578;  // Converting radians to degrees
    int tmpdeg = (int) gthetatmp % 360;
    deg = (unsigned short) tmpdeg;
    //  Serial.println(sineLUT[deg]>>1);
    while ((head == 0 && tail == 63) || (tail == head - 1));
    dacBuffer[tail] = sineLUT[deg];
    if (tail == 63)  // Tails will be incremented for loading data into buffer
      tail = 0;
    else
      tail++;
  }
  g_fudge = tonedur - ( tonesamples * SampleTimeInuS ) ;
}

//--------------------------------------------------------------

/*
   Timer Iterrupt Sub routine
   sends bits to DAC at specified time interval (1/sample rate)

*/
void TimerHandler0()
{
  if (head != tail)
  {
    GPIOB->ODR = dacBuffer[head];
    head++;
    if (head > 63)
      head = 0;
  }
  else
    GPIOB->ODR = 0;  // PORTB output register
}

/*
   AFSK trasnmitter : Audio Frequency shift keying
   tone duation 833.3 microseconds is 1/1200 (1200 bauds of AFSK transmission)
   bit 1 : no change in tone frequecy
   nit 0 : change in tone frequency
   NRZL-I : after every 5 consecutive ones one zero bit is added, not to confuse with Flag byte (0x7E: 0111 1110), it has 6 consecutive 1's
   AFSK receives 1 byte (8 bits), and processes bit by bit
   CRC is also calculated parallely
*/
void AFSK(unsigned char Byte_in, bool flag)
{
  unsigned short xor_in;
  unsigned char test = Byte_in;
  for (int i = 0; i < 8; i++)
  {
    /*
       CRC-16 computation : bit by bit
       // x16 +x12 +x5 +1 // 0b1 0001 0000 0010 0001 //0x11021 or 0x1021
       reserve of polynomial 0x8408
       disvision is done by shift and XOR operation
       i-bit Shift every time, but if change is there in last incoming bit, then do dicision by XOR with polynomial
    */
    xor_in = crc ^ (Byte_in & 1);
    crc  >>= 1;
    if (xor_in & 0x01)
      crc ^= 0x8408;

    if (Byte_in & 1)
    {
      if (toneid)
        playtone(1200, 833.3);//416.66);//833.3);
      else
        playtone(2200, 833.3);//416.66);//833.3);
      countOnes++;
      /*
         NRZL-I : after every 5 consecutive ones one zero bit is added
      */
      if (flag && (countOnes == 5))
      {
        toneid = !toneid;
        if (toneid)
          playtone(1200, 833.3);//416.66);//833.3);
        else
          playtone(2200, 833.3);//416.66);//833.3);
        countOnes = 0;
      }
    }
    else
    {
      toneid = !toneid;
      if (toneid)
        playtone(1200, 833.3);//416.66);//833.3);
      else
        playtone(2200, 833.3);//416.66);//833.3);
      countOnes = 0;
    }
    Byte_in >>= 1;
  }
}

/*
   APRS Function
   Flah bit is used to indicate whether it is flag byte or not 
*/
void APRS_TX() {
  digitalWrite(PTT, 0);

  for (int i = 0; i < 50; i++) // zeros
    AFSK(0x00, LOW);


  for (int i = 0; i < 5; i++) // Initial Flags
    AFSK(0x7E, LOW);

  crc = 0xffff; // Seed for CRC

  for (int i = 0; i < 6; i++) //Dest
    AFSK(dest[i] << 1, HIGH);
  AFSK(destSSID, 1);


  for (int i = 0; i < 6; i++) // SRC
    AFSK(src[i] << 1, HIGH);
  AFSK(srcSSID, HIGH);


  for (int j = 0; j < 8; j++) // DIGI
  {
    if (j > (Ndigi - 3))
      break;
    for (int i = 0; i < 6; i++)
      AFSK(digi[j][i] << 1, HIGH);
    AFSK((digiSSID[j]), HIGH);
  }


  AFSK(CF, HIGH);  // Ctrl Fld
  AFSK(PID, 1); // PID

  for (int i = 0; i < msgLen; i++) //// payload
    AFSK(msg[i], HIGH);

  // crc
  unsigned char crc_lo = crc ^ 0xff;
  unsigned char crc_hi = (crc >> 8) ^ 0xff;
  AFSK(crc_lo, HIGH);
  AFSK(crc_hi, HIGH);
  for (int i = 0; i < 2; i++) // Final flags
    AFSK(0x7E, 0);

  //delay(1000);
  digitalWrite(PTT, 1);
}
//--------------------------------------------------------------

//----------------------------SETUP------------------------------
void setup()
{

  Serial1.begin(115200); // for debugging the code (PA9 -RX , PA10-TX)
  Serial2.begin(9600);  // for programming Transceiver through UART (PA2-TX, PA3-RX)
  /*
     Setting pins for tranceiver
     PTT should made low for tranmission
     transceiver by default will be in receive mode
  */
  pinMode(PTT, OUTPUT);
  pinMode(PD, OUTPUT);

  /*
    Setting pins for DAC impelmented using R-2R ladder network (7-bit DAC)
  */
  pinMode(PB0, OUTPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);


  delay(100);
  digitalWrite(PTT, 0);
  delay(10);
  digitalWrite(PTT, 1);
  digitalWrite(PD, 1);

  // digitalWrite(HbyL,0);
  delay(100);
  /*
     programming the tranceiver for TX and RX frequecies
     command format : AT+DMOSETGROUP=GBW,TFV, RFV,Tx_CTCSS,SQ,Rx_CTCSS<CR><LF>
     GBW: Channels space selection. 1  25k
      TFV: Transmit frequency. Range: 400.0000~470.0000MHz
      RFV: Receive frequency. Range: 400.0000~470.0000MHz.
      Tx_CTCSS: CTCSS value in transmit
      SQ: Squelch level (0~8). 0 monitor mode which can’t be used in scanning mode.
      Rx_CTCSS: CTCSS value in receive
     AT+DMOSETVOLUME=x <CR><LF>
     Volume range (1~8).
  */
  Serial2.println("AT+DMOSETGROUP=1,435.5500,435.5500,0000,1,0000");
  //    Serial2.println("AT+DMOSETGROUP=1,420.0000,420.0000,0000,1,0000");
  Serial2.println("AT+DMOSETVOLUME=5");
  delay(100);
  double sampleTime = 1000000 / sampRate;  // multyplied by 1000000 to get the value in micro seconds

  /*
     Initalizing the Timer interrupt for data transmission to DAC
     Time duration (in micro seconds) is inverse of sample rate
  */
  ITimer0.attachInterruptInterval(int(sampleTime), TimerHandler0);
}
//--------------------------------------------------------------

//----------------------------LOOP------------------------------
void loop()
{
  while (1)
  {
    /*
       APRS message construction
    */
    delay(3000);
    strcpy(dest, "CQ    ");
    destSSID = 0x60;
    strcpy(src, "VU3BIZ");
    srcSSID = 0x60;
    Ndigi = 3;
    strcpy(digi[0], "ROISS ");
    digiSSID[0] = 0x61;
    strcpy(msg, "Hello All DE VU3BIZ, APRS Test ");
    msgLen = 30;  // 30 characters
    CF = 0x03;
    PID = 0xf0;
    /*
       APRS tarnsmission
    */
    APRS_TX();

  }
}
