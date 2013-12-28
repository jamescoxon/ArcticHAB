// Code for Atlas6 - High altitude balloon
// Based on Teensy 3.0
// RTTY using Interrupts for timing (IntervalTimer lib which is built into the Teensyduino code)
//
// jacoxon@googlemail.com

IntervalTimer timer0;

int txPin = 11, enPin = 12, ledPin = 13;
volatile uint32_t timer0count;
volatile char c;

//Variables
int32_t lat = 514981000, lon = -530000, alt = 0;
uint8_t gps_hour = 0, gps_minute = 0, gps_second = 0, gps_month = 0, gps_day = 0, lock = 0, sats = 0, navmode = 0, n = 1, GPSerror = 0, voltage, int_temp;
char superbuffer [80]; //Telem string buffer
uint8_t buf[60]; //GPS receive buffer
int count = 0;

struct t_htab { char c; int hellpat[5]; } ;

struct t_htab helltab[] = {

  {'1', { B00000100, B00000100, B01111100, B00000000, B00000000 } },
  {'2', { B01001000, B01100100, B01010100, B01001100, B01000000 } },
  {'3', { B01000100, B01000100, B01010100, B01010100, B00111100 } },
  {'4', { B00011100, B00010000, B00010000, B01111100, B00010000 } },
  {'5', { B01000000, B01011100, B01010100, B01010100, B00110100 } },
  {'6', { B00111100, B01010010, B01001010, B01001000, B00110000 } },
  {'7', { B01000100, B00100100, B00010100, B00001100, B00000100 } },
  {'8', { B00111100, B01001010, B01001010, B01001010, B00111100 } },
  {'9', { B00001100, B01001010, B01001010, B00101010, B00111100 } },
  {'0', { B00111000, B01100100, B01010100, B01001100, B00111000 } },
  {'A', { B01111000, B00101100, B00100100, B00101100, B01111000 } },
  {'B', { B01000100, B01111100, B01010100, B01010100, B00101000 } },
  {'C', { B00111000, B01101100, B01000100, B01000100, B00101000 } },
  {'D', { B01000100, B01111100, B01000100, B01000100, B00111000 } },
  {'E', { B01111100, B01010100, B01010100, B01000100, B01000100 } },
  {'F', { B01111100, B00010100, B00010100, B00000100, B00000100 } },
  {'G', { B00111000, B01101100, B01000100, B01010100, B00110100 } },
  {'H', { B01111100, B00010000, B00010000, B00010000, B01111100 } },
  {'I', { B00000000, B01000100, B01111100, B01000100, B00000000 } },
  {'J', { B01100000, B01000000, B01000000, B01000000, B01111100 } },
  {'K', { B01111100, B00010000, B00111000, B00101000, B01000100 } },
  {'L', { B01111100, B01000000, B01000000, B01000000, B01000000 } },
  {'M', { B01111100, B00001000, B00010000, B00001000, B01111100 } },
  {'N', { B01111100, B00000100, B00001000, B00010000, B01111100 } },
  {'O', { B00111000, B01000100, B01000100, B01000100, B00111000 } },
  {'P', { B01000100, B01111100, B01010100, B00010100, B00011000 } },
  {'Q', { B00111000, B01000100, B01100100, B11000100, B10111000 } },
  {'R', { B01111100, B00010100, B00010100, B00110100, B01011000 } },
  {'S', { B01011000, B01010100, B01010100, B01010100, B00100100 } },
  {'T', { B00000100, B00000100, B01111100, B00000100, B00000100 } },
  {'U', { B01111100, B01000000, B01000000, B01000000, B01111100 } },
  {'V', { B01111100, B00100000, B00010000, B00001000, B00000100 } },
  {'W', { B01111100, B01100000, B01111100, B01000000, B01111100 } },
  {'X', { B01000100, B00101000, B00010000, B00101000, B01000100 } },
  {'Y', { B00000100, B00001000, B01110000, B00001000, B00000100 } },
  {'Z', { B01000100, B01100100, B01010100, B01001100, B01100100 } },
  {'.', { B01000000, B01000000, B00000000, B00000000, B00000000 } },
  {'-', { B00010000, B00010000, B00010000, B00010000, B00010000 } },
  {',', { B10000000, B10100000, B01100000, B00000000, B00000000 } },
  {'/', { B01000000, B00100000, B00010000, B00001000, B00000100 } },
  {'@', { B00111000, B01000100, B01010100, B01000100, B00111000 } },
  {'$', { B01011000, B01010100, B11010110, B01010100, B00100100 } },
  {'*', { B00000000, B00000000, B00000100, B00001110, B00000100 } }

};

#define N_HELL  (sizeof(helltab)/sizeof(helltab[0]))

void helldelay()
{
  //Slow
  delay(64);
  delayMicroseconds(900);
  
  //Feld-Hell
  //delay(8);
  //delayMicroseconds(160);

}

void on()
{
  digitalWrite(txPin, HIGH);
  helldelay();
  digitalWrite(txPin, LOW);
}

void hellsend(char c)
{
  int i,j,q ;
  if (c == ' ') {
      for (int d=0; d<14; d++){
        helldelay();  
      }
    return ;
  }
  for (i=0; i<N_HELL; i++) {
    if (helltab[i].c == c) {
      
      for (j=0; j<=4; j++) 
      {
        byte mask = B10000000;
        for (q=0; q<=6; q++)
        {      
          if(helltab[i].hellpat[j] & mask) {
            on();
          } else {
            helldelay();
          }
          mask >>= 1;
        }
      }
      for (int d=0; d<14; d++){
        helldelay();  
      }
      return ;
    }
  }
  /* if we drop off the end, then we send a space */
}

void hellsendmsg(char *str)
{
  while (*str)
    hellsend(*str++) ;
}

// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{
	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/
	c = *string++;
	while ( c != '\0')
	{
          timer0count = 0;
          timer0.begin(timerCallback0, 19925); // 19925 (21000 - 18850 microseconds)
          while(timer0count < 11){}
          timer0.end();
          c = *string++;
	}
}

void rtty_txbit (int bit)
{
  		if (bit)
		{
		  // high
                  digitalWrite(txPin, HIGH);
                  digitalWrite(ledPin, HIGH);
		}
		else
		{
		  // low
                  digitalWrite(txPin, LOW);
                  digitalWrite(ledPin, LOW);
		}       
                
}

void timerCallback0() {
  
  // start, data, data, data, data, data, data, data, data, stop, stop
  if(timer0count == 0){
    rtty_txbit (0); // Start bit
  }
  else if(timer0count >= 1 && timer0count <= 8){
    if (c & 1) rtty_txbit(1); 
        else rtty_txbit(0);	
    c = c >> 1;
  }
  if(timer0count == 9 || timer0count == 10){
    rtty_txbit (1); // Stop bit
  }
  timer0count++;
}


void setup()
{
  pinMode(txPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  Serial1.begin(9600);
  
  analogReference(EXTERNAL); 
  analogReadAveraging(32);
  
  digitalWrite(enPin, HIGH);
  
  digitalWrite(ledPin, HIGH);
  delay(5000); //delay to allow GPS to fully boot
  digitalWrite(ledPin, LOW);
  
  rtty_txstring("$$$$");
  setupGPS();
  delay(2000);
}

void loop()
{
  int n;
  count++;
  
  gps_check_nav();
  
  if(navmode != 6 && count > 5) {
    setupGPS();
  }
  
  if((count % 100) == 0) {
    setupGPS();
  }
  
  delay(100);
  gps_check_lock();
  delay(100);
  gps_get_time();
  delay(100);
  gps_get_position();
  
  voltage = analogRead(8);
  int_temp =  analogRead(38);
  
  n=sprintf (superbuffer, "$$ATLAS,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d", count, gps_hour, gps_minute, gps_second, lat, lon, alt, sats, lock, navmode, GPSerror, voltage, int_temp);
  n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
  
  if((count % 20) == 0){
    hellsendmsg(superbuffer);
    delay(500);
    rtty_txstring("$$$$$$$$$$$$");
  }
  else {
    rtty_txstring("$$");
    rtty_txstring(superbuffer);
  }
  
}

//************Other Functions*****************

uint16_t gps_CRC16_checksum (char *string)
{
	size_t i;
	uint16_t crc;
	uint8_t c;
 
	crc = 0xFFFF;
 
	// Calculate checksum ignoring the first two $s
	for (i = 2; i < strlen(string); i++)
	{
		c = string[i];
		crc = crc_xmodem_update (crc, c);
	}
 
	return crc;
}

//from
// http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#gaca726c22a1900f9bad52594c8846115f
uint16_t crc_xmodem_update (uint16_t crc, uint8_t data)
 {
  int i;
  
  crc = crc ^ ((uint16_t)data << 8);
  for (i=0; i<8; i++)
  {
      if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
      else
          crc <<= 1;
  }
  
  return crc;
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial1.write(MSG[i]);
  }
}

void setupGPS() {
  
  Serial1.clear();
  // Taken from Project Swift (rather than the old way of sending ascii text)
  uint8_t setNMEAoff[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};
  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  
  delay(500);
  
  // Check and set the navigation mode (Airborne, 1G)
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  
  /*
  delay(500);
  
  //set GPS to Eco mode (reduces current by 4mA)
  uint8_t setEco[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x04, 0x1D, 0x85};
  sendUBX(setEco, sizeof(setEco)/sizeof(uint8_t));
  */
}

/**
 * Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
 */
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
        uint8_t* ckb)
{
    *cka = 0;
    *ckb = 0;
    for( uint8_t i = 0; i < len; i++ )
    {
        *cka += *data;
        *ckb += *cka;
        data++;
    }
}

/**
 * Verify the checksum for the given data and length.
 */
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
    uint8_t a, b;
    gps_ubx_checksum(data, len, &a, &b);
    if( a != *(data + len) || b != *(data + len + 1))
        return false;
    else
        return true;
}

/**
 * Get data from GPS, times out after 1.5 second.
 */
void gps_get_data()
{
    int i = 0;
    unsigned long startTime = millis();
    while (1) {
    // Make sure data is available to read
    if (Serial1.available()) {
      buf[i] = Serial1.read();
      i++;
    }
    // Timeout if no valid response in 1.5 seconds
    if (millis() - startTime > 1500) {
      break;
    }
    }
}
/**
 * Check the navigation status to determine the quality of the
 * fix currently held by the receiver with a NAV-STATUS message.
 */
void gps_check_lock()
{
    GPSerror = 0;
    Serial1.flush();
    Serial1.clear();
    // Construct the request to the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
        0x07, 0x16};
    sendUBX(request, 8);

    // Get the message back from the GPS
    gps_get_data();
    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
      GPSerror = 11;
    }
    if( buf[2] != 0x01 || buf[3] != 0x06 ) {
      GPSerror = 12;
    }

    // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
    if( !_gps_verify_checksum(&buf[2], 56) ) {
      GPSerror = 13;
    }
    
    if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
        lock = buf[16];
    else
        lock = 0;

    sats = buf[53];
    }
    else {
      lock = 0;
    }
}

/**
 * Poll the GPS for a position message then extract the useful
 * information from it - POSLLH.
 */
void gps_get_position()
{
    GPSerror = 0;
    Serial1.flush();
    Serial1.clear();
    // Request a NAV-POSLLH message from the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
        0x0A};
    sendUBX(request, 8);
    
    // Get the message back from the GPS
    gps_get_data();

    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        GPSerror = 21;
    if( buf[2] != 0x01 || buf[3] != 0x02 )
        GPSerror = 22;
        
    if( !_gps_verify_checksum(&buf[2], 32) ) {
      GPSerror = 23;
    }
    
    if(GPSerror == 0) {
      // 4 bytes of longitude (1e-7)
      lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
          (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
      //lon /= 1000;
      
      // 4 bytes of latitude (1e-7)
      lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
          (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
      //lat /= 1000;
      
      // 4 bytes of altitude above MSL (mm)
      alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
          (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
      alt /= 1000;
    }

}

/**
 * Get the gps_hour, gps_minute and gps_second from the GPS using the NAV-TIMEUTC
 * message.
 */
void gps_get_time()
{
    GPSerror = 0;
    Serial1.flush();
    Serial1.clear();
    // Send a NAV-TIMEUTC message to the receiver
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
        0x22, 0x67};
     sendUBX(request, 8);

    // Get the message back from the GPS
    gps_get_data();

    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        GPSerror = 31;
    if( buf[2] != 0x01 || buf[3] != 0x21 )
        GPSerror = 32;

    if( !_gps_verify_checksum(&buf[2], 24) ) {
      GPSerror = 33;
    }
    
    if(GPSerror == 0) {
      if(gps_hour > 23 || gps_minute > 59 || gps_second > 59)
      {
        GPSerror = 34;
      }
      else {
        gps_month = buf[20];
        gps_day = buf[21];
        gps_hour = buf[22];
        gps_minute = buf[23];
        gps_second = buf[24];
      }
    }
}

/**
 * Verify that the uBlox 6 GPS receiver is set to the <1g airborne
 * navigaion mode.
 */
uint8_t gps_check_nav(void)
{
    Serial1.flush();
    Serial1.clear();
    uint8_t request[8] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00,
        0x2A, 0x84};
    sendUBX(request, 8);

    // Get the message back from the GPS
    gps_get_data();

    // Verify sync and header bytes
    if( buf[0] != 0xB5 || buf[1] != 0x62 ){
      GPSerror = 41;
    }
    if( buf[2] != 0x06 || buf[3] != 0x24 ){
      GPSerror = 42;
    }
    // Check 40 bytes of message checksum
    if( !_gps_verify_checksum(&buf[2], 40) ) {
      GPSerror = 43;
    }

    // Return the navigation mode and let the caller analyse it
    navmode = buf[8];
}
