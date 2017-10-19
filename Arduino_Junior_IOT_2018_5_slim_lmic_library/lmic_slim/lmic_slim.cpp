// adjustments:
// 20171010  Dennis - clarifications and improvements
// 20171015  Marco, SF7 text formatting
//
#include <lmic_slim.h>
#include <SPI.h>
struct lmic_t LMIC;
byte channelpointer = 0;
extern  uint32_t  AESAUX[];
extern  uint32_t  AESKEY[];
#define AESkey ((uint8_t*)AESKEY)
#define AESaux ((uint8_t*)AESAUX)
uint32_t AESAUX[16/sizeof(uint32_t)];
uint32_t AESKEY[11*16/sizeof(uint32_t)];

void spi_start() {
    SPI.begin();
}
     
void radio_init () {	
    setopmode(0x00);                                // opmode SLEEP
    rxlora();     
    setopmode(0x00);                                // opmode SLEEP
}

static void setopmode (uint8_t mode) {
      writeReg(0x01, 0x88 | mode);                  //Register RegOpMode
}

static void writeReg (uint8_t addr, uint8_t data ) {
    hal_pin_nss(0);
    //digitalWrite(LED_BUILTIN,HIGH);
    SPI.transfer(addr | 0x80);
    SPI.transfer(data);    
    hal_pin_nss(1);
    //digitalWrite(LED_BUILTIN,LOW);
}

void hal_pin_nss (uint8_t val) {
    digitalWrite(SS_pin, val);                      // nss pin = PB5
}

static void rxlora () {                             // start LoRa receiver (time=LMIC.rxtime, timeout=LMIC.rxsyms, result=LMIC.frame[LMIC.dataLen])
    setopmode(0x01);  // enter standby mode (warm up))
    writeReg(0x23, 64);                             // Register LORARegPayloadMaxLength - set max payload size
    writeReg(0x33, 0x67);                           // Register LORARegInvertIQ - use inverted I/Q signal (prevent mote-to-mote communication)
    writeReg(0x1F, 0);                              // Register LORARegSymbTimeoutLsb - set symbol timeout (for single rx) - gedefinieerd op MINRX_SYMS
    writeReg(0x39, 0x34);                           // Register LORARegSyncWord - set sync word
    writeReg(0x40, 0x00|0x00|0xC0);                 // Register RegDioMapping1 - configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP
    configLoraModem();                              // configure LoRa modem (cfg1, cfg2)
    configChannel();  // configure frequency
    writeReg(0x0A, 0x08);                           // Register RegPaRamp - set PA ramp-up time 50 uSec, configure output power
    writeReg(0x09, 0xFF);                           // Register RegPaConfig - maximum power   
    writeReg(0x5A, 0x14);                           // Register RegPaDac - 0x04 is de default value. 0x07 = power boost. Alleen 2 bits worden gebruikt, daarom lezen van registerwaarde en gebruik bitwise-OR  
}

static void configLoraModem () {                    // configure LoRa modem (cfg1, cfg2)
    writeReg(0x1D, 0x72);                           // Register LORARegModemConfig1 - BW=125 en Coding Rate=4/5  
    
	writeReg(0x1E, 0b11000100);
       // Register LORARegModemConfig2 
       //          0b0000 0000
       //            nnnn ----   Spreading Factor (bit 7..4)
       //            0111 ----     7 = SF7    is the TTNmapper default
       //            1011 ----    12 = SF12   tested
       //            1100 ----    12 = SF12   tested & working
       //            ---- 0---   TxContinuousMode =0 normal mode (bit 3)  
       //            ---- -1--   RxPayloadCrcOn = 1 CRC ON (bit 2)  
       //            ---- --00   SymbTimeout(9:8)=00 default (bit 1..0)
    
	writeReg(0x26, 0x0C);                           // Register LORARegModemConfig3
}

static void txlora () {                             // start transmitter (buf=LMIC.frame, len=LMIC.dataLen)
    setopmode(0x01);                                // OPMODE_STANDBY enter standby mode (required for FIFO loading))
    writeReg(0x39, 0x34);                           // Register LORARegSyncWord - set sync word LORA_MAC_PREAMBLE
    writeReg(0x40, 0x40|0x30|0xC0);                 // Register RegDioMapping1 - MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP - set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(0x12, 0xFF);                           // Register LORARegIrqFlags - clear all radio IRQ flags
    writeReg(0x11, ~0x08);                          // Register LORARegIrqFlagsMask - IRQ_LORA_TXDONE_MASK mask all IRQs but TxDone
    writeReg(0x0E, 0x00);                           // Register LORARegFifoTxBaseAddr - initialize the payload size and address pointers
    writeReg(0x0D, 0x00);                           // Register LORARegFifoAddrPtr
    writeReg(0x22, LMIC.dataLen);                   // Register LORARegPayloadLength payload length
    writeBuf(0x00, LMIC.frame, LMIC.dataLen);       // Register RegFifo - download buffer to the radio FIFO
    setopmode(0x03);                                // OPMODE_TX - now we actually start the transmission
}

static void writeBuf (uint8_t addr, uint8_t* buf, uint8_t len) {
    hal_pin_nss(0);
    SPI.transfer(addr | 0x80);
    for (uint8_t i=0; i<len; i++) {
        SPI.transfer(buf[i]);
    }
    hal_pin_nss(1);
}

static void configChannel () {                      // set frequency: basisstap synthesizer is 32 Mhz / (2 ^ 19)
      channelpointer=LMIC.frame[10] & 0b00000111;   // Kies pseudorandom kanaal o.b.v. een byte uit de encrypted payload
      writeReg(0x06, channel[channelpointer][0]);   // Wijzig freq: Register RegFrfMsb
      writeReg(0x07, channel[channelpointer][1]);   // Register RegFrfMid
      writeReg(0x08, channel[channelpointer][2]);   // Register RegFrfLsb
}

void LMIC_setSession (uint32_t devaddr, uint8_t* nwkKey, uint8_t* artKey) {
    LMIC.devaddr = devaddr;
    memcpy(LMIC.nwkKey, nwkKey, 16);
    memcpy(LMIC.artKey, artKey, 16);
}

int LMIC_setTxData2 (uint8_t* data, uint8_t dlen) {
    memcpy(LMIC.pendTxData, data, dlen);            //  copy data naar LMIC.pendTxData
    LMIC.pendTxLen  = dlen;
    buildDataFrame();
    return 0;
}

static void buildDataFrame (void) {
    uint8_t dlen = LMIC.pendTxLen;
    int  end = 8;
    uint8_t flen = end + 5+dlen;
    LMIC.frame[0] = 0x40 | 0x00;
    LMIC.frame[5] = ( 0 | 0x80 | (end-8));
    os_wlsbf4(LMIC.frame+1, LMIC.devaddr);          // Device address in LMIC.frame[1]...[4]
    LMIC.seqnoUp += 1;
    os_wlsbf2(LMIC.frame+6, LMIC.seqnoUp-1);        // sequence number in LMIC.frame[6]...[7]
    LMIC.frame[end] = 1;                            // LMIC.pendTxPort=1
    memcpy(LMIC.frame+end+1, LMIC.pendTxData, dlen);
    delay(50);
    aes_cipher(LMIC.artKey,LMIC.devaddr, LMIC.seqnoUp-1,0, LMIC.frame+end+1, dlen);
    delay(50);
    aes_appendMic(LMIC.nwkKey, LMIC.devaddr, LMIC.seqnoUp-1, 0, LMIC.frame, flen-4);
    LMIC.dataLen = flen;
}

static void aes_appendMic (uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* pdu, int len) {
    micB0(devaddr, seqno, dndir, len);               // dndir is altijd 0
    memcpy(AESkey,key,16);
    os_wmsbf4(pdu+len, os_aes(0x02, pdu, len));
}

static void micB0 (uint32_t devaddr, uint32_t seqno, int dndir, int len) {
    memset(AESaux,0,16);
    AESaux[0]  = 0x49;
    AESaux[5]  = dndir?1:0;                          // dndir is altijd 0
    AESaux[15] = len;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
}

static void aes_cipher (uint8_t* key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t* payload, int len) {
    memset(AESaux,0,16);
    AESaux[0] = AESaux[15] = 1;                      // mode=cipher / dir=down / block counter=1
    AESaux[5] = dndir?1:0;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
    memcpy(AESkey,key,16);
    os_aes(0x04, payload, len);                      // AES_CTR =0x04
}

uint32_t os_aes (uint8_t mode, uint8_t* buf, uint16_t len) {
    switch (mode & ~0x08) {
        case 0x02:
            os_aes_cmac(buf, len, /* prepend_aux */ !(mode & 0x08));
            return os_rmsbf4(AESaux);
        case 0x04:
            os_aes_ctr(buf, len);
            break;
    }
    return 0;
}

static void os_aes_ctr (uint8_t* buf, uint16_t len) {
    uint8_t ctr[16];
    while (len) {
        memcpy(ctr, AESaux, sizeof(ctr));        
        lmic_aes_encrypt(ctr, AESkey);
        for (uint8_t i = 0; i < 16 && len > 0; i++, len--, buf++)
            *buf ^= ctr[i];
        AESaux[15]++;
    }
}

void lmic_aes_encrypt(unsigned char *Data, unsigned char *Key)
{
  unsigned char i;
  unsigned char Row,Collum;
  unsigned char Round = 0x00;
  unsigned char Round_Key[16];
  for(Collum = 0; Collum < 4; Collum++)
  {
    for(Row = 0; Row < 4; Row++)
    {
      State[Row][Collum] = Data[Row + (4*Collum)];
    }
  }
  for(i = 0; i < 16; i++)
  {
    Round_Key[i] = Key[i];
  }
  AES_Add_Round_Key(Round_Key);
  for(Round = 1; Round < 10; Round++)
  {
    for(Collum = 0; Collum < 4; Collum++)
    {
      for(Row = 0; Row < 4; Row++)
      {
        State[Row][Collum] = AES_Sub_Byte(State[Row][Collum]);
      }
    }
    AES_Shift_Rows();
    AES_Mix_Collums();
    AES_Calculate_Round_Key(Round,Round_Key);
    AES_Add_Round_Key(Round_Key);
  }
  for(Collum = 0; Collum < 4; Collum++)
  {
    for(Row = 0; Row < 4; Row++)
    {
      State[Row][Collum] = AES_Sub_Byte(State[Row][Collum]);
    }
  }
  AES_Shift_Rows();
  AES_Calculate_Round_Key(Round,Round_Key);
  AES_Add_Round_Key(Round_Key);
  for(Collum = 0; Collum < 4; Collum++)
  {
    for(Row = 0; Row < 4; Row++)
    {
      Data[Row + (4*Collum)] = State[Row][Collum];
    }
  }
}

static void AES_Add_Round_Key(unsigned char *Round_Key)
{
  unsigned char Row,Collum;
  for(Collum = 0; Collum < 4; Collum++)
  {
    for(Row = 0; Row < 4; Row++)
    {
      State[Row][Collum] = State[Row][Collum] ^ Round_Key[Row + (4*Collum)];
    }
  }
}

static unsigned char AES_Sub_Byte(unsigned char Byte)
{
  unsigned char S_Row,S_Collum;
  unsigned char S_Byte;
  S_Row = ((Byte >> 4) & 0x0F);
  S_Collum = (Byte & 0x0F);
  S_Byte = pgm_read_byte(&(S_Table [S_Row][S_Collum]));    // let op: speciale leesinstructie omdat de tabel in PROGMEM staat
  return S_Byte;
}

static void AES_Shift_Rows()
{
  unsigned char Buffer;
  Buffer = State[1][0];
  State[1][0] = State[1][1];
  State[1][1] = State[1][2];
  State[1][2] = State[1][3];
  State[1][3] = Buffer;
  Buffer = State[2][0];
  State[2][0] = State[2][2];
  State[2][2] = Buffer;
  Buffer = State[2][1];
  State[2][1] = State[2][3];
  State[2][3] = Buffer;
  Buffer = State[3][3];
  State[3][3] = State[3][2];
  State[3][2] = State[3][1];
  State[3][1] = State[3][0];
  State[3][0] = Buffer;
}

static void AES_Mix_Collums()
{
  unsigned char Row,Collum;
  unsigned char a[4], b[4];
  for(Collum = 0; Collum < 4; Collum++)
  {
    for(Row = 0; Row < 4; Row++)
    {
      a[Row] = State[Row][Collum];
      b[Row] = (State[Row][Collum] << 1);

      if((State[Row][Collum] & 0x80) == 0x80)
      {
        b[Row] = b[Row] ^ 0x1B;
      }
    }
    State[0][Collum] = b[0] ^ a[1] ^ b[1] ^ a[2] ^ a[3];
    State[1][Collum] = a[0] ^ b[1] ^ a[2] ^ b[2] ^ a[3];
    State[2][Collum] = a[0] ^ a[1] ^ b[2] ^ a[3] ^ b[3];
    State[3][Collum] = a[0] ^ b[0] ^ a[1] ^ a[2] ^ b[3];
  }
}

static void AES_Calculate_Round_Key(unsigned char Round, unsigned char *Round_Key)
{
  unsigned char i,j;
  unsigned char b;
  unsigned char Temp[4];
  unsigned char Buffer;
  unsigned char Rcon;
  for(i = 0; i < 4; i++)
  {
    Temp[i] = Round_Key[i+12];
  }
  Buffer = Temp[0];
  Temp[0] = Temp[1];
  Temp[1] = Temp[2];
  Temp[2] = Temp[3];
  Temp[3] = Buffer;
  for(i = 0; i < 4; i++)
  {
    Temp[i] = AES_Sub_Byte(Temp[i]);
  }
  Rcon = 0x01;
  while(Round != 1)
  {
    b = Rcon & 0x80;
    Rcon = Rcon << 1;
    if(b == 0x80)
    {
      Rcon = Rcon ^ 0x1b;
    }
    Round--;
  }
  Temp[0] = Temp[0] ^ Rcon;
  for(i = 0; i < 4; i++)
  {
    for(j = 0; j < 4; j++)
    {
      Round_Key[j + (4*i)] = Round_Key[j + (4*i)] ^ Temp[j];
      Temp[j] = Round_Key[j + (4*i)];
    }
  }
}

static void os_aes_cmac(uint8_t* buf, uint16_t len, uint8_t prepend_aux) {
    if (prepend_aux)
        lmic_aes_encrypt(AESaux, AESkey);
    else
        memset (AESaux, 0, 16);
    while (len > 0) {
        uint8_t need_padding = 0;
        for (uint8_t i = 0; i < 16; ++i, ++buf, --len) {
            if (len == 0) {
                AESaux[i] ^= 0x80;
                need_padding = 1;
                break;
            }
            AESaux[i] ^= *buf;
        }
        if (len == 0) {
            uint8_t final_key[16];
            memset(final_key, 0, sizeof(final_key));
            lmic_aes_encrypt(final_key, AESkey);
            uint8_t msb = final_key[0] & 0x80;
            shift_left(final_key, sizeof(final_key));
            if (msb)
                final_key[sizeof(final_key)-1] ^= 0x87;
            if (need_padding) {
                msb = final_key[0] & 0x80;
                shift_left(final_key, sizeof(final_key));
                if (msb)
                    final_key[sizeof(final_key)-1] ^= 0x87;
            }
            for (uint8_t i = 0; i < sizeof(final_key); ++i)
                AESaux[i] ^= final_key[i];
        }
        lmic_aes_encrypt(AESaux, AESkey);
    }
}

static void shift_left(uint8_t* buf, uint8_t len) {
    while (len--) {
        uint8_t next = len ? buf[1] : 0;

        uint8_t val = (*buf << 1);
        if (next & 0x80)
            val |= 1;
        *buf++ = val;
    }
}

void os_wlsbf4 (uint8_t* buf, uint32_t v) {      //! Write 32-bit quantity into buffer in little endian byte order.
    buf[0] = v;
    buf[1] = v>>8;
    buf[2] = v>>16;
    buf[3] = v>>24;
}

void os_wlsbf2 (uint8_t* buf, uint16_t v) {
    buf[0] = v;
    buf[1] = v>>8;
}

void os_wmsbf4 (uint8_t* buf, uint32_t v) {
    buf[3] = v;
    buf[2] = v>>8;
    buf[1] = v>>16;
    buf[0] = v>>24;
}

uint32_t os_rmsbf4 (uint8_t* buf) {
    return (uint32_t)((uint32_t)buf[3] | ((uint32_t)buf[2]<<8) | ((uint32_t)buf[1]<<16) | ((uint32_t)buf[0]<<24));
}
