#include <Arduino.h>
#include <GNSS.h>
#include <LowPower.h>
#include <RTC.h>
#include <Flash.h> 
#include <arch/board/board.h>
#include <arch/chip/pin.h>
SpGnss Gnss;

#define SerialMon Serial
#define SerialLoRa Serial2

// E220-900T22S(JP)へのピンアサイン
#define LoRa_ModeSettingPin_M0 PIN_D20
#define LoRa_ModeSettingPin_M1 PIN_D21
#define LoRa_AUXPin PIN_I2S0_DATA_IN

// E220-900T22S(JP)のbaud rate
#define LoRa_BaudRate 9600

#define BASE_LAT 3412345 //緯度小数点以下5桁(約1m）*100000
#define BASE_LON 13612345 //経度小数点以下5桁（約1m）*100000

const int16_t sensorID =  900;
int  sendInterval = 60;//sec

uint8_t conf[] ={0xc0, 0x00, 0x08, 
                sensorID >> 8, //ADDH
                sensorID & 0xff, //ADDL
                0b01110000, // baud_rate 9600 bps  SF:9 BW:125
                0b11100000, //subpacket_size 32, rssi_ambient_noise_flag on, transmitting_power 13 dBm
                0x00, //own_channel
                0b10000011, //RSSI on ,fix mode,wor_cycle 2000 ms
                0x00, //CRYPT
                0x00};
struct  msgStruct{ 
  uint8_t conf_0 = 0xFF;
  uint8_t conf_1 = 0xFF;
  uint8_t channel = 0x00;
  uint16_t myadress  = sensorID;
  uint16_t relative_lat;
  uint16_t relative_lon;
} msg;


  /**
   * @brief ノーマルモード(M0=0,M1=0)へ移行する
   */
void SwitchToNormalMode(void){
  digitalWrite(LoRa_ModeSettingPin_M0, 0);
  digitalWrite(LoRa_ModeSettingPin_M1, 0);
  delay(1);
  while (!board_gpio_read(LoRa_AUXPin)) {}
  delay(2);
}


  /**
   * @brief コンフィグ/sleepモード(M0=1,M1=1)へ移行する
   */
void SwitchToConfigurationMode(void){
  digitalWrite(LoRa_ModeSettingPin_M0, 1);
  digitalWrite(LoRa_ModeSettingPin_M1, 1);
  delay(1);
  while (!board_gpio_read(LoRa_AUXPin)) {}
  delay(2);
}

bool state=false;
static int gpio_handlerB(int irq, FAR void *context, FAR void *arg)
{
  state = !state;
  digitalWrite(LED1, state);
  return 0;
}

static void SleepIn(void)
{
  
  ledOff(PIN_LED0);
  ledOff(PIN_LED1);
  SwitchToConfigurationMode();
  /* Save backup data to flash */
  Gnss.saveEphemeris(); 
  Gnss.stop();
  Gnss.end();
  Serial.printf("===Total time  %"PRIu64" msec Sleep %d sec ===\n",millis(),sendInterval);
  Serial.println();
  LowPower.deepSleep(sendInterval);
}

void setup() {
  LowPower.begin();
  LowPower.clockMode(CLOCK_MODE_32MHz); 
  bootcause_e bc = LowPower.bootCause(); 
  SerialMon.begin(115200);
  delay(500);
  SerialMon.println("start");
  pinMode( LED0 ,OUTPUT);
  pinMode( LED1 ,OUTPUT);
  pinMode(LoRa_ModeSettingPin_M0, OUTPUT);
  pinMode(LoRa_ModeSettingPin_M1, OUTPUT);
  //LoRa AUX pin 割り込み設定
  board_gpio_config(LoRa_AUXPin, 0, true, false, PIN_PULLUP); 
  board_gpio_intconfig(LoRa_AUXPin, INT_BOTH_EDGE, true, gpio_handlerB); 
  board_gpio_int(LoRa_AUXPin, true);
  while (!board_gpio_read(LoRa_AUXPin)) {}
  //LoRa UART
  SerialLoRa.begin(LoRa_BaudRate);
  while(!SerialLoRa){};
  //初回のみGNSS/LoRa初期設定
  if ((bc == POR_SUPPLY) || (bc == POR_NORMAL)) {
    /* Remove backup file if power-on-reset */
    Flash.remove("gnss_backup.bin");     
  
    SwitchToConfigurationMode();
    SerialMon.printf("I send conf\r\n");
    for (size_t i = 0; i < sizeof(conf); i++){
      SerialMon.printf(" %02x",conf[i]);
    }
    SerialMon.println();
    SerialLoRa.write((uint8_t *)conf, sizeof(conf));
    delay(100);
    while (SerialLoRa.available()) {
      SerialMon.printf(" %02x",SerialLoRa.read());
    }
    while (!board_gpio_read(LoRa_AUXPin)) {}
    delay(2);
  }

  SerialLoRa.flush();
  // ノーマルモード(M0=0,M1=0)へ移行する
  SwitchToNormalMode();

  /* Activate GNSS device */
  int result;
  result = Gnss.begin();
  assert(result == 0);

  /* Start positioning */
  result = Gnss.start();
  assert(result == 0);
  Serial.println("Gnss setup OK");
}

void loop() {
  if (Gnss.waitUpdate(-1)) {
    /* Get navData. */
    SpNavData navData;
    Gnss.getNavData(&navData);
    bool posFix = ((navData.posDataExist) && (navData.posFixMode != FixInvalid));
    if (posFix ) {
      Serial.println("Position is fixed.");
      digitalWrite(LED0, HIGH);
      /*BASEからの緯度経度の差分を２Byteで送信
      およそ南北72km東西60km程度の位置を1m単位で特定可能
      */
     msg.relative_lat = int16_t(navData.latitude * 100000 -BASE_LAT);
      msg.relative_lon = int16_t(navData.longitude * 100000 - BASE_LON);

      uint8_t payload[]={msg.conf_0, msg.conf_1, msg.channel ,
                        msg.myadress & 0xff ,msg.myadress >> 8 ,
                        msg.relative_lat & 0xff, msg.relative_lat >> 8,
                        msg.relative_lon & 0xff, msg.relative_lon >> 8};
      
      for (size_t i = 0; i < sizeof(payload); i++){
        SerialMon.printf(" %02x",payload[i]);
      }
      SerialMon.println();
      SerialLoRa.flush();
      SerialLoRa.write((uint8_t *)payload, sizeof(payload));
      delay(2);//これが無いとなぜか動かない
      while (!board_gpio_read(LoRa_AUXPin)) {}
      SerialMon.printf("I send %d Byte",sizeof(payload));
      SleepIn();
    }
    delay(1000);
  }
}
