//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2020-07-01 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <Register.h>
//#include <sensors/Si7021.h>

#include <Dimmer.h>
//#include <Switch.h>

#define LED_PIN           4
#define CONFIG_BUTTON_PIN 8
#define FAN_PIN           3

#define PEERS_PER_CHANNEL    4
#define WEATHER_MSG_INTERVAL 180

#define NUM_CHANNELS         1

using namespace as;

const struct DeviceInfo PROGMEM devinfo = {
    {0xf3,0x49,0x01},       // Device ID
    "JPFAN00001",           // Device Serial
    {0xf3,0x49},            // Device Model
    0x01,                   // Firmware Version
    as::DeviceType::Dimmer, // Device Type
    {0x01,0x00}             // Info Bytes
};

typedef AvrSPI<10,11,12,13> SPIType;
typedef Radio<SPIType,2> RadioType;
typedef StatusLed<LED_PIN> LedType;
typedef AskSin<LedType,NoBattery,RadioType> HalType;

/*DEFREGISTER(Reg0,DREG_INTKEY,MASTERID_REGS,DREG_TRANSMITTRYMAX)
class MixDevList0 : public RegList0<Reg0> {
public:
  MixDevList0(uint16_t addr) : RegList0<Reg0>(addr) {}
  void defaults () {
    clear();
    transmitDevTryMax(6);
  }
};*/

typedef DimmerChannel<HalType,PEERS_PER_CHANNEL, List0> DimmerChannelType;

/*class WeatherEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, int16_t temp, uint8_t humidity, bool batlow) {
      uint8_t t1 = (temp >> 8) & 0x7f;
      uint8_t t2 = temp & 0xff;
      if ( batlow == true ) {
        t1 |= 0x80; // set bat low bit
      }
      Message::init(0xc, msgcnt, 0x70, BIDI | WKMEUP, t1, t2);
      pload[0] = humidity;
    }
};

class WeatherChannel : public Channel<HalType, List1, EmptyList, List4, PEERS_PER_CHANNEL, MixDevList0>, public Alarm {
    WeatherEventMsg msg;
    int16_t         temp;
    uint8_t         humidity;
    Si7021  sensor;
  public:
    WeatherChannel () : Channel(), Alarm(5), temp(0), humidity(0) {}
    virtual ~WeatherChannel () {}

    void measure () {
      DPRINT("Measure...\n");
      sensor.measure();
      temp = sensor.temperature();
      humidity = sensor.humidity();
      DPRINT(F("T/H = "));DDEC(temp);DPRINT(F("/"));DDECLN(humidity);
    }

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      tick = seconds2ticks(WEATHER_MSG_INTERVAL);
      measure();
      msg.init(device().nextcount(), temp, humidity, false);
      device().broadcastEvent(msg);
      clock.add(*this);
    }

    void setup(Device<HalType, MixDevList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      sensor.init();
      sysclock.add(*this);
    }

    uint8_t status () const { return 0; }
    uint8_t flags  () const { return 0; }
};*/

/*class SwChannel : public SwitchChannel<HalType,PEERS_PER_CHANNEL,MixDevList0> {
public:
  SwChannel () {};
  virtual ~SwChannel () {};

  void init () {
    BaseChannel::changed(true);
  }
  virtual void switchState(__attribute__((unused)) uint8_t oldstate,uint8_t newstate,__attribute__((unused)) uint32_t delay) {
    if ( newstate == AS_CM_JT_ON ) {
      DPRINTLN("SWITCH TURN ON");
    }
    else if ( newstate == AS_CM_JT_OFF ) {
      DPRINTLN("SWITCH TURN OFF");
    }
    BaseChannel::changed(true);
  }
};*/

class MixDevType : public ChannelDevice<HalType, VirtBaseChannel<HalType, List0>, NUM_CHANNELS, List0> {
  public:
  VirtChannel<HalType, DimmerChannelType , List0>  ch1;
  //VirtChannel<HalType, SwChannel ,         MixDevList0>  ch2;
  //VirtChannel<HalType, WeatherChannel,     MixDevList0>  ch3;
  public:
    typedef ChannelDevice<HalType, VirtBaseChannel<HalType, List0>, NUM_CHANNELS, List0> DeviceType;

    MixDevType (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr) {
      DeviceType::registerChannel(ch1, 1);
      //DeviceType::registerChannel(ch2, 2);
      //DeviceType::registerChannel(ch3, 3);
    }
    virtual ~MixDevType () {}

    DimmerChannelType&  dimmerChannel  (__attribute__((unused)) uint8_t ch) { return ch1; }
    //SwChannel&          switchChannel  (                                  ) { return ch2; }
    //WeatherChannel&     weatherChannel (                                  ) { return ch3; }

    static int const channelCount = 1;
    static int const virtualCount = 1;

    virtual void configChanged () {
      DeviceType::configChanged();
    }
};

class FAN {
  uint8_t  pin;
public:
  FAN () : pin(0) {}
  ~FAN () {}

  void init(uint8_t p) {
    pin = p;
    pinMode(pin,OUTPUT);
  }
  void set(uint8_t value) {
    uint8_t pwm = 0;
    pwm = value > 0 ? map(value, 1, 200, 32, 255) : 0;
    analogWrite(pin,pwm);
  }
};

HalType hal;
MixDevType sdev(devinfo,0x20);
DimmerControl<HalType,MixDevType,FAN > control(sdev);
ConfigToggleButton<MixDevType> cfgBtn(sdev);

void setup () {
  DINIT(57600,ASKSIN_PLUS_PLUS_IDENTIFIER);
  if( control.init(hal,FAN_PIN) ) {
    sdev.channel(1).peer(cfgBtn.peer());
  }
  buttonISR(cfgBtn,CONFIG_BUTTON_PIN);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if( worked == false && poll == false ) {
    hal.activity.savePower<Idle<true> >(hal);
  }
}
