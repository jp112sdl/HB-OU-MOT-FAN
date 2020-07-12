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

#include <Dimmer.h>

#define LED_PIN           4
#define CONFIG_BUTTON_PIN 8

#define USE_25KHZ
#define PWM_PIN              3 // 9 on 644PA (Bobuino Pinout) or 3 on 328P
#define TACHO_PIN            6 // input pin for tacho signal; set to 0 when using 3-pin fan or rpm monitoring is not wanted
#define PWR_PIN              9 // output pin to turn off a 4-pin fan; if unused (i.e. for 3-pin fan) set to 0
#define RPM_MEASURE_INTERVAL 5 // check rpm every 5 seconds

#define PEERS_PER_CHANNEL    4

using namespace as;

volatile uint32_t pulsecount = 0;
uint32_t currentPWMvalue     = 0;

const struct DeviceInfo PROGMEM devinfo = {
  {0xf3, 0x60, 0x01},     // Device ID
  "JPFAN00001",           // Device Serial
#if TACHO_PIN == 0
  {0xf3, 0x49},           // Device Model
#else
  {0xf3, 0x4f},           // Device Model
#endif
  0x01,                   // Firmware Version
  as::DeviceType::Dimmer, // Device Type
  {0x01, 0x00}            // Info Bytes
};

typedef AvrSPI<10, 11, 12, 13> SPIType;
typedef Radio<SPIType, 2> RadioType;
typedef StatusLed<LED_PIN> LedType;
typedef AskSin<LedType, NoBattery, RadioType> HalType;
typedef DimmerChannel<HalType, PEERS_PER_CHANNEL, List0> DimmerChannelType;

class FAN {
  uint8_t pwmpin;
  uint8_t pwrpin;
public:
  FAN () : pwmpin(0), pwrpin(0) {}
  ~FAN () {}

  void init(uint8_t p) {
    pwmpin = p;
    pinMode(pwmpin, OUTPUT);

#ifdef USE_25KHZ
    //https://forum.arduino.cc/index.php?topic=415167.msg2859274#msg2859274
    TCCR2A = 0;                               // TC2 Control Register A
    TCCR2B = 0;                               // TC2 Control Register B
    TIMSK2 = 0;                               // TC2 Interrupt Mask Register
    TIFR2 = 0;                                // TC2 Interrupt Flag Register
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // OC2B cleared/set on match when up/down counting, fast PWM
    TCCR2B |= (1 << WGM22) | (1 << CS21);     // prescaler 8
    OCR2A = 200;                               // TOP overflow value (Hz)
    OCR2B = 0;
#endif

#if PWR_PIN > 0
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
#endif
  }

  void set(uint8_t value) {
#if PWR_PIN > 0
    digitalWrite(PWR_PIN, value == 0 ? LOW : HIGH);
#endif

#ifdef USE_25KHZ
    OCR2B = value;
#else
    uint8_t pwm = 0;
    DPRINT("SET "); DDECLN(value);
    if (currentPWMvalue == 0) {
      analogWrite(pwmpin, 255); // give the motor a short full power pulse to start (for lower speeds)
      //delay(250);
    }
    pwm = value > 0 ? map(value, 1, 200, 32, 255) : 0;
    analogWrite(pwmpin, pwm);
#endif
    currentPWMvalue = value;
  }
};

class RPMMsg : public Message {
public:
  void init(uint8_t msgcnt, int16_t rpm) {
    Message::init(0xb, msgcnt, 0x53, BCAST, (rpm >> 8), rpm);
  }
};

DEFREGISTER(Reg1, 0x94, 0x95)
class RPMList1 : public RegList1<Reg1> {
public:
  RPMList1 (uint16_t addr) : RegList1<Reg1>(addr) {}

  bool rpmErrorMin (uint16_t value) const {
    return this->writeRegister(0x94, (value >> 8) & 0xff) && this->writeRegister(0x95, value & 0xff);
  }
  uint16_t rpmErrorMin () const {
    return (this->readRegister(0x94, 0) << 8) + this->readRegister(0x95, 0);
  }
  void defaults () {
    clear();
    rpmErrorMin(100);
  }
};

class RPMChannel : public Channel<HalType, RPMList1, EmptyList, List4, PEERS_PER_CHANNEL, List0>, public Alarm {
private:
    static void tachoISR() { pulsecount++; }
    RPMMsg    msg;
    uint16_t  rpm;
    uint16_t  rpmLast;
    uint8_t   failcnt;
    uint8_t   measurecnt;
    bool      rpmErrorState;
    bool      rpmErrorStateLast;
    uint16_t  rpmErrorMinRPM;
  public:
    RPMChannel () : Channel(), Alarm(RPM_MEASURE_INTERVAL), rpm(0), rpmLast(0), failcnt(0),measurecnt(0), rpmErrorState(false), rpmErrorStateLast(false), rpmErrorMinRPM(0) {}
    virtual ~RPMChannel () {}

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      //calculate the rpm
      uint32_t rpm = pulsecount * (60 / RPM_MEASURE_INTERVAL);

      DPRINT("PWR / RPM = ");DDEC(currentPWMvalue);DPRINT("/");DDECLN(rpm);

      // RPM error handling is active AND fan power is > 0%  AND current rpm is below threshold (rpmErrorMinRPM)
      if (rpmErrorMinRPM > 0 && currentPWMvalue > 0 && rpm < rpmErrorMinRPM) {
        //let's check at least 2 times
        if (failcnt < 2) failcnt++;
      } else {
        //if everything is ok, we can reset the fail counter and the rpmError
        failcnt = 0;
        rpmErrorState = false;
      }

      //after two checks, the failure still persists
      if (failcnt == 2) {
        msg.init(device().nextcount(), rpm);
        device().broadcastEvent(msg);
        rpmErrorState = true;
      }

      //if the error state has changed (from true->false or false->true), send an info message to the ccu
      if (rpmErrorState != rpmErrorStateLast) {
        rpmErrorStateLast = rpmErrorState;
        DPRINT("ALARM ");DDECLN(rpmErrorState);
        this->changed(true);
      }

      //only send rpm value if it differs at least 200rpm
      //or if we measured at least 5 minutes
      if ( abs( (int16_t)rpm - (int16_t)rpmLast ) > 200 || measurecnt > (300 / RPM_MEASURE_INTERVAL)) {
        measurecnt = 0;
        rpmLast = rpm;
        msg.init(device().nextcount(), rpm);
        device().broadcastEvent(msg);
      }
      measurecnt++;

      //at least, set the next timer an reset the tacho pulsecounter
      Alarm::set(seconds2ticks(RPM_MEASURE_INTERVAL));
      pulsecount = 0;
      sysclock.add(*this);
    }

    void setup(Device<HalType, List0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      // if the TACHO_PIN is not used, we won't init the pin or start the timer
      if (TACHO_PIN > 0) {
        pinMode(TACHO_PIN, INPUT);
        if( digitalPinToInterrupt(TACHO_PIN) == NOT_AN_INTERRUPT )
          enableInterrupt(TACHO_PIN,tachoISR,FALLING);
        else
          attachInterrupt(digitalPinToInterrupt(TACHO_PIN),tachoISR,FALLING);

        sysclock.add(*this);
      }
    }

    void configChanged() {
      rpmErrorMinRPM = this->getList1().rpmErrorMin();
      DPRINT(F("RPM Error min. = ")); DDECLN(rpmErrorMinRPM);
    }

    uint8_t status () const { return 0; }
    uint8_t flags  () const {
      return rpmErrorState == true ? 0x01 << 1 : 0;
    }
};

class MixDevType : public ChannelDevice<HalType, VirtBaseChannel<HalType, List0>, 2, List0> {
  public:
    VirtChannel<HalType, DimmerChannelType , List0>  ch1;
    VirtChannel<HalType, RPMChannel,         List0>  ch2;
  public:
    typedef ChannelDevice<HalType, VirtBaseChannel<HalType, List0>, 2, List0> DeviceType;

    MixDevType (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr) {
      DeviceType::registerChannel(ch1, 1);
      DeviceType::registerChannel(ch2, 2);
    }
    virtual ~MixDevType () {}

    DimmerChannelType&  dimmerChannel  (__attribute__((unused)) uint8_t ch) {
      return ch1;
    }
    RPMChannel& rpmChannel () {
      return ch2;
    }

    static int const channelCount = 1;
    static int const virtualCount = 1;
};

HalType hal;
MixDevType sdev(devinfo, 0x20);
DimmerControl<HalType, MixDevType, FAN> control(sdev);
ConfigToggleButton<MixDevType> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  if ( control.init(hal, PWM_PIN, PWR_PIN) ) {
    sdev.channel(1).peer(cfgBtn.peer());
  }
  sdev.channel(2).changed(true);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
   // hal.activity.savePower<Idle<true> >(hal);
  }
}
