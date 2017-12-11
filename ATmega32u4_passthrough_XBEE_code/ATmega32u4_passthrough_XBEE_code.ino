// Copyright 2015, Matthijs Kooijman <matthijs@stdin.nl>
//
// Permission is hereby granted, free of charge, to anyone
// obtaining a copy of this document and accompanying files, to do
// whatever they want with them without any restriction, including, but
// not limited to, copying, modification and redistribution.
//
// NO WARRANTY OF ANY KIND IS PROVIDED.
//
//
// This example shows how to use the an Arduino Leonardo and an XBee
// shield to allow a computer to talk to an XBee module, e.g. by using
// XCTU. By additionally wiring up some handshaking signals, full
// firmware updates and recovery are supported.
//
// This works well because the Leonardo (or similar 32u4-based boards)
// have a direct USB connection to the computer, offering a virtual
// serial port, keeping the hardware serial port free for talking to the
// XBee module. Additionally, since the Leonardo does not reset when the
// virtual serial port is opened, XCTU can will actually talk to this
// sketch (instead of talking to the bootloader, which happens on most
// other Arduinos that reset when serial is opened). Also, since the
// baudrate configured by the computer is known to the Leonardo, the
// baudrate for the XBee can be changed on the fly, allowing firmware
// updates. Finally, since the Leonardo can detect requests for a
// "serial break" and the handshaking pin signals, extended recovery is
// made possible.
//
// To use this sketch, connect the XBee module to the hardware serial
// port of the Arduino Leonardo. It depens a bit on the shield used, but
// most should support this configurating. DOUT on the XBee should be
// connected to pin 0 (RX) on the Leonardo, DIN to pin 1 (TX). Of course
// a level converter should be present as well.
//
// With this minimal wiring, you can send and receive data and commands
// and, in most cases replace the firmware as well (only S1 modules seem
// to require handshaking signals for firmware updates). Firmware
// recovery also works in some circumstances (when the bootloader
// detects an invalid firmware and it doesn't start the main firmware),
// but not always.
//
// To allow recovery in more cases, the DTR and RTS pins on the XBee
// have to be connected. See (or change) the constants below for where
// these pins must be connected to on the Arduino side. These pins can
// be connected directly (no level converter needed), since the code
// below only writes LOW to the pins (setting them to INPUT and relying
// on the pullups in the XBee for a high level).
//
// To support serial breaks, this code fiddles with the UART hardware
// registers directly (since Arduino's HardwareSerial does not support
// the needed API for sending breaks currently).

uint32_t brkStart = 0;
uint16_t brkLength = 0;
uint32_t baud = 9600;

uint8_t DTR_PIN = 10;
uint8_t CTS_PIN = 11;
uint8_t RTS_PIN = 12;

#define SerialUSB Serial
#define SerialHW  Serial1

void setup() {
  // Baudrate is ignored


  pinMode(CTS_PIN, INPUT);
  pinMode(8, OUTPUT);
  digitalWrite(CTS_PIN, LOW);
  digitalWrite(8, HIGH);
  
  digitalWrite(7, HIGH); // MCU power
  pinMode(7, OUTPUT);
//  digitalWrite(4, HIGH); // radio power
//  pinMode(4, OUTPUT);
//  
//  while(true);



  
  SPCR &= ~(1 << 6);
  digitalWrite(17, LOW);
  pinMode(17, INPUT);
  
  pinMode(6, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(13, INPUT);
  
  digitalWrite(6, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(13, LOW);



  digitalWrite(4, HIGH);
  pinMode(4, OUTPUT);

  delay(5000);


  SerialUSB.begin(0);
  // Initialize using an arbitrary default baudrate
  SerialHW.begin(baud);
}

void setBreak(bool enable) {
  if (enable) {
    // If a break was requested, force the TX pin low and then disable the
    // UART TX side so the digitalWrite actually takes effect
    pinMode(1, OUTPUT);
    digitalWrite(1, LOW);
    UCSR1B &= ~(1 << TXEN1);
  } else {
    // End a break by enabling the UART TX again.
    UCSR1B |= (1 << TXEN1);
  }
}

bool inBreak() {
  // We're in a break when the UART TX is disabled
  return (UCSR1B & (1 << TXEN1)) == 0;
}

void loop() {
  // Forward the DTR and RTS signals. Use OUTPUT LOW to pull low and
  // INPUT to leave the pin floating. This needs a pullup on the other
  // side, but also works when the other device is 3.3V
  pinMode(DTR_PIN, SerialUSB.dtr() ? OUTPUT : INPUT);
  pinMode(RTS_PIN, SerialUSB.rts() ? OUTPUT : INPUT);

  // Process break requests. 0xffff means to start a break, 0 to end it
  // and any other value to start a break that lasts that many ms.
  int32_t brk = SerialUSB.readBreak();
  if (brk > 0) {
    SerialHW.flush();
    setBreak(true);

    // If the break has a defined length, keep track of it so it can be
    // disabled at the right time
    if (brk != 0xffff) {
      brkLength = brk;
      brkStart = micros();
    }
  } else if (brk == 0 || (brkLength > 0 && (micros() - brkStart) >= 1000L * brkLength)) {
    // If a timed break was running and its time expires, or if the
    // break is explicitly canceled through a request with time = 0,
    // end the break
    setBreak(false);
    brkLength = 0;
  }

  // If the requested baudrate changed on SerialUSB, update it on Serial
  // too (but not while a break condition is ongoing, since calling
  // SerialHW.begin() will end the break condition
  if (!inBreak() && SerialUSB.baud() != baud) {
    baud = SerialUSB.baud();

    // Set up the TX pin as OUTPUT HIGH, so that when the UART is
    // disabled, the pin remains high
    pinMode(1, OUTPUT);
    digitalWrite(1, HIGH);

    SerialHW.end();
    SerialHW.begin(baud);
  }

  // Forward data between SerialUSB and Serial
  if (!inBreak()) {
    // Only write to Serial when TX is enabled, to prevent lockup
    if (SerialUSB.available())
      SerialHW.write(SerialUSB.read());
  }
  if (SerialHW.available())
    SerialUSB.write(SerialHW.read());
}
