//#include "pins_arduino.h"  // defines SS,MOSI,MISO,SCK
#include <Arduino.h>
#include <SPI.h>

#define RESET_LP 9
#define LED_PMODE 8

#define PIN_SPI_SS     (10)
#define PIN_SPI_MOSI   (11)
#define PIN_SPI_MISO   (12)
#define PIN_SPI_SCK    (13)

void enter_programming_mode();
void lp_isp_read(byte, byte, byte, int);
void exit_programming_mode();
void lp_isp_write(byte, byte, byte,const unsigned char* p, int);

byte datoRecibido;

//const int PIN_SPI_SS = 10;
const int ledPModePin = LED_PMODE;
const int targetResetPin = RESET_LP;

const byte PREAMBLE = 0xAA;
const byte PREAMBLE2 = 0x55;
const byte PROGRAM_ENABLE = 0xAC;
const byte PROGRAM_ENABLE_ADDR_HI = 0x53;
const byte DUMMY = 0x01;


const byte CHIP_ERASE = 0x8A;
const byte LOAD_CODE_PAGE_BUFFER = 0x51;

const byte WRITE_CODE_PAGE = 0x50;
const byte READ_CODE_PAGE = 0x30;

const byte WRITE_USER_FUSES = 0xE1;
const byte WRITE_USER_FUSES_AUTO_ERASE = 0xF1;
const byte WRITE_LOCK_BITS = 0xE4;

const byte READ_LOCK_BITS = 0x64;
const byte READ_ATMEL_SIGNATURE = 0x38;
const byte READ_USER_FUSES = 0x31;

static int programming_mode;

#define AT89LP51

// ledblink binary for either at89lpx052 or at89lp51x
// configures p1.3 as push pull output and blinks it
const unsigned char bin[] = {
#ifndef AT89LP51
        0x02, 0x00, 0x08, 0x12, 0x00, 0x83, 0x80, 0xFE, 0x75, 0x81, 0x0B, 0x12, 0x00, 0xA1, 0xE5, 0x82,
        0x60, 0x03, 0x02, 0x00, 0x03, 0x79, 0x00, 0xE9, 0x44, 0x00, 0x60, 0x1B, 0x7A, 0x00, 0x90, 0x00,
        0xA5, 0x78, 0x00, 0x75, 0xA0, 0x00, 0xE4, 0x93, 0xF2, 0xA3, 0x08, 0xB8, 0x00, 0x02, 0x05, 0xA0,
        0xD9, 0xF4, 0xDA, 0xF2, 0x75, 0xA0, 0xFF, 0xE4, 0x78, 0xFF, 0xF6, 0xD8, 0xFD, 0x78, 0x00, 0xE8,
        0x44, 0x00, 0x60, 0x0A, 0x79, 0x00, 0x75, 0xA0, 0x00, 0xE4, 0xF3, 0x09, 0xD8, 0xFC, 0x78, 0x00,
        0xE8, 0x44, 0x00, 0x60, 0x0C, 0x79, 0x00, 0x90, 0x00, 0x00, 0xE4, 0xF0, 0xA3, 0xD8, 0xFC, 0xD9,
        0xFA, 0x02, 0x00, 0x03, 0xAA, 0x82, 0xAB, 0x83, 0xAC, 0x0A, 0xAD, 0x0B, 0xC3, 0xEC, 0x9A, 0xED,
        0x64, 0x80, 0x8B, 0xF0, 0x63, 0xF0, 0x80, 0x95, 0xF0, 0x50, 0x07, 0x0C, 0xBC, 0x00, 0xED, 0x0D,
        0x80, 0xEA, 0x22, 0x53, 0xC2, 0xF7, 0x43, 0xC3, 0x08, 0xC2, 0x93, 0x90, 0x75, 0x30, 0x12, 0x00,
        0x64, 0x90, 0x75, 0x30, 0x12, 0x00, 0x64, 0x90, 0x75, 0x30, 0x12, 0x00, 0x64, 0xB2, 0x93, 0x80,
        0xEA, 0x75, 0x82, 0x00, 0x22, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
#else
        0x02, 0x00, 0x08, 0x12, 0x00, 0x83, 0x80, 0xFE, 0x75, 0x81, 0x07, 0x12, 0x00, 0xA1, 0xE5, 0x82,
        0x60, 0x03, 0x02, 0x00, 0x03, 0x79, 0x00, 0xE9, 0x44, 0x00, 0x60, 0x1B, 0x7A, 0x00, 0x90, 0x00,
        0xA5, 0x78, 0x00, 0x75, 0xA0, 0x00, 0xE4, 0x93, 0xF2, 0xA3, 0x08, 0xB8, 0x00, 0x02, 0x05, 0xA0,
        0xD9, 0xF4, 0xDA, 0xF2, 0x75, 0xA0, 0xFF, 0xE4, 0x78, 0xFF, 0xF6, 0xD8, 0xFD, 0x78, 0x00, 0xE8,
        0x44, 0x00, 0x60, 0x0A, 0x79, 0x00, 0x75, 0xA0, 0x00, 0xE4, 0xF3, 0x09, 0xD8, 0xFC, 0x78, 0x00,
        0xE8, 0x44, 0x00, 0x60, 0x0C, 0x79, 0x00, 0x90, 0x00, 0x00, 0xE4, 0xF0, 0xA3, 0xD8, 0xFC, 0xD9,
        0xFA, 0x02, 0x00, 0x03, 0xAA, 0x82, 0xAB, 0x83, 0x7C, 0x00, 0x7D, 0x00, 0xC3, 0xEC, 0x9A, 0xED,
        0x64, 0x80, 0x8B, 0xF0, 0x63, 0xF0, 0x80, 0x95, 0xF0, 0x50, 0x07, 0x0C, 0xBC, 0x00, 0xED, 0x0D,
        0x80, 0xEA, 0x22, 0x53, 0xD6, 0xF7, 0x43, 0xD7, 0x08, 0xC2, 0x93, 0x90, 0x75, 0x30, 0x12, 0x00,
        0x64, 0x90, 0x75, 0x30, 0x12, 0x00, 0x64, 0x90, 0x75, 0x30, 0x12, 0x00, 0x64, 0xB2, 0x93, 0x80,
        0xEA, 0x75, 0x82, 0x00, 0x22, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
#endif
};

// - the at89lp51x must have the boot loader jump bit (offset 0x4) set in order to start booting from the
// application at address 0x0000.
// - also clear the 12 clock compatibility mode (offset 0x6)
// This fuses array will do:
const unsigned char fuses[] =
{
        0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0 /*0xFF*/, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF,
};

unsigned char buf[1];

void setup() {
        Serial.begin(115200);

        // datasheet ch. 23.5.5
        //SPI.setBitOrder(MSBFIRST);
        //SPI.setDataMode(SPI_MODE0);

        programming_mode = 0;

        // tristate slave select pin
        pinMode(PIN_SPI_SS, OUTPUT );

        // don't keep target in reset
        pinMode(targetResetPin, OUTPUT);
        //digitalWrite(targetResetPin, LOW);

        pinMode(ledPModePin, OUTPUT);
        digitalWrite(ledPModePin, LOW);


        SPI.begin();
        SPI.setClockDivider(SPI_CLOCK_DIV64);
        SPI.setBitOrder(MSBFIRST);
        digitalWrite(targetResetPin, LOW);
        digitalWrite(SS, HIGH); // disable Slave Select
        //enter_programming_mode(); //se debe hacer primero para habilitar MISO
        //lp_isp_read(0x28,0x00, 0x00, 1);

}

void loop() {

        if(Serial.available()) {
                unsigned char c = Serial.read();
                int i;

                switch(c) {

                case 'r':
                        Serial.println("read flash...");
                        for(i=0; i < 0x100 /*0*/; i+=64) {
                                Serial.print("@"); Serial.print(i, HEX); Serial.println("");
                                unsigned char addrLo = i & 0xFF;
                                unsigned char addrHi = (i & 0xF00) >> 8;
                                lp_isp_read(READ_CODE_PAGE, addrHi, addrLo, 32);
                        }
                        break;

                case 'z':
                        //enter_programming_mode(); //se debe hacer primero para habilitar MISO
                        lp_isp_read(0x28,0x00, 0x00, 1);
                        break;

                case '9':
                        //enter_programming_mode();         //se debe hacer primero para habilitar MISO
                        lp_isp_read(0x38,0x00, 0x00, 3);
                        break;

                case 's':
                        Serial.println("Signature....");
                        lp_isp_read(READ_ATMEL_SIGNATURE, 0x00, 0x00, 3);
                        Serial.println("");
                        Serial.println("Fuses...");
                        lp_isp_read(READ_USER_FUSES, 0, 0, 6);
                        break;

                case 'l':
                        Serial.println("read lock bits...");
                        lp_isp_read(READ_LOCK_BITS, 0, 0, 1);
                        break;


                case 'e':
                        Serial.println("chip erase...");
                        lp_isp_read(CHIP_ERASE, 0, 0, 0);
                        break;

                case 'w':
                        Serial.println("write flash...");
                        for(i=0; i<sizeof(bin); i+=32) {
                                unsigned char addrLo = i & 0xFF;
                                unsigned char addrHi = (i & 0xF00) >> 8;
                                Serial.print("@"); Serial.print(i, HEX);
                                Serial.print(addrHi, HEX); Serial.print(" ");
                                Serial.print(addrLo, HEX); Serial.print(" ");
                                Serial.println("");
                                lp_isp_write(WRITE_CODE_PAGE, addrHi, addrLo, &bin[i], 32);
                        }
                        break;

                #ifdef AT89LP51
                case 'a':
                        Serial.println("set boot loader jump bit (=> start app)...");
                        lp_isp_write(WRITE_USER_FUSES_AUTO_ERASE, 0, 0, &fuses[0], 19);
                        break;
                #endif


                case 'L':
                        Serial.println("write lock bits...");
                        buf[0] = 0;
                        lp_isp_write(WRITE_LOCK_BITS, 0, 0, &buf[0], 1);
                        break;

                }
        }
}

//at89lp52
void enter_programming_mode() {

        Serial.println("enter programming mode");
        programming_mode = 1;
        // reset target
        digitalWrite(targetResetPin, HIGH);
        digitalWrite(PIN_SPI_SS, LOW);

        datoRecibido = SPI.transfer(0xAC);
        Serial.println(datoRecibido, HEX);
        //2
        datoRecibido = SPI.transfer(0X53);
        Serial.println(datoRecibido,HEX);
        //3
        datoRecibido = SPI.transfer(0x01);
        Serial.println(datoRecibido,HEX);
        Serial.println(datoRecibido,BIN);

        datoRecibido = SPI.transfer(0x01);
        Serial.println(datoRecibido,HEX);
        Serial.println(datoRecibido,BIN);

        delay(1);

        //digitalWrite(targetResetPin, LOW);
        digitalWrite(SS, HIGH); // disable Slave Select

}

void lp_isp_read(byte opcode, byte addr_hi, byte addr_lo, int bytesToRead ) {
        byte reply;
        int i;
        Serial.println("lectura de datos");
        if(!programming_mode)
                enter_programming_mode();

        // take the slave select low to select the device:
        digitalWrite(PIN_SPI_SS, LOW);

        datoRecibido = SPI.transfer(opcode);
        Serial.println(datoRecibido, HEX);
        datoRecibido = SPI.transfer(addr_hi);
        Serial.println(datoRecibido,HEX);
        datoRecibido = SPI.transfer(addr_lo);
        Serial.println(datoRecibido,HEX); //1

        for(i=0; i<bytesToRead; i++) {
                reply = SPI.transfer(0x00);

                Serial.print(reply, HEX);
                if(i%16==15)
                        Serial.println("");
                else
                        Serial.print(" ");
        }

        Serial.println("");
        digitalWrite(targetResetPin, LOW);
        digitalWrite(SS, HIGH); // disable Slave Select
        digitalWrite(ledPModePin, LOW);
        programming_mode = 0;
}

void lp_isp_write(byte opcode, byte addr_hi, byte addr_lo,
                  const unsigned char* p, int bytesToWrite ) {
        byte reply;
        int i;

        if(!programming_mode)
                enter_programming_mode();

        // take the slave select low to select the device:
        digitalWrite(PIN_SPI_SS, LOW);

        SPI.transfer(opcode);
        SPI.transfer(addr_hi);
        SPI.transfer(addr_lo);

        for(i=0; i<bytesToWrite; i++) {
                SPI.transfer(p[i]);

                Serial.print(p[i], HEX);
                if(i%16==15)
                        Serial.println("");
                else
                        Serial.print(" ");

        }

        // take the slave select high to de-select:
        digitalWrite(PIN_SPI_SS, HIGH);
        digitalWrite(ledPModePin, LOW);

}

void exit_programming_mode() {

        Serial.println("\nexit programming mode");
        programming_mode = 0;

        digitalWrite(PIN_SPI_SS, HIGH);

        SPI.end();

        // We're about to take the target out of reset
        // so configure SPI pins as input
        pinMode(MOSI, INPUT);
        pinMode(SCK, INPUT);
        pinMode(PIN_SPI_SS, INPUT);

        // get target out of reset
        digitalWrite(targetResetPin, LOW);
}
