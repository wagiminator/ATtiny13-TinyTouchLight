# TinyTouchLight - Dimmable USB Night Light with Capacitive Touch Control
TinyTouchLight is a dimmable USB night light with capacitive touch control based on the ATtiny13A. It plugs into any USB charger or power bank.

- Design Files (EasyEDA): 

![TinyTouchLight_pic3.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny13-TinyTouchLight/main/documentation/TinyTouchLight_pic3.jpg)

# Capacitive Touch Button
## Introduction
This implementation of a touch-sensitive button (touchkey) is based on the charge sharing approach similar to [Atmel's QTouchADC](http://ww1.microchip.com/downloads/en/Appnotes/doc8497.pdf) and [Tim's TinyTouchLib](https://github.com/cpldcpu/TinyTouchLib). It works without external components, only a resistor for noise reduction is used, which can also be dispensed with. The touchkey itself is a small copper area on the PCB (sense electrode), which is covered with solder mask. This sense electrode is connected to a single ADC-capable pin of the ATtiny via the resistor.

## Basic Principle
The touch sense pin (here PB4) is connected through series resistor R<sub>S</sub> to the sensor electrode capacitance, represented by C<sub>x</sub>. The switch SW represents a finger touching the key. The capacitance introduced by the finger is represented as C<sub>t</sub>. When the key is touched, C<sub>t</sub> is switched into the circuit forming a parallel capacitor with C<sub>x</sub>, changing the effective sensor capacitance to C<sub>x</sub> + C<sub>t</sub>.

![TinyTouchLight_principle.png](https://raw.githubusercontent.com/wagiminator/ATtiny13-TinyTouchLight/main/documentation/TinyTouchLight_principle.png)

It should be noted that C<sub>x</sub> and C<sub>t</sub> are not physical capacitors. C<sub>x</sub> is the effective capacitance of the sense electrode and C<sub>x</sub> + C<sub>t</sub> is the effective capacitance of the human finger touching the sensor.

## Component Selection
The series resistor R<sub>S</sub> is nominally 1kΩ, but it may be increased to higher values to improve the noise immunity of the circuit. The value of R<sub>S</sub> should be increased in steps to find the lowest value that provides adequate noise immunity. Resistance values of up to 100kΩ have proven to be useful in extremely noisy environments. For this application a 47kΩ resistor was chosen.

The value of C<sub>x</sub> should be close to that of the ADC’s internal sample-and-hold capacitor C<sub>S/H</sub> (~14pF). For best performance it is recommended that C<sub>xt</sub> should not be greater than ~60pF. If the sensor electrode is designed as a copper surface on the PCB, then it should be roughly as large as the contact surface of a finger (6-10 mm in diameter if the touchkey sensor is round, or with a 6-10 mm side if the touchkey sensor is square). There shouldn't be any traces on the other side of the PCB. The back side can have a ground plane, but this should not be a solid fill.

## Sensor Acquisition
The acquisition method works by sharing charge between the ADC’s internal sample-and-hold capacitor (C<sub>S/H</sub>) and the sense electrode capacitance (C<sub>x</sub>). When the sensor is touched the effective capacitance of the sensor electrode increases and becomes C<sub>x</sub> + C<sub>t</sub>. This affects the amount of charge shared between the capacitors. When pre-charging C<sub>x</sub> and sharing with C<sub>S/H</sub>, charge transferred to C<sub>S/H</sub> increases on touch and ADC input voltage increases. When pre-charging C<sub>S/H</sub> and sharing with C<sub>x</sub>, charge remaining on C<sub>S/H</sub> decreases on touch and ADC input voltage decreases. But the resulting signal from the averaged ADC values increases on touch. If the difference between signal and reference is greater than the user-determined threshold (delta), a touch is reported.

The charge sharing is carried out in the following sequence:
1. Precharge touchkey LOW and S/H cap HIGH.
2. Connect touchkey and S/H cap in parallel. Read the voltage via the ADC.
3. Precharge touchkey HIGH and S/H cap LOW.
4. Connect touchkey and S/H cap in parallel. Read the voltage via the ADC.
5. Calculate the voltage difference and compare it with the user-determined threshold.

### Step 1: Precharge touchkey LOW and S/H cap HIGH.
By setting the touch sense pin (PB4) to OUTPUT LOW, the touchkey is discharged. By setting the ADC muxer to a spare pin (here PB3) and setting this pin to OUTPUT HIGH, the internal S/H capacitor is charged.

![TinyTouchLight_step1.png](https://raw.githubusercontent.com/wagiminator/ATtiny13-TinyTouchLight/main/documentation/TinyTouchLight_step1.png)

```c
ADMUX   =  TC_SHC_ADC;              // connect spare pin to S/H cap
PORTB  |=  (1<<TC_SHC_PIN);         // charge S/H cap
PORTB  &= ~(1<<TC_PAD_PIN);         // prepare discharge touch pad
DDRB   |=  (1<<TC_PAD_PIN);         // discharge touch pad
_delay_us(TC_CHARGETIME);           // wait for precharge complete
```

### Step 2: Connect touchkey and S/H cap in parallel. Read the voltage via the ADC.
By setting the touch sense pin (PB4) to INPUT (no pullup) and setting the ADC muxer to this pin, charge is flowing between the capacitors until the charge is distributed proportionally between them - the charge is shared. The voltage across C<sub>S/H</sub>, due to the remaining charge, is sampled by the ADC.

![TinyTouchLight_step2.png](https://raw.githubusercontent.com/wagiminator/ATtiny13-TinyTouchLight/main/documentation/TinyTouchLight_step2.png)

```c
DDRB   &= ~(1<<TC_PAD_PIN);         // float pad input (pull up is off)
ADMUX   =  (1<<ADLAR) | TC_PAD_ADC; // connect touch pad to S/H and ADC
ADCSRA |=  (1<<ADSC);               // start voltage sampling
while (!(ADCSRA & (1<<ADIF)));      // wait for sampling complete
ADCSRA |= (1<<ADIF);                // clear ADIF
uint8_t dat1 = ADCH;                // read sampling result (voltage)
```

### Step 3: Precharge touchkey HIGH and S/H cap LOW.
By setting the touch sense pin (PB4) to OUTPUT HIGH, the touchkey is scharged, by setting the the ADC muxer to the spare pin (PB3) and setting this pin to OUTPUT HIGH, the internal S/H capacitor is discharged.

![TinyTouchLight_step3.png](https://raw.githubusercontent.com/wagiminator/ATtiny13-TinyTouchLight/main/documentation/TinyTouchLight_step3.png)

```c
ADMUX   =  TC_SHC_ADC;              // connect spare pin to S/H cap
PORTB  &= ~(1<<TC_SHC_PIN);         // discharge S/H cap
PORTB  |=  (1<<TC_PAD_PIN);         // prepare charge touch pad
DDRB   |=  (1<<TC_PAD_PIN);         // charge touch pad
_delay_us(TC_CHARGETIME);           // wait for precharge complete
```

### Step 4: Connect touchkey and S/H cap in parallel. Read the voltage via the ADC.
By setting the touch sense pin (PB4) to INPUT (no pullup) and setting the ADC muxer to this pin, charge is flowing between the capacitors until the charge is distributed proportionally between them - the charge is shared again. The resulting voltage is sampled by the ADC.

![TinyTouchLight_step4.png](https://raw.githubusercontent.com/wagiminator/ATtiny13-TinyTouchLight/main/documentation/TinyTouchLight_step4.png)

```c
DDRB   &= ~(1<<TC_PAD_PIN);         // float touch pad input
PORTB  &= ~(1<<TC_PAD_PIN);         // pull up off
ADMUX   =  (1<<ADLAR) | TC_PAD_ADC; // connect touch pad to S/H and ADC
ADCSRA |=  (1<<ADSC);               // start voltage sampling
while (!(ADCSRA & (1<<ADIF)));      // wait for sampling complete
ADCSRA |= (1<<ADIF);                // clear ADIF
uint8_t dat2 = ADCH;                // read sampling result (voltage)
```

### Step 5: Calculate the voltage difference and compare it with the user-determined threshold.
The measured and calculated voltage difference is returned by the function TC_getDelta(), which includes the code shown above. The function TC_sense() averages 16 of these measurements, calculates the difference with the non-touch bias and compares this difference with the user-defined thresholds. It also makes sure that there is no drift or stuck button. The function returns a value which represents if the touchkey is pressed, released or hold.

```c
uint8_t TC_sense(void) {
  uint16_t tmp = 0;
  for (uint8_t i=16; i; i--) {
    tmp += TC_getDelta();             // average 16 samples
    _delay_us(100);
  }
  int16_t diff = tmp - (TC_bias>>4);  // difference with bias value
  if (!TC_touch) {                    // not touched previously?
    if (diff > TC_THRESHOLD_ON) {     // new touch detected?
      TC_touch = 1;                   // set "is touched" flag
      TC_timer = 0;                   // reset touch timer
      return TC_PUSH;                 // return "new push"
    }
    TC_bias = (TC_bias - (TC_bias>>6)) + (tmp>>2); // update bias (low pass)
    return TC_OFF;                    // return "still not pushed" 
  }
  else {                              // touched previously?
    if (diff < TC_THRESHOLD_OFF) {    // touch button released?
      TC_touch = 0;                   // clear "is touched" flag
      return TC_RELEASE;              // return "touch pad released"
    }
    if (TC_timer == TC_TIMEOUT) {     // still touched but for too long?
      TC_bias  = TC_getDelta()<<8;    // maybe stuck situation, read new bias
      return TC_FAIL;                 // return "fail"
    }
    TC_timer++;                       // increase timer
    return TC_ON;                     // return "still pushed"
  }
}
```

# LED Dimmer
The LEDs are controlled via a PWM-capable pin (PB0) and a MOSFET. Timer0 generates the PWM signal, the duty cycle is controlled via the OCR0A register. The main function brings everything together:

```c
int main(void) {
  // Local variables
  uint8_t bright = 64;                // current brightness of LEDs 
  uint8_t dir    = 0;                 // current fade direction

  // Setup
  TCCR0A = (1<<COM0A1)                // clear OC0A on compare match, set at TOP
         | (1<<WGM01) | (1<<WGM00);   // fast PWM
  TCCR0B = (1<<CS00);                 // start timer without prescaler
  DDRB   = (1<<LED_PIN);              // set LED pin as output
  TC_init();                          // setup touch control
  
  // Loop
  while(1) {
    uint8_t sense = TC_sense();       // read touch pad
    if(sense == TC_PUSH) dir = !dir;  // change fade direction on new push
    if(sense == TC_ON) {              // fade on touch hold
      if(dir) {                       // fade in?
        if (bright < 196) bright++;   // increase brightness
      }
      else {                          // fade out?
        if (bright) bright--;         // decrease brightness
      }
    }    
    OCR0A = bright;                   // set brightness (PWM)
  }
}
```

# Compiling and Uploading the Firmware
Since there is no ICSP header on the board, you have to program the ATtiny before putting it into the IC socket. The [AVR Programmer Adapter](https://github.com/wagiminator/AVR-Programmer/tree/master/AVR_Programmer_Adapter) can help with this.

## If using the Arduino IDE
- Make sure you have installed [MicroCore](https://github.com/MCUdude/MicroCore).
- Go to **Tools -> Board -> MicroCore** and select **ATtiny13**.
- Go to **Tools** and choose the following board options:
  - **Clock:**  128 kHz internal osc.
  - **BOD:**    disabled
  - **Timing:** Micros disabled
- Connect your programmer to your PC and to the ATtiny.
- Go to **Tools -> Programmer** and select your ISP programmer (e.g. [USBasp](https://aliexpress.com/wholesale?SearchText=usbasp)).
- Go to **Tools -> Burn Bootloader** to burn the fuses.
- Open the TinySat sketch and click **Upload**.

## If using the precompiled hex-file
- Make sure you have installed [avrdude](https://learn.adafruit.com/usbtinyisp/avrdude).
- Connect your programmer to your PC and to the ATtiny.
- Open a terminal.
- Navigate to the folder with the hex-file.
- Execute the following command (if necessary replace "usbasp" with the programmer you use):
  ```
  avrdude -c usbasp -p t13 -U lfuse:w:0x3b:m -U hfuse:w:0xff:m -U flash:w:tinytouchlight.hex
  ```

## If using the makefile (Linux/Mac)
- Make sure you have installed [avr-gcc toolchain and avrdude](http://maxembedded.com/2015/06/setting-up-avr-gcc-toolchain-on-linux-and-mac-os-x/).
- Connect your programmer to your PC and to the ATtiny.
- Open the makefile and change the programmer if you are not using usbasp.
- Open a terminal.
- Navigate to the folder with the makefile and the sketch.
- Run "make install" to compile, burn the fuses and upload the firmware.

# References
1. [TinyTouchLib](https://github.com/cpldcpu/TinyTouchLib)
2. [Atmel AVR3001: QTouchADC](http://ww1.microchip.com/downloads/en/Appnotes/doc8497.pdf)
3. [ATtiny13A Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/doc8126.pdf)

![TinyTouchLight_pic1.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny13-TinyTouchLight/main/documentation/TinyTouchLight_pic1.jpg)
![TinyTouchLight_pic2.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny13-TinyTouchLight/main/documentation/TinyTouchLight_pic2.jpg)
![TinyTouchLight_pic4.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny13-TinyTouchLight/main/documentation/TinyTouchLight_pic4.jpg)

# License
![license.png](https://i.creativecommons.org/l/by-sa/3.0/88x31.png)

This work is licensed under Creative Commons Attribution-ShareAlike 3.0 Unported License. 
(http://creativecommons.org/licenses/by-sa/3.0/)
