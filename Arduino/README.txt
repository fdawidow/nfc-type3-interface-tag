README
The 'libraries' folder contains the acual DynamicTag/FeliCaPlug library.
The 'example' folder contains some sample projects. Please add the "FeliCaPlug" library in the Arduino IDE (Sketch->Import Library-> Add library) to be able to compile the sample projects.

NOTE: 'FeliCaPlug' is the name of the Dynamic Tag used in Japan. The terms 'NFC Dynamic Tag' and 'FeliCaPlug' mean the same thing and are interchangeable. 

The standard pin layout for the library is as below:
-------------------------
| Dynamic Tag | Arduino |
| IRQ         | 2       |
| RFDET       | 3       |
| SW          | 9       |
| SEL         | 10      |
| DATA        | 11      |
| SPICLK      | 12      |
-------------------------

If different pin layouts please modify these FeliCaPlug.h defines according to your setup:
#define FELICA_PLUG_SW_PIN          9
#define FELICA_PLUG_SEL_PIN         10
#define FELICA_PLUG_DATA_PIN        11
#define FELICA_PLUG_SPICLK_PIN      12
#define FELICA_PLUG_IRQ_PIN         2
#define FELICA_PLUG_RFDET_PIN       3
