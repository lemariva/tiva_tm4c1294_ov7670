ov7670 camera with Tiva C TM4C1294XL
====================================

This code allows taking pictures using the `ov7670` camera with `AL422 FIFO` and transfering them through serial interface to the host. The ov7670 is connected to the launchpad `Tiva C Series EK-TM4C1294XL`.

Wiring
------------
#### `ov7670 (***)` <-> `Tiva C EK-TM4C1294XL`
| ov7670         	| EK-TM4C1294XL |
| ----------------- |:-------------:|
| VCC (PIN1)	 	| +3v3			|
| GND (PIN2)     	| GND			|
| VSYNC	(PIN5)   	| PP4			|
| RRST (PIN9)    	| PA0 (*)	    |
| RCLK (PIN11    	| PP5		    |
| SCL_SCCB (PIN3)	| PN5 (**)	    |
| SDA_SCCB (PIN4)	| PN4 (**)	    |
| WEN (PIN7)      	| PM7		    |
| D0-D7 (PIN13-20) 	| PK0-PK7	    |

#### `CP2102 (****)` <-> `Tiva C EK-TM4C1294XL`
| CP2102 		| EK-TM4C1294XL |
| ------------- |:-------------:|
| TXD	        | PP0			|
| RXD	        | PP1			|
| GND	        | GND			|

+ (*)    Check JP4 -> Select CAN
+ (**)   I2C pins need pull-up resistors (e.g. 10kOhm connected to +3.3V)
+ (***)  Buy on ebay: https://goo.gl/52xkt3
+ (****) Buy on ebay: https://goo.gl/f7I005


Usage
-----
To take/get a picture you need Python 2.7:

```
python camview_tiva.py COMxx
```
`xx` -> UART port 

and you get:

![alt tag](https://raw.githubusercontent.com/lemariva/tiva_tm4c1294_ov7670/master/doc/camera_capture.PNG)

Requirements 
------------
### pyGame and pySerial:
1. install `pip`: https://pip.pypa.io/en/stable/installing/#do-i-need-to-install-pip
2. install `pyGame`: `python -m pip install pygame`
3. install `pySerial`: `python -m pip install pyserial`

### CCS6 
1. install: TivaWare C Series ver.: 2.1.3.156 | http://www.ti.com/tool/sw-tm4c

(other versions also work -> edit variable '${SW_ROOT}')

More info & Help
----------------
+ Blog: https://goo.gl/x28Lta 
* EK-TM4C1294XL: http://www.ti.com/tool/ek-tm4c1294xl
* ov7670: http://www.voti.nl/docs/OV7670.pdf
* AL422B: http://www.averlogic.com/AL422B.asp
* Camera pinout: https://goo.gl/975hZJ
* Camera schematic: https://goo.gl/nBZcSB

Changelog
---------
* 1.0 - First release.

Credit
------
Based on : https://github.com/desaster/ov7670fifotest

Contact
------
* https://goo.gl/x28Lta