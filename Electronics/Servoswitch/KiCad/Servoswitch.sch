EESchema Schematic File Version 4
LIBS:Servoswitch-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Servoswitch"
Date "2018-12-13"
Rev "1.1"
Comp "Daniel Pusztai"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 5C1237EB
P 800 1100
F 0 "J1" H 720 775 50  0000 C CNN
F 1 "CH1_IN" H 720 866 50  0000 C CNN
F 2 "" H 800 1100 50  0001 C CNN
F 3 "~" H 800 1100 50  0001 C CNN
	1    800  1100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R1
U 1 1 5C123854
P 1350 1000
F 0 "R1" V 1143 1000 50  0000 C CNN
F 1 "10k" V 1234 1000 50  0000 C CNN
F 2 "" V 1280 1000 50  0001 C CNN
F 3 "~" H 1350 1000 50  0001 C CNN
	1    1350 1000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5C1238DA
P 1200 1300
F 0 "#PWR01" H 1200 1050 50  0001 C CNN
F 1 "GND" H 1205 1127 50  0000 C CNN
F 2 "" H 1200 1300 50  0001 C CNN
F 3 "" H 1200 1300 50  0001 C CNN
	1    1200 1300
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U1
U 1 1 5C123897
P 1700 2500
F 0 "U1" H 1700 2825 50  0000 C CNN
F 1 "74HC00" H 1700 2734 50  0000 C CNN
F 2 "" H 1700 2500 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 1700 2500 50  0001 C CNN
	1    1700 2500
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U1
U 2 1 5C1238E7
P 2500 1900
F 0 "U1" H 2500 2225 50  0000 C CNN
F 1 "74HC00" H 2500 2134 50  0000 C CNN
F 2 "" H 2500 1900 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 2500 1900 50  0001 C CNN
	2    2500 1900
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U1
U 3 1 5C12393D
P 3300 1500
F 0 "U1" H 3300 1825 50  0000 C CNN
F 1 "74HC00" H 3300 1734 50  0000 C CNN
F 2 "" H 3300 1500 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 3300 1500 50  0001 C CNN
	3    3300 1500
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U1
U 4 1 5C123992
P 2500 1100
F 0 "U1" H 2500 1425 50  0000 C CNN
F 1 "74HC00" H 2500 1334 50  0000 C CNN
F 2 "" H 2500 1100 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 2500 1100 50  0001 C CNN
	4    2500 1100
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U1
U 5 1 5C1239DC
P 4900 1800
F 0 "U1" H 5130 1846 50  0000 L CNN
F 1 "74HC00" H 5130 1755 50  0000 L CNN
F 2 "" H 4900 1800 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 4900 1800 50  0001 C CNN
	5    4900 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener D1
U 1 1 5C123A99
P 5600 1750
F 0 "D1" V 5554 1829 50  0000 L CNN
F 1 "3V3" V 5645 1829 50  0000 L CNN
F 2 "" H 5600 1750 50  0001 C CNN
F 3 "~" H 5600 1750 50  0001 C CNN
	1    5600 1750
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5C123E21
P 5250 1000
F 0 "R2" V 5043 1000 50  0000 C CNN
F 1 "100" V 5134 1000 50  0000 C CNN
F 2 "" V 5180 1000 50  0001 C CNN
F 3 "~" H 5250 1000 50  0001 C CNN
	1    5250 1000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5C123E7A
P 4900 2500
F 0 "#PWR07" H 4900 2250 50  0001 C CNN
F 1 "GND" H 4905 2327 50  0000 C CNN
F 2 "" H 4900 2500 50  0001 C CNN
F 3 "" H 4900 2500 50  0001 C CNN
	1    4900 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 2300 4900 2400
Wire Wire Line
	4900 2400 5600 2400
Wire Wire Line
	5600 2400 5600 1900
Connection ~ 4900 2400
Wire Wire Line
	4900 2400 4900 2500
Wire Wire Line
	5600 1600 5600 1200
Wire Wire Line
	5600 1200 4900 1200
Wire Wire Line
	4900 1300 4900 1200
$Comp
L Connector_Generic:Conn_01x03 J5
U 1 1 5C124219
P 800 1900
F 0 "J5" H 720 1575 50  0000 C CNN
F 1 "CH1_RX" H 720 1666 50  0000 C CNN
F 2 "" H 800 1900 50  0001 C CNN
F 3 "~" H 800 1900 50  0001 C CNN
	1    800  1900
	-1   0    0    1   
$EndComp
Wire Wire Line
	1200 1300 1200 1200
Wire Wire Line
	1200 1200 1000 1200
Wire Wire Line
	1000 1000 1200 1000
$Comp
L Device:R R5
U 1 1 5C1247C1
P 1350 1800
F 0 "R5" V 1143 1800 50  0000 C CNN
F 1 "10k" V 1234 1800 50  0000 C CNN
F 2 "" V 1280 1800 50  0001 C CNN
F 3 "~" H 1350 1800 50  0001 C CNN
	1    1350 1800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5C124D31
P 1200 2100
F 0 "#PWR05" H 1200 1850 50  0001 C CNN
F 1 "GND" H 1205 1927 50  0000 C CNN
F 2 "" H 1200 2100 50  0001 C CNN
F 3 "" H 1200 2100 50  0001 C CNN
	1    1200 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2100 1200 2000
Wire Wire Line
	1200 2000 1000 2000
Wire Wire Line
	1000 1800 1200 1800
$Comp
L MCU_Microchip_ATtiny:ATtiny13-20PU U6
U 1 1 5C1253C6
P 5100 6100
F 0 "U6" H 4570 6146 50  0000 R CNN
F 1 "ATtiny13-20PU" H 4570 6055 50  0000 R CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 5100 6100 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc2535.pdf" H 5100 6100 50  0001 C CNN
	1    5100 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 5C123C2C
P 5100 7000
F 0 "#PWR024" H 5100 6750 50  0001 C CNN
F 1 "GND" H 5105 6827 50  0000 C CNN
F 2 "" H 5100 7000 50  0001 C CNN
F 3 "" H 5100 7000 50  0001 C CNN
	1    5100 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 5500 5100 5400
Wire Wire Line
	5100 7000 5100 6700
$Comp
L Device:LED D5
U 1 1 5C1240A7
P 5900 6350
F 0 "D5" V 5938 6233 50  0000 R CNN
F 1 "Green" V 5847 6233 50  0000 R CNN
F 2 "" H 5900 6350 50  0001 C CNN
F 3 "~" H 5900 6350 50  0001 C CNN
	1    5900 6350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R15
U 1 1 5C12411E
P 5900 6750
F 0 "R15" V 5693 6750 50  0000 C CNN
F 1 "100" V 5784 6750 50  0000 C CNN
F 2 "" V 5830 6750 50  0001 C CNN
F 3 "~" H 5900 6750 50  0001 C CNN
	1    5900 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5C124154
P 5900 7000
F 0 "#PWR025" H 5900 6750 50  0001 C CNN
F 1 "GND" H 5905 6827 50  0000 C CNN
F 2 "" H 5900 7000 50  0001 C CNN
F 3 "" H 5900 7000 50  0001 C CNN
	1    5900 7000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 5C12425E
P 7500 5900
F 0 "#PWR027" H 7500 5650 50  0001 C CNN
F 1 "GND" H 7505 5727 50  0000 C CNN
F 2 "" H 7500 5900 50  0001 C CNN
F 3 "" H 7500 5900 50  0001 C CNN
	1    7500 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5C1242BF
P 7500 5650
F 0 "C3" H 7615 5696 50  0000 L CNN
F 1 "100n" H 7615 5605 50  0000 L CNN
F 2 "" H 7538 5500 50  0001 C CNN
F 3 "~" H 7500 5650 50  0001 C CNN
	1    7500 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 5900 7500 5800
Wire Wire Line
	7500 5500 7500 5400
NoConn ~ 5700 5800
NoConn ~ 5700 6200
NoConn ~ 5700 6300
Text GLabel 6600 5900 2    50   Input ~ 0
MODE_IN
Text GLabel 6600 6000 2    50   Input ~ 0
MODE_OUT
Wire Wire Line
	5900 6900 5900 7000
Wire Wire Line
	5700 6100 5900 6100
Wire Wire Line
	5900 6100 5900 6200
Wire Wire Line
	5700 5900 6600 5900
Wire Wire Line
	6600 6000 6400 6000
Wire Wire Line
	5900 6500 5900 6600
Text GLabel 1200 2400 0    50   Input ~ 0
MODE_OUT
Wire Wire Line
	1200 2400 1300 2400
Wire Wire Line
	1300 2400 1300 2600
Wire Wire Line
	1300 2600 1400 2600
Connection ~ 1300 2400
Wire Wire Line
	1300 2400 1400 2400
Text GLabel 2000 1200 0    50   Input ~ 0
MODE_OUT
Wire Wire Line
	2000 1200 2200 1200
Wire Wire Line
	2200 1000 1500 1000
Wire Wire Line
	2000 2500 2100 2500
Wire Wire Line
	2100 2500 2100 2000
Wire Wire Line
	2100 2000 2200 2000
Wire Wire Line
	2200 1800 1500 1800
Wire Wire Line
	2800 1900 2900 1900
Wire Wire Line
	2900 1900 2900 1600
Wire Wire Line
	2900 1600 3000 1600
Wire Wire Line
	3000 1400 2900 1400
Wire Wire Line
	2900 1400 2900 1100
Wire Wire Line
	2900 1100 2800 1100
Text GLabel 1600 1900 2    50   Input ~ 0
UB1
Wire Wire Line
	1600 1900 1000 1900
Text GLabel 4900 1000 0    50   Input ~ 0
UB1
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 5C12FDA6
P 4300 1400
F 0 "J3" H 4220 1075 50  0000 C CNN
F 1 "CH1_OUT" H 4220 1166 50  0000 C CNN
F 2 "" H 4300 1400 50  0001 C CNN
F 3 "~" H 4300 1400 50  0001 C CNN
	1    4300 1400
	1    0    0    -1  
$EndComp
Text GLabel 3800 1400 0    50   Input ~ 0
UB1
$Comp
L power:GND #PWR03
U 1 1 5C130E14
P 3900 1700
F 0 "#PWR03" H 3900 1450 50  0001 C CNN
F 1 "GND" H 3905 1527 50  0000 C CNN
F 2 "" H 3900 1700 50  0001 C CNN
F 3 "" H 3900 1700 50  0001 C CNN
	1    3900 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1700 3900 1300
Wire Wire Line
	3900 1300 4100 1300
Wire Wire Line
	3800 1400 4100 1400
Wire Wire Line
	4100 1500 3600 1500
Wire Wire Line
	4900 1000 5100 1000
Wire Wire Line
	5600 1000 5600 1200
Connection ~ 5600 1200
Wire Wire Line
	5400 1000 5600 1000
$Comp
L Connector_Generic:Conn_01x03 J13
U 1 1 5C13C9F8
P 800 6200
F 0 "J13" H 720 5875 50  0000 C CNN
F 1 "RX_MODE" H 720 5966 50  0000 C CNN
F 2 "" H 800 6200 50  0001 C CNN
F 3 "~" H 800 6200 50  0001 C CNN
	1    800  6200
	-1   0    0    1   
$EndComp
$Comp
L Regulator_Linear:LD1117S33TR_SOT223 U5
U 1 1 5C128C54
P 3000 6000
F 0 "U5" H 3000 6242 50  0000 C CNN
F 1 "LD1117S33TR" H 3000 6151 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 3000 6200 50  0001 C CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00000544.pdf" H 3100 5750 50  0001 C CNN
	1    3000 6000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR017
U 1 1 5C12A018
P 5100 5400
F 0 "#PWR017" H 5100 5250 50  0001 C CNN
F 1 "+3.3V" H 5115 5573 50  0000 C CNN
F 2 "" H 5100 5400 50  0001 C CNN
F 3 "" H 5100 5400 50  0001 C CNN
	1    5100 5400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR020
U 1 1 5C12BD04
P 3600 5900
F 0 "#PWR020" H 3600 5750 50  0001 C CNN
F 1 "+3.3V" H 3615 6073 50  0000 C CNN
F 2 "" H 3600 5900 50  0001 C CNN
F 3 "" H 3600 5900 50  0001 C CNN
	1    3600 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 5900 3600 6000
Wire Wire Line
	3600 6000 3300 6000
$Comp
L power:GND #PWR022
U 1 1 5C12C74F
P 3000 6600
F 0 "#PWR022" H 3000 6350 50  0001 C CNN
F 1 "GND" H 3005 6427 50  0000 C CNN
F 2 "" H 3000 6600 50  0001 C CNN
F 3 "" H 3000 6600 50  0001 C CNN
	1    3000 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5C12DE64
P 1250 6100
F 0 "R13" V 1043 6100 50  0000 C CNN
F 1 "10k" V 1134 6100 50  0000 C CNN
F 2 "" V 1180 6100 50  0001 C CNN
F 3 "~" H 1250 6100 50  0001 C CNN
	1    1250 6100
	0    1    1    0   
$EndComp
Text GLabel 1500 6100 2    50   Input ~ 0
MODE_IN
Wire Wire Line
	1500 6100 1400 6100
Wire Wire Line
	3000 6300 3000 6500
$Comp
L Device:CP C2
U 1 1 5C13B0D7
P 3600 6250
F 0 "C2" H 3718 6296 50  0000 L CNN
F 1 "10u" H 3718 6205 50  0000 L CNN
F 2 "" H 3638 6100 50  0001 C CNN
F 3 "~" H 3600 6250 50  0001 C CNN
	1    3600 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 6500 3600 6500
Wire Wire Line
	3600 6500 3600 6400
Connection ~ 3000 6500
Wire Wire Line
	3000 6500 3000 6600
Wire Wire Line
	3600 6100 3600 6000
Connection ~ 3600 6000
$Comp
L Device:C C1
U 1 1 5C13E7FA
P 2400 6250
F 0 "C1" H 2515 6296 50  0000 L CNN
F 1 "100n" H 2515 6205 50  0000 L CNN
F 2 "" H 2438 6100 50  0001 C CNN
F 3 "~" H 2400 6250 50  0001 C CNN
	1    2400 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 6400 2400 6500
Wire Wire Line
	2400 6500 3000 6500
Wire Wire Line
	2400 6100 2400 6000
Wire Wire Line
	2400 6000 2700 6000
Wire Wire Line
	2400 5900 2400 6000
Connection ~ 2400 6000
$Comp
L power:+6V #PWR019
U 1 1 5C1417E2
P 2400 5900
F 0 "#PWR019" H 2400 5750 50  0001 C CNN
F 1 "+6V" H 2415 6073 50  0000 C CNN
F 2 "" H 2400 5900 50  0001 C CNN
F 3 "" H 2400 5900 50  0001 C CNN
	1    2400 5900
	1    0    0    -1  
$EndComp
$Comp
L power:+6V #PWR018
U 1 1 5C141817
P 2100 5900
F 0 "#PWR018" H 2100 5750 50  0001 C CNN
F 1 "+6V" H 2115 6073 50  0000 C CNN
F 2 "" H 2100 5900 50  0001 C CNN
F 3 "" H 2100 5900 50  0001 C CNN
	1    2100 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5C14184C
P 2100 6400
F 0 "#PWR021" H 2100 6150 50  0001 C CNN
F 1 "GND" H 2105 6227 50  0000 C CNN
F 2 "" H 2100 6400 50  0001 C CNN
F 3 "" H 2100 6400 50  0001 C CNN
	1    2100 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 6100 1100 6100
Wire Wire Line
	2100 5900 2100 6200
Wire Wire Line
	2100 6200 1000 6200
Wire Wire Line
	2100 6400 2100 6300
Wire Wire Line
	1000 6300 2100 6300
$Comp
L power:+3.3V #PWR023
U 1 1 5C14DE85
P 7500 5400
F 0 "#PWR023" H 7500 5250 50  0001 C CNN
F 1 "+3.3V" H 7515 5573 50  0000 C CNN
F 2 "" H 7500 5400 50  0001 C CNN
F 3 "" H 7500 5400 50  0001 C CNN
	1    7500 5400
	1    0    0    -1  
$EndComp
NoConn ~ 1000 1100
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 5C15324E
P 6100 1100
F 0 "J2" H 6020 775 50  0000 C CNN
F 1 "CH2_IN" H 6020 866 50  0000 C CNN
F 2 "" H 6100 1100 50  0001 C CNN
F 3 "~" H 6100 1100 50  0001 C CNN
	1    6100 1100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 5C153254
P 6650 1000
F 0 "R3" V 6443 1000 50  0000 C CNN
F 1 "10k" V 6534 1000 50  0000 C CNN
F 2 "" V 6580 1000 50  0001 C CNN
F 3 "~" H 6650 1000 50  0001 C CNN
	1    6650 1000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5C15325A
P 6500 1300
F 0 "#PWR02" H 6500 1050 50  0001 C CNN
F 1 "GND" H 6505 1127 50  0000 C CNN
F 2 "" H 6500 1300 50  0001 C CNN
F 3 "" H 6500 1300 50  0001 C CNN
	1    6500 1300
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U2
U 1 1 5C153260
P 7000 2500
F 0 "U2" H 7000 2825 50  0000 C CNN
F 1 "74HC00" H 7000 2734 50  0000 C CNN
F 2 "" H 7000 2500 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 7000 2500 50  0001 C CNN
	1    7000 2500
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U2
U 2 1 5C153266
P 7800 1900
F 0 "U2" H 7800 2225 50  0000 C CNN
F 1 "74HC00" H 7800 2134 50  0000 C CNN
F 2 "" H 7800 1900 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 7800 1900 50  0001 C CNN
	2    7800 1900
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U2
U 3 1 5C15326C
P 8600 1500
F 0 "U2" H 8600 1825 50  0000 C CNN
F 1 "74HC00" H 8600 1734 50  0000 C CNN
F 2 "" H 8600 1500 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 8600 1500 50  0001 C CNN
	3    8600 1500
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U2
U 4 1 5C153272
P 7800 1100
F 0 "U2" H 7800 1425 50  0000 C CNN
F 1 "74HC00" H 7800 1334 50  0000 C CNN
F 2 "" H 7800 1100 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 7800 1100 50  0001 C CNN
	4    7800 1100
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U2
U 5 1 5C153278
P 10200 1800
F 0 "U2" H 10430 1846 50  0000 L CNN
F 1 "74HC00" H 10430 1755 50  0000 L CNN
F 2 "" H 10200 1800 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 10200 1800 50  0001 C CNN
	5    10200 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener D2
U 1 1 5C15327E
P 10900 1750
F 0 "D2" V 10854 1829 50  0000 L CNN
F 1 "3V3" V 10945 1829 50  0000 L CNN
F 2 "" H 10900 1750 50  0001 C CNN
F 3 "~" H 10900 1750 50  0001 C CNN
	1    10900 1750
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5C153284
P 10550 1000
F 0 "R4" V 10343 1000 50  0000 C CNN
F 1 "100" V 10434 1000 50  0000 C CNN
F 2 "" V 10480 1000 50  0001 C CNN
F 3 "~" H 10550 1000 50  0001 C CNN
	1    10550 1000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5C15328A
P 10200 2500
F 0 "#PWR08" H 10200 2250 50  0001 C CNN
F 1 "GND" H 10205 2327 50  0000 C CNN
F 2 "" H 10200 2500 50  0001 C CNN
F 3 "" H 10200 2500 50  0001 C CNN
	1    10200 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 2300 10200 2400
Wire Wire Line
	10200 2400 10900 2400
Wire Wire Line
	10900 2400 10900 1900
Connection ~ 10200 2400
Wire Wire Line
	10200 2400 10200 2500
Wire Wire Line
	10900 1600 10900 1200
Wire Wire Line
	10900 1200 10200 1200
Wire Wire Line
	10200 1300 10200 1200
$Comp
L Connector_Generic:Conn_01x03 J6
U 1 1 5C153298
P 6100 1900
F 0 "J6" H 6020 1575 50  0000 C CNN
F 1 "CH2_RX" H 6020 1666 50  0000 C CNN
F 2 "" H 6100 1900 50  0001 C CNN
F 3 "~" H 6100 1900 50  0001 C CNN
	1    6100 1900
	-1   0    0    1   
$EndComp
Wire Wire Line
	6500 1300 6500 1200
Wire Wire Line
	6500 1200 6300 1200
Wire Wire Line
	6300 1000 6500 1000
$Comp
L Device:R R6
U 1 1 5C1532A1
P 6650 1800
F 0 "R6" V 6443 1800 50  0000 C CNN
F 1 "10k" V 6534 1800 50  0000 C CNN
F 2 "" V 6580 1800 50  0001 C CNN
F 3 "~" H 6650 1800 50  0001 C CNN
	1    6650 1800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5C1532A7
P 6500 2100
F 0 "#PWR06" H 6500 1850 50  0001 C CNN
F 1 "GND" H 6505 1927 50  0000 C CNN
F 2 "" H 6500 2100 50  0001 C CNN
F 3 "" H 6500 2100 50  0001 C CNN
	1    6500 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2100 6500 2000
Wire Wire Line
	6500 2000 6300 2000
Wire Wire Line
	6300 1800 6500 1800
Text GLabel 6500 2400 0    50   Input ~ 0
MODE_OUT
Wire Wire Line
	6500 2400 6600 2400
Wire Wire Line
	6600 2400 6600 2600
Wire Wire Line
	6600 2600 6700 2600
Connection ~ 6600 2400
Wire Wire Line
	6600 2400 6700 2400
Text GLabel 7300 1200 0    50   Input ~ 0
MODE_OUT
Wire Wire Line
	7300 1200 7500 1200
Wire Wire Line
	7500 1000 6800 1000
Wire Wire Line
	7300 2500 7400 2500
Wire Wire Line
	7400 2500 7400 2000
Wire Wire Line
	7400 2000 7500 2000
Wire Wire Line
	7500 1800 6800 1800
Wire Wire Line
	8100 1900 8200 1900
Wire Wire Line
	8200 1900 8200 1600
Wire Wire Line
	8200 1600 8300 1600
Wire Wire Line
	8300 1400 8200 1400
Wire Wire Line
	8200 1400 8200 1100
Wire Wire Line
	8200 1100 8100 1100
Text GLabel 6900 1900 2    50   Input ~ 0
UB2
Wire Wire Line
	6900 1900 6300 1900
Text GLabel 10200 1000 0    50   Input ~ 0
UB2
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 5C1532C6
P 9600 1400
F 0 "J4" H 9520 1075 50  0000 C CNN
F 1 "CH2_OUT" H 9520 1166 50  0000 C CNN
F 2 "" H 9600 1400 50  0001 C CNN
F 3 "~" H 9600 1400 50  0001 C CNN
	1    9600 1400
	1    0    0    -1  
$EndComp
Text GLabel 9100 1400 0    50   Input ~ 0
UB2
$Comp
L power:GND #PWR04
U 1 1 5C1532CD
P 9200 1700
F 0 "#PWR04" H 9200 1450 50  0001 C CNN
F 1 "GND" H 9205 1527 50  0000 C CNN
F 2 "" H 9200 1700 50  0001 C CNN
F 3 "" H 9200 1700 50  0001 C CNN
	1    9200 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 1700 9200 1300
Wire Wire Line
	9200 1300 9400 1300
Wire Wire Line
	9100 1400 9400 1400
Wire Wire Line
	9400 1500 8900 1500
Wire Wire Line
	10200 1000 10400 1000
Wire Wire Line
	10900 1000 10900 1200
Connection ~ 10900 1200
Wire Wire Line
	10700 1000 10900 1000
NoConn ~ 6300 1100
$Comp
L Connector_Generic:Conn_01x03 J7
U 1 1 5C155E29
P 800 3300
F 0 "J7" H 720 2975 50  0000 C CNN
F 1 "CH3_IN" H 720 3066 50  0000 C CNN
F 2 "" H 800 3300 50  0001 C CNN
F 3 "~" H 800 3300 50  0001 C CNN
	1    800  3300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R7
U 1 1 5C155E2F
P 1350 3200
F 0 "R7" V 1143 3200 50  0000 C CNN
F 1 "10k" V 1234 3200 50  0000 C CNN
F 2 "" V 1280 3200 50  0001 C CNN
F 3 "~" H 1350 3200 50  0001 C CNN
	1    1350 3200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5C155E35
P 1200 3500
F 0 "#PWR09" H 1200 3250 50  0001 C CNN
F 1 "GND" H 1205 3327 50  0000 C CNN
F 2 "" H 1200 3500 50  0001 C CNN
F 3 "" H 1200 3500 50  0001 C CNN
	1    1200 3500
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U3
U 1 1 5C155E3B
P 1700 4700
F 0 "U3" H 1700 5025 50  0000 C CNN
F 1 "74HC00" H 1700 4934 50  0000 C CNN
F 2 "" H 1700 4700 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 1700 4700 50  0001 C CNN
	1    1700 4700
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U3
U 2 1 5C155E41
P 2500 4100
F 0 "U3" H 2500 4425 50  0000 C CNN
F 1 "74HC00" H 2500 4334 50  0000 C CNN
F 2 "" H 2500 4100 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 2500 4100 50  0001 C CNN
	2    2500 4100
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U3
U 3 1 5C155E47
P 3300 3700
F 0 "U3" H 3300 4025 50  0000 C CNN
F 1 "74HC00" H 3300 3934 50  0000 C CNN
F 2 "" H 3300 3700 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 3300 3700 50  0001 C CNN
	3    3300 3700
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U3
U 4 1 5C155E4D
P 2500 3300
F 0 "U3" H 2500 3625 50  0000 C CNN
F 1 "74HC00" H 2500 3534 50  0000 C CNN
F 2 "" H 2500 3300 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 2500 3300 50  0001 C CNN
	4    2500 3300
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U3
U 5 1 5C155E53
P 4900 4000
F 0 "U3" H 5130 4046 50  0000 L CNN
F 1 "74HC00" H 5130 3955 50  0000 L CNN
F 2 "" H 4900 4000 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 4900 4000 50  0001 C CNN
	5    4900 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener D3
U 1 1 5C155E59
P 5600 3950
F 0 "D3" V 5554 4029 50  0000 L CNN
F 1 "3V3" V 5645 4029 50  0000 L CNN
F 2 "" H 5600 3950 50  0001 C CNN
F 3 "~" H 5600 3950 50  0001 C CNN
	1    5600 3950
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5C155E5F
P 5250 3200
F 0 "R8" V 5043 3200 50  0000 C CNN
F 1 "100" V 5134 3200 50  0000 C CNN
F 2 "" V 5180 3200 50  0001 C CNN
F 3 "~" H 5250 3200 50  0001 C CNN
	1    5250 3200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5C155E65
P 4900 4700
F 0 "#PWR015" H 4900 4450 50  0001 C CNN
F 1 "GND" H 4905 4527 50  0000 C CNN
F 2 "" H 4900 4700 50  0001 C CNN
F 3 "" H 4900 4700 50  0001 C CNN
	1    4900 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 4500 4900 4600
Wire Wire Line
	4900 4600 5600 4600
Wire Wire Line
	5600 4600 5600 4100
Connection ~ 4900 4600
Wire Wire Line
	4900 4600 4900 4700
Wire Wire Line
	5600 3800 5600 3400
Wire Wire Line
	5600 3400 4900 3400
Wire Wire Line
	4900 3500 4900 3400
$Comp
L Connector_Generic:Conn_01x03 J11
U 1 1 5C155E73
P 800 4100
F 0 "J11" H 720 3775 50  0000 C CNN
F 1 "CH3_RX" H 720 3866 50  0000 C CNN
F 2 "" H 800 4100 50  0001 C CNN
F 3 "~" H 800 4100 50  0001 C CNN
	1    800  4100
	-1   0    0    1   
$EndComp
Wire Wire Line
	1200 3500 1200 3400
Wire Wire Line
	1200 3400 1000 3400
Wire Wire Line
	1000 3200 1200 3200
$Comp
L Device:R R11
U 1 1 5C155E7C
P 1350 4000
F 0 "R11" V 1143 4000 50  0000 C CNN
F 1 "10k" V 1234 4000 50  0000 C CNN
F 2 "" V 1280 4000 50  0001 C CNN
F 3 "~" H 1350 4000 50  0001 C CNN
	1    1350 4000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5C155E82
P 1200 4300
F 0 "#PWR013" H 1200 4050 50  0001 C CNN
F 1 "GND" H 1205 4127 50  0000 C CNN
F 2 "" H 1200 4300 50  0001 C CNN
F 3 "" H 1200 4300 50  0001 C CNN
	1    1200 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 4300 1200 4200
Wire Wire Line
	1200 4200 1000 4200
Wire Wire Line
	1000 4000 1200 4000
Text GLabel 1200 4600 0    50   Input ~ 0
MODE_OUT
Wire Wire Line
	1200 4600 1300 4600
Wire Wire Line
	1300 4600 1300 4800
Wire Wire Line
	1300 4800 1400 4800
Connection ~ 1300 4600
Wire Wire Line
	1300 4600 1400 4600
Text GLabel 2000 3400 0    50   Input ~ 0
MODE_OUT
Wire Wire Line
	2000 3400 2200 3400
Wire Wire Line
	2200 3200 1500 3200
Wire Wire Line
	2000 4700 2100 4700
Wire Wire Line
	2100 4700 2100 4200
Wire Wire Line
	2100 4200 2200 4200
Wire Wire Line
	2200 4000 1500 4000
Wire Wire Line
	2800 4100 2900 4100
Wire Wire Line
	2900 4100 2900 3800
Wire Wire Line
	2900 3800 3000 3800
Wire Wire Line
	3000 3600 2900 3600
Wire Wire Line
	2900 3600 2900 3300
Wire Wire Line
	2900 3300 2800 3300
Text GLabel 1600 4100 2    50   Input ~ 0
UB3
Wire Wire Line
	1600 4100 1000 4100
Text GLabel 4900 3200 0    50   Input ~ 0
UB3
$Comp
L Connector_Generic:Conn_01x03 J9
U 1 1 5C155EA1
P 4300 3600
F 0 "J9" H 4220 3275 50  0000 C CNN
F 1 "CH3_OUT" H 4220 3366 50  0000 C CNN
F 2 "" H 4300 3600 50  0001 C CNN
F 3 "~" H 4300 3600 50  0001 C CNN
	1    4300 3600
	1    0    0    -1  
$EndComp
Text GLabel 3800 3600 0    50   Input ~ 0
UB3
$Comp
L power:GND #PWR011
U 1 1 5C155EA8
P 3900 3900
F 0 "#PWR011" H 3900 3650 50  0001 C CNN
F 1 "GND" H 3905 3727 50  0000 C CNN
F 2 "" H 3900 3900 50  0001 C CNN
F 3 "" H 3900 3900 50  0001 C CNN
	1    3900 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3900 3900 3500
Wire Wire Line
	3900 3500 4100 3500
Wire Wire Line
	3800 3600 4100 3600
Wire Wire Line
	4100 3700 3600 3700
Wire Wire Line
	4900 3200 5100 3200
Wire Wire Line
	5600 3200 5600 3400
Connection ~ 5600 3400
Wire Wire Line
	5400 3200 5600 3200
NoConn ~ 1000 3300
$Comp
L Connector_Generic:Conn_01x03 J8
U 1 1 5C155EB7
P 6100 3300
F 0 "J8" H 6020 2975 50  0000 C CNN
F 1 "CH4_IN" H 6020 3066 50  0000 C CNN
F 2 "" H 6100 3300 50  0001 C CNN
F 3 "~" H 6100 3300 50  0001 C CNN
	1    6100 3300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R9
U 1 1 5C155EBD
P 6650 3200
F 0 "R9" V 6443 3200 50  0000 C CNN
F 1 "10k" V 6534 3200 50  0000 C CNN
F 2 "" V 6580 3200 50  0001 C CNN
F 3 "~" H 6650 3200 50  0001 C CNN
	1    6650 3200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5C155EC3
P 6500 3500
F 0 "#PWR010" H 6500 3250 50  0001 C CNN
F 1 "GND" H 6505 3327 50  0000 C CNN
F 2 "" H 6500 3500 50  0001 C CNN
F 3 "" H 6500 3500 50  0001 C CNN
	1    6500 3500
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U4
U 1 1 5C155EC9
P 7000 4700
F 0 "U4" H 7000 5025 50  0000 C CNN
F 1 "74HC00" H 7000 4934 50  0000 C CNN
F 2 "" H 7000 4700 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 7000 4700 50  0001 C CNN
	1    7000 4700
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U4
U 2 1 5C155ECF
P 7800 4100
F 0 "U4" H 7800 4425 50  0000 C CNN
F 1 "74HC00" H 7800 4334 50  0000 C CNN
F 2 "" H 7800 4100 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 7800 4100 50  0001 C CNN
	2    7800 4100
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U4
U 3 1 5C155ED5
P 8600 3700
F 0 "U4" H 8600 4025 50  0000 C CNN
F 1 "74HC00" H 8600 3934 50  0000 C CNN
F 2 "" H 8600 3700 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 8600 3700 50  0001 C CNN
	3    8600 3700
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U4
U 4 1 5C155EDB
P 7800 3300
F 0 "U4" H 7800 3625 50  0000 C CNN
F 1 "74HC00" H 7800 3534 50  0000 C CNN
F 2 "" H 7800 3300 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 7800 3300 50  0001 C CNN
	4    7800 3300
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC00 U4
U 5 1 5C155EE1
P 10200 4000
F 0 "U4" H 10430 4046 50  0000 L CNN
F 1 "74HC00" H 10430 3955 50  0000 L CNN
F 2 "" H 10200 4000 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hc00" H 10200 4000 50  0001 C CNN
	5    10200 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener D4
U 1 1 5C155EE7
P 10900 3950
F 0 "D4" V 10854 4029 50  0000 L CNN
F 1 "3V3" V 10945 4029 50  0000 L CNN
F 2 "" H 10900 3950 50  0001 C CNN
F 3 "~" H 10900 3950 50  0001 C CNN
	1    10900 3950
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 5C155EED
P 10550 3200
F 0 "R10" V 10343 3200 50  0000 C CNN
F 1 "100" V 10434 3200 50  0000 C CNN
F 2 "" V 10480 3200 50  0001 C CNN
F 3 "~" H 10550 3200 50  0001 C CNN
	1    10550 3200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5C155EF3
P 10200 4700
F 0 "#PWR016" H 10200 4450 50  0001 C CNN
F 1 "GND" H 10205 4527 50  0000 C CNN
F 2 "" H 10200 4700 50  0001 C CNN
F 3 "" H 10200 4700 50  0001 C CNN
	1    10200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 4500 10200 4600
Wire Wire Line
	10200 4600 10900 4600
Wire Wire Line
	10900 4600 10900 4100
Connection ~ 10200 4600
Wire Wire Line
	10200 4600 10200 4700
Wire Wire Line
	10900 3800 10900 3400
Wire Wire Line
	10900 3400 10200 3400
Wire Wire Line
	10200 3500 10200 3400
$Comp
L Connector_Generic:Conn_01x03 J12
U 1 1 5C155F01
P 6100 4100
F 0 "J12" H 6020 3775 50  0000 C CNN
F 1 "CH4_RX" H 6020 3866 50  0000 C CNN
F 2 "" H 6100 4100 50  0001 C CNN
F 3 "~" H 6100 4100 50  0001 C CNN
	1    6100 4100
	-1   0    0    1   
$EndComp
Wire Wire Line
	6500 3500 6500 3400
Wire Wire Line
	6500 3400 6300 3400
Wire Wire Line
	6300 3200 6500 3200
$Comp
L Device:R R12
U 1 1 5C155F0A
P 6650 4000
F 0 "R12" V 6443 4000 50  0000 C CNN
F 1 "10k" V 6534 4000 50  0000 C CNN
F 2 "" V 6580 4000 50  0001 C CNN
F 3 "~" H 6650 4000 50  0001 C CNN
	1    6650 4000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5C155F10
P 6500 4300
F 0 "#PWR014" H 6500 4050 50  0001 C CNN
F 1 "GND" H 6505 4127 50  0000 C CNN
F 2 "" H 6500 4300 50  0001 C CNN
F 3 "" H 6500 4300 50  0001 C CNN
	1    6500 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 4300 6500 4200
Wire Wire Line
	6500 4200 6300 4200
Wire Wire Line
	6300 4000 6500 4000
Text GLabel 6500 4600 0    50   Input ~ 0
MODE_OUT
Wire Wire Line
	6500 4600 6600 4600
Wire Wire Line
	6600 4600 6600 4800
Wire Wire Line
	6600 4800 6700 4800
Connection ~ 6600 4600
Wire Wire Line
	6600 4600 6700 4600
Text GLabel 7300 3400 0    50   Input ~ 0
MODE_OUT
Wire Wire Line
	7300 3400 7500 3400
Wire Wire Line
	7500 3200 6800 3200
Wire Wire Line
	7300 4700 7400 4700
Wire Wire Line
	7400 4700 7400 4200
Wire Wire Line
	7400 4200 7500 4200
Wire Wire Line
	7500 4000 6800 4000
Wire Wire Line
	8100 4100 8200 4100
Wire Wire Line
	8200 4100 8200 3800
Wire Wire Line
	8200 3800 8300 3800
Wire Wire Line
	8300 3600 8200 3600
Wire Wire Line
	8200 3600 8200 3300
Wire Wire Line
	8200 3300 8100 3300
Text GLabel 6900 4100 2    50   Input ~ 0
UB4
Wire Wire Line
	6900 4100 6300 4100
Text GLabel 10200 3200 0    50   Input ~ 0
UB4
$Comp
L Connector_Generic:Conn_01x03 J10
U 1 1 5C155F2F
P 9600 3600
F 0 "J10" H 9520 3275 50  0000 C CNN
F 1 "CH4_OUT" H 9520 3366 50  0000 C CNN
F 2 "" H 9600 3600 50  0001 C CNN
F 3 "~" H 9600 3600 50  0001 C CNN
	1    9600 3600
	1    0    0    -1  
$EndComp
Text GLabel 9100 3600 0    50   Input ~ 0
UB4
$Comp
L power:GND #PWR012
U 1 1 5C155F36
P 9200 3900
F 0 "#PWR012" H 9200 3650 50  0001 C CNN
F 1 "GND" H 9205 3727 50  0000 C CNN
F 2 "" H 9200 3900 50  0001 C CNN
F 3 "" H 9200 3900 50  0001 C CNN
	1    9200 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 3900 9200 3500
Wire Wire Line
	9200 3500 9400 3500
Wire Wire Line
	9100 3600 9400 3600
Wire Wire Line
	9400 3700 8900 3700
Wire Wire Line
	10200 3200 10400 3200
Wire Wire Line
	10900 3200 10900 3400
Connection ~ 10900 3400
Wire Wire Line
	10700 3200 10900 3200
NoConn ~ 6300 3300
Wire Notes Line
	5900 600  5900 5000
Wire Notes Line
	600  2800 11150 2800
Wire Notes Line
	600  600  11150 600 
Wire Notes Line
	11150 600  11150 5000
Wire Notes Line
	600  600  600  5000
Wire Notes Line
	600  5000 11150 5000
$Comp
L Device:R R14
U 1 1 5C1D5686
P 6400 6350
F 0 "R14" V 6193 6350 50  0000 C CNN
F 1 "10k" V 6284 6350 50  0000 C CNN
F 2 "" V 6330 6350 50  0001 C CNN
F 3 "~" H 6400 6350 50  0001 C CNN
	1    6400 6350
	-1   0    0    1   
$EndComp
Wire Wire Line
	6400 6200 6400 6000
Connection ~ 6400 6000
Wire Wire Line
	6400 6000 5700 6000
$Comp
L power:GND #PWR026
U 1 1 5C1F4D27
P 6400 7000
F 0 "#PWR026" H 6400 6750 50  0001 C CNN
F 1 "GND" H 6405 6827 50  0000 C CNN
F 2 "" H 6400 7000 50  0001 C CNN
F 3 "" H 6400 7000 50  0001 C CNN
	1    6400 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 6500 6400 7000
Text Notes 2700 2700 0    197  ~ 0
Channel 1
Text Notes 2700 4900 0    197  ~ 0
Channel 3
Text Notes 8000 4900 0    197  ~ 0
Channel 4
Text Notes 8000 2700 0    197  ~ 0
Channel 2
$EndSCHEMATC
