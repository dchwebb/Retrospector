EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Retrospector"
Date ""
Rev "2"
Comp "Mountjoy Modular"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:PWR_FLAG #FLG05
U 1 1 5C368914
P 2250 4700
F 0 "#FLG05" H 2250 4775 50  0001 C CNN
F 1 "PWR_FLAG" H 2250 4874 50  0000 C CNN
F 2 "" H 2250 4700 50  0001 C CNN
F 3 "~" H 2250 4700 50  0001 C CNN
	1    2250 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR089
U 1 1 5C368939
P 2250 4750
F 0 "#PWR089" H 2250 4500 50  0001 C CNN
F 1 "GND" H 2250 4600 50  0000 C CNN
F 2 "" H 2250 4750 50  0001 C CNN
F 3 "" H 2250 4750 50  0001 C CNN
	1    2250 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 4700 2250 4750
$Comp
L Device:CP_Small C41
U 1 1 5C7EBE36
P 4450 1550
F 0 "C41" H 4650 1550 50  0000 C CNN
F 1 "22uF" H 4650 1450 50  0000 C CNN
F 2 "Capacitors_SMD:CP_Elec_4x5.8" H 4488 1400 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/1809131544_PANASONIC-EEEFT1V220AR_C178597.pdf" H 4450 1550 50  0001 C CNN
F 4 "C178597" H 4450 1550 50  0001 C CNN "Part_Number"
F 5 "90" H 4450 1550 50  0001 C CNN "Rotation"
	1    4450 1550
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N5819 D3
U 1 1 5C90987E
P 2150 1300
F 0 "D3" H 2150 1084 50  0000 C CNN
F 1 "1N5819" H 2150 1175 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 2150 1125 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/Changjiang-Electronics-Tech-CJ-B5819W_C8598.pdf" H 2150 1300 50  0001 C CNN
F 4 "C8598" H 2150 1300 50  0001 C CNN "Part_Number"
	1    2150 1300
	-1   0    0    1   
$EndComp
Wire Wire Line
	2300 1300 2350 1300
Wire Wire Line
	2350 1300 2350 1100
Wire Wire Line
	1950 1400 1450 1400
Wire Wire Line
	1950 1500 1450 1500
Wire Wire Line
	1950 1600 1450 1600
$Comp
L power:VCC #PWR076
U 1 1 5C9AB39C
P 2350 1100
F 0 "#PWR076" H 2350 950 50  0001 C CNN
F 1 "VCC" H 2367 1273 50  0000 C CNN
F 2 "" H 2350 1100 50  0001 C CNN
F 3 "" H 2350 1100 50  0001 C CNN
	1    2350 1100
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR071
U 1 1 5C9AB842
P 1400 1700
F 0 "#PWR071" H 1400 1800 50  0001 C CNN
F 1 "-12V" V 1400 1950 50  0000 C CNN
F 2 "" H 1400 1700 50  0001 C CNN
F 3 "" H 1400 1700 50  0001 C CNN
	1    1400 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR070
U 1 1 5C9ABB1D
P 1400 1300
F 0 "#PWR070" H 1400 1150 50  0001 C CNN
F 1 "+12V" V 1415 1428 50  0000 L CNN
F 2 "" H 1400 1300 50  0001 C CNN
F 3 "" H 1400 1300 50  0001 C CNN
	1    1400 1300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR066
U 1 1 5C9E963D
P 900 1600
F 0 "#PWR066" H 900 1350 50  0001 C CNN
F 1 "GND" H 905 1427 50  0000 C CNN
F 2 "" H 900 1600 50  0001 C CNN
F 3 "" H 900 1600 50  0001 C CNN
	1    900  1600
	-1   0    0    -1  
$EndComp
Text Notes 1250 5250 0    100  ~ 20
Power
$Comp
L power:+12V #PWR088
U 1 1 5D477E58
P 1850 4750
F 0 "#PWR088" H 1850 4600 50  0001 C CNN
F 1 "+12V" H 1750 4900 50  0000 L CNN
F 2 "" H 1850 4750 50  0001 C CNN
F 3 "" H 1850 4750 50  0001 C CNN
	1    1850 4750
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG04
U 1 1 5D477FFD
P 1850 4700
F 0 "#FLG04" H 1850 4775 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 4874 50  0000 C CNN
F 2 "" H 1850 4700 50  0001 C CNN
F 3 "~" H 1850 4700 50  0001 C CNN
	1    1850 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 4700 1850 4750
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5CB20466
P 2650 4700
F 0 "#FLG02" H 2650 4775 50  0001 C CNN
F 1 "PWR_FLAG" H 2650 4874 50  0000 C CNN
F 2 "" H 2650 4700 50  0001 C CNN
F 3 "~" H 2650 4700 50  0001 C CNN
	1    2650 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 4700 2650 4750
$Comp
L power:+3.3VA #PWR084
U 1 1 5CB34D3A
P 2650 4750
F 0 "#PWR084" H 2650 4600 50  0001 C CNN
F 1 "+3.3VA" H 2500 4900 50  0000 L CNN
F 2 "" H 2650 4750 50  0001 C CNN
F 3 "" H 2650 4750 50  0001 C CNN
	1    2650 4750
	-1   0    0    1   
$EndComp
Wire Wire Line
	1450 4700 1450 4750
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5D48BCA2
P 1450 4700
F 0 "#FLG01" H 1450 4775 50  0001 C CNN
F 1 "PWR_FLAG" H 1450 4874 50  0000 C CNN
F 2 "" H 1450 4700 50  0001 C CNN
F 3 "~" H 1450 4700 50  0001 C CNN
	1    1450 4700
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR083
U 1 1 5D48BAFD
P 1450 4750
F 0 "#PWR083" H 1450 4850 50  0001 C CNN
F 1 "-12V" H 1450 4900 50  0000 C CNN
F 2 "" H 1450 4750 50  0001 C CNN
F 3 "" H 1450 4750 50  0001 C CNN
	1    1450 4750
	-1   0    0    1   
$EndComp
Wire Wire Line
	3050 4700 3050 4750
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5CB61F63
P 3050 4700
F 0 "#FLG03" H 3050 4775 50  0001 C CNN
F 1 "PWR_FLAG" H 3050 4874 50  0000 C CNN
F 2 "" H 3050 4700 50  0001 C CNN
F 3 "~" H 3050 4700 50  0001 C CNN
	1    3050 4700
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR085
U 1 1 5CB7826E
P 3050 4750
F 0 "#PWR085" H 3050 4600 50  0001 C CNN
F 1 "VCC" H 3050 4900 50  0000 C CNN
F 2 "" H 3050 4750 50  0001 C CNN
F 3 "" H 3050 4750 50  0001 C CNN
	1    3050 4750
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR073
U 1 1 5E9CB32E
P 3950 1800
F 0 "#PWR073" H 3950 1550 50  0001 C CNN
F 1 "GND" H 4050 1650 50  0000 R CNN
F 2 "" H 3950 1800 50  0001 C CNN
F 3 "" H 3950 1800 50  0001 C CNN
	1    3950 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1400 4450 1450
Connection ~ 4450 1400
$Comp
L Device:L L1
U 1 1 5E9BEF54
P 4750 1400
F 0 "L1" V 4572 1400 50  0000 C CNN
F 1 "L" V 4663 1400 50  0000 C CNN
F 2 "Inductors_SMD:L_0805_HandSoldering" H 4750 1400 50  0001 C CNN
F 3 "~" H 4750 1400 50  0001 C CNN
F 4 "C1046" H 4750 1400 50  0001 C CNN "Part_Number"
	1    4750 1400
	0    1    1    0   
$EndComp
$Comp
L power:+3.3VA #PWR082
U 1 1 5E9BEF5F
P 4950 1300
F 0 "#PWR082" H 4950 1150 50  0001 C CNN
F 1 "+3.3VA" H 4850 1450 50  0000 L CNN
F 2 "" H 4950 1300 50  0001 C CNN
F 3 "" H 4950 1300 50  0001 C CNN
	1    4950 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 1300 4950 1400
Wire Wire Line
	4950 1400 4900 1400
Wire Wire Line
	4450 1650 4450 1750
Wire Wire Line
	4450 1400 4600 1400
Connection ~ 3950 1750
Wire Wire Line
	3950 1750 3950 1800
Wire Wire Line
	3950 1750 4450 1750
$Comp
L Eurorack_Header:Eurorack_10_pin_power J2
U 1 1 5F72E001
P 1750 1500
F 0 "J2" H 1950 1000 50  0000 C CNN
F 1 "Eurorack_10_pin_power" H 2000 1100 50  0000 C CNN
F 2 "Custom_Footprints:Eurorack_10_pin_header" H 1750 1500 50  0001 C CNN
F 3 "" H 1750 1500 50  0001 C CNN
	1    1750 1500
	-1   0    0    1   
$EndComp
Wire Wire Line
	1400 1700 1450 1700
Wire Wire Line
	1400 1300 1450 1300
Wire Wire Line
	1950 1300 2000 1300
Connection ~ 1450 1500
Wire Wire Line
	1450 1400 1450 1500
Wire Wire Line
	1450 1500 1450 1600
Connection ~ 1450 1400
Connection ~ 1450 1600
Wire Wire Line
	1450 1300 1950 1300
Connection ~ 1450 1300
Connection ~ 1950 1300
Wire Wire Line
	1450 1700 1950 1700
Connection ~ 1450 1700
Wire Wire Line
	900  1600 900  1500
$Comp
L Connector:TestPoint TP5
U 1 1 5FBC912F
P 5550 4950
F 0 "TP5" V 5550 5200 50  0000 C CNN
F 1 "TestPoint" V 5550 5500 50  0000 C CNN
F 2 "Custom_Footprints:1.3mm_Test_Point" H 5750 4950 50  0001 C CNN
F 3 "~" H 5750 4950 50  0001 C CNN
	1    5550 4950
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR018
U 1 1 5FBC9909
P 5550 4950
F 0 "#PWR018" H 5550 4800 50  0001 C CNN
F 1 "VCC" V 5550 5150 50  0000 C CNN
F 2 "" H 5550 4950 50  0001 C CNN
F 3 "" H 5550 4950 50  0001 C CNN
	1    5550 4950
	0    1    1    0   
$EndComp
Text Notes 5000 3850 0    100  ~ 20
Test Points
Wire Wire Line
	3950 1700 3950 1750
$Comp
L power:+3.3VA #PWR013
U 1 1 5FBC8BDD
P 5550 4750
F 0 "#PWR013" H 5550 4600 50  0001 C CNN
F 1 "+3.3VA" V 5550 4850 50  0000 L CNN
F 2 "" H 5550 4750 50  0001 C CNN
F 3 "" H 5550 4750 50  0001 C CNN
	1    5550 4750
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 5FBC8AAF
P 5550 4750
F 0 "TP3" V 5550 5000 50  0000 C CNN
F 1 "TestPoint" V 5550 5300 50  0000 C CNN
F 2 "Custom_Footprints:1.3mm_Test_Point" H 5750 4750 50  0001 C CNN
F 3 "~" H 5750 4750 50  0001 C CNN
	1    5550 4750
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 5FBC8425
P 5550 4550
F 0 "#PWR011" H 5550 4400 50  0001 C CNN
F 1 "+3.3V" V 5550 4650 50  0000 L CNN
F 2 "" H 5550 4550 50  0001 C CNN
F 3 "" H 5550 4550 50  0001 C CNN
	1    5550 4550
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 5FBC80EE
P 5550 4550
F 0 "TP2" V 5550 4800 50  0000 C CNN
F 1 "TestPoint" V 5550 5100 50  0000 C CNN
F 2 "Custom_Footprints:1.3mm_Test_Point" H 5750 4550 50  0001 C CNN
F 3 "~" H 5750 4550 50  0001 C CNN
	1    5550 4550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5FBC7DF8
P 5550 4350
F 0 "#PWR010" H 5550 4100 50  0001 C CNN
F 1 "GND" V 5555 4222 50  0000 R CNN
F 2 "" H 5550 4350 50  0001 C CNN
F 3 "" H 5550 4350 50  0001 C CNN
	1    5550 4350
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint TP1
U 1 1 5FBC76F3
P 5550 4350
F 0 "TP1" V 5550 4600 50  0000 C CNN
F 1 "TestPoint" V 5550 4900 50  0000 C CNN
F 2 "Custom_Footprints:1.3mm_Test_Point" H 5750 4350 50  0001 C CNN
F 3 "~" H 5750 4350 50  0001 C CNN
	1    5550 4350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1450 1500 900  1500
$Comp
L TPS5410DR:TPS5410D U1
U 1 1 63B9AA59
P 2200 3150
F 0 "U1" H 2200 3617 50  0000 C CNN
F 1 "TPS5410D" H 2200 3526 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2150 3600 50  0001 C CNN
F 3 "https://www.ti.com/lit/gpn/tps5410" H 2200 3150 50  0001 C CNN
	1    2200 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 63B9D019
P 2800 3050
F 0 "C3" H 2650 3050 50  0000 C CNN
F 1 "10nF" H 2800 2900 50  0000 C CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2800 3050 50  0001 C CNN
F 3 "~" H 2800 3050 50  0001 C CNN
F 4 "C57112" H 2800 3050 50  0001 C CNN "Part_Number"
	1    2800 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 2950 2800 2950
Wire Wire Line
	2600 3150 2800 3150
NoConn ~ 1800 3150
$Comp
L power:GND #PWR03
U 1 1 63BB87EE
P 2200 3850
F 0 "#PWR03" H 2200 3600 50  0001 C CNN
F 1 "GND" H 2300 3650 50  0000 R CNN
F 2 "" H 2200 3850 50  0001 C CNN
F 3 "" H 2200 3850 50  0001 C CNN
	1    2200 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 3550 2200 3750
Wire Wire Line
	1800 2950 1550 2950
Wire Wire Line
	1550 2950 1550 3200
Wire Wire Line
	1550 3400 1550 3750
Wire Wire Line
	1550 3750 2200 3750
Connection ~ 2200 3750
$Comp
L power:VCC #PWR01
U 1 1 63BCE4C9
P 1450 2950
F 0 "#PWR01" H 1450 2800 50  0001 C CNN
F 1 "VCC" V 1468 3077 50  0000 L CNN
F 2 "" H 1450 2950 50  0001 C CNN
F 3 "" H 1450 2950 50  0001 C CNN
	1    1450 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1450 2950 1550 2950
Connection ~ 1550 2950
Wire Wire Line
	2200 3850 2200 3750
Connection ~ 3550 3750
Wire Wire Line
	4000 3450 4000 3750
Wire Wire Line
	4000 3750 3550 3750
Wire Wire Line
	3100 2950 3200 2950
Wire Wire Line
	3100 3050 3100 2950
$Comp
L Device:D_Schottky_Small D2
U 1 1 63B9D800
P 3100 3150
F 0 "D2" V 3100 3250 50  0000 L CNN
F 1 "1N5819" V 3200 3200 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123" V 3100 3150 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/Changjiang-Electronics-Tech-CJ-B5819W_C8598.pdf" V 3100 3150 50  0001 C CNN
F 4 "C8598" H 3100 3150 50  0001 C CNN "Part_Number"
	1    3100 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	3550 3750 3100 3750
Wire Wire Line
	3550 3650 3550 3750
Connection ~ 3550 2950
Wire Wire Line
	4000 2950 4000 3250
Wire Wire Line
	3550 2950 4000 2950
Wire Wire Line
	3550 2950 3550 3050
Wire Wire Line
	3400 2950 3550 2950
$Comp
L Device:CP_Small C4
U 1 1 63BA5427
P 4000 3350
F 0 "C4" H 4088 3396 50  0000 L CNN
F 1 "150uF" H 4088 3305 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210" H 4000 3350 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/1809131544_PANASONIC-EEEFT1V220AR_C178597.pdf" H 4000 3350 50  0001 C CNN
F 4 "C178597" H 4000 3350 50  0001 C CNN "Part_Number"
F 5 "360" H 4000 3350 50  0001 C CNN "Rotation"
	1    4000 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R2
U 1 1 63BA51B4
P 3550 3550
F 0 "R2" H 3609 3596 50  0000 L CNN
F 1 "5.9k" H 3609 3505 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 3550 3550 50  0001 C CNN
F 3 "~" H 3550 3550 50  0001 C CNN
	1    3550 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 63B9EB15
P 3550 3150
F 0 "R1" H 3609 3196 50  0000 L CNN
F 1 "10k" H 3609 3105 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 3550 3150 50  0001 C CNN
F 3 "~" H 3550 3150 50  0001 C CNN
	1    3550 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:L_Small L3
U 1 1 63B9E041
P 3300 2950
F 0 "L3" V 3485 2950 50  0000 C CNN
F 1 "56uH" V 3394 2950 50  0000 C CNN
F 2 "Custom_Footprints:Inductor_SMD_6x6mm" H 3300 2950 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/1912111437_3L-COILS-SNR6045K-680M_C326343.pdf" H 3300 2950 50  0001 C CNN
F 4 "C326343" V 3300 2950 50  0001 C CNN "Part_Number"
	1    3300 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 3250 3550 3350
Wire Wire Line
	2800 2950 3100 2950
Connection ~ 2800 2950
Connection ~ 3100 2950
Wire Wire Line
	2600 3350 3550 3350
Connection ~ 3550 3350
Wire Wire Line
	3550 3350 3550 3450
Wire Wire Line
	3100 3250 3100 3750
Wire Wire Line
	3100 3750 2200 3750
Connection ~ 3100 3750
$Comp
L power:+3.3V #PWR04
U 1 1 63C1C940
P 4000 2500
F 0 "#PWR04" H 4000 2350 50  0001 C CNN
F 1 "+3.3V" H 4000 2650 50  0000 C CNN
F 2 "" H 4000 2500 50  0001 C CNN
F 3 "" H 4000 2500 50  0001 C CNN
	1    4000 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 2950 4000 2950
Connection ~ 4000 2950
Text Notes 2600 4150 0    50   ~ 0
For 5V output change:\n- Output Capacitor from 150uF to 100uF\n- Lower FB resistor from 5.9k to 3.24k\nNB 5.6k gives output of 3.4V; 6.2k gives 3.2V
Text Notes 800  3650 0    50   ~ 0
Input Capacitor\nCeramic, XR5 1206
$Comp
L Device:C_Small C2
U 1 1 63C237DB
P 1550 3300
F 0 "C2" H 1400 3300 50  0000 C CNN
F 1 "10uF" H 1450 3200 50  0000 C CNN
F 2 "Capacitors_SMD:C_1206" H 1550 3300 50  0001 C CNN
F 3 "~" H 1550 3300 50  0001 C CNN
F 4 "C57112" H 1550 3300 50  0001 C CNN "Part_Number"
	1    1550 3300
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0101
U 1 1 63C3CF98
P 2950 1200
F 0 "#PWR0101" H 2950 1050 50  0001 C CNN
F 1 "VCC" V 2968 1327 50  0000 L CNN
F 2 "" H 2950 1200 50  0001 C CNN
F 3 "" H 2950 1200 50  0001 C CNN
	1    2950 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4250 1400 4450 1400
Text Label 2650 3350 0    50   ~ 0
VSENSE
Text Label 2650 2950 0    50   ~ 0
PH
Text Label 2650 3150 0    50   ~ 0
BOOT
Text Label 4250 1400 0    50   ~ 0
VOUT
$Comp
L LD1117-3.3-4Tab:LD1117-3.3-4Tab U2
U 1 1 63C86E47
P 3950 1400
F 0 "U2" H 3950 1642 50  0000 C CNN
F 1 "LD1117-3.3-4Tab" H 3950 1551 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223-3_TabPin2" H 3950 1600 50  0001 C CNN
F 3 "http://www.diodes.com/datasheets/AP1117.pdf" H 4050 1150 50  0001 C CNN
	1    3950 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 1200 3050 1200
Wire Wire Line
	3350 1200 3450 1200
Wire Wire Line
	3450 1200 3450 1400
Wire Wire Line
	3450 1400 3650 1400
Wire Wire Line
	2900 1400 3050 1400
Wire Wire Line
	3350 1400 3450 1400
Connection ~ 3450 1400
Wire Wire Line
	4450 1150 4450 1400
$Comp
L power:+3.3V #PWR0102
U 1 1 63C98841
P 4450 850
F 0 "#PWR0102" H 4450 700 50  0001 C CNN
F 1 "+3.3V" H 4450 1000 50  0000 C CNN
F 2 "" H 4450 850 50  0001 C CNN
F 3 "" H 4450 850 50  0001 C CNN
	1    4450 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2950 4000 2850
Text Label 4200 2950 0    50   ~ 0
V_Int
Wire Wire Line
	4000 2500 4000 2550
Text Label 2900 1400 2    50   ~ 0
V_Int
$Comp
L Jumper:SolderJumper_2_Open JP4
U 1 1 63CA17BE
P 4450 1000
F 0 "JP4" V 4404 1068 50  0000 L CNN
F 1 "Jumper_Open" V 4495 1068 50  0000 L CNN
F 2 "Resistors_SMD:R_1206" H 4450 1000 50  0001 C CNN
F 3 "~" H 4450 1000 50  0001 C CNN
	1    4450 1000
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Open JP2
U 1 1 63CA47C6
P 3200 1400
F 0 "JP2" H 3200 1550 50  0000 C CNN
F 1 "Jumper_Open" H 3200 1650 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" H 3200 1400 50  0001 C CNN
F 3 "~" H 3200 1400 50  0001 C CNN
	1    3200 1400
	-1   0    0    1   
$EndComp
$Comp
L Jumper:SolderJumper_2_Bridged JP1
U 1 1 63CA67C0
P 3200 1200
F 0 "JP1" H 3200 1405 50  0000 C CNN
F 1 "Jumper_Closed" H 3200 1314 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" H 3200 1200 50  0001 C CNN
F 3 "~" H 3200 1200 50  0001 C CNN
	1    3200 1200
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Bridged JP3
U 1 1 63CA9641
P 4000 2700
F 0 "JP3" V 3954 2768 50  0000 L CNN
F 1 "Jumper_Closed" V 4045 2768 50  0000 L CNN
F 2 "Resistors_SMD:R_1206" H 4000 2700 50  0001 C CNN
F 3 "~" H 4000 2700 50  0001 C CNN
	1    4000 2700
	0    1    1    0   
$EndComp
Text Label 3450 1400 0    50   ~ 0
VIN
$Comp
L power:PWR_FLAG #FLG06
U 1 1 5CB4D472
P 3450 4700
F 0 "#FLG06" H 3450 4775 50  0001 C CNN
F 1 "PWR_FLAG" H 3450 4874 50  0000 C CNN
F 2 "" H 3450 4700 50  0001 C CNN
F 3 "~" H 3450 4700 50  0001 C CNN
	1    3450 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 4700 3450 4750
$Comp
L power:+3.3V #PWR0103
U 1 1 63CF2A51
P 3450 4750
F 0 "#PWR0103" H 3450 4600 50  0001 C CNN
F 1 "+3.3V" V 3450 4850 50  0000 L CNN
F 2 "" H 3450 4750 50  0001 C CNN
F 3 "" H 3450 4750 50  0001 C CNN
	1    3450 4750
	-1   0    0    1   
$EndComp
$EndSCHEMATC
