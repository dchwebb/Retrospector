EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Retrospector Control Board"
Date ""
Rev ""
Comp "Mountjoy Modular"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R_POT Mix_Pot1
U 1 1 5F5F0D07
P 7800 4400
F 0 "Mix_Pot1" H 7731 4446 50  0000 R CNN
F 1 "B10k" H 7731 4355 50  0000 R CNN
F 2 "Custom_Footprints:Alpha_9mm_Potentiometer_Aligned" H 7800 4400 50  0001 C CNN
F 3 "~" H 7800 4400 50  0001 C CNN
	1    7800 4400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR019
U 1 1 5F5F178F
P 7800 4150
F 0 "#PWR019" H 7800 4000 50  0001 C CNN
F 1 "+3.3VA" H 7700 4300 50  0000 L CNN
F 2 "" H 7800 4150 50  0001 C CNN
F 3 "" H 7800 4150 50  0001 C CNN
	1    7800 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 4150 7800 4250
$Comp
L power:GND #PWR022
U 1 1 5F63E50A
P 7800 4650
F 0 "#PWR022" H 7800 4400 50  0001 C CNN
F 1 "GND" H 7805 4477 50  0000 C CNN
F 2 "" H 7800 4650 50  0001 C CNN
F 3 "" H 7800 4650 50  0001 C CNN
	1    7800 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 4550 7800 4650
Wire Wire Line
	7950 4400 8050 4400
Text Label 8050 4400 0    50   ~ 0
MIX_POT
Text Label 950  4400 0    50   ~ 0
EXT_AUDIO_IN_R
Text Label 950  4300 0    50   ~ 0
EXT_AUDIO_IN_L
Text Label 2300 4600 0    50   ~ 0
EXT_AUDIO_OUT_R
Text Label 2300 4500 0    50   ~ 0
EXT_AUDIO_OUT_L
Text Label 2300 4400 0    50   ~ 0
RESET
Text Label 950  4100 0    50   ~ 0
DELAY_CV_L
Text Label 950  4200 0    50   ~ 0
DELAY_CV_R
Text Label 950  5750 0    50   ~ 0
FEEDBACK_CV
Text Label 950  5350 0    50   ~ 0
MIX_CV
Text Label 950  5250 0    50   ~ 0
MIX_POT
Text Label 950  5150 0    50   ~ 0
CLOCK
Text Label 950  5650 0    50   ~ 0
DELAY_POT_L
Text Label 950  4500 0    50   ~ 0
DELAY_POT_R
Text Label 950  5450 0    50   ~ 0
FEEDBACK_POT
Text Label 8050 1000 2    50   ~ 0
MODE1
Text Label 8050 1200 2    50   ~ 0
MODE2
Text Label 2300 5550 0    50   ~ 0
CHORUS
Text Label 2300 4200 0    50   ~ 0
MODE1
Text Label 2300 4300 0    50   ~ 0
MODE2
$Comp
L Connector:Conn_01x08_Female J1
U 1 1 608307FF
P 750 4200
F 0 "J1" H 700 4650 50  0000 C CNN
F 1 "Conn_01x08_Female" H 400 3650 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x08_Pitch2.54mm" H 750 4200 50  0001 C CNN
F 3 "~" H 750 4200 50  0001 C CNN
	1    750  4200
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x08_Female J3
U 1 1 608324C3
P 750 5450
F 0 "J3" H 700 5900 50  0000 C CNN
F 1 "Conn_01x08_Female" H 400 4900 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x08_Pitch2.54mm" H 750 5450 50  0001 C CNN
F 3 "~" H 750 5450 50  0001 C CNN
	1    750  5450
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x08_Female J2
U 1 1 608A2D7D
P 2100 4200
F 0 "J2" H 2050 4650 50  0000 C CNN
F 1 "Conn_01x08_Female" H 1750 3650 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x08_Pitch2.54mm" H 2100 4200 50  0001 C CNN
F 3 "~" H 2100 4200 50  0001 C CNN
	1    2100 4200
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 61960478
P 950 5850
F 0 "#PWR030" H 950 5600 50  0001 C CNN
F 1 "GND" V 950 5650 50  0000 C CNN
F 2 "" H 950 5850 50  0001 C CNN
F 3 "" H 950 5850 50  0001 C CNN
	1    950  5850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR031
U 1 1 61960864
P 2300 5850
F 0 "#PWR031" H 2300 5600 50  0001 C CNN
F 1 "GND" V 2300 5650 50  0000 C CNN
F 2 "" H 2300 5850 50  0001 C CNN
F 3 "" H 2300 5850 50  0001 C CNN
	1    2300 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3VA #PWR015
U 1 1 61963A13
P 950 3900
F 0 "#PWR015" H 950 3750 50  0001 C CNN
F 1 "+3.3VA" V 950 4000 50  0000 L CNN
F 2 "" H 950 3900 50  0001 C CNN
F 3 "" H 950 3900 50  0001 C CNN
	1    950  3900
	0    1    1    0   
$EndComp
Text Notes 1400 6450 0    100  ~ 20
Headers
Text Label 2300 5450 0    50   ~ 0
PINGPONG
Text Label 2300 5350 0    50   ~ 0
LINK_BTN
$Comp
L power:GND #PWR016
U 1 1 60A8280D
P 2300 4100
F 0 "#PWR016" H 2300 3850 50  0001 C CNN
F 1 "GND" V 2300 3900 50  0000 C CNN
F 2 "" H 2300 4100 50  0001 C CNN
F 3 "" H 2300 4100 50  0001 C CNN
	1    2300 4100
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x08_Female J4
U 1 1 60832E97
P 2100 5450
F 0 "J4" H 2050 5900 50  0000 C CNN
F 1 "Conn_01x08_Female" H 1750 4900 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x08_Pitch2.54mm" H 2100 5450 50  0001 C CNN
F 3 "~" H 2100 5450 50  0001 C CNN
	1    2100 5450
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5FA6DF94
P 950 4000
F 0 "#PWR018" H 950 3750 50  0001 C CNN
F 1 "GND" V 950 3800 50  0000 C CNN
F 2 "" H 950 4000 50  0001 C CNN
F 3 "" H 950 4000 50  0001 C CNN
	1    950  4000
	0    -1   -1   0   
$EndComp
Text Label 950  4600 0    50   ~ 0
FILTER_POT
$Comp
L power:+3.3VA #PWR020
U 1 1 5F8725F2
P 10450 4150
F 0 "#PWR020" H 10450 4000 50  0001 C CNN
F 1 "+3.3VA" H 10350 4300 50  0000 L CNN
F 2 "" H 10450 4150 50  0001 C CNN
F 3 "" H 10450 4150 50  0001 C CNN
	1    10450 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 4150 10450 4250
$Comp
L power:GND #PWR023
U 1 1 5F8725FD
P 10450 4650
F 0 "#PWR023" H 10450 4400 50  0001 C CNN
F 1 "GND" H 10455 4477 50  0000 C CNN
F 2 "" H 10450 4650 50  0001 C CNN
F 3 "" H 10450 4650 50  0001 C CNN
	1    10450 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 4550 10450 4650
Wire Wire Line
	10600 4400 10700 4400
$Comp
L Device:R_POT Feedback_Pot1
U 1 1 5F874318
P 9150 4400
F 0 "Feedback_Pot1" H 9081 4446 50  0000 R CNN
F 1 "B10k" H 9081 4355 50  0000 R CNN
F 2 "Custom_Footprints:Alpha_9mm_Potentiometer_Aligned" H 9150 4400 50  0001 C CNN
F 3 "~" H 9150 4400 50  0001 C CNN
	1    9150 4400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR021
U 1 1 5F87431E
P 9150 4150
F 0 "#PWR021" H 9150 4000 50  0001 C CNN
F 1 "+3.3VA" H 9050 4300 50  0000 L CNN
F 2 "" H 9150 4150 50  0001 C CNN
F 3 "" H 9150 4150 50  0001 C CNN
	1    9150 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 4150 9150 4250
$Comp
L power:GND #PWR024
U 1 1 5F874325
P 9150 4650
F 0 "#PWR024" H 9150 4400 50  0001 C CNN
F 1 "GND" H 9155 4477 50  0000 C CNN
F 2 "" H 9150 4650 50  0001 C CNN
F 3 "" H 9150 4650 50  0001 C CNN
	1    9150 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 4550 9150 4650
Wire Wire Line
	9300 4400 9400 4400
$Comp
L Device:R_POT Delay_Pot_L1
U 1 1 5F874E6B
P 8350 5550
F 0 "Delay_Pot_L1" H 8281 5596 50  0000 R CNN
F 1 "B10k" H 8281 5505 50  0000 R CNN
F 2 "Custom_Footprints:Alpha_9mm_Potentiometer_Aligned" H 8350 5550 50  0001 C CNN
F 3 "~" H 8350 5550 50  0001 C CNN
	1    8350 5550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR025
U 1 1 5F874E71
P 8350 5300
F 0 "#PWR025" H 8350 5150 50  0001 C CNN
F 1 "+3.3VA" H 8250 5450 50  0000 L CNN
F 2 "" H 8350 5300 50  0001 C CNN
F 3 "" H 8350 5300 50  0001 C CNN
	1    8350 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 5300 8350 5400
$Comp
L power:GND #PWR028
U 1 1 5F874E78
P 8350 5800
F 0 "#PWR028" H 8350 5550 50  0001 C CNN
F 1 "GND" H 8355 5627 50  0000 C CNN
F 2 "" H 8350 5800 50  0001 C CNN
F 3 "" H 8350 5800 50  0001 C CNN
	1    8350 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 5700 8350 5800
Wire Wire Line
	8500 5550 8600 5550
Text Label 10700 4400 0    50   ~ 0
FILTER_POT
Text Label 9400 4400 0    50   ~ 0
FEEDBACK_POT
Text Label 8600 5550 0    50   ~ 0
DELAY_POT_L
$Comp
L Device:R_POT Delay_Pot_R1
U 1 1 5F875D32
P 10100 5550
F 0 "Delay_Pot_R1" H 10031 5596 50  0000 R CNN
F 1 "B10k" H 10031 5505 50  0000 R CNN
F 2 "Custom_Footprints:Alpha_9mm_Potentiometer_Aligned" H 10100 5550 50  0001 C CNN
F 3 "~" H 10100 5550 50  0001 C CNN
	1    10100 5550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR026
U 1 1 5F875D38
P 10100 5300
F 0 "#PWR026" H 10100 5150 50  0001 C CNN
F 1 "+3.3VA" H 10000 5450 50  0000 L CNN
F 2 "" H 10100 5300 50  0001 C CNN
F 3 "" H 10100 5300 50  0001 C CNN
	1    10100 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 5300 10100 5400
$Comp
L power:GND #PWR029
U 1 1 5F875D3F
P 10100 5800
F 0 "#PWR029" H 10100 5550 50  0001 C CNN
F 1 "GND" H 10105 5627 50  0000 C CNN
F 2 "" H 10100 5800 50  0001 C CNN
F 3 "" H 10100 5800 50  0001 C CNN
	1    10100 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 5700 10100 5800
Wire Wire Line
	10250 5550 10350 5550
$Comp
L thonkiconn:AudioJack2_Ground_Switch J5
U 1 1 5F8775FB
P 1150 1100
F 0 "J5" H 1155 1442 50  0000 C CNN
F 1 "Audio_In_L" H 1155 1351 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 1150 1100 50  0001 C CNN
F 3 "~" H 1150 1100 50  0001 C CNN
	1    1150 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5F877E00
P 1450 1300
F 0 "#PWR03" H 1450 1050 50  0001 C CNN
F 1 "GND" H 1455 1127 50  0000 C CNN
F 2 "" H 1450 1300 50  0001 C CNN
F 3 "" H 1450 1300 50  0001 C CNN
	1    1450 1300
	1    0    0    -1  
$EndComp
Text Label 1550 1100 0    50   ~ 0
EXT_AUDIO_IN_L
Wire Wire Line
	1350 1000 1450 1000
Wire Wire Line
	1450 1000 1450 1200
Wire Wire Line
	1350 1100 1550 1100
Wire Wire Line
	1350 1200 1450 1200
Connection ~ 1450 1200
Wire Wire Line
	1450 1200 1450 1300
$Comp
L power:GND #PWR07
U 1 1 5F879412
P 1450 2200
F 0 "#PWR07" H 1450 1950 50  0001 C CNN
F 1 "GND" H 1455 2027 50  0000 C CNN
F 2 "" H 1450 2200 50  0001 C CNN
F 3 "" H 1450 2200 50  0001 C CNN
	1    1450 2200
	1    0    0    -1  
$EndComp
Text Label 1550 2100 0    50   ~ 0
EXT_AUDIO_IN_L
Wire Wire Line
	1350 1900 1450 1900
Wire Wire Line
	1350 2000 1550 2000
Text Label 1550 2000 0    50   ~ 0
EXT_AUDIO_IN_R
Wire Wire Line
	1450 1900 1450 2200
$Comp
L thonkiconn:AudioJack2_Ground_Switch J11
U 1 1 5F879408
P 1150 2000
F 0 "J11" H 1155 2342 50  0000 C CNN
F 1 "Audio_In_R" H 1155 2251 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 1150 2000 50  0001 C CNN
F 3 "~" H 1150 2000 50  0001 C CNN
	1    1150 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 2100 1550 2100
Text Label 6250 1250 2    50   ~ 0
EXT_AUDIO_OUT_R
Text Label 6250 2400 2    50   ~ 0
EXT_AUDIO_OUT_L
Text Label 3100 2000 0    50   ~ 0
DELAY_CV_R
Text Label 4650 1100 0    50   ~ 0
FEEDBACK_CV
Text Label 4650 2000 0    50   ~ 0
MIX_CV
$Comp
L thonkiconn:AudioJack2_Ground_Switch J6
U 1 1 5F886822
P 2700 1100
F 0 "J6" H 2705 1442 50  0000 C CNN
F 1 "Delay_CV_L" H 2705 1351 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 2700 1100 50  0001 C CNN
F 3 "~" H 2700 1100 50  0001 C CNN
	1    2700 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5F88682C
P 3000 1300
F 0 "#PWR04" H 3000 1050 50  0001 C CNN
F 1 "GND" H 3005 1127 50  0000 C CNN
F 2 "" H 3000 1300 50  0001 C CNN
F 3 "" H 3000 1300 50  0001 C CNN
	1    3000 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 1000 3000 1000
Wire Wire Line
	3000 1000 3000 1200
Wire Wire Line
	2900 1100 3100 1100
Wire Wire Line
	2900 1200 3000 1200
Connection ~ 3000 1200
Wire Wire Line
	3000 1200 3000 1300
$Comp
L thonkiconn:AudioJack2_Ground_Switch J8
U 1 1 5F88887C
P 2700 2000
F 0 "J8" H 2705 2342 50  0000 C CNN
F 1 "Delay_CV_R" H 2705 2251 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 2700 2000 50  0001 C CNN
F 3 "~" H 2700 2000 50  0001 C CNN
	1    2700 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5F888886
P 3000 2200
F 0 "#PWR08" H 3000 1950 50  0001 C CNN
F 1 "GND" H 3005 2027 50  0000 C CNN
F 2 "" H 3000 2200 50  0001 C CNN
F 3 "" H 3000 2200 50  0001 C CNN
	1    3000 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 1900 3000 1900
Wire Wire Line
	3000 1900 3000 2100
Wire Wire Line
	2900 2000 3100 2000
Wire Wire Line
	2900 2100 3000 2100
Connection ~ 3000 2100
Wire Wire Line
	3000 2100 3000 2200
$Comp
L thonkiconn:AudioJack2_Ground_Switch J7
U 1 1 5F88AC3D
P 4250 2000
F 0 "J7" H 4255 2342 50  0000 C CNN
F 1 "Mix_CV" H 4255 2251 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 4250 2000 50  0001 C CNN
F 3 "~" H 4250 2000 50  0001 C CNN
	1    4250 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5F88AC43
P 4550 2200
F 0 "#PWR09" H 4550 1950 50  0001 C CNN
F 1 "GND" H 4555 2027 50  0000 C CNN
F 2 "" H 4550 2200 50  0001 C CNN
F 3 "" H 4550 2200 50  0001 C CNN
	1    4550 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1900 4550 1900
Wire Wire Line
	4550 1900 4550 2100
Wire Wire Line
	4450 2000 4650 2000
Wire Wire Line
	4450 2100 4550 2100
Connection ~ 4550 2100
Wire Wire Line
	4550 2100 4550 2200
$Comp
L thonkiconn:AudioJack2_Ground_Switch J10
U 1 1 5F88B48E
P 4250 1100
F 0 "J10" H 4255 1442 50  0000 C CNN
F 1 "Feedback_CV" H 4255 1351 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 4250 1100 50  0001 C CNN
F 3 "~" H 4250 1100 50  0001 C CNN
	1    4250 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5F88B494
P 4550 1300
F 0 "#PWR05" H 4550 1050 50  0001 C CNN
F 1 "GND" H 4555 1127 50  0000 C CNN
F 2 "" H 4550 1300 50  0001 C CNN
F 3 "" H 4550 1300 50  0001 C CNN
	1    4550 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1000 4550 1000
Wire Wire Line
	4550 1000 4550 1200
Wire Wire Line
	4450 1100 4650 1100
Wire Wire Line
	4450 1200 4550 1200
Connection ~ 4550 1200
Wire Wire Line
	4550 1200 4550 1300
Text Label 3100 1100 0    50   ~ 0
DELAY_CV_L
$Comp
L thonkiconn:AudioJack2_Ground_Switch J13
U 1 1 5F891904
P 6600 2400
F 0 "J13" H 6550 2650 50  0000 C CNN
F 1 "Audio_Out_L" H 6750 2150 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 6600 2400 50  0001 C CNN
F 3 "~" H 6600 2400 50  0001 C CNN
	1    6600 2400
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5F89190A
P 6300 2600
F 0 "#PWR010" H 6300 2350 50  0001 C CNN
F 1 "GND" H 6305 2427 50  0000 C CNN
F 2 "" H 6300 2600 50  0001 C CNN
F 3 "" H 6300 2600 50  0001 C CNN
	1    6300 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 2500 6300 2500
Wire Wire Line
	6400 2400 6250 2400
Wire Wire Line
	6300 2500 6300 2600
NoConn ~ 6400 2300
Wire Wire Line
	6400 1350 6300 1350
Wire Wire Line
	6400 1250 6250 1250
NoConn ~ 6400 1150
$Comp
L Switch:SW_SPDT_MSM DELAY_MODE1
U 1 1 693BF917
P 8400 1100
F 0 "DELAY_MODE1" H 8400 1385 50  0000 C CNN
F 1 "SW_SPDT_MSM" H 8400 1294 50  0000 C CNN
F 2 "Custom_Footprints:SPDT_SubMiniature_Aligned" H 8400 1100 50  0001 C CNN
F 3 "~" H 8400 1100 50  0001 C CNN
	1    8400 1100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8200 1000 8050 1000
Wire Wire Line
	8200 1200 8050 1200
Wire Wire Line
	8700 1100 8600 1100
$Comp
L Switch:SW_Push RESET1
U 1 1 5F8AB203
P 10300 2450
F 0 "RESET1" H 10300 2735 50  0000 C CNN
F 1 "SW_Push" H 10300 2644 50  0000 C CNN
F 2 "Custom_Footprints:SW_PUSH_6mm_aligned" H 10300 2650 50  0001 C CNN
F 3 "~" H 10300 2650 50  0001 C CNN
	1    10300 2450
	1    0    0    -1  
$EndComp
Text Label 10000 2450 2    50   ~ 0
RESET
Wire Wire Line
	6300 1350 6300 1500
$Comp
L power:GND #PWR06
U 1 1 5F8949E3
P 6300 1500
F 0 "#PWR06" H 6300 1250 50  0001 C CNN
F 1 "GND" H 6305 1327 50  0000 C CNN
F 2 "" H 6300 1500 50  0001 C CNN
F 3 "" H 6300 1500 50  0001 C CNN
	1    6300 1500
	1    0    0    -1  
$EndComp
$Comp
L thonkiconn:AudioJack2_Ground_Switch J12
U 1 1 5F8949DD
P 6600 1250
F 0 "J12" H 6550 1500 50  0000 C CNN
F 1 "Audio_Out_R" H 6750 1000 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 6600 1250 50  0001 C CNN
F 3 "~" H 6600 1250 50  0001 C CNN
	1    6600 1250
	-1   0    0    1   
$EndComp
Wire Wire Line
	8700 1100 8700 1200
$Comp
L power:GND #PWR02
U 1 1 5F8B04D7
P 8700 1200
F 0 "#PWR02" H 8700 950 50  0001 C CNN
F 1 "GND" H 8705 1027 50  0000 C CNN
F 2 "" H 8700 1200 50  0001 C CNN
F 3 "" H 8700 1200 50  0001 C CNN
	1    8700 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 2450 10600 2550
$Comp
L power:GND #PWR012
U 1 1 5F8B30C5
P 10600 2550
F 0 "#PWR012" H 10600 2300 50  0001 C CNN
F 1 "GND" H 10605 2377 50  0000 C CNN
F 2 "" H 10600 2550 50  0001 C CNN
F 3 "" H 10600 2550 50  0001 C CNN
	1    10600 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10500 2450 10600 2450
Wire Wire Line
	10000 2450 10100 2450
$Comp
L thonkiconn:AudioJack2_Ground_Switch J9
U 1 1 5F8B66FC
P 1150 2950
F 0 "J9" H 1155 3292 50  0000 C CNN
F 1 "Clock" H 1155 3201 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 1150 2950 50  0001 C CNN
F 3 "~" H 1150 2950 50  0001 C CNN
	1    1150 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5F8B6706
P 1450 3150
F 0 "#PWR014" H 1450 2900 50  0001 C CNN
F 1 "GND" H 1455 2977 50  0000 C CNN
F 2 "" H 1450 3150 50  0001 C CNN
F 3 "" H 1450 3150 50  0001 C CNN
	1    1450 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 2850 1450 2850
Wire Wire Line
	1450 2850 1450 3050
Wire Wire Line
	1350 2950 1550 2950
Wire Wire Line
	1350 3050 1450 3050
Connection ~ 1450 3050
Wire Wire Line
	1450 3050 1450 3150
Text Label 1550 2950 0    50   ~ 0
CLOCK
Wire Notes Line
	500  3550 11200 3550
Text Notes 8700 6450 0    100  ~ 20
Potentiometers
Text Notes 9300 3350 0    100  ~ 20
Switches and Buttons
Text Notes 6400 3350 0    100  ~ 20
Outputs
Text Notes 4350 3350 0    100  ~ 20
CV Inputs
Wire Notes Line
	7200 500  7200 6500
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5F6CF78D
P 1000 7350
F 0 "#FLG0101" H 1000 7425 50  0001 C CNN
F 1 "PWR_FLAG" H 1000 7523 50  0000 C CNN
F 2 "" H 1000 7350 50  0001 C CNN
F 3 "~" H 1000 7350 50  0001 C CNN
	1    1000 7350
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0101
U 1 1 5F6CFD38
P 1000 7350
F 0 "#PWR0101" H 1000 7200 50  0001 C CNN
F 1 "+3.3VA" H 900 7550 50  0000 L CNN
F 2 "" H 1000 7350 50  0001 C CNN
F 3 "" H 1000 7350 50  0001 C CNN
	1    1000 7350
	-1   0    0    1   
$EndComp
Text Label 10350 5550 0    50   ~ 0
DELAY_POT_R
Wire Notes Line
	5350 450  5350 3550
$Comp
L power:GND #PWR0102
U 1 1 6025DB85
P 1550 7350
F 0 "#PWR0102" H 1550 7100 50  0001 C CNN
F 1 "GND" H 1550 7150 50  0000 C CNN
F 2 "" H 1550 7350 50  0001 C CNN
F 3 "" H 1550 7350 50  0001 C CNN
	1    1550 7350
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 6025E675
P 1550 7350
F 0 "#FLG0102" H 1550 7425 50  0001 C CNN
F 1 "PWR_FLAG" H 1550 7523 50  0000 C CNN
F 2 "" H 1550 7350 50  0001 C CNN
F 3 "~" H 1550 7350 50  0001 C CNN
	1    1550 7350
	1    0    0    -1  
$EndComp
$Comp
L TB62781FNG:TB62781FNG U1
U 1 1 6014CECA
P 4800 5000
F 0 "U1" H 4850 5665 50  0000 C CNN
F 1 "TB62781FNG" H 4850 5574 50  0000 C CNN
F 2 "Housings_SSOP:SSOP-20_4.4x6.5mm_Pitch0.65mm" H 4800 5000 50  0001 C CNN
F 3 "" H 4800 5000 50  0001 C CNN
	1    4800 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4100 5900 4100
Wire Wire Line
	5900 4100 5900 4600
Wire Wire Line
	5900 4600 5400 4600
Wire Wire Line
	6100 4300 5950 4300
Wire Wire Line
	5950 4300 5950 4700
Wire Wire Line
	5950 4700 5400 4700
Wire Wire Line
	6100 4500 6000 4500
Wire Wire Line
	6000 4500 6000 4800
Wire Wire Line
	6000 4800 5400 4800
Wire Wire Line
	6100 5000 6050 5000
Wire Wire Line
	6050 5000 6050 4900
Wire Wire Line
	6050 4900 5400 4900
Wire Wire Line
	6100 5200 6000 5200
Wire Wire Line
	6000 5200 6000 5000
Wire Wire Line
	6000 5000 5400 5000
Wire Wire Line
	6100 5400 5950 5400
Wire Wire Line
	5950 5400 5950 5100
Wire Wire Line
	5950 5100 5400 5100
Wire Wire Line
	6100 5900 5900 5900
Wire Wire Line
	5900 5900 5900 5200
Wire Wire Line
	5900 5200 5400 5200
Wire Wire Line
	5400 5300 5850 5300
Wire Wire Line
	5850 5300 5850 6100
Wire Wire Line
	5850 6100 6100 6100
Wire Wire Line
	5400 5400 5800 5400
Wire Wire Line
	5800 5400 5800 6300
Wire Wire Line
	5800 6300 6100 6300
$Comp
L power:+3.3V #PWR017
U 1 1 601748FF
P 2300 4000
F 0 "#PWR017" H 2300 3850 50  0001 C CNN
F 1 "+3.3V" V 2315 4128 50  0000 L CNN
F 2 "" H 2300 4000 50  0001 C CNN
F 3 "" H 2300 4000 50  0001 C CNN
	1    2300 4000
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR035
U 1 1 601757AE
P 5400 5500
F 0 "#PWR035" H 5400 5350 50  0001 C CNN
F 1 "+3.3V" V 5415 5628 50  0000 L CNN
F 2 "" H 5400 5500 50  0001 C CNN
F 3 "" H 5400 5500 50  0001 C CNN
	1    5400 5500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR034
U 1 1 60185E59
P 4200 5600
F 0 "#PWR034" H 4200 5350 50  0001 C CNN
F 1 "GND" H 4205 5427 50  0000 C CNN
F 2 "" H 4200 5600 50  0001 C CNN
F 3 "" H 4200 5600 50  0001 C CNN
	1    4200 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 5600 4200 5500
Text Label 4000 4700 2    50   ~ 0
LED_SPI_DATA
Text Label 4000 4800 2    50   ~ 0
LED_SPI_CLK
Text Label 2300 5150 0    50   ~ 0
LED_SPI_DATA
Text Label 2300 5250 0    50   ~ 0
LED_SPI_CLK
Wire Wire Line
	6500 4300 6550 4300
Wire Wire Line
	6550 4300 6550 5200
Wire Wire Line
	6550 5200 6500 5200
Connection ~ 6550 5200
Wire Wire Line
	6550 5200 6550 6100
Wire Wire Line
	6550 6100 6500 6100
Wire Wire Line
	4000 4700 4300 4700
Wire Wire Line
	4000 4800 4300 4800
$Comp
L Device:R_Small R1
U 1 1 601AE819
P 4150 5200
F 0 "R1" V 4100 5050 50  0000 C CNN
F 1 "3k3" V 4150 5200 30  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 4150 5200 50  0001 C CNN
F 3 "~" H 4150 5200 50  0001 C CNN
	1    4150 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 5200 4300 5200
$Comp
L Device:R_Small R2
U 1 1 601B38BD
P 4150 5300
F 0 "R2" V 4100 5150 50  0000 C CNN
F 1 "3k3" V 4150 5300 30  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 4150 5300 50  0001 C CNN
F 3 "~" H 4150 5300 50  0001 C CNN
	1    4150 5300
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R3
U 1 1 601B3B6F
P 4150 5400
F 0 "R3" V 4100 5250 50  0000 C CNN
F 1 "3k3" V 4150 5400 30  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 4150 5400 50  0001 C CNN
F 3 "~" H 4150 5400 50  0001 C CNN
	1    4150 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 5300 4300 5300
Wire Wire Line
	4250 5400 4300 5400
Wire Wire Line
	4050 5200 3900 5200
Wire Wire Line
	3900 5200 3900 5300
Wire Wire Line
	3900 5500 4200 5500
Wire Wire Line
	4050 5300 3900 5300
Connection ~ 3900 5300
Wire Wire Line
	3900 5300 3900 5400
Wire Wire Line
	4050 5400 3900 5400
Connection ~ 3900 5400
Wire Wire Line
	3900 5400 3900 5500
$Comp
L thonkiconn:AudioJack2_Ground_Switch J14
U 1 1 601D5C89
P 2700 2950
F 0 "J14" H 2705 3292 50  0000 C CNN
F 1 "Filter" H 2705 3201 50  0000 C CNN
F 2 "Custom_Footprints:THONKICONN_hole" H 2700 2950 50  0001 C CNN
F 3 "~" H 2700 2950 50  0001 C CNN
	1    2700 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR033
U 1 1 601D629B
P 3000 3150
F 0 "#PWR033" H 3000 2900 50  0001 C CNN
F 1 "GND" H 3005 2977 50  0000 C CNN
F 2 "" H 3000 3150 50  0001 C CNN
F 3 "" H 3000 3150 50  0001 C CNN
	1    3000 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 2850 3000 2850
Wire Wire Line
	3000 2850 3000 3050
Wire Wire Line
	2900 2950 3100 2950
Wire Wire Line
	2900 3050 3000 3050
Connection ~ 3000 3050
Wire Wire Line
	3000 3050 3000 3150
Text Label 3100 2950 0    50   ~ 0
FILTER_CV
Text Label 2300 5650 0    50   ~ 0
HP_MODE
$Comp
L power:+5V #PWR036
U 1 1 601E5036
P 6800 4950
F 0 "#PWR036" H 6800 4800 50  0001 C CNN
F 1 "+5V" H 6815 5123 50  0000 C CNN
F 2 "" H 6800 4950 50  0001 C CNN
F 3 "" H 6800 4950 50  0001 C CNN
	1    6800 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 4950 6800 5200
Wire Wire Line
	6550 5200 6800 5200
$Comp
L power:+5V #PWR027
U 1 1 601EAD52
P 2300 3900
F 0 "#PWR027" H 2300 3750 50  0001 C CNN
F 1 "+5V" V 2315 4028 50  0000 L CNN
F 2 "" H 2300 3900 50  0001 C CNN
F 3 "" H 2300 3900 50  0001 C CNN
	1    2300 3900
	0    1    1    0   
$EndComp
Text Notes 4250 6450 0    100  ~ 20
LED Controller
$Comp
L power:+3.3V #PWR0103
U 1 1 6015D1C1
P 4150 4900
F 0 "#PWR0103" H 4150 4750 50  0001 C CNN
F 1 "+3.3V" V 4165 5028 50  0000 L CNN
F 2 "" H 4150 4900 50  0001 C CNN
F 3 "" H 4150 4900 50  0001 C CNN
	1    4150 4900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4150 4900 4300 4900
Text Notes 3400 5000 0    50   ~ 0
Address 3
Text Label 950  5550 0    50   ~ 0
FILTER_CV
Text Label 2300 5750 0    50   ~ 0
LP_MODE
$Comp
L Switch:SW_Push LOCK1
U 1 1 603B0AD6
P 10350 1250
F 0 "LOCK1" H 10350 1535 50  0000 C CNN
F 1 "SW_Push" H 10350 1444 50  0000 C CNN
F 2 "Custom_Footprints:SW_PUSH_6mm_aligned" H 10350 1450 50  0001 C CNN
F 3 "~" H 10350 1450 50  0001 C CNN
	1    10350 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 1250 10650 1350
$Comp
L power:GND #PWR0106
U 1 1 603B1146
P 10650 1350
F 0 "#PWR0106" H 10650 1100 50  0001 C CNN
F 1 "GND" H 10655 1177 50  0000 C CNN
F 2 "" H 10650 1350 50  0001 C CNN
F 3 "" H 10650 1350 50  0001 C CNN
	1    10650 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 1250 10650 1250
Wire Wire Line
	10050 1250 10150 1250
Text Label 10050 1250 2    50   ~ 0
LINK_BTN
$Comp
L Switch:SW_SPDT_MSM WIDTH_MODE1
U 1 1 603BBA9F
P 8400 2000
F 0 "WIDTH_MODE1" H 8400 2285 50  0000 C CNN
F 1 "SW_SPDT_MSM" H 8400 2194 50  0000 C CNN
F 2 "Custom_Footprints:SPDT_SubMiniature_Aligned" H 8400 2000 50  0001 C CNN
F 3 "~" H 8400 2000 50  0001 C CNN
	1    8400 2000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8200 1900 8050 1900
Wire Wire Line
	8200 2100 8050 2100
Wire Wire Line
	8700 2000 8600 2000
Wire Wire Line
	8700 2000 8700 2100
$Comp
L power:GND #PWR0107
U 1 1 603BBAAD
P 8700 2100
F 0 "#PWR0107" H 8700 1850 50  0001 C CNN
F 1 "GND" H 8705 1927 50  0000 C CNN
F 2 "" H 8700 2100 50  0001 C CNN
F 3 "" H 8700 2100 50  0001 C CNN
	1    8700 2100
	1    0    0    -1  
$EndComp
Text Label 8050 1900 2    50   ~ 0
CHORUS
Text Label 8050 2100 2    50   ~ 0
PINGPONG
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 603D657A
P 2050 7350
F 0 "#FLG0103" H 2050 7425 50  0001 C CNN
F 1 "PWR_FLAG" H 2050 7523 50  0000 C CNN
F 2 "" H 2050 7350 50  0001 C CNN
F 3 "~" H 2050 7350 50  0001 C CNN
	1    2050 7350
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 603D674B
P 2050 7350
F 0 "#PWR0108" H 2050 7200 50  0001 C CNN
F 1 "+3.3V" H 1950 7550 50  0000 L CNN
F 2 "" H 2050 7350 50  0001 C CNN
F 3 "" H 2050 7350 50  0001 C CNN
	1    2050 7350
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0109
U 1 1 603D6C4D
P 2550 7350
F 0 "#PWR0109" H 2550 7200 50  0001 C CNN
F 1 "+5V" H 2500 7550 50  0000 L CNN
F 2 "" H 2550 7350 50  0001 C CNN
F 3 "" H 2550 7350 50  0001 C CNN
	1    2550 7350
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 603D76E6
P 2550 7350
F 0 "#FLG0104" H 2550 7425 50  0001 C CNN
F 1 "PWR_FLAG" H 2550 7523 50  0000 C CNN
F 2 "" H 2550 7350 50  0001 C CNN
F 3 "~" H 2550 7350 50  0001 C CNN
	1    2550 7350
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPDT_MSM FILTER_MODE1
U 1 1 603E6A1C
P 8400 2900
F 0 "FILTER_MODE1" H 8400 3185 50  0000 C CNN
F 1 "SW_SPDT_MSM" H 8400 3094 50  0000 C CNN
F 2 "Custom_Footprints:SPDT_SubMiniature_Aligned" H 8400 2900 50  0001 C CNN
F 3 "~" H 8400 2900 50  0001 C CNN
	1    8400 2900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8200 2800 8050 2800
Wire Wire Line
	8200 3000 8050 3000
Wire Wire Line
	8700 2900 8600 2900
Wire Wire Line
	8700 2900 8700 3000
$Comp
L power:GND #PWR0110
U 1 1 603E6A2A
P 8700 3000
F 0 "#PWR0110" H 8700 2750 50  0001 C CNN
F 1 "GND" H 8705 2827 50  0000 C CNN
F 2 "" H 8700 3000 50  0001 C CNN
F 3 "" H 8700 3000 50  0001 C CNN
	1    8700 3000
	1    0    0    -1  
$EndComp
Text Label 8050 3000 2    50   ~ 0
HP_MODE
Text Label 8050 2800 2    50   ~ 0
LP_MODE
$Comp
L Device:LED_RGBA D1
U 1 1 603E9B4B
P 6300 4300
F 0 "D1" H 6300 4797 50  0000 C CNN
F 1 "LED_RGBA" H 6300 4706 50  0000 C CNN
F 2 "Custom_Footprints:LED_RGB_PLCC4" H 6300 4250 50  0001 C CNN
F 3 "~" H 6300 4250 50  0001 C CNN
	1    6300 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_RGBA D2
U 1 1 603EB66F
P 6300 5200
F 0 "D2" H 6300 5697 50  0000 C CNN
F 1 "LED_RGBA" H 6300 5606 50  0000 C CNN
F 2 "Custom_Footprints:LED_RGB_PLCC4" H 6300 5150 50  0001 C CNN
F 3 "~" H 6300 5150 50  0001 C CNN
	1    6300 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_RGBA D3
U 1 1 603EC23C
P 6300 6100
F 0 "D3" H 6300 6597 50  0000 C CNN
F 1 "LED_RGBA" H 6300 6506 50  0000 C CNN
F 2 "Custom_Footprints:LED_RGB_PLCC4" H 6300 6050 50  0001 C CNN
F 3 "~" H 6300 6050 50  0001 C CNN
	1    6300 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C1
U 1 1 603EDA8B
P 6800 5300
F 0 "C1" H 6850 5200 50  0000 L CNN
F 1 "47uF" H 6600 5200 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_5x5.8" H 6800 5300 50  0001 C CNN
F 3 "~" H 6800 5300 50  0001 C CNN
	1    6800 5300
	1    0    0    -1  
$EndComp
Connection ~ 6800 5200
$Comp
L power:GND #PWR0111
U 1 1 603EEA73
P 6800 5500
F 0 "#PWR0111" H 6800 5250 50  0001 C CNN
F 1 "GND" H 6805 5327 50  0000 C CNN
F 2 "" H 6800 5500 50  0001 C CNN
F 3 "" H 6800 5500 50  0001 C CNN
	1    6800 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 5500 6800 5400
Wire Notes Line
	3200 7800 3200 3550
$Comp
L Device:R_POT Tone_Pot1
U 1 1 5F8725E8
P 10450 4400
F 0 "Tone_Pot1" H 10381 4446 50  0000 R CNN
F 1 "B10k" H 10381 4355 50  0000 R CNN
F 2 "Custom_Footprints:Alpha_9mm_Potentiometer_Aligned" H 10450 4400 50  0001 C CNN
F 3 "~" H 10450 4400 50  0001 C CNN
	1    10450 4400
	1    0    0    -1  
$EndComp
Wire Notes Line
	500  6900 3200 6900
$Comp
L power:GND #PWR0105
U 1 1 6022D36C
P 4300 4600
F 0 "#PWR0105" H 4300 4350 50  0001 C CNN
F 1 "GND" V 4300 4400 50  0000 C CNN
F 2 "" H 4300 4600 50  0001 C CNN
F 3 "" H 4300 4600 50  0001 C CNN
	1    4300 4600
	0    1    1    0   
$EndComp
Connection ~ 4200 5500
Wire Wire Line
	4200 5500 4300 5500
Wire Wire Line
	4150 5000 4300 5000
$Comp
L power:GND #PWR0104
U 1 1 6015D81B
P 4150 5000
F 0 "#PWR0104" H 4150 4750 50  0001 C CNN
F 1 "GND" V 4150 4800 50  0000 C CNN
F 2 "" H 4150 5000 50  0001 C CNN
F 3 "" H 4150 5000 50  0001 C CNN
	1    4150 5000
	0    1    1    0   
$EndComp
Connection ~ 4300 5000
Wire Wire Line
	4300 5100 4300 5000
$EndSCHEMATC
