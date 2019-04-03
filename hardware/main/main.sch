EESchema Schematic File Version 4
LIBS:main-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R_Small R105
U 1 1 5CA26933
P 7050 4250
F 0 "R105" V 6800 4250 50  0000 C CNN
F 1 "68k" V 6900 4250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7050 4250 50  0001 C CNN
F 3 "~" H 7050 4250 50  0001 C CNN
	1    7050 4250
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C112
U 1 1 5CA26DBA
P 7450 3800
F 0 "C112" V 7450 3550 50  0000 C CNN
F 1 "100n" V 7350 3550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7450 3800 50  0001 C CNN
F 3 "~" H 7450 3800 50  0001 C CNN
	1    7450 3800
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_3_Open JP101
U 1 1 5CA28CFD
P 3000 4050
F 0 "JP101" V 2950 4150 50  0000 L CNN
F 1 "SolderJumper_3_Open" V 3200 3600 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical_SMD_Pin1Left" H 3000 4050 50  0001 C CNN
F 3 "~" H 3000 4050 50  0001 C CNN
	1    3000 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	6100 3550 6100 4050
Wire Wire Line
	6250 4250 6850 4250
Wire Wire Line
	7150 4250 7250 4250
$Comp
L Device:R_Small R106
U 1 1 5CA36108
P 7450 4250
F 0 "R106" V 7700 4250 50  0000 C CNN
F 1 "33k" V 7600 4250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7450 4250 50  0001 C CNN
F 3 "~" H 7450 4250 50  0001 C CNN
	1    7450 4250
	0    1    -1   0   
$EndComp
Wire Wire Line
	7350 4250 7250 4250
Connection ~ 7250 4250
Wire Wire Line
	7250 3800 7350 3800
$Comp
L power:GND #PWR0107
U 1 1 5CA41536
P 7800 4500
F 0 "#PWR0107" H 7800 4250 50  0001 C CNN
F 1 "GND" H 7850 4300 50  0000 C CNN
F 2 "" H 7800 4500 50  0001 C CNN
F 3 "" H 7800 4500 50  0001 C CNN
	1    7800 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 4250 3000 4850
Wire Wire Line
	7250 3800 7250 4250
$Comp
L Connector:TestPoint TP101
U 1 1 5CA66FC6
P 3000 2750
F 0 "TP101" H 2950 3100 50  0000 L CNN
F 1 "TestPoint" H 2850 3000 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 3200 2750 50  0001 C CNN
F 3 "~" H 3200 2750 50  0001 C CNN
	1    3000 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP104
U 1 1 5CA6C390
P 6850 2750
F 0 "TP104" H 6750 3100 50  0000 L CNN
F 1 "TestPoint" H 6750 3000 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 7050 2750 50  0001 C CNN
F 3 "~" H 7050 2750 50  0001 C CNN
	1    6850 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 2750 6850 4250
Connection ~ 6850 4250
Wire Wire Line
	6850 4250 6950 4250
$Comp
L Connector:TestPoint TP105
U 1 1 5CA7056A
P 7250 2750
F 0 "TP105" H 7150 3100 50  0000 L CNN
F 1 "TestPoint" H 7150 3000 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 7450 2750 50  0001 C CNN
F 3 "~" H 7450 2750 50  0001 C CNN
	1    7250 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 2750 7250 3800
Connection ~ 7250 3800
$Comp
L Connector:Conn_01x02_Female J103
U 1 1 5CA83904
P 10000 4250
F 0 "J103" H 9900 3900 50  0000 C CNN
F 1 "Conn_01x02_Female" H 9900 4000 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical_SMD_Pin1Left" H 10000 4250 50  0001 C CNN
F 3 "~" H 10000 4250 50  0001 C CNN
	1    10000 4250
	1    0    0    1   
$EndComp
Wire Wire Line
	9800 4250 9650 4250
Wire Wire Line
	9650 4250 9650 4350
$Comp
L power:GND #PWR0106
U 1 1 5CA8ADDC
P 9650 4350
F 0 "#PWR0106" H 9650 4100 50  0001 C CNN
F 1 "GND" H 9700 4150 50  0000 C CNN
F 2 "" H 9650 4350 50  0001 C CNN
F 3 "" H 9650 4350 50  0001 C CNN
	1    9650 4350
	-1   0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U101
U 5 1 5CAAA3A9
P 1800 6200
F 0 "U101" H 1600 6150 50  0000 R CNN
F 1 "LM324" H 1600 6200 50  0000 R CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 1750 6300 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 1850 6400 50  0001 C CNN
	5    1800 6200
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C102
U 1 1 5CAAC8E2
P 2700 6200
F 0 "C102" H 2600 6150 50  0000 R CNN
F 1 "100n" H 2600 6200 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2700 6200 50  0001 C CNN
F 3 "~" H 2700 6200 50  0001 C CNN
	1    2700 6200
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5CAB2174
P 1700 6650
F 0 "#PWR0102" H 1700 6400 50  0001 C CNN
F 1 "GND" H 1750 6450 50  0000 C CNN
F 2 "" H 1700 6650 50  0001 C CNN
F 3 "" H 1700 6650 50  0001 C CNN
	1    1700 6650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1700 6500 1700 6550
Wire Wire Line
	1700 6550 2200 6550
Wire Wire Line
	2700 6300 2700 6550
Connection ~ 1700 6550
$Comp
L power:+5V #PWR0101
U 1 1 5CAB5C3A
P 1700 5750
F 0 "#PWR0101" H 1700 5600 50  0001 C CNN
F 1 "+5V" H 1750 5950 50  0000 C CNN
F 2 "" H 1700 5750 50  0001 C CNN
F 3 "" H 1700 5750 50  0001 C CNN
	1    1700 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5850 2200 5850
Wire Wire Line
	2700 5850 2700 6100
Connection ~ 1700 5850
Wire Wire Line
	1700 5850 1700 5900
Text Label 9350 4150 0    50   ~ 0
OUTPUT
Text Label 6350 4250 0    50   ~ 0
MIDDLE
$Comp
L Connector:Conn_01x02_Female J101
U 1 1 5CAC8BDC
P 3700 6250
F 0 "J101" H 3600 5900 50  0000 C CNN
F 1 "Conn_01x02_Female" H 3600 6000 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical_SMD_Pin1Left" H 3700 6250 50  0001 C CNN
F 3 "~" H 3700 6250 50  0001 C CNN
	1    3700 6250
	1    0    0    1   
$EndComp
Wire Wire Line
	3500 6250 3500 6550
Wire Wire Line
	3500 6550 2700 6550
Connection ~ 2700 6550
Text Notes 3650 2350 0    50   ~ 0
BLOCO DE PRIMEIRA ORDEM
Text Notes 6900 2250 0    50   ~ 0
BLOCO DE SEGUNDA ORDEM
Wire Notes Line
	5550 2350 3500 2350
Wire Notes Line
	6700 4750 6700 2350
Wire Notes Line
	6700 2350 8650 2350
Wire Notes Line
	8650 2350 8650 4750
Wire Notes Line
	8650 4750 6700 4750
Text Label 2550 4050 0    50   ~ 0
INPUT
$Comp
L power:GND #PWR0105
U 1 1 5CA835BB
P 2450 4250
F 0 "#PWR0105" H 2450 4000 50  0001 C CNN
F 1 "GND" H 2500 4050 50  0000 C CNN
F 2 "" H 2450 4250 50  0001 C CNN
F 3 "" H 2450 4250 50  0001 C CNN
	1    2450 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 4150 2450 4250
Wire Wire Line
	2300 4150 2450 4150
$Comp
L Connector:Conn_01x02_Female J102
U 1 1 5CA7F6A7
P 2100 4050
F 0 "J102" H 1950 4250 50  0000 C CNN
F 1 "Conn_01x02_Female" H 1950 4150 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical_SMD_Pin1Left" H 2100 4050 50  0001 C CNN
F 3 "~" H 2100 4050 50  0001 C CNN
	1    2100 4050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2850 4050 2300 4050
Wire Wire Line
	6100 4450 6100 4850
Wire Wire Line
	3000 4850 6100 4850
Wire Wire Line
	2200 6100 2200 5850
Connection ~ 2200 5850
Wire Wire Line
	2200 5850 2700 5850
Wire Wire Line
	2200 6300 2200 6550
Connection ~ 2200 6550
Connection ~ 8500 4150
Wire Wire Line
	9800 4150 8850 4150
$Comp
L Connector:TestPoint TP106
U 1 1 5CA74222
P 7650 2750
F 0 "TP106" H 7750 3100 50  0000 R CNN
F 1 "TestPoint" H 7750 3000 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 7850 2750 50  0001 C CNN
F 3 "~" H 7850 2750 50  0001 C CNN
	1    7650 2750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7650 4250 7800 4250
Connection ~ 7650 4250
Wire Wire Line
	7550 4250 7650 4250
Wire Wire Line
	7650 2750 7650 4250
Connection ~ 8500 3800
Wire Wire Line
	8500 2750 8500 3800
$Comp
L Connector:TestPoint TP107
U 1 1 5CA72F09
P 8500 2750
F 0 "TP107" H 8400 3100 50  0000 L CNN
F 1 "TestPoint" H 8300 3000 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 8700 2750 50  0001 C CNN
F 3 "~" H 8700 2750 50  0001 C CNN
	1    8500 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 3800 7550 3800
Wire Wire Line
	7800 3800 8500 3800
Connection ~ 7800 3800
Wire Wire Line
	7800 4050 7800 3800
Wire Wire Line
	7850 4050 7800 4050
$Comp
L Device:C_Small C113
U 1 1 5CA40DA4
P 7800 4400
F 0 "C113" H 7900 4450 50  0000 L CNN
F 1 "100n" H 7900 4400 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7800 4400 50  0001 C CNN
F 3 "~" H 7800 4400 50  0001 C CNN
	1    7800 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3800 8500 4150
Wire Wire Line
	8500 4150 8450 4150
Connection ~ 7800 4250
Wire Wire Line
	7850 4250 7800 4250
$Comp
L Amplifier_Operational:LM324 U101
U 2 1 5CA37D44
P 8150 4150
F 0 "U101" H 8150 3750 50  0000 C CNN
F 1 "LM324" H 8150 3850 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 8100 4250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 8200 4350 50  0001 C CNN
	2    8150 4150
	1    0    0    1   
$EndComp
Wire Wire Line
	7800 4250 7800 4300
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5CB0351E
P 2700 5750
F 0 "#FLG0101" H 2700 5825 50  0001 C CNN
F 1 "PWR_FLAG" H 2700 5950 50  0000 C CNN
F 2 "" H 2700 5750 50  0001 C CNN
F 3 "~" H 2700 5750 50  0001 C CNN
	1    2700 5750
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5CB0506F
P 2700 6650
F 0 "#FLG0102" H 2700 6725 50  0001 C CNN
F 1 "PWR_FLAG" H 2700 6850 50  0000 C CNN
F 2 "" H 2700 6650 50  0001 C CNN
F 3 "~" H 2700 6650 50  0001 C CNN
	1    2700 6650
	-1   0    0    1   
$EndComp
Wire Wire Line
	2700 6550 2700 6650
Wire Wire Line
	1700 6550 1700 6650
Wire Wire Line
	1700 5750 1700 5850
$Comp
L Device:C_Small C111
U 1 1 5CB18E41
P 7450 3650
F 0 "C111" V 7500 3900 50  0000 C CNN
F 1 "100n" V 7400 3900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7450 3650 50  0001 C CNN
F 3 "~" H 7450 3650 50  0001 C CNN
	1    7450 3650
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C110
U 1 1 5CB1909F
P 7450 3500
F 0 "C110" V 7500 3250 50  0000 C CNN
F 1 "100n" V 7400 3250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7450 3500 50  0001 C CNN
F 3 "~" H 7450 3500 50  0001 C CNN
	1    7450 3500
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C108
U 1 1 5CB193ED
P 7450 3350
F 0 "C108" V 7500 3600 50  0000 C CNN
F 1 "100n" V 7400 3600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7450 3350 50  0001 C CNN
F 3 "~" H 7450 3350 50  0001 C CNN
	1    7450 3350
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C107
U 1 1 5CB19D5E
P 7450 3200
F 0 "C107" V 7500 2950 50  0000 C CNN
F 1 "100n" V 7400 2950 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7450 3200 50  0001 C CNN
F 3 "~" H 7450 3200 50  0001 C CNN
	1    7450 3200
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C104
U 1 1 5CB1A074
P 7450 3050
F 0 "C104" V 7500 3300 50  0000 C CNN
F 1 "100n" V 7400 3300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7450 3050 50  0001 C CNN
F 3 "~" H 7450 3050 50  0001 C CNN
	1    7450 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	7550 3050 7550 3200
Connection ~ 7550 3800
Connection ~ 7550 3200
Wire Wire Line
	7550 3200 7550 3350
Connection ~ 7550 3350
Wire Wire Line
	7550 3350 7550 3500
Connection ~ 7550 3500
Wire Wire Line
	7550 3500 7550 3650
Connection ~ 7550 3650
Wire Wire Line
	7550 3650 7550 3800
Connection ~ 7350 3800
Connection ~ 7350 3200
Wire Wire Line
	7350 3200 7350 3350
Connection ~ 7350 3350
Wire Wire Line
	7350 3350 7350 3500
Connection ~ 7350 3500
Wire Wire Line
	7350 3500 7350 3650
Connection ~ 7350 3650
Wire Wire Line
	7350 3650 7350 3800
$Comp
L Device:C_Small C103
U 1 1 5CB1DD3F
P 7450 2900
F 0 "C103" V 7550 2650 50  0000 C CNN
F 1 "100n" V 7450 2650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7450 2900 50  0001 C CNN
F 3 "~" H 7450 2900 50  0001 C CNN
	1    7450 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	7550 2900 7550 3050
Connection ~ 7550 3050
Wire Wire Line
	7350 2900 7350 3050
Connection ~ 7350 3050
Wire Wire Line
	7350 3050 7350 3200
$Comp
L Device:R_Small R107
U 1 1 5CB22462
P 7450 4600
F 0 "R107" V 7200 4600 50  0000 C CNN
F 1 "39k" V 7300 4600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7450 4600 50  0001 C CNN
F 3 "~" H 7450 4600 50  0001 C CNN
	1    7450 4600
	0    -1   1    0   
$EndComp
Wire Wire Line
	7550 4600 7650 4600
Wire Wire Line
	7650 4600 7650 4250
Wire Wire Line
	7250 4250 7250 4600
Wire Wire Line
	7250 4600 7350 4600
$Comp
L Device:D_Zener_Small D101
U 1 1 5CBA03D6
P 8850 4400
F 0 "D101" V 8800 4500 50  0000 L CNN
F 1 "D_Zener_Small" V 8850 4500 50  0000 L CNN
F 2 "Diode_SMD:D_SMB-SMC_Universal_Handsoldering" V 8850 4400 50  0001 C CNN
F 3 "~" V 8850 4400 50  0001 C CNN
	1    8850 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	8850 4300 8850 4150
Connection ~ 8850 4150
Wire Wire Line
	8850 4150 8500 4150
$Comp
L power:GND #PWR0108
U 1 1 5CBA3889
P 8850 4650
F 0 "#PWR0108" H 8850 4400 50  0001 C CNN
F 1 "GND" H 8900 4450 50  0000 C CNN
F 2 "" H 8850 4650 50  0001 C CNN
F 3 "" H 8850 4650 50  0001 C CNN
	1    8850 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 4500 8850 4650
$Comp
L Device:CP1_Small C101
U 1 1 5CBBBC42
P 2200 6200
F 0 "C101" H 2300 6250 50  0000 L CNN
F 1 "10u" H 2300 6200 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric_Pad1.42x2.65mm_HandSolder" H 2200 6200 50  0001 C CNN
F 3 "~" H 2200 6200 50  0001 C CNN
	1    2200 6200
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4148W D102
U 1 1 5CBBDCB4
P 3150 5850
F 0 "D102" H 3150 6100 50  0000 C CNN
F 1 "1N4148W" H 3150 6000 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 3150 5675 50  0001 C CNN
F 3 "https://www.vishay.com/docs/85748/1n4148w.pdf" H 3150 5850 50  0001 C CNN
	1    3150 5850
	1    0    0    -1  
$EndComp
Connection ~ 2700 5850
Wire Wire Line
	2700 5750 2700 5850
Wire Wire Line
	3500 6150 3500 5850
Wire Wire Line
	3500 5850 3300 5850
Wire Wire Line
	3000 5850 2700 5850
$Comp
L Connector:TestPoint TP109
U 1 1 5CBCA3DC
P 2200 5400
F 0 "TP109" H 2300 5550 50  0000 L CNN
F 1 "TestPoint" H 2300 5450 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 2400 5400 50  0001 C CNN
F 3 "~" H 2400 5400 50  0001 C CNN
	1    2200 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 5400 2200 5850
$Comp
L Connector:TestPoint TP111
U 1 1 5CBCDB18
P 2200 6950
F 0 "TP111" H 2100 6950 50  0000 R CNN
F 1 "TestPoint" H 2100 7050 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 2400 6950 50  0001 C CNN
F 3 "~" H 2400 6950 50  0001 C CNN
	1    2200 6950
	-1   0    0    1   
$EndComp
Wire Wire Line
	2200 6550 2700 6550
Wire Wire Line
	2200 6950 2200 6550
$Comp
L Connector:TestPoint TP110
U 1 1 5CBDABAA
P 3500 5400
F 0 "TP110" H 3600 5550 50  0000 L CNN
F 1 "TestPoint" H 3600 5450 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 3700 5400 50  0001 C CNN
F 3 "~" H 3700 5400 50  0001 C CNN
	1    3500 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5400 3500 5850
Connection ~ 3500 5850
$Comp
L Jumper:SolderJumper_3_Open JP102
U 1 1 5CA2762A
P 6100 4250
F 0 "JP102" V 6150 4350 50  0000 L CNN
F 1 "SolderJumper_3_Open" V 5850 3850 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical_SMD_Pin1Left" H 6100 4250 50  0001 C CNN
F 3 "~" H 6100 4250 50  0001 C CNN
	1    6100 4250
	0    -1   1    0   
$EndComp
$Comp
L Amplifier_Operational:LM324 U101
U 3 1 5CCE2776
P 7300 5800
F 0 "U101" H 7300 5400 50  0000 C CNN
F 1 "LM324" H 7300 5500 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 7250 5900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 7350 6000 50  0001 C CNN
	3    7300 5800
	1    0    0    1   
$EndComp
Wire Wire Line
	7000 5900 6900 5900
$Comp
L power:GND #PWR0109
U 1 1 5CCF602D
P 6900 6000
F 0 "#PWR0109" H 6900 5750 50  0001 C CNN
F 1 "GND" H 6950 5800 50  0000 C CNN
F 2 "" H 6900 6000 50  0001 C CNN
F 3 "" H 6900 6000 50  0001 C CNN
	1    6900 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 5900 6900 6000
Wire Wire Line
	6900 5500 6900 5700
Wire Wire Line
	6900 5700 7000 5700
Wire Wire Line
	7700 5500 7700 5800
Wire Wire Line
	7700 5800 7600 5800
Wire Wire Line
	6900 5500 7700 5500
$Comp
L Graphic:Logo_Open_Hardware_Small #LOGO101
U 1 1 5CD50677
P 5250 5450
F 0 "#LOGO101" H 5250 5725 50  0001 C CNN
F 1 "Logo_Open_Hardware_Small" H 5250 5225 50  0001 C CNN
F 2 "" H 5250 5450 50  0001 C CNN
F 3 "~" H 5250 5450 50  0001 C CNN
	1    5250 5450
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H104
U 1 1 5CD57CFF
P 2800 1600
F 0 "H104" H 2900 1650 50  0000 L CNN
F 1 "MountingHole" H 2900 1600 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 2800 1600 50  0001 C CNN
F 3 "~" H 2800 1600 50  0001 C CNN
	1    2800 1600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H103
U 1 1 5CD58028
P 2800 1400
F 0 "H103" H 2900 1450 50  0000 L CNN
F 1 "MountingHole" H 2900 1400 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 2800 1400 50  0001 C CNN
F 3 "~" H 2800 1400 50  0001 C CNN
	1    2800 1400
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H102
U 1 1 5CD581D8
P 2800 1200
F 0 "H102" H 2900 1250 50  0000 L CNN
F 1 "MountingHole" H 2900 1200 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 2800 1200 50  0001 C CNN
F 3 "~" H 2800 1200 50  0001 C CNN
	1    2800 1200
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H101
U 1 1 5CD58402
P 2800 1000
F 0 "H101" H 2900 1050 50  0000 L CNN
F 1 "MountingHole" H 2900 1000 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 2800 1000 50  0001 C CNN
F 3 "~" H 2800 1000 50  0001 C CNN
	1    2800 1000
	1    0    0    -1  
$EndComp
Connection ~ 4000 3450
Wire Wire Line
	3800 3450 4000 3450
$Comp
L Device:R_Small R101
U 1 1 5CA60BF7
P 3700 3450
F 0 "R101" V 3950 3450 50  0000 C CNN
F 1 "10k" V 3850 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3700 3450 50  0001 C CNN
F 3 "~" H 3700 3450 50  0001 C CNN
	1    3700 3450
	0    1    -1   0   
$EndComp
Wire Wire Line
	4000 3800 4000 3700
Wire Wire Line
	4200 3850 4200 4050
$Comp
L Device:C_Small C109
U 1 1 5CB0FB9C
P 4200 3750
F 0 "C109" H 4400 3800 50  0000 C CNN
F 1 "100n" H 4400 3700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4200 3750 50  0001 C CNN
F 3 "~" H 4200 3750 50  0001 C CNN
	1    4200 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C105
U 1 1 5CB0ECD8
P 4000 3900
F 0 "C105" H 3800 3950 50  0000 C CNN
F 1 "100n" H 3800 3850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4000 3900 50  0001 C CNN
F 3 "~" H 4000 3900 50  0001 C CNN
	1    4000 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C106
U 1 1 5CA609C2
P 4000 3600
F 0 "C106" H 3800 3650 50  0000 C CNN
F 1 "100n" H 3800 3550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4000 3600 50  0001 C CNN
F 3 "~" H 4000 3600 50  0001 C CNN
	1    4000 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 4050 4100 4050
Wire Wire Line
	4000 4050 4000 4000
Wire Wire Line
	4000 3450 4200 3450
Wire Wire Line
	4000 3500 4000 3450
$Comp
L Connector:TestPoint TP103
U 1 1 5CA527F7
P 4600 2750
F 0 "TP103" H 4550 3100 50  0000 L CNN
F 1 "TestPoint" H 4450 3000 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 4800 2750 50  0001 C CNN
F 3 "~" H 4800 2750 50  0001 C CNN
	1    4600 2750
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U101
U 1 1 5CA24F80
P 5000 3550
F 0 "U101" H 5000 3800 50  0000 C CNN
F 1 "LM324" H 5000 3900 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 4950 3650 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5050 3750 50  0001 C CNN
	1    5000 3550
	1    0    0    -1  
$EndComp
Wire Notes Line
	5550 4750 5550 2350
Wire Notes Line
	3500 2350 3500 4750
Wire Notes Line
	3500 4750 5550 4750
Wire Wire Line
	3000 2750 3000 3450
Wire Wire Line
	4700 3650 4600 3650
Wire Wire Line
	4600 3650 4600 3800
Wire Wire Line
	4600 3800 5000 3800
Connection ~ 4600 3800
Wire Wire Line
	4600 3800 4600 3900
Wire Wire Line
	4600 4100 4600 4200
$Comp
L power:GND #PWR0103
U 1 1 5CAB245D
P 4600 4200
F 0 "#PWR0103" H 4600 3950 50  0001 C CNN
F 1 "GND" H 4650 4000 50  0000 C CNN
F 2 "" H 4600 4200 50  0001 C CNN
F 3 "" H 4600 4200 50  0001 C CNN
	1    4600 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3450 4700 3450
Connection ~ 4200 3450
Wire Wire Line
	5300 3550 5400 3550
Wire Wire Line
	3600 3450 3000 3450
Connection ~ 3000 3450
Wire Wire Line
	3000 3450 3000 3850
Wire Wire Line
	5200 3800 5400 3800
Connection ~ 5400 3550
Wire Wire Line
	5400 3550 6100 3550
Wire Wire Line
	4600 2750 4600 3650
Connection ~ 4600 3650
$Comp
L Connector:TestPoint TP102
U 1 1 5CA4FE81
P 4200 2750
F 0 "TP102" H 4150 3100 50  0000 L CNN
F 1 "TestPoint" H 4050 3000 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 4400 2750 50  0001 C CNN
F 3 "~" H 4400 2750 50  0001 C CNN
	1    4200 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 2750 4200 3450
Wire Wire Line
	4200 3450 4200 3650
Wire Wire Line
	4100 4050 4100 4150
$Comp
L power:GND #PWR0104
U 1 1 5CA5D446
P 4100 4150
F 0 "#PWR0104" H 4100 3900 50  0001 C CNN
F 1 "GND" H 4150 3950 50  0000 C CNN
F 2 "" H 4100 4150 50  0001 C CNN
F 3 "" H 4100 4150 50  0001 C CNN
	1    4100 4150
	1    0    0    -1  
$EndComp
Connection ~ 4100 4050
Wire Wire Line
	4100 4050 4200 4050
$Comp
L Device:R_Small R102
U 1 1 5CB5C28F
P 5100 3800
F 0 "R102" V 4900 3800 50  0000 C CNN
F 1 "10k" V 5000 3800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5100 3800 50  0001 C CNN
F 3 "~" H 5100 3800 50  0001 C CNN
	1    5100 3800
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R103
U 1 1 5CA30F84
P 4600 4000
F 0 "R103" H 4450 4100 50  0000 C CNN
F 1 "10k" H 4500 4000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4600 4000 50  0001 C CNN
F 3 "~" H 4600 4000 50  0001 C CNN
	1    4600 4000
	-1   0    0    1   
$EndComp
Wire Wire Line
	5400 3800 5400 3550
$Comp
L Connector:TestPoint TP108
U 1 1 5CA7F834
P 5400 2750
F 0 "TP108" H 5350 3100 50  0000 L CNN
F 1 "TestPoint" H 5250 3000 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 5600 2750 50  0001 C CNN
F 3 "~" H 5600 2750 50  0001 C CNN
	1    5400 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 2750 5400 3550
$Comp
L Amplifier_Operational:LM324 U101
U 4 1 5CAB1DC7
P 8400 5800
F 0 "U101" H 8400 5400 50  0000 C CNN
F 1 "LM324" H 8400 5500 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 8350 5900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 8450 6000 50  0001 C CNN
	4    8400 5800
	1    0    0    1   
$EndComp
Wire Wire Line
	8100 5900 8000 5900
$Comp
L power:GND #PWR0110
U 1 1 5CAB1DD2
P 8000 6000
F 0 "#PWR0110" H 8000 5750 50  0001 C CNN
F 1 "GND" H 8050 5800 50  0000 C CNN
F 2 "" H 8000 6000 50  0001 C CNN
F 3 "" H 8000 6000 50  0001 C CNN
	1    8000 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 5900 8000 6000
Wire Wire Line
	8000 5500 8000 5700
Wire Wire Line
	8000 5700 8100 5700
Wire Wire Line
	8800 5500 8800 5800
Wire Wire Line
	8800 5800 8700 5800
Wire Wire Line
	8000 5500 8800 5500
$EndSCHEMATC