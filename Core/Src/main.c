/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
//#include "math_helper.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
uint8_t volume_level = 5; // Default volume level
uint8_t trig_vol = 5; // Default volume level

uint16_t trig_freq = 100;

int8_t current_row = -1, current_col = -1;
uint8_t key_pressed = 0;
uint8_t key_detected = 0;
char key;
uint8_t push_button_en = 0;
uint8_t new_sample = 0;

uint8_t echo_en = 0;

char keys[4][4] = {{'3', '2', '1', '0'},
                    {'7', '6', '5', '4'},
                    {'B', 'A', '9', '8'},
                    {'F', 'E', 'D', 'C'}};

const float32_t sine[SAMPLES] = {
		0,	0.130526192220052,	0.258819045102521,	0.382683432365090,	0.500000000000000,	0.608761429008721,	0.707106781186548,
		0.793353340291235,	0.866025403784439,	0.923879532511287,	0.965925826289068,	0.991444861373810,	1,	0.991444861373811,
		0.965925826289068,	0.923879532511287,	0.866025403784439,	0.793353340291235,	0.707106781186548,	0.608761429008721,
		0.500000000000000,	0.382683432365090,	0.258819045102521,	0.130526192220052,	0,	-0.130526192220051,	-0.258819045102520,
		-0.382683432365089,	-0.499999999999999,	-0.608761429008721,	-0.707106781186548,	-0.793353340291235,	-0.866025403784439,
		-0.923879532511287,	-0.965925826289068,	-0.991444861373810,	-1,	-0.991444861373811,	-0.965925826289068,	-0.923879532511287,
		-0.866025403784439,	-0.793353340291236,	-0.707106781186548,	-0.608761429008721,	-0.500000000000000,	-0.382683432365090,
		-0.258819045102522,	-0.130526192220053
};

float32_t square[SAMPLES];

const float32_t logarithmic_chirp[320] = {
		0.000000000000000061,	0.002622410992047861,	0.005253663973466970,	0.007893770384930297,	0.010542741395035495,	0.013200587895525877,	0.015867320496454066,	0.018542949521290073,
		0.021227485001971542,	0.023920936673895138,	0.026623313970853074,	0.029334626019908643,	0.032054881636210709,	0.034784089317753723,	0.037522257240071598,	0.040269393250875855,
		0.043025504864628375,	0.045790599257054837,	0.048564683259595690,	0.051347763353792118,	0.054139845665610427,	0.056940935959702531,	0.059751039633601337,	0.062570161711849828,
		0.065398306840066575,	0.068235479278943648,	0.071081682898178900,	0.073936921170339814,	0.076801197164660218,	0.079674513540768196,	0.082556872542344922,	0.085448275990715375,
		0.088348725278367082,	0.091258221362398390,	0.094176764757897533,	0.097104355531246703,	0.100040993293358240,	0.102986677192832010,	0.105941405909045980,	0.108905177645166230,
		0.111877990121087980,	0.114859840566297130,	0.117850725712659680,	0.120850641787131110,	0.123859584504392860,	0.126877549059407400,	0.129904530119898690,	0.132940521818751430,
		0.135985517746334080,	0.139039510942737950,	0.142102493889940090,	0.145174458503884160,	0.148255396126476810,	0.151345297517508140,	0.154444152846483080,	0.157551951684374300,
		0.160668682995289720,	0.163794335128054890,	0.166928895807713030,	0.170072352126936720,	0.173224690537355760,	0.176385896840798810,	0.179555956180445340,	0.182734853031894270,
		0.185922571194139130,	0.189119093780459800,	0.192324403209221870,	0.195538481194587030,	0.198761308737133020,	0.201992866114384050,	0.205233132871247170,	0.208482087810360570,
		0.211739708982344370,	0.215005973675965020,	0.218280858408200220,	0.221564338914212730,	0.224856390137231970,	0.228156986218334190,	0.231466100486134670,	0.234783705446379690,
		0.238109772771442410,	0.241444273289723230,	0.244787176974952890,	0.248138452935395580,	0.251498069402956710,	0.254865993722190930,	0.258242192339209860,	0.261626630790492030,
		0.265019273691591620,	0.268420084725748410,	0.271829026632395280,	0.275246061195565440,	0.278671149232197430,	0.282104250580339830,	0.285545324087251580,	0.288994327597401960,
		0.292451217940364990,	0.295915950918612280,	0.299388481295203350,	0.302868762781368150,	0.306356748023990040,	0.309852388592980640,	0.313355634968552230,	0.316866436528383590,
		0.320384741534681720,	0.323910497121136620,	0.327443649279772870,	0.330984142847692230,	0.334531921493712690,	0.338086927704900790,	0.341649102772995210,	0.345218386780727190,
		0.348794718588032520,	0.352378035818156910,	0.355968274843654950,	0.359565370772282730,	0.363169257432780890,	0.366779867360555120,	0.370397131783246010,	0.374020980606193880,
		0.377651342397795690,	0.381288144374756830,	0.384931312387234990,	0.388580770903877330,	0.392236442996751310,	0.395898250326170650,	0.399566113125414350,	0.403239950185338420,
		0.406919678838884410,	0.410605214945482130,	0.414296472875345100,	0.417993365493664670,	0.421695804144698540,	0.425403698635752780,	0.429116957221065130,	0.432835486585582130,
		0.436559191828633180,	0.440287976447505720,	0.444021742320914510,	0.447760389692375140,	0.451503817153472210,	0.455251921627031540,	0.459004598350192470,	0.462761740857380200,
		0.466523240963184150,	0.470288988745136360,	0.474058872526396560,	0.477832778858340690,	0.481610592503056990,	0.485392196415748600,	0.489177471727042850,	0.492966297725213780,
		0.496758551838309250,	0.500554109616195060,	0.504352844712508190,	0.508154628866524960,	0.511959331884944910,	0.515766821623591440,	0.519576963969030530,	0.523389622820107150,
		0.527204660069405030,	0.531021935584629400,	0.534841307189911630,	0.538662630647041900,	0.542485759636628150,	0.546310545739186690,	0.550136838416161340,	0.553964484990880020,
		0.557793330629441700,	0.561623218321546380,	0.565453988861259300,	0.569285480827721570,	0.573117530565801950,	0.576949972166696630,	0.580782637448476910,	0.584615355936589420,
		0.588447954844309340,	0.592280259053150400,	0.596112091093235260,	0.599943271123626440,	0.603773616912622660,	0.607602943818024150,	0.611431064767369080,	0.615257790238142090,
		0.619082928237961740,	0.622906284284749700,	0.626727661386881850,	0.630546860023327600,	0.634363678123782030,	0.638177911048790960,	0.641989351569874020,	0.645797789849653410,
		0.649603013421986450,	0.653404807172108140,	0.657202953316791350,	0.660997231384523490,	0.664787418195706640,	0.668573287842887610,	0.672354611671016960,	0.676131158257749170,
		0.679902693393781730,	0.683668980063242500,	0.687429778424128110,	0.691184845788802130,	0.694933936604551380,	0.698676802434213370,	0.702413191936877570,	0.706142850848662460,
		0.709865521963579990,	0.713580945114492330,	0.717288857154159800,	0.720988991936399870,	0.724681080297347790,	0.728364850036839040,	0.732040025899910680,	0.735706329558433620,
		0.739363479592880620,	0.743011191474238440,	0.746649177546067850,	0.750277147006723990,	0.753894805891742180,	0.757501857056394940,	0.761098000158428880,	0.764682931640995540,
		0.768256344715771980,	0.771817929346292900,	0.775367372231492210,	0.778904356789468790,	0.782428563141483460,	0.785939668096195860,	0.789437345134148760,	0.792921264392515420,
		0.796391092650110770,	0.799846493312681210,	0.803287126398485760,	0.806712648524170680,	0.810122712890953390,	0.813516969271127150,	0.816895063994893090,	0.820256639937531280,
		0.823601336506926020,	0.826928789631450890,	0.830238631748229430,	0.833530491791779850,	0.836803995183058700,	0.840058763818912760,	0.843294416061954100,	0.846510566730867220,
		0.849706827091166740,	0.852882804846411770,	0.856038104129895340,	0.859172325496819990,	0.862285065916973510,	0.865375918767918860,	0.868444473828712590,	0.871490317274166260,
		0.874513031669661770,	0.877512195966544280,	0.880487385498096800,	0.883438171976119850,	0.886364123488128100,	0.889264804495180530,	0.892139775830360640,	0.894988594697921020,
		0.897810814673113080,	0.900605985702712770,	0.903373654106265470,	0.906113362578062300,	0.908824650189867690,	0.911507052394417540,	0.914160101029702910,	0.916783324324059180,
		0.919376246902079860,	0.921938389791372770,	0.924469270430179120,	0.926968402675872660,	0.929435296814361430,	0.931869459570409790,	0.934270394118903560,	0.936637600097074200,
		0.938970573617708970,	0.941268807283364040,	0.943531790201601380,	0.945759008001275100,	0.947949942849885320,	0.950104073472023970,	0.952220875168933280,	0.954299819839202090,
		0.956340376000621160,	0.958342008813221960,	0.960304180103520260,	0.962226348389994210,	0.964107968909812760,	0.965948493646846980,	0.967747371360983650,	0.969504047618768740,
		0.971217964825405680,	0.972888562258134030,	0.974515276101013520,	0.976097539481141750,	0.977634782506330400,	0.979126432304266880,	0.980571913063189360,	0.981970646074102120,
		0.983322049774557390,	0.984625539794035220,	0.985880529000944810,	0.987086427551279730,	0.988242642938953360,	0.989348580047844540,	0.990403641205582440,	0.991407226239099710,
		0.992358732531984260,	0.993257555083659870,	0.994103086570423680,	0.994894717408374870,	0.995631835818261310,	0.996313827892278070,	0.996940077662846650,	0.997509967173408010,
};


const float32_t firCoeffs32[NUM_TAPS] = {
	  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
	  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
	  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
};

const float32_t coeffTable[950] = {
		/* Band 1, -9 dB gain */
		0.997590, -1.995180, 0.997590, 2.000000, -1.000000, 0.997590, -1.980927, 0.983438, 1.976016, -0.976300,
		/* Band 1, -8 dB gain */
		0.997862, -1.995725, 0.997862, 2.000000, -1.000000, 0.997862, -1.981052, 0.983296, 1.976696, -0.976964,
		0.998133, -1.996266, 0.998133, 2.000000, -1.000000, 0.998133, -1.981161, 0.983141, 1.977357, -0.977611,
		0.998402, -1.996805, 0.998402, 2.000000, -1.000000, 0.998402, -1.981254, 0.982972, 1.977999, -0.978239,
		0.998670, -1.997341, 0.998670, 2.000000, -1.000000, 0.998670, -1.981332, 0.982789, 1.978624, -0.978850,
		0.998937, -1.997875, 0.998937, 2.000000, -1.000000, 0.998937, -1.981394, 0.982591, 1.979230, -0.979443,
		0.999204, -1.998408, 0.999204, 2.000000, -1.000000, 0.999204, -1.981441, 0.982380, 1.979819, -0.980021,
		0.999469, -1.998939, 0.999469, 2.000000, -1.000000, 0.999469, -1.981472, 0.982154, 1.980392, -0.980582,
		0.999735, -1.999470, 0.999735, 2.000000, -1.000000, 0.999735, -1.981488, 0.981914, 1.980948, -0.981128,
		1.000000, -2.000000, 1.000000, 2.000000, -1.000000, 1.000000, -1.981488, 0.981658, 1.981488, -0.981658,
		1.000265, -2.000530, 1.000265, 2.000000, -1.000000, 1.000265, -1.981473, 0.981388, 1.982014, -0.982174,
		1.000531, -2.001062, 1.000531, 2.000000, -1.000000, 1.000531, -1.981443, 0.981103, 1.982524, -0.982675,
		1.000797, -2.001594, 1.000797, 2.000000, -1.000000, 1.000797, -1.981397, 0.980802, 1.983020, -0.983163,
		1.001064, -2.002127, 1.001064, 2.000000, -1.000000, 1.001064, -1.981335, 0.980485, 1.983501, -0.983636,
		1.001331, -2.002663, 1.001331, 2.000000, -1.000000, 1.001331, -1.981258, 0.980153, 1.983970, -0.984097,
		1.001600, -2.003200, 1.001600, 2.000000, -1.000000, 1.001600, -1.981165, 0.979804, 1.984424, -0.984545,
		1.001871, -2.003741, 1.001871, 2.000000, -1.000000, 1.001871, -1.981056, 0.979439, 1.984866, -0.984980,
		1.002142, -2.004285, 1.002142, 2.000000, -1.000000, 1.002142, -1.980931, 0.979057, 1.985296, -0.985403,
		1.002416, -2.004832, 1.002416, 2.000000, -1.000000, 1.002416, -1.980790, 0.978659, 1.985713, -0.985814,
		0.990527, -1.966018, 0.975790, 1.985977, -0.986122, 0.990527, -1.937818, 0.949629, 1.916491, -0.921315,
		0.991593, -1.968075, 0.976771, 1.985795, -0.985948, 0.991593, -1.938312, 0.949134, 1.919385, -0.923988,
		0.992655, -1.970134, 0.977760, 1.985625, -0.985785, 0.992655, -1.938731, 0.948575, 1.922192, -0.926586,
		0.993711, -1.972199, 0.978759, 1.985466, -0.985633, 0.993711, -1.939074, 0.947948, 1.924913, -0.929111,
		0.994764, -1.974270, 0.979768, 1.985320, -0.985495, 0.994764, -1.939339, 0.947255, 1.927549, -0.931563,
		0.995814, -1.976351, 0.980790, 1.985187, -0.985371, 0.995814, -1.939528, 0.946494, 1.930102, -0.933942,
		0.996862, -1.978442, 0.981825, 1.985068, -0.985260, 0.996862, -1.939639, 0.945663, 1.932572, -0.936250,
		0.997908, -1.980546, 0.982873, 1.984964, -0.985164, 0.997908, -1.939672, 0.944763, 1.934961, -0.938487,
		0.998954, -1.982665, 0.983938, 1.984874, -0.985083, 0.998954, -1.939626, 0.943793, 1.937271, -0.940654,
		1.000000, -1.984800, 0.985018, 1.984800, -0.985018, 1.000000, -1.939502, 0.942752, 1.939502, -0.942752,
		1.001047, -1.986953, 0.986115, 1.984741, -0.984968, 1.001047, -1.939299, 0.941639, 1.941658, -0.944781,
		1.002096, -1.989125, 0.987229, 1.984698, -0.984934, 1.002096, -1.939017, 0.940454, 1.943738, -0.946744,
		1.003148, -1.991318, 0.988362, 1.984670, -0.984915, 1.003148, -1.938656, 0.939197, 1.945745, -0.948640,
		1.004204, -1.993532, 0.989513, 1.984658, -0.984913, 1.004204, -1.938215, 0.937868, 1.947681, -0.950472,
		1.005263, -1.995770, 0.990682, 1.984662, -0.984925, 1.005263, -1.937695, 0.936466, 1.949547, -0.952241,
		1.006329, -1.998031, 0.991871, 1.984680, -0.984953, 1.006329, -1.937095, 0.934991, 1.951345, -0.953948,
		1.007400, -2.000318, 0.993079, 1.984713, -0.984996, 1.007400, -1.936416, 0.933443, 1.953077, -0.955594,
		1.008478, -2.002631, 0.994306, 1.984760, -0.985052, 1.008478, -1.935658, 0.931821, 1.954745, -0.957181,
		1.009564, -2.004971, 0.995553, 1.984820, -0.985123, 1.009564, -1.934820, 0.930127, 1.956351, -0.958711,
		0.966534, -1.866346, 0.906596, 1.932002, -0.935733, 0.966534, -1.761754, 0.831431, 1.680788, -0.746363,
		0.970252, -1.873134, 0.909509, 1.931427, -0.935322, 0.970252, -1.762977, 0.829961, 1.691136, -0.754252,
		0.973965, -1.879949, 0.912454, 1.930912, -0.934976, 0.973965, -1.763937, 0.828291, 1.701165, -0.761962,
		0.977674, -1.886800, 0.915435, 1.930459, -0.934695, 0.977674, -1.764629, 0.826418, 1.710879, -0.769489,
		0.981383, -1.893695, 0.918458, 1.930071, -0.934481, 0.981383, -1.765047, 0.824337, 1.720282, -0.776831,
		0.985093, -1.900640, 0.921530, 1.929746, -0.934334, 0.985093, -1.765186, 0.822047, 1.729380, -0.783986,
		0.988808, -1.907645, 0.924655, 1.929485, -0.934253, 0.988808, -1.765040, 0.819543, 1.738177, -0.790953,
		0.992529, -1.914715, 0.927839, 1.929289, -0.934239, 0.992529, -1.764602, 0.816823, 1.746681, -0.797732,
		0.996259, -1.921860, 0.931088, 1.929156, -0.934291, 0.996259, -1.763869, 0.813885, 1.754898, -0.804323,
		1.000000, -1.929086, 0.934406, 1.929086, -0.934406, 1.000000, -1.762833, 0.810725, 1.762833, -0.810725,
		1.003755, -1.936401, 0.937799, 1.929078, -0.934585, 1.003755, -1.761488, 0.807343, 1.770493, -0.816941,
		1.007528, -1.943812, 0.941272, 1.929129, -0.934824, 1.007528, -1.759830, 0.803737, 1.777886, -0.822972,
		1.011319, -1.951325, 0.944828, 1.929237, -0.935121, 1.011319, -1.757852, 0.799906, 1.785018, -0.828819,
		1.015132, -1.958947, 0.948472, 1.929401, -0.935475, 1.015132, -1.755549, 0.795850, 1.791897, -0.834486,
		1.018970, -1.966685, 0.952209, 1.929619, -0.935882, 1.018970, -1.752916, 0.791568, 1.798530, -0.839975,
		1.022836, -1.974543, 0.956040, 1.929887, -0.936339, 1.022836, -1.749948, 0.787061, 1.804925, -0.845289,
		1.026731, -1.982528, 0.959969, 1.930203, -0.936845, 1.026731, -1.746640, 0.782330, 1.811089, -0.850432,
		1.030660, -1.990645, 0.963999, 1.930564, -0.937395, 1.030660, -1.742987, 0.777378, 1.817030, -0.855408,
		1.034625, -1.998898, 0.968132, 1.930968, -0.937987, 1.034625, -1.738985, 0.772206, 1.822755, -0.860219,
		0.920303, -1.585898, 0.752613, 1.732542, -0.789053, 0.920303, -1.254890, 0.632373, 1.046514, -0.495094,
		0.928926, -1.600342, 0.757327, 1.730792, -0.789364, 0.928926, -1.254072, 0.629919, 1.068635, -0.507055,
		0.937598, -1.614936, 0.762098, 1.729211, -0.789864, 0.937598, -1.252690, 0.627141, 1.090188, -0.518931,
		0.946322, -1.629695, 0.766937, 1.727798, -0.790548, 0.946322, -1.250726, 0.624036, 1.111176, -0.530705,
		0.955102, -1.644633, 0.771858, 1.726549, -0.791410, 0.955102, -1.248160, 0.620600, 1.131605, -0.542361,
		0.963942, -1.659767, 0.776871, 1.725460, -0.792444, 0.963942, -1.244973, 0.616830, 1.151480, -0.553883,
		0.972848, -1.675114, 0.781992, 1.724527, -0.793642, 0.972848, -1.241144, 0.612723, 1.170807, -0.565257,
		0.981823, -1.690690, 0.787234, 1.723743, -0.794997, 0.981823, -1.236652, 0.608280, 1.189596, -0.576472,
		0.990872, -1.706514, 0.792612, 1.723104, -0.796499, 0.990872, -1.231475, 0.603499, 1.207855, -0.587516,
		1.000000, -1.722604, 0.798141, 1.722604, -0.798141, 1.000000, -1.225593, 0.598381, 1.225593, -0.598381,
		1.009212, -1.738978, 0.803836, 1.722235, -0.799913, 1.009212, -1.218982, 0.592929, 1.242820, -0.609058,
		1.018514, -1.755656, 0.809715, 1.721991, -0.801808, 1.018514, -1.211620, 0.587144, 1.259547, -0.619542,
		1.027910, -1.772658, 0.815793, 1.721866, -0.803817, 1.027910, -1.203485, 0.581033, 1.275784, -0.629825,
		1.037407, -1.790004, 0.822087, 1.721853, -0.805931, 1.037407, -1.194553, 0.574602, 1.291543, -0.639903,
		1.047009, -1.807712, 0.828614, 1.721946, -0.808142, 1.047009, -1.184801, 0.567857, 1.306835, -0.649774,
		1.056723, -1.825804, 0.835390, 1.722136, -0.810441, 1.056723, -1.174206, 0.560809, 1.321671, -0.659434,
		1.066555, -1.844299, 0.842433, 1.722419, -0.812820, 1.066555, -1.162746, 0.553469, 1.336064, -0.668881,
		1.076512, -1.863217, 0.849759, 1.722787, -0.815272, 1.076512, -1.150398, 0.545850, 1.350023, -0.678115,
		1.086598, -1.882578, 0.857384, 1.723234, -0.817789, 1.086598, -1.137141, 0.537969, 1.363562, -0.687135,
		0.721551, -0.944198, 0.366817, 1.564831, -0.646726, 0.721551, 0.944198, 0.366817, -1.564831, -0.646726,
		0.748146, -0.991441, 0.385846, 1.553009, -0.639214, 0.748146, 0.991441, 0.385846, -1.553009, -0.639214,
		0.775740, -1.040702, 0.405875, 1.540911, -0.631637, 0.775740, 1.040702, 0.405875, -1.540911, -0.631637,
		0.804368, -1.092052, 0.426942, 1.528533, -0.624000, 0.804368, 1.092052, 0.426942, -1.528533, -0.624000,
		0.834067, -1.145563, 0.449085, 1.515874, -0.616311, 0.834067, 1.145563, 0.449085, -1.515874, -0.616311,
		0.864875, -1.201312, 0.472342, 1.502930, -0.608577, 0.864875, 1.201312, 0.472342, -1.502930, -0.608577,
		0.896831, -1.259376, 0.496755, 1.489700, -0.600805, 0.896831, 1.259376, 0.496755, -1.489700, -0.600805,
		0.929976, -1.319836, 0.522366, 1.476182, -0.593003, 0.929976, 1.319836, 0.522366, -1.476182, -0.593003,
		0.964351, -1.382775, 0.549216, 1.462375, -0.585182, 0.964351, 1.382775, 0.549216, -1.462375, -0.585182,
		1.000000, -1.448278, 0.577350, 1.448278, -0.577350, 1.000000, 1.448278, 0.577350, -1.448278, -0.577350,
		1.036967, -1.516434, 0.606814, 1.433892, -0.569519, 1.036967, 1.516434, 0.606814, -1.433892, -0.569519,
		1.075297, -1.587334, 0.637655, 1.419216, -0.561698, 1.075297, 1.587334, 0.637655, -1.419216, -0.561698,
		1.115038, -1.661071, 0.669920, 1.404252, -0.553901, 1.115038, 1.661071, 0.669920, -1.404252, -0.553901,
		1.156237, -1.737743, 0.703659, 1.389001, -0.546139, 1.156237, 1.737743, 0.703659, -1.389001, -0.546139,
		1.198944, -1.817448, 0.738923, 1.373467, -0.538428, 1.198944, 1.817448, 0.738923, -1.373467, -0.538428,
		1.243212, -1.900290, 0.775764, 1.357651, -0.530779, 1.243212, 1.900290, 0.775764, -1.357651, -0.530779,
		1.289091, -1.986374, 0.814237, 1.341560, -0.523210, 1.289091, 1.986374, 0.814237, -1.341560, -0.523210,
		1.336637, -2.075809, 0.854397, 1.325197, -0.515736, 1.336637, 2.075809, 0.854397, -1.325197, -0.515736,
		1.385904, -2.168706, 0.896300, 1.308568, -0.508373, 1.385904, 2.168706, 0.896300, -1.308568, -0.508373
};

static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

static float32_t biquadStateBand1[8];
static float32_t biquadStateBand2[8];
static float32_t biquadStateBand3[8];
static float32_t biquadStateBand4[8];
static float32_t biquadStateBand5[8];

int band_gainDB[5] = {0, 0, 0, 0, 0};

float32_t input;
float32_t prevSample;

//uint32_t blockSize = BLOCK_SIZE;
//uint32_t numBlocks = SAMPLES/BLOCK_SIZE;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Per the adafruit keypad library, it seems like they allow polling the keypad
// at 20 us!! i.e. _KEYPAD_SETTLING_DELAY=20us
// We should be able to do close to that too
void check_keys(){
  uint16_t row_pins[] = { ROW0_Pin, ROW1_Pin, ROW2_Pin, ROW3_Pin };

  HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  key_detected = 0;
  for (int i = 0; i < 4; i++) {

    // Clear Row0 to Row3; Only valid b/c they are all on GPIOC
    HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin, GPIO_PIN_RESET);

    // Set desired Row
    HAL_GPIO_WritePin(ROW0_GPIO_Port, row_pins[i], GPIO_PIN_SET);
    HAL_Delay(0);

    if (HAL_GPIO_ReadPin(COL0_GPIO_Port, COL0_Pin)) {
      current_row = i;
      current_col = 0;
      key_detected = 1;
    }
    if (HAL_GPIO_ReadPin(COL1_GPIO_Port, COL1_Pin)) {
      current_row = i;
      current_col = 1;
      key_detected = 1;
    }
    if (HAL_GPIO_ReadPin(COL2_GPIO_Port, COL2_Pin)) {
      current_row = i;
      current_col = 2;
      key_detected = 1;
    }
    if (HAL_GPIO_ReadPin(COL3_GPIO_Port, COL3_Pin)) {
      current_row = i;
      current_col = 3;
      key_detected = 1;
    }
  }
  key = keys[current_row][current_col];
  // This ensures that after scanning the rows, they are all set
  // to high, so the interrupt detects if ANY key is pressed
  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin, GPIO_PIN_SET);

  HAL_Delay(1);

  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	arm_fir_instance_f32 lp;

	arm_biquad_casd_df1_inst_f32 S1;
	arm_biquad_casd_df1_inst_f32 S2;
	arm_biquad_casd_df1_inst_f32 S3;
	arm_biquad_casd_df1_inst_f32 S4;
	arm_biquad_casd_df1_inst_f32 S5;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Call FIR init function to initialize the instance structure. */
  arm_fir_init_f32(&lp, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], BLOCK_SIZE);

  arm_biquad_cascade_df1_init_f32(&S1, NUMSTAGES, &coeffTable[190*0 + 10*(band_gainDB[0] + 9)], biquadStateBand1);
  arm_biquad_cascade_df1_init_f32(&S2, NUMSTAGES, &coeffTable[190*1 + 10*(band_gainDB[1] + 9)], biquadStateBand2);
  arm_biquad_cascade_df1_init_f32(&S3, NUMSTAGES, &coeffTable[190*2 + 10*(band_gainDB[2] + 9)], biquadStateBand3);
  arm_biquad_cascade_df1_init_f32(&S4, NUMSTAGES, &coeffTable[190*3 + 10*(band_gainDB[3] + 9)], biquadStateBand4);
  arm_biquad_cascade_df1_init_f32(&S5, NUMSTAGES, &coeffTable[190*4 + 10*(band_gainDB[4] + 9)], biquadStateBand5);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  float32_t output = 0;
  float index = 0;
  float incr = 1;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DAC_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  // Set Row0 to Row3; Only valid b/c they are all on GPIOC

  HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin, GPIO_PIN_SET);

  HAL_Delay(1);

  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim7);
//  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start(&htim5);

  HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_2, (0 << 8));

//	float32_t  *inputF32;
//	inputF32 = &sine[0];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (key_pressed){
		// We cannot clear key_pressed here since we haven't
		// figured out which key was actually pressed
		// This is because scanning through keys generates
		// interrupts which re-enables key_pressed, which
		// we won't be able to properly clear
		check_keys();
		key_pressed = 0;

		if (key_detected) {
			if (key == 'A'){
				// double the frequency
				incr = incr*2;
			}
			else if (key == 'B'){
				// half the frequency (haven't exactly figured out how to go below 1 kHz)
				incr = incr/2;
			}
//			else if (key == 'E'){
//				// Increase volume (if not maximum)
//				if (volume_level < 10){
//					volume_level++;
//				}
//			}
//			else if (key == 'F'){
//				// Decrease volume (if not minimum)
//				if (volume_level > 0){
//					volume_level--;
//				}
//
//			// Control volume of triangular wave
//			}
			else if (key == 'C'){
				if (trig_vol < 11){
					trig_vol++;
				}
				HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_1, (trig_vol << 8));
			}
			else if (key == 'D'){
				if (trig_vol > 0){
					trig_vol--;
				}
				HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_1, (trig_vol << 8));

			// Control frequency of triangular wave
			}
			//	          else if (key == '8'){
			//	            // increase freq
			//	            if (trig_freq > 1){
			//	              trig_freq = trig_freq >> 1;
			//	            }
			//	            htim5.Init.Period = trig_freq;
			//	            if (HAL_TIM_Base_Init(&htim5) != HAL_OK){
			//	              Error_Handler();
			//	            }
			//	          }
			//	          else if (key == '9'){
			//	            // increase freq
			//	            if (trig_freq < 1000){
			//	              trig_freq = trig_freq << 1;
			//	            }
			//	            htim5.Init.Period = trig_freq;
			//	            if (HAL_TIM_Base_Init(&htim5) != HAL_OK){
			//	              Error_Handler();
			//	            }
			//	          }

			// ******* Gains of Equalizers *******//

			// Band 1
			else if (key == '0'){
				if (band_gainDB[0] < MAX_GAIN){
					band_gainDB[0]++;
					arm_biquad_cascade_df1_init_f32(&S1, NUMSTAGES, &coeffTable[190*0 + 10*(band_gainDB[0] + 9)], biquadStateBand1);
				}
			}else if (key == '1'){
				if (band_gainDB[0] > -MAX_GAIN){
					band_gainDB[0]--;
					arm_biquad_cascade_df1_init_f32(&S1, NUMSTAGES, &coeffTable[190*0 + 10*(band_gainDB[0] + 9)], biquadStateBand1);
				}
			}

			// band 2
			else if (key == '2'){
				if (band_gainDB[1] < MAX_GAIN){
					band_gainDB[1]++;
					arm_biquad_cascade_df1_init_f32(&S2, NUMSTAGES, &coeffTable[190*1 + 10*(band_gainDB[1] + 9)], biquadStateBand2);
				}
			}else if (key == '3'){
				if (band_gainDB[1] > -MAX_GAIN){
					band_gainDB[1]--;
					arm_biquad_cascade_df1_init_f32(&S2, NUMSTAGES, &coeffTable[190*1 + 10*(band_gainDB[1] + 9)], biquadStateBand2);
				}
			}

			// band 3
			else if (key == '4'){
				if (band_gainDB[2] < MAX_GAIN){
					band_gainDB[2]++;
					arm_biquad_cascade_df1_init_f32(&S3, NUMSTAGES, &coeffTable[190*2 + 10*(band_gainDB[2] + 9)], biquadStateBand3);
				}
			}else if (key == '5'){
				if (band_gainDB[2] > -1){
					band_gainDB[2]--;
					arm_biquad_cascade_df1_init_f32(&S2, NUMSTAGES, &coeffTable[190*2 + 10*(band_gainDB[2] + 9)], biquadStateBand3);
				}
			}

			// band 4
			else if (key == '6'){
				if (band_gainDB[3] < MAX_GAIN){
					band_gainDB[3]++;
					arm_biquad_cascade_df1_init_f32(&S4, NUMSTAGES, &coeffTable[190*3 + 10*(band_gainDB[3] + 9)], biquadStateBand4);
				}
			}else if (key == '7'){
				if (band_gainDB[3] > -MAX_GAIN){
					band_gainDB[3]--;
					arm_biquad_cascade_df1_init_f32(&S4, NUMSTAGES, &coeffTable[190*3 + 10*(band_gainDB[3] + 9)], biquadStateBand4);
				}
			}

			// band 5
			else if (key == '8'){
				if (band_gainDB[4] < MAX_GAIN){
					band_gainDB[4]++;
					arm_biquad_cascade_df1_init_f32(&S5, NUMSTAGES, &coeffTable[190*4 + 10*(band_gainDB[4] + 9)], biquadStateBand5);
				}
			}else if (key == '9'){
				if (band_gainDB[4] > -MAX_GAIN){
					band_gainDB[4]--;
					arm_biquad_cascade_df1_init_f32(&S5, NUMSTAGES, &coeffTable[190*4 + 10*(band_gainDB[4] + 9)], biquadStateBand5);
				}
			}

			else if (key == 'E'){
			}
			else if (key == 'F'){
			}
			// Clear keypress and keydetect
			key_detected = 0;
			key_pressed = 0;
			current_row = -1;
			current_col = -1;
		}
	}

	if (new_sample){
	new_sample = 0;

	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, 100);
	input = (float32_t)HAL_ADC_GetValue(&hadc3) - 2048.0;
	//
	//	  	  	  if (filter_en){
	//	  	  		  arm_fir_f32(&lp, waveform + (uint32_t)(index)%SAMPLES, &output, BLOCK_SIZE);
	//	  	  	  }else{
	//	  	  		  output = waveform[(uint32_t)(index) % SAMPLES] * (volume_level / 10.0);
	//	  	  	  }
	//
	//	  	  	  output *= 4095;
	//	  	  	  output += 2047;
	//	  		  output = input;

	arm_biquad_cascade_df1_f32(&S1, &input, &output, 1);
	arm_biquad_cascade_df1_f32(&S2, &output, &output, 1);
	arm_biquad_cascade_df1_f32(&S3, &output, &output, 1);
	arm_biquad_cascade_df1_f32(&S4, &output, &output, 1);
	arm_biquad_cascade_df1_f32(&S5, &output, &output, 1);

	// Select sine waveform
	const float32_t *waveform = (volume_level % 2 == 0) ? sine : square;

  // Cap output
//  if (output > 4096){
//    output = 4096;
//  }
//  else if (output < 0) {
//    output = 0;
//  }

	//output += waveform[(uint32_t)(index) % SAMPLES] * (volume_level / 10.0);




//  y[n] = x[n -2] + y[n - 1] + y[n - 2];

	if (push_button_en){
		output += 0.25f*biquadStateBand1[0] + 0.125f*biquadStateBand1[1] + 0.25f*biquadStateBand1[2] + 0.125f*biquadStateBand1[3];
	}

	output += 2048.0;
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t)output);


//	  	  	  index += incr;
//	  	  	  if (index >= SAMPLES){
//	  	  		  index -= SAMPLES; // wrap around
//	  	  	  }
	  	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T5_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 7000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1750-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW0_Pin ROW1_Pin ROW2_Pin ROW3_Pin */
  GPIO_InitStruct.Pin = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : COL0_Pin COL1_Pin COL2_Pin COL3_Pin */
  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin|COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
