
int sineLUT[360] =  { 128,130,132,134,136,139,141,143,
                      145,147,150,152,154,156,158,160,
                      163,165,167,169,171,173,175,177,
                      179,181,183,185,187,189,191,193,
                      195,197,199,201,202,204,206,208,
                      209,211,213,214,216,218,219,221,
                      222,224,225,227,228,229,231,232,
                      233,234,236,237,238,239,240,241,
                      242,243,244,245,246,247,247,248,
                      249,249,250,251,251,252,252,253,
                      253,253,254,254,254,255,255,255,
                      255,255,255,255,255,255,255,255,
                      254,254,254,253,253,253,252,252,
                      251,251,250,249,249,248,247,247,
                      246,245,244,243,242,241,240,239,
                      238,237,236,234,233,232,231,229,
                      228,227,225,224,222,221,219,218,
                      216,214,213,211,209,208,206,204,
                      202,201,199,197,195,193,191,189,
                      187,185,183,181,179,177,175,173,
                      171,169,167,165,163,160,158,156,
                      154,152,150,147,145,143,141,139,
                      136,134,132,130,128,125,123,121,
                      119,116,114,112,110,108,105,103,
                      101,99,97,95,92,90,88,86,
                      84,82,80,78,76,74,72,70,
                      68,66,64,62,60,58,56,54,
                      53,51,49,47,46,44,42,41,
                      39,37,36,34,33,31,30,28,
                      27,26,24,23,22,21,19,18,
                      17,16,15,14,13,12,11,10,
                      9,8,8,7,6,6,5,4,
                      4,3,3,2,2,2,1,1,
                      1,0,0,0,0,0,0,0,
                      0,0,0,0,1,1,1,2,
                      2,2,3,3,4,4,5,6,
                      6,7,8,8,9,10,11,12,
                      13,14,15,16,17,18,19,21,
                      22,23,24,26,27,28,30,31,
                      33,34,36,37,39,41,42,44,
                      46,47,49,51,53,54,56,58,
                      60,62,64,66,68,70,72,74,
                      76,78,80,82,84,86,88,90,
                      92,95,97,99,101,103,105,108,
                      110,112,114,116,119,121,123,125 };

//----------------------------------------------
// Carrier Estimation Variables - Costas Loop --
//----------------------------------------------

// Local Oszilator Signals
int LoI;
int LoQ;

// Filter Coefficiants
float b[3] = {1, 2, 1}; // Numerator
float a[3] = {1, 1.69113816433026142860285290225874632597 , -0.732831786314550148730972978228237479925}; // Denominator:

// FilterArray
float filterArrayA [3] = {0, 0, 0};
float filterArrayB [3] = {0, 0, 0};
float filterArrayC [3] = {0, 0, 0};

//Gain
float gain = 0.010423405496072171358412639108337316429 ;

//--------------------
// Clock Estimation --
//--------------------
bool digitalDemodSignal;

bool trigger = false;
bool triggerOld = false;
bool serialDataDelayed = false;
int upcounter = 0;
bool enable = false;


int i = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(D4, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  static int counter = 10;

  //Sampling the inputsignal
  int sample = sampling() - 98;
  //int sample = TestsignalGen()- 127;

//---------------
// Costas Loop --
//---------------

  //Multiply sampled inputsignal with LO
  int pi = (int)(((long) sample * (long(LoI)))>>7);
  int pq = (int)(((long) sample * (long(LoQ)))>>7);
  
    int lpf_pi;
    int lpf_pq;

  //Low pass filtering the pi-product and pq-product
    lpf_pi = (int)IIRFilter(filterArrayA, (float)pi);
    lpf_pq = (int)IIRFilter(filterArrayB, (float)pq);

  //Multiplication of the two filteroutputs
    int toLoopFilter = (int)(((long)lpf_pi * (long)lpf_pq)>>6); 
    
  //Loopfilter 
    int NCO_Control = (int)IIRFilter(filterArrayC, (float)toLoopFilter);
  Serial.print(" ");
  Serial.print(sample);
  //Serial.print(" ");
  //Serial.print(LoI);
  //Serial.print(" ");
  //Serial.print(LoQ);
  //Serial.print(" ");
  //Serial.print(lpf_pq);
  Serial.print(" ");
  Serial.print(lpf_pi);
  Serial.print(" ");
  Serial.print(NCO_Control);
  
  NCO(NCO_Control, &LoI, &LoQ);

  lpf_pi>0 ? digitalDemodSignal = true : digitalDemodSignal = false;

//-----------------
// Symbol Timing --
//-----------------
  bool bums = LineDecoder(digitalDemodSignal);

  bums ? digitalWrite(D4, HIGH) : digitalWrite(D4, LOW); 

  Serial.print(" ");
  Serial.print(trigger*50);
  Serial.print(" ");
  Serial.println(bums*150);
  
}


//-------------------------
//-- Sampling the Signal --
//-------------------------
int sampling (){
  return (analogRead(A0) / 4); 
}


//----------------------------------
//-- Number Controlled Oscillator --
//----------------------------------
void NCO (int stepSize, int *iVal, int *qVal){

    i = (i + stepSize + 175) % 3600; 
    
    int iValue = sineLUT[i/10];
    *iVal = iValue - 127;
    int qValue = sineLUT[(i/10 + 90) % 360];
    *qVal = qValue - 127;
}

//----------------
//-- IIR Filter --
//----------------
float IIRFilter (float filterarray[], float newSample){

  filterarray[0] = (newSample*a[0]) + (a[1]*filterarray[1]) + (a[2]*filterarray[2]);
  float out = ((b[0]*filterarray[0]) + (b[1]*filterarray[1]) + (b[2]*filterarray[2]))*gain;
  filterarray[2] = filterarray[1];
  filterarray[1] = filterarray[0];
  
  return out;
}

//--------------------------
//-- Testsignal Generator --
//--------------------------
int TestsignalGen (){
  static int p = 0;
  static int phi = 20;
  static float j = 0;

  if ((p%300) == 0){
    j+=180;
  }
  if (j < 359 - phi){
    j = (j+ phi);
  }else {
    j = 0;
  }
  p++;
  delay (100);
  return sineLUT[(int)j];
}

//----------------------
// Manchester Decoder --
//----------------------
bool LineDecoder(bool serialData){

  static bool serialDataOut;
  
  //Saving Trigger
  if (upcounter < 70){
    if (upcounter == 0) {
      enable == true ? upcounter++ : upcounter = upcounter;
    }
    else{
      upcounter++;
    }
  }
  else {
    upcounter = 0;
    triggerOld = trigger;
    serialDataOut = serialDataDelayed;
  }
  //Calculating new trigger
  trigger = serialData xor serialDataDelayed;

  if (triggerOld == 0 && trigger == 1){
    serialDataDelayed = serialData;
    enable = true;
  }else {
    enable = false;
  }
  return serialDataOut;
}

//---------------------------
// Number Controlled Clock --
//---------------------------

bool NCC (int err_s){
  
  const int T = 10000;
  static int t = 0;
  bool clk;

  t < (T - err_s)/2 ? clk = false : clk = true;

  t = (t + 1) % (T-err_s);

  return clk;
  
}
