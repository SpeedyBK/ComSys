  /*
  static int bums = 0;
  
  i = (i + 1) % 360;
  i == 0 ? periods = (periods + 1) % 6 : periods = periods;
  if (periods == 0 && i == 0){ 
    //InpData = !digitalRead(InterruptPin);
    InpData =  dataA[bums];
    bums = (bums + 1) % 8;
  }

  PORTC = sineLUT[ManchesterEncoding(InpData, periods, i)] >> 1;
  PORTB = sineLUT[ManchesterEncoding(InpData, periods, i)] >> 3;
  */


int dlay = 1;
int i = 0;

bool dataA [8] = {0, 0, 0, 0, 0, 1, 1, 0};

int periods = 0;
bool InpData = 0;

enum ST {Wait, Preamble, Data, InterframeGap};  
typedef enum ST States;

const int sineLUT[360] = {128,130,132,134,136,139,141,143,
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
                          101, 99, 97, 95, 92, 90, 88, 86,
                           84, 82, 80, 78, 76, 74, 72, 70,
                           68, 66, 64, 62, 60, 58, 56, 54,
                           53, 51, 49, 47, 46, 44, 42, 41,
                           39, 37, 36, 34, 33, 31, 30, 28,
                           27, 26, 24, 23, 22, 21, 19, 18,
                           17, 16, 15, 14, 13, 12, 11, 10,
                            9,  8,  8,  7,  6,  6,  5,  4,
                            4,  3,  3,  2,  2,  2,  1,  1,
                            1,  0,  0,  0,  0,  0,  0,  0,
                            0,  0,  0,  0,  1,  1,  1,  2,
                            2,  2,  3,  3,  4,  4,  5,  6,
                            6,  7,  8,  8,  9, 10, 11, 12,
                           13, 14, 15, 16, 17, 18, 19, 21,
                           22, 23, 24, 26, 27, 28, 30, 31,
                           33, 34, 36, 37, 39, 41, 42, 44,
                           46, 47, 49, 51, 53, 54, 56, 58,
                           60, 62, 64, 66, 68, 70, 72, 74,
                           76, 78, 80, 82, 84, 86, 88, 90,
                           92, 95, 97, 99,101,103,105,108,
                          110,112,114,116,119,121,123,125};

#define InterruptPin 2

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  DDRB = B111111;
  DDRC = B111111;
  pinMode(6, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  ControllFSM();
}

int ManchesterEncoding (bool data, int periods, int phase){

  if (periods < 2 && data == false) {
    return phase;
  }
  else if (periods < 2 && data == true) {
    return (phase + 180) % 360;  
  }
  else if (periods < 4 && data == false) {
    return (phase + 180) % 360; 
  }
  else if (periods < 4 && data == true){
    return phase;  
  }
}

int ControllFSM () {

  static ST State = Wait; 
  bool preamble [6] = {1, 0, 1, 0, 1, 1};
  int bits = 0;
  int debug = 0;
  static uint8_t data = 0;

  switch (State) {

    case Wait: // Just wait for data, and send dc voltage.
      if (Serial.available()> 0){
        data = Serial.read();
        if (data == '\n'){
          State = Wait;
        }else {
          State = Preamble;
        }
        Serial.println("True");
      }
      PORTC = 128 >> 1;
      PORTB = 128 >> 3;
    break;
    case Preamble: //Send 1010(Cariersync)11(Start of Frame)
            
      while (bits < 6) {
        delay(dlay);
        i = (i + 1) % 360;
        i == 0 ? periods = (periods + 1) % 4 : periods = periods;
        if (periods == 0 && i == 0){ 
          bits++;
        }
        InpData =  preamble[bits];
        PORTC = sineLUT[ManchesterEncoding(InpData, periods, i)] >> 1;
        PORTB = sineLUT[ManchesterEncoding(InpData, periods, i)] >> 3;
        digitalWrite(6, InpData);
      }  
      State = Data;
   
    break;
    case Data:  
      while (bits < 8) {
        delay(dlay);
        i = (i + 1) % 360;
        i == 0 ? periods = (periods + 1) % 4 : periods = periods;
        if (periods == 0 && i == 0){ 
          bits++;
        }
        InpData = bitRead(data, 7-bits);
        PORTC = sineLUT[ManchesterEncoding(InpData, periods, i)] >> 1;
        PORTB = sineLUT[ManchesterEncoding(InpData, periods, i)] >> 3;
        digitalWrite(6, InpData);
      }  
      State = InterframeGap;
    break;
    case InterframeGap:
      PORTC = 128 >> 1;
      PORTB = 128 >> 3;
      data = 0;
      digitalWrite(6, LOW);
      delay (1000);
      State = Wait;
    break;
  }
  return 0;
}
