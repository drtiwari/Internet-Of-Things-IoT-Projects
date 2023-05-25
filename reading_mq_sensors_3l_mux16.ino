//Multiplexer definitions
#define     S0                          D0
#define     S1                          D1
#define     S2                          D2
#define     S3                          D3
#define     analogpin                   A0
///////////////////////////////////////////////////////////////////////////////////
//MQ Gas sensors definitions
int         MQ_PIN                =     analogpin; //MQ Pin designation in calculation

int         RL_VALUE              =     10;                              //load resistance on the board, in kilo ohms
float       Ro;                                    //Ro is extracted from calibratiom
float       GAS[3];                                //Targeted gas

int         READ_SAMPLE_INTERVAL  =      50;      //number of samples in normal operation
int         READ_SAMPLE_TIMES     =      5;       //time interval(in millisecond) between each samples in normal operation

/*****************************Globals***********************************************/
float       LPGCurve[3]           =      {2.3,0.20,-0.46};     //calibration curve of LPG from the MQ2 datasheet, Curve data format:{ x, y, slope}
float       COCurve[3]            =      {2.3,0.72,-0.34};     //calibration curve of CO from the MQ2 datasheet
float       SmokeCurve[3]         =      {2.3,0.53,-0.44};     //calibration curve of Smoke from the MQ2 datasheet
float       CH4Curve[3]           =      {3.3, 0,  -0.38};     //calibration curve of CH4 from the MQ6 datasheet
float       H2Curve[3]            =      {2.3,0.93,-1.44};     //calibration curve of H2 from the MQ8 datasheet

/****************** MQResistanceCalculation ****************************************/
float MQResistanceCalculation(int raw_adc) {
  return (((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/*****************************  MQRead *********************************************/
float MQRead(int mq_pin) {
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}

/*****************************  MQGetPercentage **********************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
///////////////////////////////////////////////////////////////////////////////////
void setup() {

  pinMode(analogpin, INPUT);
  pinMode(S0,OUTPUT);
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
  pinMode(S3,OUTPUT);  
  
  Serial.begin(115200);
  
}

void loop() {

  digitalWrite(S0,LOW);
  digitalWrite(S1,LOW);
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW); {
    //Serial.print("2#MQ2: "); 
    Ro = 1.11;
    Serial.print( "LPG: " );
    Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,LPGCurve));
    Serial.println( "ppm" );
    Serial.print( "CO: " );
    Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,COCurve));
    Serial.println( "ppm" );
    Serial.print( "Smoke: " );
    Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,SmokeCurve));
    Serial.println( "ppm" ); 
  }
  
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW); {
   //Serial.print("2#MQ6: ");
   Ro = 0.54;
   Serial.print( "CH4: " );
   Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,CH4Curve));
   Serial.println( "ppm" );
  }
  
  digitalWrite(S0,LOW);
  digitalWrite(S1,HIGH);
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW); { 
    //Serial.print("3#MQ8: ");
    Ro = 0.22;
    Serial.print( "Hydrogen: " );
    Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,H2Curve));
    Serial.println( "ppm" );
  }
  Serial.println( " " );
  delay(2000);
  
  }
