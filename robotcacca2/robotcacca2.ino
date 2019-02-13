#include <Wire.h>

#define SLAVE_ADDRESS 0x04

#define FRONT_TRIG 14
#define FRONT_ECHO 15
#define RIGHT_TRIG 16
#define RIGHT_ECHO 17
#define LEFT_TRIG 2
#define LEFT_ECHO 3


#define SAVEDATA 10// 過去何回分の値を保存するか
#define K_V  0.5                // 赤外線強度の係数
#define K_DELTA 0.4             // 赤外線方角の係数
#define COART_WIDTH 122         // コートの横幅
#define COART_DEPTH 172         // コートの奥行幅
#define BODY_WIDTH 22           // マシンの横幅
#define BODY_DEPTH 22           // マシンの奥行幅
#define THRESHOLD_POS_DCSN 60     // 自己位置推定のための閾値
#define THRESHOLD_POS_DCSN2 60   // 自己位置推定のための閾値2
#define TRUE  1
#define FALSE 0
#define K_PHI 8.5             // ゴール方角の係数



int i;
char val;
int flag=0,index=0;
int readFromEv3[8];
char writeFromArduino[8];
char buf[8];
int deg = 0;


int irdirc[SAVEDATA];       // Input: 赤外線ボールの方向の履歴(0が最新)
int irdist[SAVEDATA];           // Input: 赤外線ボールの距離の履歴(0が最新)
int cmps=0;                // Input: コンパス(0-360)
int us[4];              // Input: 超音波センサ(0:前 1:左 2:後 3:左)(単位はcm)
int leftMotor  = 0;        // Output:左モーター(-100-100)
int rightMotor = 0;             // Output:右モーター(-100-100)
int dribbler = 1;         // Output:ドリブラー(0:やらない, 1:やる)
int dribble = 0;          // Input: ドリブル中か(0:違う, 1:そう)
int kick=0;                     // Output:キッカー(0:やらない, 1:やる)
int m_width=50;              // Variable:自己位置（横）
int m_depth=50;              // Variable:自己位置（奥行）
int waygoal = 0;                // Variable:ゴールの方向(最初においた向きを記憶)
int fwd, rht;                   // fwd:前を向いているか、rht:右を向いているか
int angle_trans[180] = {  0,  6, 10, 13, 19, 23, 26, 31, 35, 38, 42, 45, 49, 53, 57, 60, 64, 68, 71, 74, 77, 80, 84, 87, 90,
                         94, 97,101,105,108,112,117,122,126,129,132,135,138,142,146,149,152,154,157,160,164,167,169,171,175,
                        178,181,184,187,189,191,193,196,198,200,201,203,204,206,208,210,210,211,212,212,212,212,213,214,214,
                        215,215,216,217,217,217,218,218,218,218,218,218,218,218,218,218,219,219,219,219,220,220,220,220,220,
                        220,220,221,221,221,221,221,221,221,221,221,222,222,222,222,222,222,223,223,223,223,224,224,224,226,
                        226,226,226,227,227,227,228,228,228,229,229,230,231,231,231,232,232,233,234,236,238,239,241,242,244,
                        246,249,253,255,257,259,262,264,267,271,274,277,281,284,288,292,295,299,303,307,311,315,319,324,328,
                        331,335,340,344,348};



/************************************/
/* 超音波センサから距離を取得する by.ohira
/************************************/
float getDistance(int trig, int echo){
  // 超音波の出力終了
  digitalWrite(trig,LOW);
  delayMicroseconds(1);
  // 超音波を出力
  digitalWrite(trig,HIGH);
  delayMicroseconds(11);
  // 超音波を出力終了
  digitalWrite(trig,LOW);
  // 出力した超音波が返って来る時間を計測
  int t = pulseIn(echo,HIGH);
  // 計測した時間と音速から反射物までの距離を計算
  float distance = t*0.017;
  
  return distance;
}

/************************************/
/* EV3から受け取ったセンターの値を変数に格納する
/************************************/
void inputSensor(){
  irdirc[0] = readFromEv3[0];
  irdist[0] = readFromEv3[1] + 128;
  
  if(readFromEv3[3] == 0){
    cmps = angle_trans[(readFromEv3[2]+180)/2]-180;
  }
  else if(readFromEv3[3] > 0){
    cmps = readFromEv3[3] + 127;
  }
  else if(readFromEv3[3] < 0){
    cmps = readFromEv3[3] - 128;
  }
  
  us[0] = getDistance(FRONT_TRIG, FRONT_ECHO); // 前
  us[1] = getDistance(LEFT_TRIG, LEFT_ECHO); // 左
  us[2] = readFromEv3[4] + 128; // 後ろ
  us[3] = getDistance(RIGHT_TRIG, RIGHT_ECHO); // 右

  //dribble = readFromEv3[5];

  static int aa=0;
  if(aa % 100 == 0){
  /*
  Serial.print("irdirc=");
  Serial.println(irdirc[0]);
  Serial.print("irdist=");
  Serial.println(irdist[0]);
  Serial.print("leftmotor=");
  Serial.println(leftMotor);
  Serial.print("rightmotor=");
  Serial.println(rightMotor);
  Serial.print("dribble=");
  Serial.println(dribble);
  */
/*  Serial.print("cmps=");
  Serial.println(cmps);
  Serial.print("us={");
  Serial.print(us[0]);
  Serial.print(",");
  Serial.print(us[1]);
  Serial.print(",");
  Serial.print(us[2]);
  Serial.print(",");
  Serial.print(us[3]);
  Serial.println("}");
  */
  }
  aa++;
 
}

/************************************/
/* 自己位置推定を行う
/************************************/

void myPosition(){ // 自己位置推定
  
  double cmpsPi;
  double cosPi, sinPi;
  double est[4][2]; // センサ4つから予想される自己位置（奥行と横) (図170716の①と②)
  int i, j;
  int same1[4][4], same2[4][4];     // 各センサから推定される①と②が、他と一致したか否か
  int samecount1[4], samecount2[4]; // 各センサから推定される①と②が、他と一致した数
  int maxindex1, maxindex2;         // samecountにおいて最も大きい値の添え字
  int sensor1or2[4]={0,0,0,0};      // 各センサが①を表してるか②を表しているか
  int stable[2]={0,0};                     // 安定して得られたデータか否か
  static int myposStable[2];            // 過去のデータ保存場所 (推定座標)(信用できる値しか保存しない）
  // static int usSave[SAVEDATA][4];       // 過去のデータ保存場所 (赤外線実測値)
  int temp, tempindex;
  int maxcount1, maxcount2;

  /*
  static int aa=0;
  if(aa % 100 == 0){
  Serial.print("m_depth=");
  Serial.println(m_depth);
  Serial.print("m_width=");
  Serial.println(m_width);
  }
  aa++;
  */
  m_width = 0;
  m_depth = 0; 



  // -180～180を-Pi～Piに変換
  cmpsPi = (cmps-waygoal) * PI / 180;

  cosPi = cos(cmpsPi);
  sinPi = sin(cmpsPi);
  
  // ロボが前を見ていれば fwd=1, 後ろを見ていれば fwd=-1
  if(cosPi >= 0){
    fwd = 1;
  }else{
    fwd = -1;
  }
  
  // ロボが右を見ていれば rht=1, 左を見ていれば rht=-1
  if(sinPi >= 0){
    rht = 1;
  }else{
    rht = -1;
  }

  // 各センサから推定される①と②を計算
  est[0][0] = (us[0]+BODY_DEPTH/2) * fabs(cosPi);
  est[0][1] = (us[0]+BODY_DEPTH/2) * fabs(sinPi);
  est[1][0] = (us[1]+BODY_WIDTH/2) * fabs(sinPi);
  est[1][1] = COART_WIDTH - (us[1]+BODY_WIDTH/2) * fabs(cosPi);
  est[2][0] = COART_DEPTH - (us[2]+BODY_WIDTH/2) * fabs(cosPi);
  est[2][1] = COART_WIDTH - (us[2]+BODY_WIDTH/2) * fabs(sinPi);
  est[3][0] = COART_DEPTH - (us[3]+BODY_DEPTH/2) * fabs(sinPi);
  est[3][1] = (us[3]+BODY_DEPTH/2) * fabs(cosPi);


  /*
  static int aa=0;
  if(aa % 100 == 0){
  Serial.print("fwd1=");
  Serial.print(est[0][0]);
  Serial.print(",2=");
  Serial.println(est[0][1]);
  Serial.print("left1=");
  Serial.print(est[1][0]);
  Serial.print(",2=");
  Serial.println(est[1][1]);
  Serial.print("bck1=");
  Serial.print(est[2][0]);
  Serial.print(",2=");
  Serial.println(est[2][1]);
  Serial.print("rht1=");
  Serial.print(est[3][0]);
  Serial.print(",2=");
  Serial.println(est[3][1]);
  Serial.println("");
  }
  aa++;
  */
  

  // 各センサから推定された①の一致数をカウント
  for(i=0;i<=3;i++){
    samecount1[i]=0;
    for(j=0;j<=3;j++){
      if(pow(est[i][0]-est[j][0],2) < THRESHOLD_POS_DCSN){
        same1[i][j] = 1;
        samecount1[i]++;
      }else{
        same1[i][j] = 0;
      }
    }
  }

  // 他のセンサと最も①の一致数の多いセンサ番号を取得
  maxindex1=0;
  for(i=1;i<=3;i++){
      if(samecount1[maxindex1] < samecount1[i]){
          maxindex1=i;
      }
  }

  // 各センサから推定された②の一致数をカウント  
  for(i=0;i<=3;i++){
    samecount2[i]=0;
    for(j=0;j<=3;j++){
      if(pow(est[i][1]-est[j][1],2) < THRESHOLD_POS_DCSN){
        same2[i][j] = 1;
        samecount2[i]++;
      }else{
        same2[i][j] = 0;
      }
    }
  }

  // 他のセンサと最も②の一致数の多いセンサ番号を取得
  maxindex2=0;
  for(i=1;i<=3;i++){
      if(samecount2[maxindex2] < samecount2[i]){
          maxindex2=i;
      }
  }

  maxcount1 = samecount1[maxindex1];
  maxcount2 = samecount2[maxindex2];

  // ②が4つ一致している場合、一致している②を信用、①は過去の値を使う
  if(maxcount2 == 4){
    temp=0;
    for(i=0;i<=3;i++){
      temp += est[i][1];
    }
    temp /= 4;
    m_width = (COART_WIDTH/2-temp)*rht;
    myposStable[1] = m_width;
  }
  
  
  // ②が3つ一致している場合、一致している②を信用、①は過去と比較する
  if(maxcount2 == 3){
    temp=0;
    for(i=0;i<=3;i++){
      if(same2[maxindex2][i]==1){
        temp += est[i][1];
      }else{
        tempindex = i;
      }
    }
    temp /= 3;
    m_width = (COART_WIDTH/2-temp)*rht;
    myposStable[1] = m_width;
    temp = (COART_DEPTH/2-est[tempindex][0])*fwd;
    if(pow(myposStable[0]-temp,2) < THRESHOLD_POS_DCSN2){
      m_depth = temp;
      myposStable[0] = m_depth;
    }
  }

  // ①が4つ一致している場合、一致している①を信用、②は過去の値を使う
  if(maxcount1 == 4){
    temp=0;
    for(i=0;i<=3;i++){
      temp += est[i][0];
    }
    temp /= 4;
    m_depth = (COART_DEPTH/2-temp)*fwd;
    myposStable[0] = m_depth;
    
  }
  
  // ①が3つ一致している場合、一致している①を信用、②は過去と比較する
  if(maxcount1 == 3){
    temp=0;
    for(i=0;i<=3;i++){
      if(same1[maxindex1][i]==1){
        temp += est[i][0];
      }else{
        tempindex = i;
      }
    }
    temp /= 3;
    m_depth = (COART_DEPTH/2-temp)*fwd;
    myposStable[0] = m_depth;
    temp = (COART_WIDTH/2-est[tempindex][1])*rht;
    if(pow(myposStable[1]-temp,2) < THRESHOLD_POS_DCSN2){
      m_width = temp;
      myposStable[1] = m_width;
    }
  }

  // ①が2つ以上、②が2つ以上で全てのセンサが①か②に対応していれば、①、②ともに信用
  if(maxcount1 >= 2 && maxcount2 >= 2){
    temp=0;
    for(i=0;i<=3;i++){
      if(same1[maxindex1][i]==1 || same2[maxindex2][i]==1){
        temp++;
      }else{
        break;
      }
    }
    // 全てのセンサがいずれかに対応していれば
    if(temp==4){
      temp=0;
      for(i=0;i<=3;i++){
        if(same1[maxindex1][i]==1){
          temp += est[i][0];
        }
      }
      temp /= maxcount1;
      m_depth = (COART_DEPTH/2-temp)*fwd;
      myposStable[0] = m_depth;

      temp=0;
      for(i=0;i<=3;i++){
        if(same2[maxindex2][i]==1){
          temp += est[i][1];
        }
      }
      temp /= maxcount2;
      m_width = (COART_WIDTH/2-temp)*rht;
      myposStable[1] = m_width;
    }
  }
  if(m_depth == 0){
    m_depth = myposStable[0];
  }
  if(m_width == 0){
    m_width = myposStable[1];
  }
}  




/************************************/
/* ドリブルしているか判定する
/************************************/
void judgeDribble(){
  /*前方距離センサかIRSeekerの情報から判定する？*/
  if(readFromEv3[5] == 1){
    dribble = TRUE;
  }
}

/************************************/
/* ボールを追いかける
/************************************/
void chaseBall(){

  if(irdirc[0] != 0 ){
    //leftMotor = (char)( K_V * 50 + K_DELTA * (irdirc[0] - 5) );
    //rightMotor = (char)( K_V * 50 - K_DELTA * (irdirc[0] - 5) );
    static int aa=0;
  if(aa % 100 == 0){
  Serial.println(irdist[0]);
  }
  aa++;
    
    leftMotor = (char)( K_V * ((255-irdist[0])/2.55) + K_DELTA * ((irdirc[0] - 5)*25) );
    rightMotor = (char)( K_V * ((255-irdist[0])/2.55) - K_DELTA * ((irdirc[0] - 5)*25) );
  
  }
  else{
    leftMotor = (char)30;
    rightMotor = (char)-30;
  }
}


/************************************/
/* ドリブルしながらゴールへ向かう
/************************************/
void shoot(int md, int mw, int compass){
  /*
  double gap;
  if(mw == 0){
    gap = waygoal - compass; 
  }
  else if(rht == 1){
    gap = - ( 90 - 180 * PI / atan( (COART_DEPTH /2 - md ) / mw) ) - (waygoal - compass);
      }
  else{
    gap =  ( 90 - 180 * PI / atan( (COART_DEPTH /2 - md ) / (- mw) ) ) - (waygoal - compass);
    if(gap > 180){
      gap = gap - 360;
    }
  }
  
  if(gap < -180){
    gap = gap + 360;
  }
  else if(gap >= 180){
    gap = gap - 360;
  }
      
  leftMotor  = - ( K_V * 50 + K_PHI * gap );
  rightMotor = - ( K_V * 50 - K_PHI * gap );
  */
  leftMotor  = 30;
  rightMotor = 30;
}



void outputMotor(){
  writeFromArduino[6] = leftMotor;
  writeFromArduino[7] = rightMotor;
}

/************************************/
/* setup(起動時に1度だけ呼ばれる)
/************************************/
void setup() 
{
  
    Serial.begin(9600);         // start serial for output
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
   

  pinMode(FRONT_TRIG,OUTPUT);
  pinMode(FRONT_ECHO,INPUT);
  pinMode(RIGHT_TRIG,OUTPUT);
  pinMode(RIGHT_ECHO,INPUT);
  pinMode(LEFT_TRIG,OUTPUT);
  pinMode(LEFT_ECHO,INPUT);
  
  inputSensor();
  waygoal = cmps;
  
                                  /*ドリブルを始める*/
}


/************************************/
/* loop(繰り返し呼ばれるメイン処理)
/************************************/
void loop()
{
  for(int i = 0; i < SAVEDATA - 1; i++){
    irdirc[i+1] = irdirc[i];
    irdist[i+1] = irdist[i];
  }
  
  inputSensor();

  myPosition();
  
  judgeDribble();
  

  if(dribble == FALSE){
    chaseBall();
  }
  else{
    shoot(m_depth, m_width, cmps);
  }

  outputMotor();
}


/************************************/
/* EV3からシリアル受信する
/************************************/
void receiveData(int byteCount)
{
    i = 0;
    while(Wire.available()>0) 
    {
      val=(char)Wire.read();
      readFromEv3[i] = (int)val;
      flag=1;
      i++;
    }
}




/************************************/
/* EV3へシリアル送信する
/************************************/
// callback for sending data
void sendData()
{
  Wire.write(writeFromArduino,8);
}
