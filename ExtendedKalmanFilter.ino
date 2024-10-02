#include <Wire.h>
#define pi 3.14159
#define R 6378137 //Dünya’nın yarıçapını R = 6378137 metre olarak kabul edebiliriz.
#define SIZE 10

const float Deg2rad = pi/180;
const float Gravity = -9.81;
//GPS için
// enlem = latitude, boylam = longitude , irtifa = altitude 
byte previousLetter,currentLetter,flagIndex,counter, storage[53];
float longitudeRad,latitudeRad,altitudeM;
float PECEF[3]={0,0,0};//P konum demek ve ECEF kendi frame
float Pref[3] = {4209450, 2309522, 4198728}; //{Xref, Yref, Zref} aldığımız referan kendi ilk konumuz (ben evim konumu aldım ama ECEF frame'ide)
float TransformationMatrix[9];

// gyroskop için 
float RateRoll, RatePitch, RateYaw; //açısal hız 
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;// açısal hız kalibrasyonu için  
int RateCalibrationNumber; 

// AccX ivmeölçerden ivme için -- DeltaVelX ivme integrali için delta hız değişkeni
float AccX, AccY, AccZ, DeltaVelX, DeltaVelY, DeltaVelZ;

float deltat = 0;  //saniye cinsinden döngü süresi
unsigned long now = 0, last = 0; //micros() zamanlayıcı için



struct Quaternion {
  float w, x, y, z;
};

struct Velocity {
  float N, E, D;
};
struct Position {
  float N, E, D;
};

//IMU değişkenleri için ilk değer verme 
Quaternion stateQuat = {1,0,0,0};
Quaternion deltaQuat = {0,0,0};
Quaternion quat = {1,0,0,0};

Velocity stateVel = {0,0,0};
Velocity deltaVelNav = {0,0,0};

Position statePos = {0,0,0};
//GPS için
Velocity VelocityMeasurement;
Position positionMeasurement;
Position positionPrevios;

//DCM matrisi ( Body to Navigation)
float Tbn[3][3] = {{0,0,0},
                   {0,0,0},
                   {0,0,0}
  };

float fJacobian[SIZE][SIZE]={{deltaQuat.w,0,0,0,0,0,0,0,0,0},
                             {0,deltaQuat.x,0,0,0,0,0,0,0,0},
                             {0,0,deltaQuat.y,0,0,0,0,0,0,0},
                             {0,0,0,deltaQuat.z,0,0,0,0,0,0},
                             {0,0,0,0,1,0,0,0,0,0},
                             {0,0,0,0,0,1,0,0,0,0},
                             {0,0,0,0,0,0,1,0,0,0},
                             {0,0,0,0,deltat,0,0,1,0,0},
                             {0,0,0,0,0,deltat,0,0,1,0},
                             {0,0,0,0,0,0,deltat,0,0,1}
};
float P[SIZE][SIZE]={{1,0,0,0,0,0,0,0,0,0},
                     {0,1,0,0,0,0,0,0,0,0},
                     {0,0,1,0,0,0,0,0,0,0},
                     {0,0,0,1,0,0,0,0,0,0},
                     {0,0,0,0,1,0,0,0,0,0},
                     {0,0,0,0,0,1,0,0,0,0},
                     {0,0,0,0,0,0,1,0,0,0},
                     {0,0,0,0,0,0,0,1,0,0},
                     {0,0,0,0,0,0,0,0,1,0},
                     {0,0,0,0,0,0,0,0,0,1}
};
float Q[SIZE][SIZE]={{0.001,0,0,0,0,0,0,0,0,0},
                     {0,0.001,0,0,0,0,0,0,0,0},
                     {0,0,0.001,0,0,0,0,0,0,0},
                     {0,0,0,0.001,0,0,0,0,0,0},
                     {0,0,0,0,0.001,0,0,0,0,0},
                     {0,0,0,0,0,0.001,0,0,0,0},
                     {0,0,0,0,0,0,0.001,0,0,0},
                     {0,0,0,0,0,0,0,0.001,0,0},
                     {0,0,0,0,0,0,0,0,0.001,0},
                     {0,0,0,0,0,0,0,0,0,0.001}
};
float Rvariance[SIZE][SIZE]={{0.01,0,0,0,0,0,0,0,0,0},//sistem varyansları şuan resgele verdim  
                             {0,0.01,0,0,0,0,0,0,0,0},
                             {0,0,0.01,0,0,0,0,0,0,0},
                             {0,0,0,0.01,0,0,0,0,0,0},
                             {0,0,0,0,0.325,0,0,0,0,0},
                             {0,0,0,0,0,0.333,0,0,0,0},
                             {0,0,0,0,0,0,0.432,0,0,0},
                             {0,0,0,0,0,0,0,0.342,0,0},
                             {0,0,0,0,0,0,0,0,0.56,0},
                             {0,0,0,0,0,0,0,0,0,0.55}
};

float H[SIZE][SIZE]={{1,0,0,0,0,0,0,0,0,0},
                     {0,1,0,0,0,0,0,0,0,0},
                     {0,0,1,0,0,0,0,0,0,0},
                     {0,0,0,1,0,0,0,0,0,0},
                     {0,0,0,0,1,0,0,0,0,0},
                     {0,0,0,0,0,1,0,0,0,0},
                     {0,0,0,0,0,0,1,0,0,0},
                     {0,0,0,0,0,0,0,1,0,0},
                     {0,0,0,0,0,0,0,0,1,0},
                     {0,0,0,0,0,0,0,0,0,1}
};
float eye[SIZE][SIZE]={{1,0,0,0,0,0,0,0,0,0},
                       {0,1,0,0,0,0,0,0,0,0},
                       {0,0,1,0,0,0,0,0,0,0},
                       {0,0,0,1,0,0,0,0,0,0},
                       {0,0,0,0,1,0,0,0,0,0},
                       {0,0,0,0,0,1,0,0,0,0},
                       {0,0,0,0,0,0,1,0,0,0},
                       {0,0,0,0,0,0,0,1,0,0},
                       {0,0,0,0,0,0,0,0,1,0},
                       {0,0,0,0,0,0,0,0,0,1}
};

float denominator[SIZE][SIZE];
float numerator[SIZE][SIZE];
float K[SIZE][SIZE];
float Z[SIZE];

void readIMU(void);//MPU6050 sensörden ivme ve açısal hızı hesapla
Quaternion Rot2quat(float roll, float pitch, float yaw);//gyroskop'tan açısal hızları Quaternion'a dönüştür
Quaternion QuatMult(Quaternion A,Quaternion B);// quaternion multiply (çarpmak)
Quaternion normQuat(Quaternion stateQuat);//quaternion normalize (normalleştirmek)
void Quat2Tbn(Quaternion stateQuat, float Tbn[3][3]);// quaternion to DCM matrisi
void stateVelAndPos(float deltat);//ivmeölçerden aldığımız ivmeyi integralini alarak hıza ve konuma dönüştür
Position positionNED(void); // GPS'ten LLA dan NED eksenine konumu dönüştür hesapla (ana kod içerisinde veriler çekildi)
void ZSensor(float Z[SIZE], Velocity v, Position p, Quaternion q);// GPS'ten çekilen verileri Z matrisine yerleştir 
void UncertaintyOfThePrediction(float fJacobian[SIZE][SIZE], float P[SIZE][SIZE], float Q[SIZE][SIZE]);// kalman Prediction step P matrisi işle ve yeni P'ye yerleştir
void kalmanGainDenominator(float H[SIZE][SIZE], float P[SIZE][SIZE], float Rvariance[SIZE][SIZE],float denominator[SIZE][SIZE]);//kalman kazancı paydası bu
void kalmanGainNumerator(float P[SIZE][SIZE], float H[SIZE][SIZE], float numerator[SIZE][SIZE]);//kalman kazancı payı bu
void inverseMatrix(float matrix[SIZE][SIZE],float inverse[SIZE][SIZE]);//kalman kazancı paydası tersini bul
void kalmanGain( float numerator[SIZE][SIZE], float denominator[SIZE][SIZE], float K[SIZE][SIZE]);//kalman kazancı bul ve K matrisine yerleştir
void statesUpdate(float Z[SIZE], Velocity stateVel,Position statePos, Quaternion stateQuat, float K[SIZE][SIZE], float H[SIZE][SIZE]);//durumları güncelle
void UncertaintyOfThePredictionUpdate(float eye[SIZE][SIZE], float P[SIZE][SIZE], float K[SIZE][SIZE], float H[SIZE][SIZE]);//P matrisi güncelle 


void setup(){

     Serial.begin(9600);
     Wire.begin(); //burada herhangi bir adres yazmıyoruz çünkü biz Master -- haberleşme protokolu başla demek
     Serial2.begin(9600); //GPS için
     delay(250);

     //İvmeölçer başlat 
     Wire.beginTransmission(0x68);
     Wire.write(0x6B);// Slave'ten 0x6B ilk adrese sahip olan dizi yaz demek
     Wire.write(0x00); //  güç modunda çalıştırdığımızda tüm değerleri sıfıra çekmesi için 
     Wire.endTransmission();
 
    //açısal hızı kalibre et -- 4 sanye buyunca 4000 veri al demek
     for (RateCalibrationNumber = 0; RateCalibrationNumber < 4000;RateCalibrationNumber++){
      readIMU();
      RateCalibrationRoll += RateRoll;
      RateCalibrationPitch += RatePitch;
      RateCalibrationYaw += RateYaw;
       
      delay(1);//1ms bekle arayla veri topla
     }
     
     RateCalibrationRoll /= 4000;
     RateCalibrationPitch /= 4000;
     RateCalibrationYaw /= 4000;

     //GPS NMEA protokolu için ilk harf al ve sakla (her döngüde yeni değer ile  takas yabılması için )
      if (Serial2.available())
         previousLetter=Serial2.read();
     
}


void loop(){
    

    
  if (Serial2.available()) {// eğer GPS veri veriyorsa işlem yap 
    currentLetter=Serial2.read();// veri oku ve ikinci harfa yerleştir

    if (flagIndex==0){ //istenilen verileri çemesi bitince bir bayrak bitti ise 0 olur ve sürekli takas yapar G ve A üst üste gelenekadar 
         if (previousLetter=='G' && currentLetter=='A'){ //$GPGGA NMEA'sı geldi mi soruyoruz geldiğise veri 
                                                      //kullanmaya başla istediğim verileri burada
            counter=53;
            flagIndex=1;
          }else {
            previousLetter=currentLetter;
          }

    }else {
      storage[53-counter]=currentLetter;
      counter--;
      if (counter==0){
        flagIndex=0;
        //storage[1] ASCII Kod 
        //numarası gelir biz normal sayı çıkması için -48 yaparız ardından onlar 
        //basamak olması için 10 ile çarpıyoruz ve birleri artırıyoruz aynı şekil-- örneği 10*(storage[11]-48) + storage[12]-48

        now = micros();
        deltat = (now - last) * 1.0e-6; //son güncellemeden bu yana geçen zaman (saniye)
        last = now;

        // Yer Merkezli Enlem-Boylam (LLA) Referans Modeli oluşturma                                                                                                                                                                                             // pi/180 normal dönüşüm - 60 dakikayı dereceye dönüştürmek 100000 ise basamak kaydırdık  
        latitudeRad=float((10*(storage[11]-48) + storage[12]-48)*6000000 + (storage[13]-48)*1000000 + (storage[14]-48)*10000 + (storage[16]-48)*10000 + (storage[17]-48)*1000 + (storage[18]-48)*100 + (storage[19]-48)*10 + (storage[20]-48))*pi/1080000000.0; // 60*180*100000=1080000000.0 //[rad]

        longitudeRad=float((100*(storage[24]-48)+10*(storage[25]-48)+ storage[26]-48)*6000000 + (storage[27]-48)*1000000 + (storage[28]-48)*100000 + (storage[30]-48)*10000 + (storage[31]-48)*1000 + (storage[32]-48)*100 + (storage[33]-48)*10 + (storage[34]-48))*pi/1080000000.0;//[rad]

        altitudeM = float(100*(storage[48]-48) + 10*(storage[49]-48) + (storage[50]-48) + (storage[52]-48)/10.0);//[M]

        // NED ekseninde konum hesabı
        positionPrevios = positionMeasurement;
        positionMeasurement = positionNED();//[M]
        VelocityMeasurement.N = (positionMeasurement.N - positionPrevios.N)/deltat; //[M/S] gps'ten konumu alıp hıza dönüştürmek ( gps'im hızı veremiyor olduğu için öyle yaptım)
        VelocityMeasurement.E = (positionMeasurement.E - positionPrevios.E)/deltat;
        VelocityMeasurement.D = (positionMeasurement.D - positionPrevios.D)/deltat;

        readIMU();
        RateRoll -= RateCalibrationRoll;
        RatePitch -= RateCalibrationPitch;
        RateYaw -= RateCalibrationYaw;

        //sates(x)k+1 = stat(x,u)k
        deltaQuat = Rot2quat(RateRoll, RatePitch, RateYaw);
        Quaternion quat_temp = QuatMult(stateQuat, deltaQuat);
        stateQuat = normQuat(quat_temp);

        Quat2Tbn(stateQuat, Tbn);//DCM matrisi
  
        stateVelAndPos(deltat);//sates(x)k+1 = stat(x,u)k
        //Prediction
        UncertaintyOfThePrediction(fJacobian, P, Q);//direkt sonuç P matrise kayıt ediliyor 

        // Güncelleme adımı 
        //Ölçümler -- Z matrisi                              //quat olması lazım ama daha belli değil burası kesinleştirmedim
        ZSensor(Z, VelocityMeasurement, positionMeasurement, stateQuat);//GPS'ten alınan veriler direkt Z matrisine aktarılıyor 

        //Kalman kazancı hesabı
        //K = P*H'/(H*P*H' + R);
        kalmanGainDenominator( P, H, Rvariance, denominator);//denominator=H*P*H' + R
        kalmanGainNumerator( P, H, numerator);//numerator = P*H'
        kalmanGain(numerator, denominator, K);//K = numerator/denominator çıkan değer K matrisine yerleştir

        //states(i+1) = states(i) + K*(Z - H*states(i));
        statesUpdate(Z, stateVel, statePos, stateQuat, K,H);

        //P(i+1) = (eye(10) - K*H)*P(i);
        UncertaintyOfThePredictionUpdate( eye, P, K, H);

        // Serial.print(" deltat [s] : ");
        // Serial.print(deltat,5); //GPS ne kadar hızda işlediğini görmek için 

        //Duruş verileri göster
        Serial.print(" stateQuat.w : ");
        Serial.print(stateQuat.w);
        Serial.print(" stateQuat.x : ");
        Serial.print(stateQuat.x);
        Serial.print(" stateQuat.y : ");
        Serial.print(stateQuat.y);
        Serial.print(" stateQuat.z : ");
        Serial.print(stateQuat.z);

        //Hız verileri göster 
        Serial.print(" stateVel.N [m/s] : ");
        Serial.print(stateVel.N);
        Serial.print(" stateVel.E [m/s] : ");
        Serial.print(stateVel.E);
        Serial.print(" stateVel.Z [m/s] : ");
        Serial.print(stateVel.D);

        // Konum verileri göster 
        Serial.print(" statePos.N [m] : ");
        Serial.print(statePos.N);
        Serial.print(" statePos.E [m] : ");
        Serial.print(statePos.E);
        Serial.print(" statePos.D [m] : ");
        Serial.println(statePos.D);
  
 
      }
    }
  }
 
}



void readIMU(void){
  //Buradaki kullanılan tüm adresler MPU6050 sensör datasheet'inden detaylıca gözlemleyebilirsiniz 
  Wire.beginTransmission(0x68);// bu dört satırı alçak geçiş filtresini açmak
  Wire.write(0x1A);
  Wire.write(0x05); //5 byte yaz
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //burada 8g'yi ayarlamak (çözünürlüğü ayarlamak)
  Wire.write(0x1C);//ivmeölçer çıkışlarını 1C adresinde(register) saklanır bu adresten gelen ölçüm aralığı AFS_SEL'e (JİROSKOP GÜRÜLTÜ PERFORMANSI) göre 2 yani 10 yani 16 bit aralığ o da bizim sonraki satır tanımı
  Wire.write(0x10);//0x10 adresi iki tabanlıda gösterimi 0001 0000 o da 16 bit eder //
  Wire.endTransmission(); 

  Wire.beginTransmission(0x68); //ivmeölçer değerlerinden çekmesini başlamak
  Wire.write(0x3B); // ivmeölçer değerleri ilk saklanan değerin adresi başla diyorum AccX (ACCEL_XOUT_H)
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6); // bu fonkisyon 0x68 dresten iletilen verilerinden 6 baytı talep edeceğiz -- Normalde biz Master olarak biz veri göndeririz ama bu fonkisyonda Slave'i Master yaptık ve Master'ı Slave yaptık

  //Yüksek biti (MSB) 8 bit kaydırarak LSB bitinden veri çekmek
  int16_t AccXLSB = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  int16_t AccYLSB = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int16_t AccZLSB = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
   
  //jiroskop çıkışını yapılandırın ve sensörden dönüş hızı ölçümlerini çekin //bunu anlamadım
  Wire.beginTransmission(0x68);  
  Wire.write(0x1B);  // jiroskop çözünlüğü ayarlanması
  Wire.write(0x08); // FS_SEL değeri 4. bitte bunun için biz 16ye çevirdik 0x08 çıktı
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6); 

  Wire.beginTransmission(0x68); 
  Wire.write(0x43); //jiroskop değerleri hesaplamak için dizinin ilk adresi --- ivmeölçer 0x3B aynı mantığı
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);

 //Yüksek biti (MSB) 8 bit kaydırarak LSB bitinden veri çekmek
  int16_t GyroXLSB = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  int16_t GyroYLSB = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  int16_t GyroZLSB = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  RateRoll = (float)GyroXLSB/65.5; //[derece/s]
  RatePitch = (float)GyroXLSB/65.5;
  RateYaw = (float)GyroXLSB/65.5;
  RateRoll = RateRoll*Deg2rad; //[rad/s]
  RatePitch = RatePitch*Deg2rad;
  RateYaw = RateYaw*Deg2rad;

  AccX = (float)AccXLSB/4096;// fiziksel değere dönüştürmek
  AccY = (float)AccYLSB/4096;
  AccZ = (float)AccZLSB/4096; //[g] 

  AccX = AccX*9.81 + 0.58; //[m/s^2] // artı ve eksi sayılar sensör kalibrasyonu için (0'ra çekmesi için) örneği -0.582
  AccY = AccY*9.81 - 0.02;
  AccZ = AccZ*9.81 + 0.70;

  DeltaVelX = AccX*deltat;//[m/s]
  DeltaVelY = AccY*deltat;
  DeltaVelZ = AccZ*deltat;

 
 

}


Position positionNED(void){
  Position Posned;
  //LLA’den ECEF Dönüşümü ve referans noktası çıkarma
  //𝑋=(𝑅+𝐻)*cos⁡(𝜙)*cos⁡(𝜆) - Pref
  PECEF[0] = (R + altitudeM)*cos(latitudeRad)*cos(longitudeRad) - Pref[0];//Pref[0]=4209452 X yönü
  //𝑌=(𝑅+𝐻)*cos⁡(𝜙)*sin(𝜆) 
  PECEF[1] = (R + altitudeM)*cos(latitudeRad)*sin(longitudeRad) - Pref[1];//Pref[1]=2309517 Y yönü
  //Z=(𝑅+𝐻)*sin⁡(𝜙)
  PECEF[2] = (R + altitudeM)*sin(latitudeRad) - Pref[2];//Pref[2]=41987 Z yönü
  //ECEF ekseninde (metre cinsinden) verilen noktanın NED eksenine dönüştüren Transformasyon matrisi hesabı
  TransformationMatrix[0] = -sin(latitudeRad)*cos(longitudeRad); TransformationMatrix[1] = -sin(latitudeRad)*sin(longitudeRad); TransformationMatrix[2] = cos(latitudeRad);
  TransformationMatrix[3] = -sin(longitudeRad);                  TransformationMatrix[4] = cos(longitudeRad);                   TransformationMatrix[5] = 0;
  TransformationMatrix[6] = -cos(latitudeRad)*cos(longitudeRad); TransformationMatrix[7] = -cos(latitudeRad)*sin(longitudeRad); TransformationMatrix[8] = -sin(latitudeRad);
  // NED ekseninde konum hesabı PNED = TransformationMatrix(PECEF - Pref)
  Posned.N = TransformationMatrix[0]*PECEF[0] + TransformationMatrix[1]*PECEF[1] + TransformationMatrix[2]*PECEF[2];
  Posned.E = TransformationMatrix[3]*PECEF[0] + TransformationMatrix[4]*PECEF[1] + TransformationMatrix[5]*PECEF[2];
  Posned.D = TransformationMatrix[6]*PECEF[0] + TransformationMatrix[7]*PECEF[1] + TransformationMatrix[8]*PECEF[2];

 return Posned;
}


// Quaternion duruş hesabı ama sensör füzyondan sonra düşük frekansta çalıştığım için değiştirmem gerekir bu sadece yüksek frekansta geçerli
struct Quaternion Rot2quat(float roll, float pitch, float yaw) {

  Quaternion Rot2quatResult;

  Rot2quatResult.w = 1;
  Rot2quatResult.x = roll/2.0;
  Rot2quatResult.y = pitch/2.0;
  Rot2quatResult.z = yaw/2.0;

  return Rot2quatResult;
}


Quaternion QuatMult(Quaternion A,Quaternion B){

  Quaternion QuatMultResult;
  QuatMultResult.w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z;
  QuatMultResult.x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y;
  QuatMultResult.y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x;
  QuatMultResult.z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w;

  return QuatMultResult;

}

Quaternion normQuat(Quaternion stateQuat){
 float quatMag = sqrt(stateQuat.w*stateQuat.w + stateQuat.x*stateQuat.x +
                       stateQuat.y*stateQuat.y + stateQuat.z*stateQuat.z);

  stateQuat.w = stateQuat.w/quatMag;
  stateQuat.x = stateQuat.x/quatMag;
  stateQuat.y = stateQuat.y/quatMag;
  stateQuat.z = stateQuat.z/quatMag;
  
  return stateQuat;

}

void Quat2Tbn(Quaternion stateQuat, float Tbn[3][3]){ //DCM

  float w = stateQuat.w;
  float x = stateQuat.x;
  float y = stateQuat.y;
  float z = stateQuat.z;

  Tbn[0][0] = w*w + x*x - y*y - z*z;
  Tbn[0][1] = 2*(x*y - w*z);
  Tbn[0][2] = 2*(x*z + w*y);

  Tbn[1][0] = 2*(x*y + w*z);
  Tbn[1][1] = w*w - x*x + y*y - z*z;
  Tbn[1][2] = 2*(y*z - w*x);

  Tbn[2][0] = 2*(x*z - w*y);
  Tbn[2][1] = 2*(y*z + w*x);
  Tbn[2][2] = w*w - x*x - y*y + z*z;


}

void ZSensor(float Z[SIZE], Velocity v, Position p, Quaternion q){

  Z[0]=q.w;
  Z[1]=q.x;
  Z[2]=q.y;
  Z[3]=q.z;
  Z[4]=v.N;
  Z[5]=v.E;
  Z[6]=v.D;
  Z[7]=p.N;
  Z[8]=p.E;
  Z[9]=p.D;
}



void stateVelAndPos(float deltat){

  deltaVelNav.N = Tbn[0][0]*DeltaVelX + Tbn[0][1]*DeltaVelY + Tbn[0][2]*DeltaVelZ;
  deltaVelNav.E = Tbn[1][0]*DeltaVelX + Tbn[1][1]*DeltaVelY + Tbn[1][2]*DeltaVelZ;
  deltaVelNav.D = Tbn[2][0]*DeltaVelX + Tbn[2][1]*DeltaVelY + Tbn[2][2]*DeltaVelZ + Gravity*deltat;

  stateVel.N = stateVel.N + deltaVelNav.N;
  stateVel.E = stateVel.E + deltaVelNav.E;
  stateVel.D = stateVel.D + deltaVelNav.D;

  statePos.N = statePos.N + stateVel.N*deltat;
  statePos.E = statePos.E + stateVel.E*deltat; 
  statePos.D = statePos.D + stateVel.D*deltat; 

}

void UncertaintyOfThePrediction(float fJacobian[SIZE][SIZE], float P[SIZE][SIZE], float Q[SIZE][SIZE]){
 float temp[SIZE][SIZE];

  for (int j=0; j<SIZE ; j++){

     for (int i=0; i<SIZE ; i++){
            temp[j][i] = fJacobian[j][0]*P[0][i] + fJacobian[j][1]*P[1][i] + fJacobian[j][2]*P[2][i] + fJacobian[j][3]*P[3][i] + fJacobian[j][4]*P[4][i] + 
                         fJacobian[j][5]*P[5][i] + fJacobian[j][6]*P[6][i] + fJacobian[j][7]*P[7][i] + fJacobian[j][8]*P[8][i] + fJacobian[j][9]*P[9][i];
      }
 }

  for (int j=0; j<SIZE ; j++){

     for (int i=0; i<SIZE ; i++){
            P[j][i] = temp[j][0]*fJacobian[i][0] + temp[j][1]*fJacobian[i][1] + temp[j][2]*fJacobian[i][2] + temp[j][3]*fJacobian[i][3] + temp[j][4]*fJacobian[i][4] + 
                                  temp[j][5]*fJacobian[i][5] + temp[j][6]*fJacobian[i][6] + temp[j][7]*fJacobian[i][7] + temp[j][8]*fJacobian[i][8] + temp[j][9]*fJacobian[i][9];
      }
 }

  for(int i=0; i<SIZE ; i++){
    P[i][i] = P[i][i] + Q[i][i];
  }


}
//K = P*H'/(H*P*H' + R);
void kalmanGainDenominator(float H[SIZE][SIZE], float P[SIZE][SIZE], float Rvariance[SIZE][SIZE],float denominator[SIZE][SIZE]){
 float temp[SIZE][SIZE];

  for (int j=0; j<10 ; j++){

     for (int i=0; i<SIZE ; i++){
            temp[j][i] = H[j][0]*P[0][i] + H[j][1]*P[1][i] + H[j][2]*P[2][i] + H[j][3]*P[3][i] + H[j][4]*P[4][i] + 
                         H[j][5]*P[5][i] + H[j][6]*P[6][i] + H[j][7]*P[7][i] + H[j][8]*P[8][i] + H[j][9]*P[9][i];
      }
 }

  for (int j=0; j<SIZE ; j++){

     for (int i=0; i<SIZE ; i++){
            denominator[j][i] = temp[j][0]*H[i][0] + temp[j][1]*H[i][1] + temp[j][2]*H[i][2] + temp[j][3]*H[i][3] + temp[j][4]*H[i][4] + 
                                  temp[j][5]*H[i][5] + temp[j][6]*H[i][6] + temp[j][7]*H[i][7] + temp[j][8]*H[i][8] + temp[j][9]*H[i][9];
      }
 }

  for(int i=0; i<SIZE ; i++){
    denominator[i][i] = denominator[i][i] + Rvariance[i][i];
  }


}


//K = P*H'/(H*P*H' + R);
void kalmanGainNumerator(float P[SIZE][SIZE], float H[SIZE][SIZE], float numerator[SIZE][SIZE]){

  for (int j=0; j<SIZE ; j++){

     for (int i=0; i<SIZE ; i++){
            numerator[j][i] = P[j][0]*H[i][0] + P[j][1]*H[i][1] + P[j][2]*H[i][2] + P[j][3]*H[i][3] + P[j][4]*H[i][4] + 
                                  P[j][5]*H[i][5] + P[j][6]*H[i][6] + P[j][7]*H[i][7] + P[j][8]*H[i][8] + P[j][9]*H[i][9];
      }
 }

}

void kalmanGain( float numerator[SIZE][SIZE], float denominator[SIZE][SIZE], float K[SIZE][SIZE]){
float inv[SIZE][SIZE];

  inverseMatrix(denominator, inv);

      for (int j=0; j<SIZE ; j++){
        for (int i=0; i<SIZE ; i++){
               K[j][i] = numerator[j][0]*inv[i][0] + numerator[j][1]*inv[i][1] + numerator[j][2]*inv[i][2] + numerator[j][3]*inv[i][3] + numerator[j][4]*inv[i][4] + 
                         numerator[j][5]*inv[i][5] + numerator[j][6]*inv[i][6] + numerator[j][7]*inv[i][7] + numerator[j][8]*inv[i][8] + numerator[j][9]*inv[i][9];

      }
 } 
 


}
//bu fonksiyon ben yapmadım internetten hazır. yaptığı iş matrisin tersi bulma 
void inverseMatrix(float matrix[SIZE][SIZE],float inverse[SIZE][SIZE]) { 
    // Birim matris oluşturma
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            if (i == j)
                inverse[i][j] = 1;
            else
                inverse[i][j] = 0;
        }
    }

    // Gauss-Jordan eliminasyonu
    for (int i = 0; i < SIZE; i++) {
        float temp = matrix[i][i];
        for (int j = 0; j < SIZE; j++) {
            matrix[i][j] /= temp;
            inverse[i][j] /= temp;
        }
        for (int k = 0; k < SIZE; k++) {
            if (k != i) {
                temp = matrix[k][i];
                for (int j = 0; j < SIZE; j++) {
                    matrix[k][j] -= matrix[i][j] * temp;
                    inverse[k][j] -= inverse[i][j] * temp;
                }
            }
        }
    }

}

//states(i+1) = states(i) + K*(Z - H*states)
void statesUpdate(float Z[SIZE], Velocity stateVel,Position statePos, Quaternion stateQuat, float K[SIZE][SIZE], float H[SIZE][SIZE]){
 float temp[SIZE];
 float temp2[SIZE];

  for (int j=0; j<SIZE ; j++){ //(Z - H*states) = temp
          temp[j] =Z[j] -  (H[j][0]*stateQuat.w + H[j][1]*stateQuat.x + H[j][2]*stateQuat.y + H[j][3]*stateQuat.z + H[j][4]*stateVel.N + 
                            H[j][5]*stateVel.E + H[j][6]*stateVel.D + H[j][7]*statePos.N + H[j][8]*statePos.E + H[j][9]*statePos.D);
  }
  for (int j=0; j<SIZE ; j++){ 
          temp2[j] =K[j][0]*temp[0] + K[j][1]*temp[1] + K[j][2]*temp[2] + K[j][3]*temp[3] + K[j][4]*temp[4] + 
                   K[j][5]*temp[5] + K[j][6]*temp[6] + K[j][7]*temp[7] + K[j][8]*temp[8] + K[j][9]*temp[9];
  }

  //states(i+1) = states(i) + K*(Z - H*states)
  stateQuat.w = stateQuat.w + temp2[0];
  stateQuat.x = stateQuat.x + temp2[1];
  stateQuat.y = stateQuat.y + temp2[2];
  stateQuat.z = stateQuat.z + temp2[3];
  stateVel.N = stateVel.N + temp2[4];
  stateVel.E = stateVel.E + temp2[5];
  stateVel.D = stateVel.D + temp2[6];
  statePos.N = statePos.N + temp2[7];
  statePos.E = statePos.E + temp2[8];
  statePos.D = statePos.D + temp2[9];

}

//P = (eye(10) - K*H)*P;
void UncertaintyOfThePredictionUpdate(float eye[SIZE][SIZE], float P[SIZE][SIZE], float K[SIZE][SIZE], float H[SIZE][SIZE]){
  float temp[SIZE][SIZE];
  float temp2[SIZE][SIZE];

  for (int j=0; j<SIZE ; j++){
     for (int i=0; i<SIZE ; i++){
            temp[j][i] = eye[j][i] - (K[j][0]*H[i][0] + K[j][1]*H[i][1] + K[j][2]*H[i][2] + K[j][3]*H[i][3] + K[j][4]*H[i][4] + 
                                      K[j][5]*H[i][5] + K[j][6]*H[i][6] + K[j][7]*H[i][7] + K[j][8]*H[i][8] + K[j][9]*H[i][9]);
      }
  }
  for (int j=0; j<SIZE ; j++){
     for (int i=0; i<SIZE ; i++){
            temp2[j][i] = temp[j][0]*P[i][0] + temp[j][1]*P[i][1] + temp[j][2]*P[i][2] + temp[j][3]*P[i][3] + temp[j][4]*P[i][4] + 
                          temp[j][5]*P[i][5] + temp[j][6]*P[i][6] + temp[j][7]*P[i][7] + temp[j][8]*P[i][8] + temp[j][9]*P[i][9];
      }
  }
  for (int j=0; j<SIZE ; j++){
     for (int i=0; i<SIZE ; i++){
           P[j][i] = temp2[j][i];
      }
  }
  


}
