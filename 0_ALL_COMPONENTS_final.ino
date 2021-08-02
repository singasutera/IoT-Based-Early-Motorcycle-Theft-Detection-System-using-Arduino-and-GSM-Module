#include "TinyGPS++.h"
#include <SoftwareSerial.h>
#include <String.h>
#include <Wire.h>
#include <MPU6050.h>

SoftwareSerial gprsSerial(7,8);
SoftwareSerial GPSSerial(2,3);
TinyGPSPlus gps;
MPU6050 mpu; //SCL A5 SDA A4

#define fsrPin A0
int pirSt = LOW;
int val = 0;
int fsrR;
String stPIR;
String stK;
String stJ;
String stP;
String datAMsk = "";
String stALst = "";
String datSMsk = "";
String stSLst = "";
String Lat;
String Lng;

void setup() {
  Serial.begin(9600);

  GPSSerial.begin(9600);
  gprsSerial.begin(9600);
    
  Wire.begin();
    
  pinMode(11, OUTPUT);//alarm
  pinMode(4, INPUT);//pir
  digitalWrite(11, LOW);//alarm
  digitalWrite(4, LOW);//pir
  
  stPIR="0";
  stK = "0";
  stJ = "0";
  stP = "0"; 
  stSLst="0";
  Lat = "-6.265436";
  Lng = "106.989580";

  datAMsk.reserve(200);
  datSMsk.reserve(200);
}

void loop() {
      MasukDataGPS();
      MasukDataPIR();
      MasukDataFSR();
      MasukDataAcc();
      SIM800LSEND();
      AlarmONOFF();  
}

void MasukDataGPS(){
  while(GPSSerial.available()>0)     //Selama ada karakter masuk ke modul GPS
  {
    gps.encode(GPSSerial.read());  //Masukkan data NMEA ke library satu karakter per satu karakter
    if(gps.location.isUpdated())     //Untuk mengurangi kondisi supaya hanya dieksekusi ketika data NMEA selesai masuk
    {
     //Mengambil informasi terakhir dari objek gps yang berasal dari data yang diambil oleh modul GPS
     Serial.print("Bujur Lintang:");
     Lat = String(gps.location.lat(),6);
     Lng = String(gps.location.lng(),6);
     Serial.print(Lat); Serial.print(","); Serial.println(Lng);
     Serial.println("");
     delay(2000);
    }
  }
}

void ShowResponse(){
  while(gprsSerial.available()!=0)
  Serial.write(gprsSerial.read());
  delay(5000); 
}

void GetResponse(int wait){
  datAMsk = "";
  Serial.print("response : ");
  long timeNOW2 = millis();
  while(millis()-timeNOW2 < wait){
    while(gprsSerial.available()>0){
      datAMsk += (char)gprsSerial.read();
    }
    delay(1);
  }
  Serial.println(datAMsk);
  Serial.println();
}

String parse(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  } 

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void MasukDataPIR(){
  val = digitalRead(4);
  if (val == HIGH) {
    if (pirSt == LOW) {
      Serial.println("GERAKAN MANUSIA TERDETEKSI");
      stPIR = "1";
      stK = "1";
      pirSt = HIGH;
      Serial.print("Status Kunci = ");
      Serial.println(stK);
    }
  } 
  else {
    if (pirSt == HIGH) {
      Serial.println("GERAKAN BERHENTI");
      stPIR = "0";
      pirSt = LOW;
      Serial.print("Status Kunci = ");
      Serial.println(stK);
    }
  }
}

void MasukDataFSR(){
  fsrR = analogRead(fsrPin);
  Serial.print("Analog reading = ");
  Serial.print(fsrR);
  
  if (fsrR < 10) {
    Serial.println(" - Tidak ada tekanan");
    stJ = "0";
  } else if (fsrR < 200) {
    Serial.println(" - Sentuhan ringan");
    stJ = "0";
  } else if (fsrR < 500) {
    Serial.println(" - Tekanan ringan");
    stJ = "0";
  } else if (fsrR < 800) {
    Serial.println(" - Tekanan sedang");
    stJ = "1";
  } else {
    Serial.println(" - Tekanan berat");
    stJ = "1";
  } 
  
  Serial.print("Status Jok = ");
  Serial.println(stJ);
}

void MasukDataAcc(){
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Sensor MPU6050 tidak terdeteksi, cek pengkabelan.");
    delay(500);
  }
  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);//power supply delay selama 3ms
  mpu.setIntFreeFallEnabled(false);//mematikan hardware interrupt  
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);//set high pass filter
  mpu.setMotionDetectionThreshold(1);//set limit deteksi pergerakan ke 4mg(dibagi 2)
  mpu.setMotionDetectionDuration(5);//set durasi minimal 5ms
  mpu.setZeroMotionDetectionThreshold(4);//set limit deteksi kediaman ke 8mg(dibagi 2)
  mpu.setZeroMotionDetectionDuration(2);//set durasi minimal 2s
  
  Vector rawAccel = mpu.readRawAccel();
  Activites act = mpu.readActivites();
  if (act.isActivity)
  {
    Serial.println ("GERAKAN MOTOR TERDETEKSI");
    stP = "1";
    Serial.print("Status Pergerakan = ");
    Serial.println(stP);
  } else
  {
    Serial.println ("NORMAL");
    stP = "0";
    Serial.print("Status Pergerakan = ");
    Serial.println(stP);
  }
  delay(1000);
}

void AlarmONOFF(){
    while(stK&&stJ=="0"){
    if(stP="1"){
      StopAlarm();
    }
    else{
      StartAlarm();
    }
  }

  while(stK=="0"){
    if(stJ="1"){
      StartAlarm();
    }
    else{
      StopAlarm();
    }
  }

  while(stK=="1"){
    StartAlarm(); 
  }

  SIM800LREQUEST_Alarm();

  if(stALst="1"){
    StartAlarm();
  }
  else if(stALst="0"){
    StopAlarm();
    stK="0";
  }
}

void StartAlarm() {
  tone(11,450); 
  
  gprsSerial.println("AT");
  ShowResponse();
 
  gprsSerial.println("AT+CPIN?"); //sets the password of the mobile device
  ShowResponse();
 
  gprsSerial.println("AT+CREG?"); //gives info abt registration status
  ShowResponse();
 
  gprsSerial.println("AT+CGATT?"); //attach or detach device to packet domain service
  ShowResponse();
 
  gprsSerial.println("AT+CIPSHUT"); //close GPRS PDP context
  ShowResponse();
 
  gprsSerial.println("AT+CIPSTATUS"); //returns the corrent connection status
  ShowResponse();
 
  gprsSerial.println("AT+CIPMUX=0"); //single IP connection
  ShowResponse();
 
  gprsSerial.println("AT+CSTT=\"internet\"");//sets up the apn, user name and password for the PDP context.
  ShowResponse();
 
  gprsSerial.println("AT+CIICR");//brings up the GPRS or CSD call depending on the configuration previously set by the AT+CSTT command.
  ShowResponse();
  
  gprsSerial.println("AT+CIFSR");//returns the local IP addres
  ShowResponse();

  gprsSerial.println("AT+CIPSPRT=0");//set whether echo prompt ">" after issuing "AT+CIPSEND" command.
  ShowResponse();
  
  gprsSerial.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");//starts TCP or UDP connection
  ShowResponse();
 
  gprsSerial.println("AT+CIPSEND");//send the data over the TCP or UDP connection.
  ShowResponse();
  
  String alarmON="GET https://api.thingspeak.com/update?api_key=VNBV9VJOHRXJL2UL&field6=,1";
  Serial.println(alarmON);
  gprsSerial.println(alarmON);//begin send data to remote server
  ShowResponse();
 
  gprsSerial.println((char)26);//sending
  delay(5000);//waiting for reply, important! the time is base on the condition of internet 
  gprsSerial.println();
  ShowResponse();
 
  gprsSerial.println("AT+CIPSHUT");//close the GPRS PDP context.
  ShowResponse();

//KIRIM SMS KE PONSEL PENGGUNA
  gprsSerial.println("AT+CMGF=1"); //set format pesan SMS sebagai text
  ShowResponse();
  gprsSerial.println("AT+CMGS=\"+6281318690931\""); //nomor ponsel pengguna sistem
  ShowResponse();
  gprsSerial.println("PERINGATAN! Ada aktivitas mencurigakan di motor Anda!");//isi SMS
  ShowResponse();
  gprsSerial.write(26);
}

void StopAlarm() {
  noTone(11);
  
  gprsSerial.println("AT");
  ShowResponse();
 
  gprsSerial.println("AT+CPIN?"); //sets the password of the mobile device
  ShowResponse();
 
  gprsSerial.println("AT+CREG?"); //gives info abt registration status
  ShowResponse();
 
  gprsSerial.println("AT+CGATT?"); //attach or detach device to packet domain service
  ShowResponse();
 
  gprsSerial.println("AT+CIPSHUT"); //close GPRS PDP context
  ShowResponse();
 
  gprsSerial.println("AT+CIPSTATUS"); //returns the corrent connection status
  ShowResponse();
 
  gprsSerial.println("AT+CIPMUX=0"); //single IP connection
  ShowResponse();
 
  gprsSerial.println("AT+CSTT=\"internet\"");//sets up the apn, user name and password for the PDP context.
  ShowResponse();
 
  gprsSerial.println("AT+CIICR");//brings up the GPRS or CSD call depending on the configuration previously set by the AT+CSTT command.
  ShowResponse();
  
  gprsSerial.println("AT+CIFSR");//returns the local IP addres
  ShowResponse();

  gprsSerial.println("AT+CIPSPRT=0");//set whether echo prompt ">" after issuing "AT+CIPSEND" command.
  ShowResponse();
  
  gprsSerial.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");//starts TCP or UDP connection
  ShowResponse();
 
  gprsSerial.println("AT+CIPSEND");//send the data over the TCP or UDP connection.
  ShowResponse();
  
  String alarmOFF="GET https://api.thingspeak.com/update?api_key=VNBV9VJOHRXJL2UL&field6=,0";
  Serial.println(alarmOFF);
  gprsSerial.println(alarmOFF);//begin send data to remote server
  ShowResponse();
 
  gprsSerial.println((char)26);//sending
  delay(5000);//waitting for reply, important! the time is base on the condition of internet 
  gprsSerial.println();
  ShowResponse();
 
  gprsSerial.println("AT+CIPSHUT");//close the GPRS PDP context.
  ShowResponse();
}

void SIM800LSEND(){
  gprsSerial.println("AT");
  ShowResponse();
 
  gprsSerial.println("AT+CPIN?"); //sets the password of the mobile device
  ShowResponse();
 
  gprsSerial.println("AT+CREG?"); //gives info abt registration status
  ShowResponse();
 
  gprsSerial.println("AT+CGATT?"); //attach or detach device to packet domain service
  ShowResponse();
 
  gprsSerial.println("AT+CIPSHUT"); //close GPRS PDP context
  ShowResponse();
 
  gprsSerial.println("AT+CIPSTATUS"); //returns the corrent connection status
  ShowResponse();
 
  gprsSerial.println("AT+CIPMUX=0"); //single IP connection
  ShowResponse();
 
  gprsSerial.println("AT+CSTT=\"internet\"");//sets up the apn, user name and password for the PDP context.
  ShowResponse();
 
  gprsSerial.println("AT+CIICR");//brings up the GPRS or CSD call depending on the configuration previously set by the AT+CSTT command.
  ShowResponse();
  
  gprsSerial.println("AT+CIFSR");//returns the local IP addres
  ShowResponse();

  gprsSerial.println("AT+CIPSPRT=0");//set whether echo prompt ">" after issuing "AT+CIPSEND" command.
  ShowResponse();
  
  gprsSerial.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");//starts TCP or UDP connection
  ShowResponse();
 
  gprsSerial.println("AT+CIPSEND");//send the data over the TCP or UDP connection.
  ShowResponse();
  
//  String str="GET https://api.thingspeak.com/update?api_key=VNBV9VJOHRXJL2UL&field1=" + String(stPIR) + "&field2=" + String(stJ) + "&field3=" + String(stP);// + "&field4=" + Lat + "," + Lng;
//  String str="GET https://api.thingspeak.com/update?api_key=VNBV9VJOHRXJL2UL&field1=0&field2=0&field3=0&field4=-6.265436,106.989580";
  String str="GET https://api.thingspeak.com/update?api_key=VNBV9VJOHRXJL2UL&field1=" + stK + "&field2=" + stJ + "&field3=" + stP + "&field4=" + Lat + "," + Lng;
  Serial.println(str);
  gprsSerial.println(str);//begin send data to remote server
  ShowResponse();
 
  gprsSerial.println((char)26);//sending
  delay(5000);//waitting for reply, important! the time is base on the condition of internet 
  gprsSerial.println();
  ShowResponse();
 
  gprsSerial.println("AT+CIPSHUT");//close the GPRS PDP context.
  ShowResponse();
}

void SIM800LREQUEST_Alarm(){
  gprsSerial.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  ShowResponse();

  gprsSerial.println("AT+SAPBR=3,1,\"APN\",\"internet\"");
  ShowResponse();

  gprsSerial.println("AT+SAPBR=1,1");
  ShowResponse();

  gprsSerial.println("AT+HTTPINIT");
  ShowResponse();

  Serial.println("AT+HTTPPARA=\"URL\",\"http://api.thingspeak.com/channels/1070612/fields/6/last.txt");
  gprsSerial.print("AT+HTTPPARA=\"URL\",\"http://api.thingspeak.com/channels/1070612/fields/6/last.txt");
  gprsSerial.println("\"");
  ShowResponse();
  
  // set http action type 0 = GET, 1 = POST, 2 = HEAD
  gprsSerial.println("AT+HTTPACTION=0");
  GetResponse(15000);

  gprsSerial.println("AT+HTTPREAD=0");
  GetResponse(5000);  // respon di simpan kedalam variabel dataMasuk

  Serial.print("Full data :");
  Serial.println(datAMsk);

  stALst = parse(datAMsk, ',', 1);
  
  Serial.print("Status Alarm :");
  Serial.println(stALst);

  gprsSerial.println("AT+HTTPTERM");
  ShowResponse();

  gprsSerial.println("AT+SAPBR=0,1");
  ShowResponse();  
}
