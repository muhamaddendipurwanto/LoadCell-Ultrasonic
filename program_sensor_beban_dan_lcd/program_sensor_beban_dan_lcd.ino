#include <SPI.h>
#include <Wire.h>
#include<HX711_ADC.h>
#include<EEPROM.h>
#include<LiquidCrystal_I2C.h>

// deklrasi pin yang digunakan
#define trigPin 12
#define echoPin 11

LiquidCrystal_I2C lcd(0x27,16,2);
const int HX711_dout =4;  //D6
const int HX711_sck = 5;   //D7

HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
long t;
// variable - variabel
int jarak = 100.00;
int tinggi;
int hasil;
int j;

void setup() {
  Serial.begin(9600);
  lcd.begin();
  pinMode(HX711_dout, INPUT);
  delay(10);
  Serial.println();
  Serial.println("Memulai...");
  Serial.println("CLEARDATA");                               // clear excel sheet
  Serial.println("LABEL,Waktu ,Jarak Lendutan (mm), Berat"); // label for ms-excel
  lcd.begin();
  LoadCell.begin();
  lcd.setCursor(0,0);
  lcd.print("Brt: ");
  lcd.setCursor(12,0);
  lcd.print("0");
  lcd.setCursor(14,0);
  lcd.print("gr");
  float calibrationValue; 
  calibrationValue = 696.0; 
  EEPROM.get(calVal_eepromAdress, calibrationValue);
  long stabilizingtime = 2000; 
  boolean _tare = true; 
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, cek kabel MCU>HX711 pastikan sudah tepat");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Startup selesai");
  }
}

void loop() {
  int j;
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; 
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      int i = LoadCell.getData();
      if(i<0){
        i=0;
      }
      tampil(i);
      j=i;
      newDataReady = 0;
      t = millis();
    }
  }
  if(Serial.available() > 0){
    float i;
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tara selesai");
  }

long duration, gape;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  gape = (duration / 29.0);
  tinggi = (jarak * gape) / 2;
  hasil = tinggi / 10;
  lcd.setCursor(0, 4);
  lcd.print("Jrk:  ");
  lcd.print(hasil);
  lcd.print(" ");
  lcd.print("mm");
  lcd.print("   ");
  Serial.print("DATA, TIME,");
  Serial.print(hasil);
  Serial.print(",");
  Serial.println(j);
  delay(500);


} //end void loop


void tampil(int j){
  lcd.setCursor(4,0);
  lcd.print("          ");
  if(j<10){
    lcd.setCursor(12,0);  
  }else if(j<100 && j>=10){
    lcd.setCursor(11,0);
  }else if(j<1000 && j>=100){
    lcd.setCursor(10,0);
  }else if(j<10000 && j>=1000){
    lcd.setCursor(9,0);
  }else if(j<100000 && j>=10000){
    lcd.setCursor(8,0);
  }else if(j<1000000 && j>=100000){
    lcd.setCursor(7,0);
  }else if(j<10000000 && j>=1000000){
    lcd.setCursor(6,0);
  }else if(j<100000000 && j>=10000000){
    lcd.setCursor(5,0);
  }else{
    lcd.setCursor(4,0);
  }
  lcd.print(j);
}
