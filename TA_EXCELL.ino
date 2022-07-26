/*
|PROGRAM ARDUINO UNO UNTUK ALAT UKUR DEFLEKSI 
*/
#include <SPI.h>
#include <Wire.h>
#include<HX711_ADC.h>
#include<EEPROM.h>
#include<LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);

//deklrasi pin modul hx711 ke arduino 
const int HX711_dout = 4;
const int HX711_sck = 5;

// deklrasi pin trig dan pin echo ke arduino uno
#define trigPin 12
#define echoPin 11

//deklarasi pin load cell
HX711_ADC LoadCell(HX711_dout, HX711_sck);

//penyimpan nilai CalVal pada Eeprom Adress
const int calVal_eepromAdress = 0;
//deklrasi untuk fungsi reset tare 
long t;

// variable - variabel jarak 
int jarak = 100.00; //batas
int tinggi;
float hasil;
int j;
int inisiasi;

//fungsi millis()
unsigned long interval=2000; // waktu yang kita butuhkan untuk menunggu
unsigned long previousMillis=0; // millis() mengembalikan unsigned long.

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  lcd.begin();
  pinMode(HX711_dout, INPUT);
  delay(10);
  Serial.println();
  Serial.println("Memulai...");
  //fungsi pengiriman ke excell
  Serial.println("CLEARDATA");                              // hapus lembar excel
  Serial.println("LABEL,Waktu,Berat, Jarak Lendutan(mm)"); // label untuk ms-excel
  LoadCell.begin(); //pengaktifan load cell
  lcd.setCursor(0,0); //mengatur posisi kursor pada baris 1 posisi 0 
  lcd.print("Brt: "); //menulis kata Brt
  lcd.setCursor(12,0); //mengatur posisi kursor pada baris 12 posisi 0
  lcd.print("0"); //menulis kata "0"
  lcd.setCursor(14,0); //mengatur posisi kursor pada baris 14 posisi 0
  lcd.print("gr"); //menulis kata "gr"
  float calibrationValue;  //deklarasi nilai kalibrasi 
  calibrationValue = 696.0; //isi kalibrasi 696.0
  EEPROM.get(calVal_eepromAdress, calibrationValue); //baca semua tipe data atau objek dar  EEPROM  
  //calVal_eepromAdress ,lokasi untuk membaca
  //calibrationValue , data yang di baca berupa nilai float
  long stabilizingtime = 2000; 
  boolean _tare = true; //fungsi true untuk nilai benar/ fungsi reset nilai
  LoadCell.start(stabilizingtime, _tare); //loadcell mendapat waktu 2000m untuk stabil dan 

  //apabila Load Cell.getTareTimeoutFlag atau wiring hx711 tidak benar akan menampilkan Timeout, cek kabel MCU>HX711 pastikan sudah tepat
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, cek kabel MCU>HX711 pastikan sudah tepat");
    while (1);
  }
  else {
    //apabila Load Cell.getTareTimeoutFlag atau wiring hx711  benar akan menampilkan startup selesai
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Startup selesai");
  }
}

void loop() {
  //fungsi perulangan berkali

  int j; //deklarasi j
  static boolean newDataReady = 0; 
  const int serialPrintInterval = 0;  //menambah nilai untuk memperlambat aktivitas cetak serial
  
  if (LoadCell.update()) newDataReady = true; //jika LoadCell mengupdate data bernilai benar 
  if (newDataReady) {   //jika data baru ready 
    if (millis() > t + serialPrintInterval) {  //jika nilai millis lebih besar > aktivitas cetak serial
      int i = LoadCell.getData(); //maka load cell akan mengupdate nilai data
      if(i<0){ //jika i lebih kecil dari 0 maka akan menampilan nilai 0
        i=0;
      }
      tampil(i); //fungsi tampil untuk menampilkan di LCD
      inisiasi = i; //insiasi bernilai i data load cell
      j=i; //j bernilai data load cell di untuk dikirimkan ke excell
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


  //code sensor jarak
  long duration, gape; //deklrasi duration dan gape
  digitalWrite(trigPin, LOW); //fungsi memberi nilai LOW pada pin Trigger
  delayMicroseconds(2); //delay 2 ms
  digitalWrite(trigPin, HIGH); //fungsi memberi nilai HIGH pada pin trigger
  delayMicroseconds(10); //delay 10 ms
  digitalWrite(trigPin, LOW); //fungsi memberi nilai LOW pada pin Trigger
  duration = pulseIn(echoPin, HIGH); //deklarasi variable duration dengan isian nilai error pulsa yang lebih besar 
  gape = (duration / 29.0); //deklarasi variable gape di isi dengan hasil duration di bagi 29.0
  tinggi = (jarak * gape) / 2; //deklarasi variable tinggi di isi dengan  hasil jarak maksimal 100 dikali hasil variable gape


//deklarasi  inisiasi hasil beban untuk jarak
hasil = 0;
  if(inisiasi >0 && inisiasi <= 1200){
  hasil=0.3;
  }else if(inisiasi >1200 && inisiasi <= 2200){
    hasil = 0.65;
  }else if(inisiasi >2200 && inisiasi <= 3200){
    hasil = 0.95;
  }else if(inisiasi >3200 && inisiasi <= 4300){
    hasil = 1.30;
  }else if (inisiasi > 5000){
    hasil = 1.8;
  } else {
    hasil = 0;
  }
  
//fungsi untuk menampilkan isi ke LCD
  lcd.setCursor(0, 4);
  lcd.print("Jrk:  ");
  lcd.print(hasil);
  lcd.print(" ");
  lcd.print("mm");
  lcd.print("   ");
  
  unsigned long currentMillis = millis(); // ambil waktu saat ini
 // periksa apakah waktu "interval" telah berlalu (1000 milidetik)
 if ((unsigned long)(currentMillis - previousMillis) >= interval) {
   //bagian pengiriman data ke excell   
  Serial.print("DATA, TIME,");
  Serial.print(inisiasi);
  Serial.print(",");
  Serial.println(hasil);
  // simpan waktu "saat ini"
   previousMillis = millis();
 }
}

//fungsi tampil untuk menampilkan hasil berat ke LCD 
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
