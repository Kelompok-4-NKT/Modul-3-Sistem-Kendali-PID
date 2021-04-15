/************************************************/

// Modul Praktikum 3 - Sistem Kendali 
// Nama Tim			: NKT
// Nama Anggota 1	: Fakhrul Efendi
// Nama Anggota 2	: Iqbal Syaifullah A.
// Versi Program	: 3.0.1

/************************************************/
//Deklarasi variabel sensor
int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;
int sensor6 = A5;
int baca_sensor[6];
int button1 = 8;
int button2 = 9;
int sendat[6];
int sensorBit;

//Variabel motor
int pinEnable1 = 4;//Harus diset HIGH apabila akan diaktifkan
int pinEnable2 = 2;//Harus diset HIGH apabila akan diaktifkan
//Motor Kiri
int motor_kiri1 = 5;//Input motor driver 1 L293D
int motor_kiri2 = 6;//Input motor driver 2 L293D
//Motor Kanan
int motor_kanan1 = 3;//Input motor driver 3 L293D
int motor_kanan2 = 11;//Input motor driver 4 L293D

//Variabel Bantuan
int x;

//Variabel PID
int Kp = 5, Ki = 0, Kd = 0;
int LastError = 0; //Nilai 0 berarti tidak ada error
int error = 0;
int rate_i, rate_d; // Variabel rate untuk I dan D
int KMkanan, KMkiri, KSetPoint = 150;

void setup(){
  //Pinmode input sensor
  pinMode(sensor1, INPUT);//Set pin A0 sebagai input
  pinMode(sensor2, INPUT);//Set pin A1 sebagai input
  pinMode(sensor3, INPUT);//Set pin A2 sebagai input
  pinMode(sensor4, INPUT);//Set pin A3 sebagai input
  pinMode(sensor5, INPUT);//Set pin A4 sebagai input
  pinMode(sensor6, INPUT);//Set pin A5 sebagai input
  //Pinmode output motor
  pinMode(pinEnable1, OUTPUT);//Set pin 4 sebagai output
  pinMode(pinEnable2, OUTPUT);//Set pin 2 sebagai output
  pinMode(motor_kiri1, OUTPUT);//Set pin 5 sebagai output
  pinMode(motor_kiri2, OUTPUT);//Set pin 6 sebagai output
  pinMode(motor_kanan1, OUTPUT);//Set pin 3 sebagai output
  pinMode(motor_kanan2, OUTPUT);//Set pin 11 sebagai output
  //Inisialisasi komunikasi serial
  Serial.begin(9600);//Baud Rate standar 9600
  }

  void readsensor(){ //Fungsi untuk membaca sensor kemudian menyimpan pada array
    baca_sensor[0] = analogRead(sensor1);
    baca_sensor[1] = analogRead(sensor2);
    baca_sensor[2] = analogRead(sensor3);
    baca_sensor[3] = analogRead(sensor4);
    baca_sensor[4] = analogRead(sensor5);
    baca_sensor[5] = analogRead(sensor6);
    
  delay(10);
  /* Menampilkan data Sensor ke Serial Monitor /
  /  Data Sensor 1-6                           /
  /  Formatting tampilan data sensor           /
  /                                            /
  /	for (x=0; x<=5; x++) {                     /
  /		Serial.println(baca_sensor[x]);        /
  /		Serial.print(" ");                     /
  /		}                                      /
  /                                           */
  for (x=0; x<=5; x++)
  {
    if (baca_sensor[x] < 33)
    {
      sendat[x]=1; //Gelap
    }
    else{
      sendat[x]=0; //Terang
    }
  }
    sensorBit = 0;
    for (x=0; x<=5; x++){
      sensorBit += sendat[x] + (0 << x);
      Serial.println(sensorBit, BIN);
    }
  }
                     
void pv() {
  switch (sensorBit) {
    // i adalah gelap, 0 adalah terang                          
    // contoh = 0b100000 berarti sensor 1 gelap, sisanya terang 
    // Sensor 1 2 3 4 5 6                                      
 
    case 0b100000: error = -3; break;
    case 0b110000: error = -2; break;
    case 0b010000: error = -1; break;
    //		 ||
    case 0b001100: error = 0; break;
    case 0b001000: error = 0; break;
    case 0b000100: error = 0; break;
    //		 ||
    case 0b000010: error = 1; break;
    case 0b000011: error = 2; break;
    case 0b000001: error = 3; break;
    
    default: error; break;
  }
}
void loop() {
	digitalWrite(pinEnable1, HIGH);
	digitalWrite(pinEnable2, HIGH);
    //Fungsi deteksi garis
  	follow_line();
}
  
void follow_line() {
   readsensor(); //Fungsi membaca input sensor
   pv(); // Fungsi membaca output process value
   //Fungsi MV(t) persamaan PID
   rate_d = error - LastError; //Rate untuk D
   rate_i = error + LastError; //Rate untuk I
   LastError = error;
   int moveControl = (Kp * error) + (Ki * rate_i) + (Kd * rate_d);
   
   KMkanan = KSetPoint - moveControl; //Kecepatan motor Kanan
   KMkiri = KSetPoint + moveControl; //Kecepatan motor Kiri
   
   setMotor(KMkiri, KMkanan); //Fungsi motor 
}
void setMotor(int pwmKiri, int pwmKanan)
{
  int maxpwm;
  //maxpwm di-set 255
  if (pwmKiri > pwmKanan){
    pwmKiri = maxpwm;
  }
  else if (pwmKiri < -maxpwm){
    pwmKiri = -maxpwm;
  }
  // Fungsi agar motor tidak bergerak mundur
  if (pwmKiri < 0){
    pwmKiri *= -1;
    analogWrite(motor_kiri2, pwmKiri);
    analogWrite(motor_kiri1, 0);  
  }
  else
  {
    analogWrite(motor_kiri2, 0);
    analogWrite(motor_kiri1, pwmKiri);
  }
  if (pwmKanan > maxpwm) {
    pwmKanan = maxpwm;
  }
  else if (pwmKanan < -maxpwm)
  {
    pwmKanan = -maxpwm;
  }
  if (pwmKanan < 0)
  {
    pwmKanan *= -1;
    analogWrite(motor_kanan2, 0);
    analogWrite(motor_kanan1, pwmKanan);
  }
  else
  {
    analogWrite(motor_kanan2, 0);
    analogWrite(motor_kanan1, pwmKanan);
  }
}
