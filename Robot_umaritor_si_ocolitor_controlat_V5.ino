#include <FastLED.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h> 
#include <LedControl.h>
#include <NewPing.h>                                // librarie senzor us de distanta
#include <Arduino.h>                                // librarie Arduino
#include <Servo.h>                                  // librarie servo

//Fast Led bara nanopixeli
#define LED_PIN 8
#define NUM_LEDS 8
CRGB leds[NUM_LEDS];
int index1;
int index2;
int i;
unsigned long initialtime;

//LDC Display
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//pini si initializare Afisaj Matrice LED
int DIN = 35;
int CS = 6;
int CLK = 7;
LedControl lc = LedControl(DIN,CLK,CS,0);
byte inainteMatrice[] ={ 0x18, 0x3c, 0x7e, 0xff, 0x18, 0x18, 0x18, 0x18 };
byte inapoiMatrice[] ={ 0x18, 0x18, 0x18, 0x18, 0xff, 0x7e, 0x3c, 0x18 };
byte stangaMatrice[] ={ 0x8, 0x18, 0x38, 0x7f, 0xff, 0x3f, 0x18, 0x8 }; 
byte dreaptaMatrice[] ={ 0x10, 0x18, 0x1e, 0xff, 0xfe, 0x1c, 0x18, 0x10 };
byte stopMatrice[] ={ 0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81 };

//senzor US
#define TRIG_PIN  34                                // pin trigger senzor us
#define ECHO_PIN     5                              // pin echo senzor us
#define MAX_distantaUS 50                             // distanta maxima pentru senzorul us
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_distantaUS);    // initializare senzor us

//control variables
boolean confirmareMiscareInainte = false;
int distantaUS = 100;

//servo
Servo motoras;
int pos = 0;

//output PWM pentru control motoare
#define vitezaPWM 55

//motoare + punte h
int motorStangaFata = 22;
int motorStangaSpate = 23;
int motorDreaptaFata = 24;
int motorDreaptaSpate = 25;
int enA = 2;
int enB = 3;

//contorRotatii senzori obstacole
int contorRotatii = 0;

//senzori IR pentru urmarire linie
int s1 = 26;
int s2 = 27;
int s3 = 28;
int s4 = 29;
int s5 = 30;
int buton = 31;
int near = 32;
int inputSelectie = 33;

String cuvantSelectie;

void setup() {
  Serial.begin(9600);

  //setup fast led
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds,NUM_LEDS);
  FastLED.setBrightness(255);
  index1 = 0;
  index2 = 0;
  initialtime = millis();
  
  //LCD Display
  lcd.init();                      // initialize the lcd 
  lcd.backlight();                  //init lumina de fundal
  
  //setup Modul afisaj Matrice LED
  lc.shutdown(0, false); // setam controller-ul sa se reseteze
  lc.setIntensity(0, 15); // setam intensitatea led-urilor
  lc.clearDisplay(0); //golim contentul de pe ecran
  
  //setup robot ocolitor
  motoras.attach(4);
  servoPozMijloc();
  delay(2000);
  
  
  
  //config motor + h
  pinMode(motorStangaFata, OUTPUT);
  pinMode(motorStangaSpate, OUTPUT);
  pinMode(motorDreaptaFata, OUTPUT);
  pinMode(motorDreaptaSpate, OUTPUT);

  // Configurare pini senzori de obstacole
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  pinMode(near, INPUT);
  pinMode(buton, INPUT);
  pinMode(inputSelectie, INPUT);
}

void loop() {
  distantaUS = readPing();
  //la fiecare iteratie se citeste starea senzorilor IR
  int sensor1 = digitalRead(s1);
  int sensor2 = digitalRead(s2);
  int sensor3 = digitalRead(s3);
  int sensor4 = digitalRead(s4);
  int sensor5 = digitalRead(s5);
  int sensorNear = digitalRead(near);
  int sensorButon = digitalRead(buton);
  int selectie = digitalRead(inputSelectie);

   
  //monitorizareSenzori();
  //        delay(500);

  if (selectie == HIGH) {
    Serial.println("Mod urmaritor");
    cuvantSelectie = "Mod urmaritor";
    if ((sensor1 == HIGH) && (sensor2 == LOW) && (sensor3 == LOW) && (sensor4 == LOW) && (sensor5 == HIGH))
    {
      inainte();
    }
    else if ((sensor1 == HIGH) && (sensor2 == HIGH) && (sensor3 == LOW) && (sensor4 == LOW) && (sensor5 == HIGH))
    {
      inainte();
    }
    else if ((sensor1 == HIGH) && (sensor2 == LOW) && (sensor3 == LOW) && (sensor4 == HIGH) && (sensor5 == HIGH))
    {
      inainte();
    }
    else if ((sensor1 == LOW) && (sensor2 == LOW) && (sensor3 == HIGH) && (sensor4 == HIGH) && (sensor5 == HIGH))
    {
      stanga();
    }
    else if ((sensor1 == LOW) && (sensor2 == HIGH) && (sensor3 == HIGH) && (sensor4 == HIGH) && (sensor5 == HIGH))
    {
      stanga();
    }
    else if ((sensor1 == HIGH) && (sensor2 == HIGH) && (sensor3 == HIGH) && (sensor4 == LOW) && (sensor5 == LOW))
    {
      dreapta();
    }
    else if ((sensor1 == HIGH) && (sensor2 == HIGH) && (sensor3 == HIGH) && (sensor4 == HIGH) && (sensor5 == LOW))
    {
      dreapta();
    }
    else {
      stop();
    }
    
  }
  else {
    Serial.println("Mod ocolitor");
    cuvantSelectie = "Mod ocolitor";
    lcd.setCursor(0,0);
    lcd.print("Mod 2 ocolitor");
    int distantaDreapta = 0;
    int distantaStanga = 0;
    delay(40);

    if (distantaUS <= 25) {
      stop();
      delay(100);
      miscareInapoi();
      delay(200);
      stop();
      delay(200);
      distantaDreapta = citireLaDreapta();
      delay(200);
      distantaStanga = citireLaStanga();
      delay(200);

      if (distantaDreapta >= distantaStanga) {
        Serial.println("intoarcere la dreapta");
        intoarcereLaDreapta();
        stop();
        contorRotatii++;
      }
      else

      {
        Serial.println("intoarcere la stanga");
        intoarcereLaStanga();
        stop();
        contorRotatii++;
      }
    }
    else
    {
      miscareInainte();
    }
    
    //distantaUS = readPing();  -- cred ca vine stearsa

    if (contorRotatii > 6) {
      stop();
      delay(200);
      distantaDreapta = citireLaDreapta();
      delay(200);
      distantaStanga = citireLaStanga();
      delay(200);

      if (distantaDreapta >= distantaStanga)
      {
        intoarcereLaDreapta();
        intoarcereLaDreapta();
        intoarcereLaDreapta();
      }
      else
      {
        intoarcereLaStanga();
        intoarcereLaStanga();
        intoarcereLaStanga();
      }
      contorRotatii = 0;
    }
  }
}

int citireLaDreapta() {
  servoPozStanga();
  delay(500);
  int distantaUS = readPing();
  delay(100);
  servoPozMijloc();
  return distantaUS;
}

int citireLaStanga() {
  servoPozDreapta();
  delay(500);
  int distantaUS = readPing();
  delay(100);
  servoPozMijloc();
  return distantaUS;
  delay(100);
}

int readPing() {
  delay(100);
  int cm = sonar.ping_cm();
  if (cm == 0)
  {
    cm = 250;
  }
  return cm;
}

void servoPozStanga() {
  pos = 44;
  motoras.write(pos);
}

void servoPozMijloc() {
  pos = 84;
  motoras.write(pos);
}

void servoPozDreapta() {
  pos = 124;
  motoras.write(pos);
}

void inainte() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(cuvantSelectie);
  lcd.setCursor(0,2);
  lcd.print("Miscare inainte");
  printByte(inainteMatrice);
  analogWrite(enA, vitezaPWM);
  analogWrite(enB, vitezaPWM);
  digitalWrite(motorStangaFata, HIGH);
  digitalWrite(motorStangaSpate, LOW);
  digitalWrite(motorDreaptaFata, HIGH);
  digitalWrite(motorDreaptaSpate, LOW);
}
void inapoi() {
  marsarierNanopixeli();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(cuvantSelectie);
  lcd.setCursor(0,2);
  lcd.print("Miscare inapoi");
  printByte(inapoiMatrice);
  analogWrite(enA, vitezaPWM);
  analogWrite(enB, vitezaPWM);
  digitalWrite(motorStangaFata, LOW);
  digitalWrite(motorStangaSpate, HIGH);
  digitalWrite(motorDreaptaFata, LOW);
  digitalWrite(motorDreaptaSpate, HIGH);
}
void dreapta() {
  semnalizareDreaptaNanopixeli();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(cuvantSelectie);
  lcd.setCursor(0,2);
  lcd.print("Miscare la dreapta");
  printByte(dreaptaMatrice);
  analogWrite(enA, vitezaPWM + 15);
  analogWrite(enB, vitezaPWM + 15);
  digitalWrite(motorStangaFata, HIGH);
  digitalWrite(motorStangaSpate, LOW);
  digitalWrite(motorDreaptaFata, LOW);
  digitalWrite(motorDreaptaSpate, LOW);
}

void stanga() {
  semnalizareStangaNanopixeli();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(cuvantSelectie);
  lcd.setCursor(0,2);
  lcd.print("Miscare la stanga");
  printByte(stangaMatrice);
  analogWrite(enA, vitezaPWM + 15);
  analogWrite(enB, vitezaPWM + 15);
  digitalWrite(motorStangaFata, LOW);
  digitalWrite(motorStangaSpate, LOW);
  digitalWrite(motorDreaptaFata, HIGH);
  digitalWrite(motorDreaptaSpate, LOW);
}

void stop() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(cuvantSelectie);
  lcd.setCursor(0,2);
  lcd.print("Miscare oprita");
  printByte(stopMatrice);
  analogWrite(enA, vitezaPWM);
  analogWrite(enB, vitezaPWM);
  digitalWrite(motorStangaFata, LOW);
  digitalWrite(motorStangaSpate, LOW);
  digitalWrite(motorDreaptaFata, LOW);
  digitalWrite(motorDreaptaSpate, LOW);
}

void miscareInainte() {
  if (!confirmareMiscareInainte)
  {
    confirmareMiscareInainte = true;
    inainte();
    inainte();
  }
}

void miscareInapoi() {
  confirmareMiscareInainte = false;
  inapoi();
  inapoi();
}

void intoarcereLaDreapta() {
  printByte(dreaptaMatrice);
  inapoi();
  delay(500);
  dreapta();
  delay(500);
  dreapta();
}

void intoarcereLaStanga() {
  printByte(stangaMatrice);
  inapoi();
  delay(500);
  stanga();
  delay(500);
  stanga();
}

//functie Afisare Matrice LED
void printByte(byte character []) {
  int i = 0;
  for(i=0;i<8;i++) {
    lc.setRow(0, i, character[i]);
  }
}

void marsarierNanopixeli() {
  leds[0] = CRGB(14, 11, 2);
  leds[1] = CRGB(14, 11, 2); 
  leds[2] = CRGB(14, 11, 2); 
  leds[3] = CRGB(14, 11, 2); 
  leds[4] = CRGB(14, 11, 2); 
  leds[5] = CRGB(14, 11, 2); 
  leds[6] = CRGB(14, 11, 2);  
  leds[7] = CRGB(14, 11, 2); 
  FastLED.show();
}

void semnalizareStangaNanopixeli() {
   for (int i = 8; i > 0; i--) {
    leds[i] = CRGB(255, 200, 20);
    FastLED.show();
    delay(50);
  }

  for (int i = 0; i < 8; i++) {
    leds[i] = CRGB(0, 0, 0);
    FastLED.show();
    delay(50);
  }
}

void semnalizareDreaptaNanopixeli() {
  for (int i = 0; i < 8; i++) {
    leds[i] = CRGB(255, 200, 20);
    FastLED.show();
    delay(50);
  }

  for (int i = 8; i > 0; i--) {
    leds[i] = CRGB(0, 0, 0);
    FastLED.show();
    delay(50);
  }
}


void monitorizareSenzori() {
  // Monitorizare senzori de obstacole
  Serial.print(" Senzori: ");
  Serial.print(" S1-");
  if (digitalRead(s1)) {
    Serial.print("OFF");
  } else {
    Serial.print("ON");
  }

  Serial.print(" Senzori: ");
  Serial.print(" S2-");
  if (digitalRead(s2)) {
    Serial.print("OFF");
  } else {
    Serial.print("ON");
  }

  Serial.print(" Senzori: ");
  Serial.print(" S3-");
  if (digitalRead(s3)) {
    Serial.print("OFF");
  } else {
    Serial.print("ON");
  }

  Serial.print(" Senzori: ");
  Serial.print(" S4-");
  if (digitalRead(s4)) {
    Serial.print("OFF");
  } else {
    Serial.print("ON");
  }

  Serial.print(" Senzori: ");
  Serial.print(" S5-");
  if (digitalRead(s5)) {
    Serial.print("OFF");
  } else {
    Serial.print("ON");
  }

  Serial.println("");

  Serial.print(" Buton: ");
  if (digitalRead(buton)) {
    Serial.print("OFF");
  } else {
    Serial.print("ON");
  }

  Serial.println("");
  Serial.print(" Senzor Obstacol: ");
  if (digitalRead(near)) {
    Serial.print("OFF");
  } else {
    Serial.print("ON");
  }
  Serial.println("");
  Serial.println("");

  delay(100);
}
