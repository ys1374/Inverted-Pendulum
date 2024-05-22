










// !!!!!!!!!!!!!!!!DO NOT CHANGE ANYTHING !!!!!!!!!!!!!!
// © 2021 All rights reserved for Yasin Salehi
//if you change even a dot in this script
//i won't stay down i'll bring you to marshall court
//and i beat you there then i will feed you to the SHARKS!!!!!
//HA HA HA HA HA ....  
//oh my god did you just believe that?
//HA HA HA HA HA ....
//i'm so funny oh lord...
//pardon my sense of humor of course you can change anything 
//but i recommendyou not to do so
//i wrote the script myself and now i don't even know what is it about
//anyway if you made any improvement by NOT CHANGEING the script
//feel free to write to me
//salehi.yasin@gmail.com
//Salehi-REGARDS
//stay safe - wear mask
























//-----------------------------------------------------
#include <Encoder.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <Keypad.h>

char hexaKeys[4][4] = {  {1, 2, 3, 13}, {4, 5, 6, 14}, {7, 8, 9, 15}, {11, 10, 12, 16}};
byte rowPins[4] =  {14, 15, 16, 17};
byte colPins[4] = {2, 3, 4, 5};
const int m = 5;
const int n = m - 1;
int k_triger = 0;
int cruser_column;
int chose = 0;

double theta = 0;
double theta_Setpoint = 0;
double Voltage;
double theta_kp, theta_ki, theta_kd;
double beta = 0;
double beta_Setpoint = 0;
double beta_kp = 1.25, beta_ki = 0, beta_kd = .1;

int time1 = 11;

Encoder pend_encoder(18, 19);
Encoder rod_encoder(20, 21);

PID theta_controller(&theta, &Voltage, &theta_Setpoint, theta_kp, theta_ki, theta_kd, DIRECT);
PID beta_controller(&beta, &theta_Setpoint, &beta_Setpoint, beta_kp, beta_ki, beta_kd, DIRECT);

LiquidCrystal lcd(50, 51, 8, 9, 10, 11);

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, 4, 4);

void setup() {

  Serial.begin(115200);

  lcd.begin(20, 4);

  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  lcd_first_print();
  defult_chose();
  controller_asignment();
    lcd.clear();
  while (time1 > 0) {
    time1 = time1 - 1;
        
    lcd.setCursor(4, 0);
    lcd.print("**WARNING**");
    lcd.setCursor(2, 1);
    lcd.print("The Process will");
        lcd.setCursor(6, 2);
    lcd.print("start in");
    lcd.setCursor(9, 3);
    lcd.print(time1);
    delay(1000);
    lcd.clear();
  }

  pend_encoder.write(5000);
  rotation_180();

    lcd.setCursor(4, 0);
    lcd.print("**WARNING**");
    lcd.setCursor(2, 1);
    lcd.print("Please DO NOT let");
        lcd.setCursor(0, 2);
    lcd.print("full cycle rotation");
    lcd.setCursor(5, 3);
    lcd.print("DISTURB IT");


}
void loop() {

  theta = pend_encoder.read();
  beta = rod_encoder.read();


  if (-1000 < theta & theta < 1000)
  {
    theta_controller.Compute();

    assign_Voltage_fun(Voltage);
    print_simple();

  }
  else
  {

    turn_stop();//-----------------------------------------
  }
}
//********************************************************************************
//***************************FUNCTIONS 1st part***********************************
//********************************************************************************

void turn_cw(int value) {
  analogWrite(6, value);
  analogWrite(7, 0);
}

void turn_ccw(int value) {
  analogWrite(6, 0);
  analogWrite(7, value);
}

void turn_stop() {
  analogWrite(6, 0);
  analogWrite(7, 0);
}
//--------------------------------
void assign_Voltage_fun(double Voltage_fun) {
  if (Voltage_fun < 1 & Voltage_fun > -1) {
    turn_stop;
  }
  if (Voltage_fun >= 1) {
    turn_cw(Voltage_fun);
  }
  if (Voltage_fun <= -1) {
    turn_ccw(abs(Voltage_fun));
  }
}
//------------------------------------
void print_simple() {
  Serial.print(theta) ;
  Serial.print("|||||||") ;
  Serial.print(beta) ;
  Serial.print("|||||||") ;
  Serial.print(theta_Setpoint) ;
  Serial.println("") ;
}
//------------------------------------
void rotation_180() {
  pend_encoder.write(5000);
  int theta_func;
  int beta_func;
  theta_func = pend_encoder.read();
  while (theta_func < 9000 &  theta_func > 1000) {
    theta_func = pend_encoder.read();
    beta_func = rod_encoder.read();

    while (beta_func != 50) {
      beta_func = rod_encoder.read();
      turn_cw(80);

      theta_func = pend_encoder.read();
      if (theta_func == 9000 || theta_func == 1000) {
        if (theta_func == 9000) {
          pend_encoder.write(-1000);
        }
        if (theta_func == -9000) {
          pend_encoder.write(1000);
        }
        return;
      }
    }

    while (beta_func != 0) {
      beta_func = rod_encoder.read();
      turn_ccw(200);
      theta_func = pend_encoder.read();
      if (theta_func == 9000 || theta_func == 1000) {
        if (theta_func == 9000) {
          pend_encoder.write(-1000);
        }
        if (theta_func == -9000) {
          pend_encoder.write(1000);
        }
        return;
      }
    }

  }
}
//------------------------------------------------------
void controller_asignment() {
  theta_controller.SetMode(AUTOMATIC);
  theta_controller.SetOutputLimits(-255, 255);
  theta_controller.SetSampleTime(10);
  theta_controller.SetTunings(theta_kp, theta_ki, theta_kd);

  beta_controller.SetMode(AUTOMATIC);
  beta_controller.SetOutputLimits(-150, +150);
  beta_controller.SetSampleTime(3);
  beta_controller.SetTunings(beta_kp, beta_ki, beta_kd);
}

//********************************************************************************
//***************************FUNCTIONS 2nd part***********************************
//********************************************************************************

//FUNCTION read k value from keypad  --------------------------------------------
double k(int cruser_row) {
  double k1, k2, k;
  int k1_digits[m] = {0};
  int k1_digits_old[m] = {0};

  int k2_digits[m] = {0};
  int k2_digits_old[m] = {0};

  int oldkey  = 0;
  int exit_1  = 0;
  int dot_key = 0;
  int p = 0;
  int lcd_cruser = 4;


  while (exit_1 == 0) {

    int Key_k = customKeypad.getKey();

    if (Key_k != 0 & Key_k == 12) {
      exit_1 = exit_1 + 1;
      Serial.println("12 is pushed");
      delay(500);
    }
    if (Key_k != 0  & Key_k == 11) {
      lcd.setCursor(lcd_cruser, cruser_row);
      lcd.print(".");
      dot_key = dot_key + 1;
      Serial.println("11 is pushed");
      delay(500);
    }


    // >1 digits---------------------------------------------------
    if (Key_k != 0 & dot_key == 0) {

      //push 1-9 number keys(key 1-9)------------------------------
      if (Key_k < 10) {
        lcd.setCursor(lcd_cruser, cruser_row);
        lcd.print(Key_k);
        k1_digits[0] = Key_k;

        for (int i = 0; i < n ; i++) {
          k1_digits[i + 1] = k1_digits_old[i];
        }


        for (int j = 0; j < n + 1 ; j++) {
          k1_digits_old[j] = k1_digits[j];
        }


        for (int t = 0; t < n + 1; t++) {
          Serial.print(k1_digits[p]);
        }
        Serial.println("");

      }
      //push 0 number (key 10)------------------------------
      if (Key_k == 10) {
        k1_digits[0] = 0;
        lcd.setCursor(lcd_cruser, cruser_row);
        lcd.print(0);
        for (int i = 0; i < n ; i++) {
          k1_digits[i + 1] = k1_digits_old[i];
        }


        for (int j = 0; j < n + 1 ; j++) {
          k1_digits[j] = k1_digits_old[j];
        }


        for (int t = 0; t < n + 1; t++) {
          Serial.print(k1_digits[p]);
        }
        Serial.println("");

      }
      //--------------------------------------------------------
      k1 = k_1(k1_digits);
      delay(500);
      lcd_cruser = lcd_cruser + 1;

    }
    // 0< <1 digits------------------------------------------------------
    if (Key_k != 0  & dot_key == 1) {

      //push 1-9 number keys(key 1-9)------------------------------
      if (Key_k < 10) {

        lcd.setCursor(lcd_cruser, cruser_row);
        lcd.print(Key_k);

        k2_digits[p] = Key_k;
        p = p + 1;

        for (int i = 0; i < n + 1; i++) {
          Serial.print(k2_digits[i]);
        }
        Serial.println("");


      }
      //push 0 number (key 10)------------------------------
      if (Key_k == 10) {

        k2_digits[p] = 0;
        p = p + 1;

        lcd.setCursor(lcd_cruser, cruser_row);
        lcd.print(0);

        for (int i = 0; i < n + 1; i++) {
          Serial.print(k2_digits[i]);
        }
        Serial.println("");

      }
      //--------------------------------------------------------
      k2 = k_2(k2_digits);
      delay(500);
      lcd_cruser = lcd_cruser + 1;

    }
    //--------------------------------------------------------------
  }

  k =  k1 + k2;

  return k;

}
//FUNCTION tunrn digits to number for >1 numbers--------------------------------------------------------------
int k_1(int k_11[m]) {
  float num = k_11[0] + (k_11[1] * 1e1) + (k_11[2] * 1e2) + (k_11[3] * 1e3) + (k_11[4] * 1e4);
  return num;
}
//FUNCTION tunrn digits to number for 0< <1 numbers----------------------------------------------------------
double k_2(int k_22[m]) {
  double num;
  //  if (k_22[0] == 0 & k_22[1] == 0 & k_22[2] == 0 & k_22[3] == 0 & k_22[4] == 0) {
  //    return num = 0.0;
  //  }

  num = (k_22[0] / 1e1) + (k_22[1] / 1e2) + (k_22[2] / 1e3) + (k_22[3] / 1e4) + (k_22[4] / 1e5);
  return num;
}
//FUNCTION lcd first print -----------------------------------------------------------------------------------
void lcd_first_print() {
  lcd.setCursor(0, 0);
  lcd.print("B.Sc. Final Proj. of");

  lcd.setCursor(0, 2);
  lcd.print("YASIN SALEHI");

  lcd.setCursor(0, 3);
  lcd.print("1398-1400");

  delay(7000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Donated to");

  lcd.setCursor(0, 2);
  lcd.print("Kurdistan University");


  delay(5000);
  lcd.clear();
  lcd.setCursor(0, 0);
}
//FUNCTION get k values -----------------------------------------------------------------------------------
void get_k_values() {
  lcd.setCursor(0, 0);
  lcd.print("Please Enter K Gain ");
  lcd.setCursor(0, 1);
  lcd.print("Values");
  delay(3000);
  lcd.clear();

  if (k_triger == 0) {
    cruser_column = 0;
    lcd.setCursor(15, cruser_column); lcd.print("*10^3");
    lcd.setCursor(0, cruser_column);
    lcd.print("Kp=");
    theta_kp = k(cruser_column);
    k_triger = k_triger + 1;
    Serial.println(""); Serial.print("theta_kp is set"); Serial.print(theta_kp, 5); Serial.println("");
  }

  if (k_triger == 1) {
    cruser_column = 1;
    lcd.setCursor(15, cruser_column); lcd.print("*10^3");
    lcd.setCursor(0, cruser_column);
    lcd.print("Ki=");
    theta_ki = k(cruser_column);
    k_triger = k_triger + 1;
    Serial.println(""); Serial.print("theta_ki is set"); Serial.print(theta_ki, 5); Serial.println("");

  }


  if (k_triger == 2) {
    cruser_column = 2;
    lcd.setCursor(15, cruser_column); lcd.print("*10^3");
    lcd.setCursor(0, cruser_column);
    lcd.print("Kd=");
    theta_kd = k(cruser_column);
    k_triger = k_triger + 1;
    Serial.println(""); Serial.print("kd is set"); Serial.print(theta_kd, 5); Serial.println("");
  }
}

//FUNCTION defult K chose----------------------------
void defult_chose() {
  lcd.print("Do you whant to");
  lcd.setCursor(0, 1);
  lcd.print("Continue with defult");
  lcd.setCursor(0, 2);
  lcd.print("K Gain Values ?");
  lcd.setCursor(0, 3);
  lcd.print("Yes-->1  No-->2");

  while (chose < 1) {
    int Key = customKeypad.getKey();
    if (Key == 1) {
      lcd.clear();
      theta_kp = 1.2, theta_ki = .8, theta_kd = .05;
 //     theta_kp = 1.674, theta_ki = 16.872, theta_kd = .042;

      lcd.setCursor(0, 0);
      lcd.print("Kp= "); lcd.print(theta_kp);
      lcd.setCursor(15, 0); lcd.print("*10^3");
      lcd.setCursor(0, 1);
      lcd.print("Ki= "); lcd.print(theta_ki);
      lcd.setCursor(15, 1); lcd.print("*10^3");
      lcd.setCursor(0, 2);
      lcd.print("Kd= "); lcd.print(theta_kd);
      lcd.setCursor(15, 2); lcd.print("*10^3");
      delay(3000);

      chose = 1;
    }
    if (Key == 2) {
      lcd.clear();
      get_k_values();
      chose = 2;
    }
  }
}
