#include <Servo.h>

#define BUTTON_PIN 2    // Pin du bouton poussoir
#define SERVO_PIN A2     // Pin de contrôle du servomoteur
#define Led_PIN 3

Servo myservo; // Crée un objet servo pour contrôler un servo moteur
int buttonState = 0;    // Variable pour stocker l'état précédent du bouton poussoir
int servoState = 0;     // Variable pour stocker l'état du servomoteur
int servoAngle = 0;  // Angle de rotation du servomoteur
const int pinCapteurForce = A1; // Broche analogique à laquelle le capteur de force est connecté
int seuilForce = 600; // Seuil de force pour arrêter le servo moteur
unsigned long previousMillis = 0; // Variable pour stocker le temps écoulé depuis le dernier changement d'état de la LED
const long interval = 500;
const long interval2 = 5000;  
// Intervalle de temps pour le clignotement de la LED (en millisecondes)
unsigned long currentMillis = millis(); 
const int BATTERYPIN = A0; //pin de la batterie

const float TensionMin = 3; //tension min
const float TensionMax = 2; //tension max

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

int seconds = 0;

LiquidCrystal_I2C lcd_1(0x3F, 16, 2);

void setup() {
  Serial.begin(9600); // Attache le servo moteur à la broche 9
  pinMode(pinCapteurForce, INPUT); // Configure la broche du capteur de force en entrée
  pinMode(BUTTON_PIN, INPUT);  // Configure le bouton poussoir comme une entrée
  myservo.attach(SERVO_PIN);// Attache le servomoteur au pin
  pinMode(Led_PIN, OUTPUT);
  Serial.begin(9600);
  lcd_1.begin();
  lcd_1.backlight();
  lcd_1.clear();

  
} 
int getBattery ()
{
  float b = analogRead(BATTERYPIN); //valeur analogique
  int minValue = (1023 * TensionMin) / 5; 
  int maxValue = (1023 * TensionMax) / 5; 


  b = ((b - minValue) / (maxValue - minValue)) * 100; //mettre en pourcentage

  if (b > 100) //max is 100%
    b = 100;

  else if (b < 0) //min is 0%
    b = 0;
  int valeur = b;
  return b;
}
void loop() {
  int currentButtonState = digitalRead(BUTTON_PIN);  // Lit l'état actuel du bouton poussoir 
  int LedState = digitalRead(Led_PIN);
   // Si le bouton poussoir est enfoncé et l'état précédent était relâché
  if (currentButtonState == HIGH && buttonState == LOW) {
    // Inverse l'état du servomoteur à chaque pression du bouton
    if (servoState == 0) {
      servoAngle = 180;  // Définir l'angle à 90 degrés (exemple)
      servoState = 1;   // Activer le servomoteur  
    } else {
      servoAngle = 0;   // Définir l'angle à 0 degrés (exemple)
      servoState = 0;   // Désactiver le servomoteur
    }
    myservo.write(servoAngle);  // Envoyer la position au servomoteur
  }
  if (currentButtonState == LOW ) {
    // Allume la LED et vérifie l'intervalle pour l'éteindre
    digitalWrite(Led_PIN, HIGH);
    }
   if (LedState == HIGH) {
       
      if (millis()-currentMillis >= interval) {
         digitalWrite(Led_PIN,LOW);
         currentMillis = millis();
       }
    }
  int readforce = analogRead(pinCapteurForce); // Lit la valeur du capteur de force
  Serial.println(readforce);
  if (readforce >= seuilForce) {
    // Si la force détectée dépasse le seuil, arrête le servo moteur
     myservo.detach();
  }
  else{
    myservo.attach(SERVO_PIN);
  }
  
  buttonState = currentButtonState;
  Serial.print(getBattery());
 lcd_1.setCursor(0,0);
 lcd_1.print("batterie:");
 lcd_1.print(getBattery());
 lcd_1.print("%");
}
