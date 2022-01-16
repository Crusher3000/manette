#include <Arduino.h>
// connect to wifi spot and mqtt server
#include <FreeRTOS.h>
#include <WiFi.h>
#include <PubSubClient.h>
//------------------------------------

// For OLED SSD1306-------------------
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GrayOLED.h>
//------------------------------------

// SSID and password for wifi spot----
const char* ssid = "AntoDB";
const char* password = "3049oQ7%";
//------------------------------------

// ID mqtt server --------------------
const char* mqtt_server = "192.168.137.189";
//------------------------------------

// connecting to wifi ----------------
WiFiClient espClient;
PubSubClient client(espClient);
//------------------------------------

String clientId = "Hello"; // ID client MQTT

TaskHandle_t Task1; // task oled
TaskHandle_t Task2; // task other

#define SCREEN_WIDTH 128 // OLED display width, in pixels 
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)

#define v_y 33 // pinout y joystick
#define v_x 32 // pinout x joystick
#define btn 25 // pinout button joystick
#define led 27 // pinout led
 
char data_x[5];   // char values x joystick to send
char data_y[5];   // char values y joystick to send
char data_T0[5];  // char values touch button (T0) to send
char data_T2[5];  // char values touch button (T2) to send
char send_btn[5]; // char values button to send
int value_x;      // variable x joystick
int value_y;      // variable y joystick
int battery;
int battery_map;
char str_battery[5];
char test_battery[5];
//--------- Touch button---------------------------
int value_T0;  // variable touch button (T0)
int value_T2;  // variable touch button (T2)
int value_VT0; // values touch button (T0)
int value_VT2; // values touch button (T2)

int counter1;  //evite des erreurs d'envoie sur T0
int counter2;  //evite des erreurs d'envoie sur T2

//********************************************
int send_VT0; // values of T0 send to MQTT
int send_VT2; // values of T2 send to MQTT
//********************************************
int pre_send_VT0; // values used to send (values of T0)when its differents 
int pre_send_VT2; // values used to send (values of T2)when its differents 
char puplish_VT0[5]; // send to MQTT in form of char*
char puplish_VT2[5]; // send to MQTT in form of char*
//--------------------------------------------------

//---------------------data-------------------------
int state1_joy = 0;   // valeur par defaut 0 utilisé pour le systeme d'envoie de donnée
int pre1_joy = 0;     // valeur par defaut 0 utilisé pour le systeme d'envoie de donnée
char etat1_joy[5];    // utiliser pour envoyer les données
int state2_joy = 0;   // valeur par defaut 0 utilisé pour le systeme d'envoie de donnée
int pre2_joy = 0;     // valeur par defaut 0 utilisé pour le systeme d'envoie de donnée
char etat2_joy[5];    // utiliser pour envoyer les données
int speed_v;          // utiliser pour récupérer la valeur du topic speed
int distance;         // utiliser pour récupérer la valeur du topic distance
char str_speed[10];   // utiliser pour la convertion et de pouvoir imprimer sur le oled
char str_distance[10];// utiliser pour la convertion et de pouvoir imprimer sur le oled
//--------------------------------------------------
int etat_led = 0;       // etat de la led par défaut
int ledState = LOW;     // the current state of LED
int lastButtonState;    // the previous state of button
int currentButtonState; // the current state of button 
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins) 

int OLEDclock;
int oldOLEDclock;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(String topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  
  char buffer1[length+1];// On crée une variable local de buffer
  for (int i = 0; i < length+1; i++) {// On relis tous les caractères reçus sur le topic
    buffer1[i] = (char)payload[i];// On enregistre tous les caractères que l'on a besoin (uniquement)
  }
  if (String(topic) == "speed") {// On vérifie si c'est le bon topic
    speed_v = atoi(buffer1);
    sprintf(str_speed, "%2u km/h", speed_v);
  }
  if (String(topic) == "distance") {// On vérifie si c'est le bon topic
    distance = atoi(buffer1);
    sprintf(str_distance, "%4u cm ", distance);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect

    if (client.connect("Hola")) {
      Serial.println("connected");  
      // Subscribe or resubscribe to a topic
      
      //subscribe to topic room/lamp
      client.subscribe("speed");
      //subscribe to topic room_R
      client.subscribe("distance");

      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void printToOLED(int x, int y,  char *message){
  /*
  *Function to print on SSD1306 oled
  */
  display.setCursor(x, y); // place the cursor 
  display.setTextColor(WHITE,BLACK); //Superposer les texte
  display.print(message);// write the text
  display.display();// put some delay
}
void Task1code( void * parameter ){
  /*
   * task use to print on oled 
   * anything imformation that need to  
   * be print on oled are done with 
   * this task
   */
  const TickType_t xDelayTask1 = 1 / portTICK_PERIOD_MS ;// delay
  for(;;){
    
    printToOLED(5,16,"Speed    :");  // afficher "Speed    :" sur le oled aux coordonnées 5 , 16
    printToOLED(70,16,str_speed);    // afficher la valeur dans str_speed sur le oled aux coordonnées 70 , 16
    printToOLED(5,26,"Distance :");  // afficher "Distance :" sur le oled aux coordonnées 5 , 26
    printToOLED(70,26,str_distance); // afficher la valeur dans str_distance sur le oled aux coordonnées 70 , 26
    printToOLED(5,36,"Klaxon   :");  // afficher "Klaxon   :" sur le oled aux coordonnées 5 , 36
    printToOLED(5,46,"Power    :");  // afficher "Power    :" sur le oled aux coordonnées 5 , 46
    printToOLED(70,46,str_battery);  // afficher la valeur dans str_battery sur le oled aux coordonnées 70 , 46
    printToOLED(85,46,"%");          // afficher "%" sur le oled aux coordonnées 85 , 46

    vTaskDelay( xDelayTask1 );// delay
    
    if (ledState == 1) // si ledState == 1
    {
    printToOLED(70, 36 ,"ON "); // afficher ON sur le oled aux coordonnées 70 , 36
    }
    else if (ledState!= 1)// si ledState est différent de 1
    {
      printToOLED(70, 36 ,"OFF");// afficher OFF sur le oled aux coordonnées 70 , 36
    }
    }
  vTaskDelay( xDelayTask1 ); // delay
  }

void Task2code( void * parameter ){
/*
cette tache faire tout sauf les affichages sur le oled
dans cette tache, on retrouve la lecture des touch button
du joystick, la lecture des topic mais il y a aussi les 
traitements de données et l'envoie des données
*/
   
  const TickType_t xDelayTask2 = 50 / portTICK_PERIOD_MS ;   // remplace la fonction delay (50 ms)
  const TickType_t xDelayTask2_2 = 50 / portTICK_PERIOD_MS ; // remplace la fonction delay (50 ms)
  const TickType_t xDelayTask2_3 = 50 / portTICK_PERIOD_MS ; // remplace la fonction delay (50 ms)
  for(;;){
    
    if (!client.connected()) {// Si le client pour le MQTT en WiFi n'est pas connecté
    reconnect(); // On appelle la fonction qui demande une reconnexion
  }
  client.loop();  
  
    value_x = analogRead(v_x); // lecture du joystick X
    value_y = analogRead(v_y); // lecture du joystick Y

//======================= Lecture de la batterie ====================================

    battery = analogRead(34); // lire des valeur analogie sur le pine 34
    battery_map = map(battery, 2000,3700,0,100); // remis à l'échelle 
    snprintf(str_battery,5,"%u",battery_map); // convertir le int en char* dans battery_map et le mettre dans str_battery

//===================================================================================

//======================= Touche button =============================================

    value_T0 = touchRead(T0); // lecture du touch button T0
    value_T2 = touchRead(T2); // lecture du touch button T2


    if (value_T0 <= 37) // si valeur touch button T0 <= 37
    {
      counter1 +=1; // ajoute +1 au counter1
      if (counter1 >3) // si reste appuyer ( counter1 >3)
      {
        send_VT0 = 1; // mettre 1 à send_VT0
      }
    }
    if (value_T2 <= 37) // si valeur touch button T2 <= 37
    {
      counter2+=1; // ajoute +1 au counter2
       if (counter2 >3) // si reste appuyer ( counter2 >3)
      {
        send_VT2 = 1; // mettre 1 à send_VT2
      }
    }
    if (value_T0 >= 50) // si valeur touch button T0 >= 50
    {
      send_VT0 = 0; // mettre 0 à send_VT0
      counter1 = 0; // remettre à 0 le counter1
    }
    if (value_T2 >= 40) // si valeur touch button T2 >= 50
    {
      send_VT2 = 0; // mettre 0 à send_VT2
      counter2 = 0; // remettre à 0 le counter2
    }
    
    snprintf(puplish_VT0,5,"%u",send_VT0); // convertir le int en char* dans send_VT0 et le mettre dans puplish_VT0
    snprintf(puplish_VT2,5,"%u",send_VT2); // convertir le int en char* dans send_VT2 et le mettre dans puplish_VT2

//== Pour envoyer les données que quand il y a un changement =========================

    if (send_VT0 != pre_send_VT0) // si send_VT0 est différent que pre_send_VT0
    {
      client.publish("moteur_cam1", puplish_VT0); // publie la donnée dans puplish_VT0 au topic moteur_cam1
      
    }

    if (send_VT2 != pre_send_VT2) // si send_VT2 est différent que pre_send_VT2
    {
      client.publish("moteur_cam2", puplish_VT2); // publie la donnée dans puplish_VT2 au topic moteur_cam2
    }
   
    pre_send_VT2 = send_VT2; // copier la donnée dans send_VT2 et la mettre dans pre_send_VT2
    pre_send_VT0 = send_VT0; // copier la donnée dans send_VT0 et la mettre dans pre_send_VT0

//=====================================================================================

    if (value_x>=2000){state1_joy = 1;} //si valeur joystick X lue est >= 2000, mettre 1 dans state1_joy
    if (value_x<=500){state1_joy = 2;} //si valeur joystick X lue est <= 500, mettre 2 dans state1_joy
    if (value_x<=2000 and value_x>=500){state1_joy = 0;} // si le joystick ne bouge pas, mettre 0 dans state1_joy
    
    if (value_y>=2000){state2_joy = 1;} //si valeur joystick Y lue est >= 2000, mettre 1 dans state2_joy
    if (value_y<=500){state2_joy = 2;} //si valeur joystick Y lue est <= 500, mettre 2 dans state2_joy
    if (value_y<=2000 and value_y>=500){state2_joy = 0;} // si le joystick ne bouge pas, mettre 0 dans state2_joy
    
    snprintf(etat1_joy,5,"%u",state1_joy); // convertir le int en char* dans state1_joy et le mettre dans etat1_joy
    snprintf(etat2_joy,5,"%u",state2_joy); // convertir le int en char* dans state2_joy et le mettre dans etat2_joy
    
//--------------------- Send only if joystick values are differences ----------------------------
    if (state1_joy != pre1_joy) // si state1_joy est different de pre1_joy
    {
      client.publish("drive",etat1_joy); // publie la valeur dans etat1_joy au topic drive
    }
    pre1_joy = state1_joy; // copier la donnée dans state1_joy et la mettre dans pre1_joy
    
    if (state2_joy != pre2_joy) // si state2_joy est different de pre2_joy
    {
      client.publish("turn",etat2_joy); // publie la valeur dans etat2_joy au topic turn
    }
    pre2_joy = state2_joy; // copier la donnée dans state2_joy et la mettre dans pre2_joy
//-------------------------------------------------------------------------------------------------

//----------------------utilisation posible pour une amélioration----------------------------------

    sprintf(data_x,"%4u",value_x);// convertir le int en char* dans value_x et le mettre dans data_x
    sprintf(data_y,"%4u",value_y);// convertir le int en char* dans value_y et le mettre dans data_y

    sprintf(data_T0,"%3u",value_T0);// convertir le int en char* dans value_T0 et le mettre dans data_T0
    sprintf(data_T2,"%3u",value_T2);// convertir le int en char* dans value_T2 et le mettre dans data_T2

//--------------------------------------------------------------------------------------------------

//--------------- Gestion toogle du bouton ------------------------
    lastButtonState    = currentButtonState; // save the last state
    currentButtonState = digitalRead(btn); // read new state

    if(lastButtonState == HIGH && currentButtonState == LOW) 
    {
      
      ledState = !ledState; // copier l'inverse de la donnée dans ledState et la mettre dans ledState
      if (ledState == 1) // si lesState est égal à 1
      {
        etat_led = 1; // mettre 1 dans etat_led
      }
      else if (ledState!= 1) // si ledState est différent de 1
      {
        etat_led = 0; // mettre 0 dans etat_led
      }
      vTaskDelay( xDelayTask2_2 );// delay toggle
      // control LED arccoding to the toggled state
      snprintf(send_btn,5,"%u",ledState); // convert values to send
      client.publish("buzzer",send_btn); // publish on topic "buzzer"
      digitalWrite(led, ledState); // change state led
    }

    vTaskDelay( xDelayTask2 ); // delay for(;;)
  }
}

void setup() {

  Serial.begin(115200); // initialise le Serial
  pinMode(btn,INPUT_PULLUP); // initialise le bouton
  pinMode(led,OUTPUT); // initialise la led
  
  setup_wifi(); // appele à la fonction setup_wifi()
  client.setServer(mqtt_server, 1883); // On se connecte au serveur MQTT
  client.setCallback(callback); // set la fonction callback
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  { // Address 0x3D for 128x64 
  for(;;); // Don't proceed, loop forever 
  }
  delay(2000); // delai de 2 secondes
  display.clearDisplay(); // nettoyer l'écran oled 
  xTaskCreatePinnedToCore(Task1code,"Task1",10000,NULL,1,&Task1,1); // creer et lance la tache
  delay(500); // delai de 500 mili secondes

  xTaskCreatePinnedToCore(Task2code,"Task2",10000,NULL,1,&Task2,0); // creer et lance la tache
  delay(500); // delai de 500 mili secondes
}

void loop() {
}