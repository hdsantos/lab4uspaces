#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_system.h" // Para usar timers
#include <ArduinoJson.h>
#include "time.h"       // Para sincronismo com NTP (ex: ESP32->Time) 
#include "sntp.h"
#include <MQUnifiedsensor.h>
#include "Adafruit_LTR390.h" //Sensor UV trabalho com I2C e não precisa que os pinos sejam declarados

// Configuração WiFi, MQTT, JSON, NTP e o nó
const char *NODEname = "/node1"; // para efeitos da publicação no MQTT - NÂO ESTÁ A SER USADO
const char *SSID = "GILV36";
const char *PWD = "1960aa1995";
const char *MQTTServer = "192.168.1.53";
const char *MQTT_user = "iotdev";
const char *MQTT_pass = "Qwe123!";
//const char *MQTTServer = "192.168.233.156";
//const char *MQTT_user = "mqtt_user";
//const char *MQTT_pass = "123abc!";
int MQTTPort = 1883;
StaticJsonDocument<384> dist;  // Comporta até 6 sensores de posição
StaticJsonDocument<512> amb;  // DHT com Temperatura, Humidade, Index de calor, PhotR, MQ-135 (6 pares) e LTR390-UV
StaticJsonDocument<64> snd;   // Detetor de Som apenas com um par som:valor (além do timestamp)
const char* ntpServer1 = "3.pt.pool.ntp.org";
const char* ntpServer2 = "0.europe.pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;
// TimeZone rule for Europe/Lisbon including daylight adjustment rules (optional)
const char* time_zone = "WET-0WEST-1,M3.5.0/01:00:00,M10.5.0/02:00:00";

// Sensor MQ-135 (CO, CO2, Alchool, Toluen, NH4)
// Estes parametros devem ser ajustados para a placa usada, assim como o valor R0
// determinado pela calibração e 
#define MQ_135R0  7.52           // Determinado durante a fase de calibração (usado no setup) - ver MQ-135.ino
#define Board ("ESP32")
#define Pin135 (13)                //Analog input 2 (ADC2_4)
#define RatioMQ135CleanAir (3.6)  //RS / R0   = 10 ppm 
#define ADC_Bit_Resolution (12)   // 12 bit ADC 
#define Voltage_Resolution (4.66) //   Volt resolution to calc the voltage
#define Type ("MQ-135")           //Type of sensor used
//Declare Sensor
MQUnifiedsensor MQ135(Board, Voltage_Resolution,   ADC_Bit_Resolution, Pin135, Type);

// define num. pinos e configurações específicas de sensores
// -- Sensores de distancia (vector para acomodar vários sensores). Adicionar pinos na lista de Trigger e Echo
const int MaxSensDist = 6; // Numero maximo de sensores de distancia
const int NumSensDist = 1; // numero atual de sensores de distancia
int trigPin[NumSensDist] = {14}; //Trigger pins
int echoPin[NumSensDist] = {12}; //Echo pins

// -- Sensor de temp e humidade
#include "DHT.h"
const int TempHumD = 16;
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
DHT dht(TempHumD, DHTTYPE); // Inicializa o sensor DHT

// -- Sensor fotoresist (luminosidade) RM2.20
const int PhotRAn = 17;

// -- Detetor de som
//const int DetSomG = 15; // Gate: saida acima ou abaixo do threshold (hw) - nao vou usar
const int DetSomEnv = 4; // Envelope: nível do som
// leitura dos sensores de ambiente a cada 10 segundos
const int ambCont = 10; 
const int sampleWindow = 50;  //janela de amostragem para o ruído: 50ms (20MHz)
int Sound = 0;                // ultima leitura

// LTR390 sensor de UV - inicializa objeto
Adafruit_LTR390 ltr = Adafruit_LTR390();

// definição de variáveis
hw_timer_t *timer = NULL; // Declara um timer
float distance[NumSensDist]; // Vetor para ler array de sensoser de distancia
float Temp = 0.0;
float Hum = 0.0;
float CalorIndex = 0.0;
int Luz = 0;
int amb_ctr = ambCont; // Variável para controlar a execução dos sensores de ciclo longo no loop (ambiente)
char data[256]; // Buffer para enviar string a MQTT -> MAX 256 bytes
char pubDist[] = "/node1/dist";
char pubAmb[] = "/node1/amb";
char pubSnd[] = "/node1/snd";

// --- Funcoes auxiliares
// Bloco para ligar a WiFi
// connectToWiFi()
void connectToWiFi() {
  Serial.print("Connecting to ");
  WiFi.begin(SSID, PWD);
  Serial.println(SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("Connected.");  
}

// Bloco de registo cliente MQTT incluindo subscrever um tópico
// setupMQTT() para iniciair a ligação
// reconnect() para garantir que estabelece a ligaçaõ antes de aceder 
// MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); // Cria objeto MQTT

void setupMQTT() {
  mqttClient.setServer(MQTTServer, MQTTPort);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
      Serial.println("Reconnecting to MQTT Broker..");
      String clientId = "NODE1-";
      clientId += String(random(0xffff), HEX); // change ID each time of reconnecting
      if (mqttClient.connect(clientId.c_str(), MQTT_user, MQTT_pass)) {
      //if (mqttClient.connect(clientId.c_str())) {
        Serial.println("Connected.");
        // subscribe to topic
        //mqttClient.subscribe("/swa/commands"); // segundo para qos=0,1 (default 0)
      } 
  }
}

// Leitura dos sensores de distância
float LeSensorDist(int TrigP, int EchoP) {
  long duration;
  float dist;
  // Clears the trigPin
  digitalWrite(TrigP, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TrigP, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigP, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(EchoP, HIGH);
  // Calculating the distance
  dist = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Dist: ");
  Serial.println(dist);
  return(dist);
}

// Leitura do sensor de temp e humidade DHT
int LeSensorTH(float* t, float* h, float* cIndex) {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  *h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  *t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float f = dht.readTemperature(true);
  // Check if any reads failed and exit early
  //if (isnan(h) || isnan(t) || isnan(f)) {
  if (isnan(*h) || isnan(*t)) {
    return 0;
  }
  // Compute heat index in Fahrenheit (the default)
  //float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  *cIndex = dht.computeHeatIndex(*t, *h, false);

  Serial.print(" Hum: ");
  Serial.print(*h);
  Serial.print(" Temp: ");
  Serial.print(*t);
  Serial.print("°C ");
  //Serial.print(f);
  Serial.print(" Heat index: ");
  Serial.print(*cIndex);
  Serial.print("°C ");
  return 1;
}

// Verifica se tempo local via NTP está sync e espera, se não estiver
void printLocalTime() {
  struct tm timeinfo;
  int firstTime = true;

  while(!getLocalTime(&timeinfo)){
    if (firstTime) {
      Serial.println("Waiting for time sync...");
      firstTime = !firstTime;
    }
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t) {
  Serial.println("Got time adjustment from NTP!");
  //printLocalTime();
}

// Mede o tempo em ms que decorreu desde o valor em prevT até ao momento atual.
// Atualiza o valor de prevT com o tempo atual. Escreve o delat na consola se prt=true 
void measureDelay(uint64_t *prevT, bool prt) {
  uint64_t actT = timerReadMilis(timer);
  if (prt) {
    Serial.print(" # Delta = ");
    Serial.println(actT - *prevT);
  }
  *prevT = actT;
}
// --- fim de funções auxiliares

void setup() {
  int i;
  Serial.begin(115200); // Starts the serial communication
  connectToWiFi();
  
  // Ativa todos os pinos dos sensores de distancia (lista)
  for (i = 0; i < NumSensDist; i++) { 
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }
  pinMode(TempHumD, INPUT);
  pinMode(PhotRAn, INPUT);
  //pinMode(DetSomG, INPUT);
  pinMode(DetSomEnv, INPUT);
  dht.begin(); // Inicializa o sensor de Temp/Humidade DHTxx
  
  // Inicializa o sensor de qualidade do ar MQ-135
  // https://github.com/miguel5612/MQSensorsLib/blob/master/examples/ESP32/esp32.ino
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.setR0(MQ_135R0);        // Valor obtido na calibração prévia, para este sensor e a board
  MQ135.init();

  // Inicializa sensor UV LTR390
  if (ltr.begin()) {
    ltr.setMode(LTR390_MODE_UVS);
    if (ltr.getMode() == LTR390_MODE_ALS)
      Serial.println("Modo ALS");
      else
        Serial.println("Modo UVS");
    ltr.setGain(LTR390_GAIN_3);    //Ganho pode ser 1,3,6,9, ou 18
    Serial.print("Ganho: ");
    switch (ltr.getGain()) {
      case LTR390_GAIN_1: Serial.println(1); break;
      case LTR390_GAIN_3: Serial.println(3); break;
      case LTR390_GAIN_6: Serial.println(6); break;
      case LTR390_GAIN_9: Serial.println(9); break;
      case LTR390_GAIN_18: Serial.println(18); break;
    }
    ltr.setResolution(LTR390_RESOLUTION_16BIT);
    Serial.print("Resolution : ");
    switch (ltr.getResolution()) {
      case LTR390_RESOLUTION_13BIT: Serial.println(13); break;
      case LTR390_RESOLUTION_16BIT: Serial.println(16); break;
      case LTR390_RESOLUTION_17BIT: Serial.println(17); break;
      case LTR390_RESOLUTION_18BIT: Serial.println(18); break;
      case LTR390_RESOLUTION_19BIT: Serial.println(19); break;
      case LTR390_RESOLUTION_20BIT: Serial.println(20); break;
    }
    ltr.setThresholds(100, 1000);
    ltr.configInterrupt(true, LTR390_MODE_UVS);
  }  
  else {
    Serial.println("Sensor UV LTR390 não encontrado");
    while(1) delay(10);
  }

  setupMQTT(); 
  // set notification call-back function
  sntp_set_time_sync_notification_cb(timeavailable);
  // This will set configured ntp servers and constant TimeZone/daylightOffset
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  timer = timerBegin(0, 80, true); // Configura o timer 0, div 80, countUP
  // Assegura que o tempo local foi devidamente sincronizado
  printLocalTime();
}

// O loop tem um ciclo de 0,5 segundos. Os sensores de ambiente não necessitam de ser lidos
// com essa frequência. São usados contadores globais para controlar a execução destes sensores
void loop() {
  uint64_t iniTime, prevTime, loopTime;
  time_t currentTime;
  struct tm timeinfo;
  float CO, Alcohol, CO2, Toluen, NH4, Aceton;
  unsigned long startMillis = millis();   // para o calculo do ruido em db
  float peakToPeak = 0;
  uint signalMax = 0;
  uint signalMin = 4095;

  getLocalTime(&timeinfo);
  time(&currentTime);       // Transforma estrutura localtime em Unix time (Epoch: seg. desde 1/1/1970) - TAMESTAMP
  //Serial.print("->Time: ");
  //Serial.println(currentTime);
  timerWrite(timer, 0);             //reset do timer para o loop
  iniTime = timerReadMilis(timer);  // Guarda valor inicial do timer em ms
  //Serial.print("Ini time is = ");
  //Serial.println(iniTime);
  prevTime = iniTime;
  
  // Leitura dos sensores de distancia => Tempo medido para 1 sensor 14ms
  // e construção do objeto JSON
  dist["time"] = currentTime;
  for (int i = 0; i < NumSensDist; i++) {
    distance[i] = LeSensorDist(trigPin[i], echoPin[i]);
    dist["SDist"]["id"+String(i)] = distance[i];
    }
  serializeJson(dist, data);
  // envia para MQTT
  if (!mqttClient.connected())
    reconnect();
  mqttClient.loop();
  mqttClient.publish(pubDist, data);
  // atualiza dados temporais
  measureDelay(&prevTime, true); // Mede mas não mostra o tempo gasto na etape (debug). Atualiza tempo atual
  
  // Leitura do sensor de deteção de som. A amostragem vai ser à velocidade máxima
  // mas a saída será gerada apenas se houver variação superior a 5%
  while (millis() - startMillis < sampleWindow) {
    uint rdSom = analogRead(DetSomEnv);
    if (rdSom < 4095) {       // vai ignorar valores erroneos!
      if (rdSom > signalMax)  // e ajusta valores maximo e minimo
        signalMax = rdSom;
      else if (rdSom < signalMin)
        signalMin = rdSom;
    }
  }
  peakToPeak = signalMax - signalMin;
  int db = map(peakToPeak,20,900,49.5,90);  //calibra para deciBels
  //int db = map(peakToPeak,20,200,49.5,20);
  //uint rdSom = analogRead(DetSomEnv);
  Serial.print(" db: ");
  //Serial.print(Sound); Serial.print(" ");
  Serial.println(db);
  //if (abs(Sound - rdSom) >= (int) (0.05 * Sound)) { // Apenas altera se o valor for diferente 5%
  //  Sound = rdSom;                                  // relativamente ao previamente guardado
  snd["time"] = currentTime;
  snd["DB"] = db;
  serializeJson(snd, data);
  if (!mqttClient.connected())
    reconnect();
  mqttClient.loop();
  mqttClient.publish(pubSnd, data);
  measureDelay(&prevTime, false);  // Mede mas não mostra o tempo gasto na etape (debug). Atualiza tempo atual
  //} */
  
  // Leitura de sensores de ambiente e construção do objeto JSON
  if (amb_ctr > 1)
    amb_ctr--;           // atualiza contador de passagens (apenas executa quand chega a 1)
    else {
      amb_ctr = ambCont; // Reinicializa o contador de passagens
      // Leitura do sensor TH => Tempo medido 24 ou 25ms
      amb["time"] = currentTime; // a menos que se use um ciclo de mais que 1 segundo!
      if (!LeSensorTH(&Temp, &Hum, &CalorIndex)) 
        Serial.println("Erro ao ler sensor TH, valores não alterados.");
      amb["DHT"]["Temp"] = Temp;
      amb["DHT"]["Hum"] = Hum;
      amb["DHT"]["HeatIdx"] = CalorIndex;
      // Leitura do sensor fotoresistivo => Tempo medido 0ms
      Luz = analogRead(PhotRAn);
      Serial.print(" IntLuz: ");
      Serial.println(Luz);
      amb["PhR"] = Luz;
      MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
      // A leitura de cada valor é precedida da ativação dos parametros 'a' e 'b' para dar valores em ppm
      // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
      /* Motivation: We have added 400 PPM because when the library is calibrated it assumes the current
       state of the air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is
       around 400 PPM.
       https://www.lavanguardia.com/natural/20190514/462242832581/concentracion-dioxido-cabono-co2-atmosfera-bate-record-historia-humanidad.html
      */
      MQ135.setA(605.18); MQ135.setB(-3.937);
      amb["mq"]["CO"] = CO = MQ135.readSensor();
      MQ135.setA(77.255); MQ135.setB(-3.18);
      amb["mq"]["alc"] = Alcohol = MQ135.readSensor();
      MQ135.setA(110.47); MQ135.setB(-2.862);
      amb["mq"]["CO2"] = CO2 = MQ135.readSensor() + 400;
      MQ135.setA(44.947); MQ135.setB(-3.445);
      amb["mq"]["tol"] = Toluen = MQ135.readSensor();
      MQ135.setA(102.2 ); MQ135.setB(-2.473);
      amb["mq"]["NH4"] = NH4 = MQ135.readSensor();
      MQ135.setA(34.668); MQ135.setB(-3.369);
      amb["mq"]["ace"] = Aceton = MQ135.readSensor();
      Serial.print(" | "); Serial.print(CO); 
      Serial.print(" | "); Serial.print(Alcohol);
      Serial.print(" | "); Serial.print(CO2); 
      Serial.print(" | "); Serial.print(Toluen); 
      Serial.print(" | "); Serial.print(NH4); 
      Serial.print(" | "); Serial.print(Aceton);
      Serial.println(" |"); 
   
      if (ltr.newDataAvailable()) {
        int valorUV = ltr.readUVS();
        Serial.printf("Valor UV: %d\n", valorUV);
        amb["UV"] = valorUV;
      }

      serializeJson(amb, data);
      // envia para MQTT
      if (!mqttClient.connected())
        reconnect();
      mqttClient.loop();
      mqttClient.publish(pubAmb, data);
      measureDelay(&prevTime, false);  // Mede mas não mostra o tempo gasto na etape (debug). Atualiza tempo atual         
    } 
  //--------- loop time eval & delay
  Serial.print("loop time is = ");
  int loopT = (prevTime - iniTime);
  Serial.println(loopT); //should be less than 500 - NOT TESTED
  if (loopT > 0 && loopT < 500)
    delay(500 - loopT);
}
