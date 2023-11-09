/* 
  V2 23/10/2023
    - removi a parte do código relativa ao sensor de hum/temp DHT, pois o BME280 realiza as mesmas leituras
  v1 22/09/2023
    - leitura de sensores de distância passa a ser em cm (valor inteiro)
    - apenas é enviado para o MQTT o array de leituras, se houver algum sensore que reporte
      um valor diferente (adicionado o vector prev_dist)
    - aumentado o contador de iterações para leitura dos sensores de ambiente (valor 20, para 10seg)
      mas justifica-se aumentar ainda mais, depois dos testes
    - valores decimais normalizados para 1 ou 2 casas decimais; alguns para inteiros (em particular a distância)
*/
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_system.h" // Para usar timers
#include <ArduinoJson.h>
#include "time.h"       // Para sincronismo com NTP (ex: ESP32->Time) 
#include "sntp.h"
#include "Adafruit_LTR390.h" //Sensor UV trabalho com I2C e não precisa que os pinos sejam declarados
#include "Adafruit_BME280.h" //Sensor Temp/Press/Hum com I2C
#include <MQUnifiedsensor.h>

// Configuração WiFi, MQTT, JSON, NTP e o nó
/*const char *SSID = "GILV36";
const char *PWD = "1960aa1995";
const char *MQTTServer = "192.168.1.53";
const char *MQTT_user = "iotdev";
const char *MQTT_pass = "Qwe123!";*/
const char *SSID = "ASUS WL-500";
const char *PWD = "1UD5WYDAD7HA0imq";
const char *MQTTServer = "192.168.233.156";
const char *MQTT_user = "mqtt_user";
const char *MQTT_pass = "123abc!";
int MQTTPort = 1883;
StaticJsonDocument<384> dist;  // Comporta até 6 sensores de posição
StaticJsonDocument<512> amb;  // BME280 (press; temp; hum), PhotR, MQ-135 (6 pares) e LTR390-UV
StaticJsonDocument<64> snd;   // Detetor de Som apenas com um par som:valor (além do timestamp)
const char* ntpServer1 = "3.pt.pool.ntp.org";
const char* ntpServer2 = "0.europe.pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;
// TimeZone rule for Europe/Lisbon including daylight adjustment rules (optional)
const char* time_zone = "WET-0WEST-1,M3.5.0/01:00:00,M10.5.0/02:00:00";

// Sensor MQ-135 (CO, CO2, Alchool, Toluen, NH4)
// Sensor MQ-131 ()
// Estes parametros devem ser ajustados para a placa usada, assim como o valor R0
// determinado pela calibração, que tem que executar no setup()
#define Board "ESP-32"
#define Type1 "MQ-135"         // Type of sensor used
#define Voltage_Resolution 5 // Volt resolution to calc the voltage
#define Pin135 34              
#define ADC_Bit_Resolution 12  // 12 bit ADC 
#define RatioMQ135CleanAir 3.6 // RS / R0 = 10 ppm 
//Declare air quality Sensors
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin135, Type1);

// define num. pinos e configurações específicas de sensores
// -- Sensores de distancia (vector para acomodar vários sensores). Adicionar pinos na lista de Trigger e Echo
const int MaxSensDist = 6; // Numero maximo de sensores de distancia
const int NumSensDist = 4; // numero atual de sensores de distancia
int trigPin[NumSensDist] = {14,15,4,17}; //Trigger pins
int echoPin[NumSensDist] = {12,13,36,37}; //Echo pins

// -- Sensor Pressão, Temperatura e Humidade BME280 (I2C)
Adafruit_BME280 bme; // Inicializa um objecto -> bme
#define SEALEVELPRESSURE_HPA (1013.25) // Pressão média ao nível do mar

// -- Sensor fotoresist (luminosidade) RM2.20
const int PhotRAn = 32; 

// -- Detetor de som
//const int DetSomG = 15; // Gate: saida acima ou abaixo do threshold (hw) - nao vou usar
const int DetSomEnv = 33; // Envelope: nível do som
const int sampleWindow = 25;  //janela de amostragem para o ruído: 50ms (20MHz?!) 25 parece ser suficiente
int Sound = 0;                // ultima leitura

// leitura dos sensores de ambiente a cada 10 segundos
const int ambCont = 10; 

// LTR390 sensor de UV - inicializa objeto
Adafruit_LTR390 ltr = Adafruit_LTR390();

// definição de variáveis
hw_timer_t *timer = NULL; // Declara um timer
float distance[NumSensDist]; // Vetor para ler array de sensor de distancia (cm, valor inteiro)
int prev_dist[NumSensDist];  // Vetor de leituras previas de distancia (inicializar a 0)
float bTemp = 0.0;
float bHum = 0.0;
uint bPress = 0;
float bAlt = 0.0;
int Luz = 0;
int amb_ctr = ambCont; // Variável para controlar a execução dos sensores de ciclo longo no loop (ambiente)
char data[256]; // Buffer para enviar string a MQTT -> MAX 256 bytes
char pubDist[] = "/node1/dist";
char pubAmb[] = "/node1/amb";
char pubSnd[] = "/node1/snd";

// --- Funcoes auxiliares
// Bloco para ligar a WiFi
void connectToWiFi() {
  Serial.print("Connecting to ");
  WiFi.begin(SSID, PWD);
  Serial.print(SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected.");  
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
  //Serial.print("Dist: ");
  //Serial.println(dist);
  return(dist);
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
  float calcR0 = 0;     // Auxiliar para calculo R0 do MQ-135
  bool status;
  Serial.begin(115200); // Starts the serial communication
 
  // Inicializa o sensor de qualidade do ar MQ-135
  // https://github.com/miguel5612/MQSensorsLib/blob/master/examples/ESP32/esp32.ino
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init();
  /* If the RL value is different from 10K please assign your RL value with the following method:
    MQ135.setRL(10);
  */
  Serial.println("A calibrar o sensor de ambiente MQ-135");
  for(i = 1; i <= 10; i++) {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  Serial.printf(" Valor de R0: %f", calcR0);
  MQ135.setR0(calcR0/10);
  Serial.println("  Terminado!.");
  if(isinf(calcR0)) {
    Serial.println("Erro: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while(1); }
  if(calcR0 == 0) {
    Serial.println("Erro: Conection issue, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while(1); }

  // Deve ser feito após a calibracao do R0. Pelas experiencias que fiz, ao contrario
  // nao funcionava
  // Inicializa WiFi
  connectToWiFi();
  
  // Ativa todos os pinos dos sensores de distancia (lista) e os dos restantes sensores
  for (i = 0; i < NumSensDist; i++) { 
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
    prev_dist[i] = 0;
  }
  pinMode(PhotRAn, INPUT);
  pinMode(DetSomEnv, INPUT);
  //pinMode(DetSomG, INPUT); // Nao usado
 
  // Inicializa sensor UV LTR390 (I2C)
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
    //while(1) delay(10);
  }

  // Inicializa o sensor BME280
  status = bme.begin(); 
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  //while (1);
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
//
void loop() {
  uint64_t iniTime, prevTime, loopTime;
  time_t currentTime;
  struct tm timeinfo;
  float CO, Alcohol, CO2, Toluen, NH4, Aceton;
  unsigned long startMillis = millis();   // variáveis auxiliares para o calculo do ruido em db
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
  
  // Leitura dos sensores de distancia => Tempo de resposta medido para 1 sensor 14ms
  // e construção do objeto JSON
  // Apenas envia para MQTT se pelo menos um dos sensores reportar um valor diferente
  bool toSend = false;
  dist["time"] = currentTime;
  for (int i = 0; i < NumSensDist; i++) {
    distance[i] = (int)LeSensorDist(trigPin[i], echoPin[i]); // Converte em inteiro (cm)
    if (prev_dist[i] != distance[i]) {
      prev_dist[i] = distance[i];
      toSend = true;
    }
    dist["SDist"]["id"+String(i)] = distance[i];
  }
  if (toSend) {
    serializeJson(dist, data);
    // envia para MQTT
    if (!mqttClient.connected())
      reconnect();
    mqttClient.loop();
    mqttClient.publish(pubDist, data);
    // atualiza dados temporais
    measureDelay(&prevTime, true); // Mede mas não mostra o tempo gasto na etape (debug).
                                   //Atualiza tempo atual
  }

  // Leitura do sensor de deteção de som. A amostragem vai ser à velocidade máxima
  // O ciclo gasta cerca de 50ms para ler a amplitude do sinal do micro amplificado
  //Serial.println("A ler ruido");
  while (millis() - startMillis < sampleWindow) {
    uint rdSom = analogRead(DetSomEnv);
    //Serial.print(rdSom); Serial.print(" ");
    if (rdSom < 4095) {       // vai ignorar valores erroneos!
      if (rdSom > signalMax)  // e ajusta valores maximo e minimo
        signalMax = rdSom;
      else if (rdSom < signalMin)
        signalMin = rdSom;
    }
  }
  //Serial.printf("\nRuido: SMax: %d SMin: %d\n", signalMax, signalMin);
  peakToPeak = signalMax - signalMin;
  int db = map(peakToPeak,30,400,49.5,90);  // calibra para deciBels
  //int db = map(peakToPeak,50,500,49.5,20); // valores usados na fonte inicial
  //Serial.print(" db: ");
  //Serial.println(db);
  snd["time"] = currentTime;
  snd["DB"] = db;
  serializeJson(snd, data);
  if (!mqttClient.connected())
    reconnect();
  mqttClient.loop();
  mqttClient.publish(pubSnd, data);
  measureDelay(&prevTime, false);  // Mede mas não mostra o tempo gasto na etape (debug). Atualiza tempo atual
  
  // Leitura de sensores de ambiente e construção do objeto JSON
  if (amb_ctr > 1)
    amb_ctr--;           // atualiza contador de passagens (apenas executa quando chega a 1)
    else {
      amb_ctr = ambCont; // Reinicializa o contador de passagens
      amb["time"] = currentTime; // a menos que se use um ciclo de mais que 1 segundo!
      // Leitura do sensor BME280
      bTemp = bme.readTemperature();
      bHum = bme.readHumidity();
      bPress = bme.readPressure()/100; // Le a pressão em hPa (hectoPascal = millibar);
      bAlt = bme.readAltitude(SEALEVELPRESSURE_HPA); // Estima a altitude com base na pressão média ao nível do mar
      //Serial.printf("\n BME Temp: %.2f°C Hum: %0.2f Press: %dhPa Alt: %.0fm\n", bTemp, bHum, bPress, bAlt);
      amb["BME"]["Temp"] = bTemp;
      amb["BME"]["Hum"] = bHum;
      amb["BME"]["Press"] = bPress;
      amb["BME"]["Alt"] = bAlt;

      // Leitura do sensor fotoresistivo => Tempo medido 0ms
      Luz = analogRead(PhotRAn);
      //Serial.print(" IntLuz: ");
      //Serial.println(Luz);
      amb["PhR"] = Luz;

      // Leitura do sensor de qualidade do ar
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
      //Serial.print(" | "); Serial.print(CO); 
      //Serial.print(" | "); Serial.print(Alcohol);
      //Serial.print(" | "); Serial.print(CO2); 
      //Serial.print(" | "); Serial.print(Toluen); 
      //Serial.print(" | "); Serial.print(NH4); 
      //Serial.print(" | "); Serial.print(Aceton);
      //Serial.println(" |"); 
/*   
      // Leitura do sensor de UV
      if (ltr.newDataAvailable()) {
        int valorUV = ltr.readUVS();
        Serial.printf("Valor UV: %d\n", valorUV);
        amb["UV"] = valorUV;
      }
*/
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
