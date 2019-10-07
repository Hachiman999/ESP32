/*---------------------------------------------------------------------------------
Sensor DHT
+ = 3.3v entre 5v 
-= gnd
out = pin4 por la linea de 3.3v del esp32 en caso de arduino algun A0, A1,A2
Sensor Temperatura
s = 3.3v entre 5v 
- = gnd
|||| = pin2 por la linea de 3.3v del esp32 en caso de arduino algun A0, A1,A2
Sensor gy-max30102
vin = 3.3v entre 5v 
gnd = gnd
scl=          en caso de arduino algun A0, A1,A2
sda=          en caso de arduino algun A0, A1,A2
*/
//Librerias usadas
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include "LedMatrix.h"
#include "MPU9250.h"
//----------------------------------puertos
const int piel_temperatura= 12;
const int Resistencia_galbanica= 4;
const int humedad_temperatura =26;
const int Buzzer =27;
const int ledPIN_wifi = 2;
const int ledPIN_post = 15;
//-------------------------------define
#define ONE_WIRE_BUS  piel_temperatura // El cable de datos se conecta al pin 2 del Arduino

#define DHTTYPE DHT11   // DHT 22  (AM2302), AM2321

#define GSR Resistencia_galbanica //pin del GSR

//matriz
#define NUMBER_OF_DEVICES 2 //numero de pantallas en serie
#define CS_PIN 25
#define CLK_PIN 32
#define MISO_PIN 1 //esto hace relleno pero no se usa
#define MOSI_PIN 33

//----------------------------coneccion con el wifi
const char* ssid = "WiFi-UAO";
const char* password =  "";
//----------------------------coneccion con el servidor
const String ip ="http://11.11.27.131:8080/";
//----------------------------variables globales
//esta es el tamaño de datos que va enviar el json
const size_t capacidad = JSON_OBJECT_SIZE(2) + 21*JSON_OBJECT_SIZE(3) + 2*JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(10);
int Conteo_conexiones =0;//los intentos de conexion a la red wifi
//1111111111111111111111111111111111111111111111111111111111111111111111111111
uint8_t DHTPin = humedad_temperatura;//pin del sensor de humedad y temperatura
float ambient_humidity,ambient_temperature=0;
//2222222222222222222222222222222222222222222222222222222222222222
int gsr=0;
//ledddddddddddddddddddd
bool estado_led_1,estado_led_2=false;
//matriz
LedMatrix ledMatrix = LedMatrix(NUMBER_OF_DEVICES, CLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
bool cambio = true;
String temp_ma;
//---------------------------------------------
// Definir modelo y pin del DTH
DHT dht(DHTPin, DHTTYPE);
// Definir una instancia de OneWire para comunicarse
// con cualquier dispositivo OneWire
OneWire oneWire(ONE_WIRE_BUS);
// Pasar la referencia a OneWire del
// Sensor de temperatura Dallas
DallasTemperature sensor_temperatura(&oneWire);
// Definir cliente NTP para obtener tiempo
WiFiUDP ntpUDP;//Network Time Protocol Protocolo de datagramas de usuario
NTPClient timeClient(ntpUDP);//Network Time Protocol Client
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
// Variables para guardar fecha y hora
String formato,fecha,hora;//formato de datos
int freq = 2000;int channel = 0;int resolution = 8;//variables de lbuzzer
//-------------------setup
void setup() {
Serial.begin(115200);
Serial.println("`7MM°°°YMM   .M°°°bgd `7MM°°°°Mq.");
Serial.println("  MM    `7  ,MI    °Y   MM   `MM.");
Serial.println("  MM   d    `MMb.       MM   ,M9  pd°°b.   pd*||*b.");
Serial.println("  MMmmMM      `YMMNq.   MMmmdM9  (O)  `8b (O)   j8");
Serial.println("  MM   Y  , .     `MM   MM            ,89     ,;j9");
Serial.println("  MM     ,M Mb     dM   MM          °°Yb.  ,-='");
Serial.println(".JMMmmmmMMM P°Ybmmd°  .JMML.           88 Ammmmmmm ");
Serial.println("                                 (O)  .M'");
Serial.println("                                  bmmmd'");
//Led wifi
pinMode(ledPIN_wifi, OUTPUT);
pinMode(ledPIN_post, OUTPUT);
//buzzer
ledcSetup(channel, freq, resolution);
ledcAttachPin(Buzzer, channel);
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
//iniciamos la captacion de datos del dth
 dht.begin();
 // Establece valor por defecto del chip 9 bit. Si tiene problemas pruebe aumentar
 sensor_temperatura.begin(); 
 //inicializamos la libreria de la matriz
 ledMatrix.init();
//Usamos el metodo buscar_wifi para buscar el wifi
 buscar_wifi();
//Inicialice un NTPClient para obtener tiempo     
timeClient.begin(); timeClient.setTimeOffset(-18000);

}

void loop() {

if(estado_led_1){digitalWrite(ledPIN_wifi, HIGH);}else{digitalWrite(ledPIN_wifi, LOW); }

 
if(WiFi.status()== WL_CONNECTED){//comprobamos que seguimos conectados ala red wifi
  digitalWrite(ledPIN_post, LOW);//led del post
// Leemos la temperatura o la humedad toma alrededor de 250 milisegundos
 ambient_humidity = dht.readHumidity(); ambient_temperature = dht.readTemperature();
//comprobamos que el sensor si nos este mandando algo
Serial.println(" Humedad: ");Serial.print(ambient_humidity);
Serial.println(" ");
Serial.println(" Temperatura: ");Serial.print(ambient_temperature);
// if (isnan(ambient_humidity) || isnan(ambient_temperature)) { Serial.println("Error al leer del sensor DHT"); return;}  
 sensor_temperatura.requestTemperatures();  // Enviar el comando para obtener la temperatura

 Serial.println(" ");
 Serial.println("La temperatura en el dispositivo 2 es: ");
 Serial.print(sensor_temperatura.getTempCByIndex(0)); 
 bool fiebre=false;
if(sensor_temperatura.getTempCByIndex(0)>37.8){ fiebre=true;}else{fiebre=false;}
re_buzzer(fiebre);
String f= "temperatiura : "+(String)sensor_temperatura.getTempCByIndex(0);
matrizx2(f);
  gsr = DatosGSR();
  Serial.println(" ");
 Serial.println("Resistencia Galbanica: ");Serial.print(gsr);
  // put your main code here, to run repeatedly:
  // read the sensor
  IMU.readSensor();

//  //aceleracion
  Serial.print("aceleracion en X es:");
  float ax = IMU.getAccelX_mss();
  Serial.println(ax);
  Serial.print("");

  Serial.print("aceleracion en Y es:");
  float ay=IMU.getAccelY_mss();
  Serial.println(ay);
  Serial.print("");

  Serial.print("aceleracion en Z es:");
  float az=IMU.getAccelZ_mss();
  Serial.println(az);
  Serial.println("");
  
  //Giros
  float gx = IMU.getGyroX_rads();
  Serial.print(gx);
  Serial.println("\t");
  float gy = IMU.getGyroY_rads();
  Serial.print(gy);
  Serial.println("\t");
  float gz = IMU.getGyroZ_rads();
  Serial.print(gz);
  Serial.println("\t"); 

  //campo magnetico
  float mx = IMU.getMagX_uT();
  Serial.print(mx);
  Serial.println("\t");
  float my= IMU.getMagY_uT();
  Serial.print(my);
  Serial.println("\t");
  float mz = IMU.getMagZ_uT();
  Serial.print(mz);
  Serial.println("\t");

  //sensor de teperatura
  Serial.print("la temperatura del circuito es: ");
  float tem_C = IMU.getTemperature_C();
  Serial.println(tem_C);

//resibimos un string que es array de chars que tambien es un objeto json a traves del metodo devolver_json
 String info = devolver_json(
  sensor_temperatura.getTempCByIndex(0), //body temperature value
 gsr, //body resistenace value
  70, //body BPM value
  95.4, //body sp02 value
  ambient_temperature, //ambient temperature value
  ambient_humidity, //ambient humidity value
  ax,//device acceleration ax
  ay,//device acceleration ay
  az,//device acceleration az
  gx,//device gyro gx
  gy,//device gyro gy
  gz,//device gyro gz
  mx,//device magnetometer mx
  my,//device magnetometer my
  mz,//device magnetometer mz
  234.5,//device pressure value
  950.3,//device altitude value
  tem_C);//device temperature value  
//le enviamos los datos al servidor 
enviar_dato_cliente(info);
  }else{
    Serial.println("Error in WiFi connection");  
    Serial.println("Reeconectando:");  
    buscar_wifi();
    }
 //delay(1000);
}
//***************************************************************************************************************************************************************************
void buscar_wifi(){
//mandamos el nombre de la red wifi y la contraseña
WiFi.begin(ssid, password);
//Cuenta hasta 50 si no se puede conectar lo cancela
while(WiFi.status() !=WL_CONNECTED and Conteo_conexiones <50  ){ Conteo_conexiones++; delay(500); Serial.print("#"); }
//cuando se conecte me dira la aprobacion y la ip
if(WiFi.status() ==WL_CONNECTED){Conteo_conexiones==0;Serial.println("WiFi conectado"); Serial.println(WiFi.localIP());estado_led_1=true;     }}//fin buscar wifi
//***************************************************************************************************************************************************************************
String devolver_json(float v1, int v2, int v3, float v4, float v5, float v6, int v7, int v8, int v9, float v10, float v11,
float v12, int v13, int v14, int v15, float v16, float v17,float v18){
  
DynamicJsonDocument doc(capacidad);

//le mandamos la fecha y la hora del metodo tiempo_actual
Serial.println("");
Serial.println("Hora actual : "); Serial.print(tiempo_actual());
doc["time"] = tiempo_actual();

JsonObject body = doc.createNestedObject("body");

JsonObject body_temperature = body.createNestedObject("temperature");
body_temperature["value"] = v1;
body_temperature["type"] = "float";
body_temperature["unit"] = "°C";

JsonObject body_resistance = body.createNestedObject("resistance");
body_resistance["value"] = v2;
body_resistance["type"] = "int";
body_resistance["unit"] = "Ohm";

JsonObject body_BPM = body.createNestedObject("BPM");
body_BPM["value"] = v3;
body_BPM["type"] = "int";
body_BPM["unit"] = "Dimensionless";

JsonObject body_SpO2 = body.createNestedObject("SpO2");
body_SpO2["value"] = v4;
body_SpO2["type"] = "float";
body_SpO2["unit"] = "%";

JsonObject ambient = doc.createNestedObject("ambient");

JsonObject ambient_temperature = ambient.createNestedObject("temperature");
ambient_temperature["value"] = v5;
ambient_temperature["type"] = "float";
ambient_temperature["unit"] = "°C";

JsonObject ambient_humidity = ambient.createNestedObject("humidity");
ambient_humidity["value"] = v6;
ambient_humidity["type"] = "float";
ambient_humidity["unit"] = "%";

JsonObject device = doc.createNestedObject("device");
device["UUID"] = "550e8400-e29b-41d4-a716-446655440000";

JsonObject device_acceleration = device.createNestedObject("acceleration");

JsonObject device_acceleration_ax = device_acceleration.createNestedObject("ax");
device_acceleration_ax["value"] = v7;
device_acceleration_ax["type"] = "int";
device_acceleration_ax["unit"] = "mg";

JsonObject device_acceleration_ay = device_acceleration.createNestedObject("ay");
device_acceleration_ay["value"] = v8;
device_acceleration_ay["type"] = "int";
device_acceleration_ay["unit"] = "mg";

JsonObject device_acceleration_az = device_acceleration.createNestedObject("az");
device_acceleration_az["value"] = v9;
device_acceleration_az["type"] = "int";
device_acceleration_az["unit"] = "mg";

JsonObject device_gyro = device.createNestedObject("gyro");

JsonObject device_gyro_gx = device_gyro.createNestedObject("gx");
device_gyro_gx["value"] = v10;
device_gyro_gx["type"] = "float";
device_gyro_gx["unit"] = "deg/s";

JsonObject device_gyro_gy = device_gyro.createNestedObject("gy");
device_gyro_gy["value"] = v11;
device_gyro_gy["type"] = "float";
device_gyro_gy["unit"] = "deg/s";

JsonObject device_gyro_gz = device_gyro.createNestedObject("gz");
device_gyro_gz["value"] = v12;
device_gyro_gz["type"] = "float";
device_gyro_gz["unit"] = "deg/s";

JsonObject device_magnetometer = device.createNestedObject("magnetometer");

JsonObject device_magnetometer_mx = device_magnetometer.createNestedObject("mx");
device_magnetometer_mx["value"] = v13;
device_magnetometer_mx["type"] = "int";
device_magnetometer_mx["unit"] = "mG";

JsonObject device_magnetometer_my = device_magnetometer.createNestedObject("my");
device_magnetometer_my["value"] = v14;
device_magnetometer_my["type"] = "int";
device_magnetometer_my["unit"] = "mG";

JsonObject device_magnetometer_mz = device_magnetometer.createNestedObject("mz");
device_magnetometer_mz["value"] = v15;
device_magnetometer_mz["type"] = "int";
device_magnetometer_mz["unit"] = "mG";

JsonObject device_pressure = device.createNestedObject("pressure");
device_pressure["value"] = v16;
device_pressure["type"] = "float";
device_pressure["unit"] = "mb";

JsonObject device_altitude = device.createNestedObject("altitude");
device_altitude["value"] = v17;
device_altitude["type"] = "float";
device_altitude["unit"] = "m";

JsonObject device_temperature = device.createNestedObject("temperature");
device_temperature["value"] = v18;
device_temperature["type"] = "float";
device_temperature["unit"] = "°C";

//char info[1200];
String info;
serializeJson(doc, info);
//Serial.println(info);
return info;
 
  }
  //*************************************************************************************************************************************************************************
int DatosGSR() { 
long sum=0;
for(int i=0;i<10;i++) //Average the 10 measurements to remove the glitch
{
int sensorValue=analogRead(GSR);
sum += sensorValue;
delay(5);
}
int gsr_average = sum/10;
int form = ((4095+2*gsr_average)*10000)/(2043-gsr_average);

return form ; }// fin de DatosGSR
//***************************************************************************************************************************************************************************
String tiempo_actual(){
while(!timeClient.update()) { timeClient.forceUpdate(); }
// Necesitamos extraer fecha y hora
formato = timeClient.getFormattedDate();
//aqui obtenemos la fecha
int splitT = formato.indexOf("T");
fecha = formato.substring(0, splitT);
//aqui obtenemos la hora
hora = formato.substring(splitT+1, formato.length()-1);
String fecha_hora = fecha+" "+ hora ;
return fecha_hora;}//fin de tiempo_actual
//***************************************************************************************************************************************************************************
void enviar_dato_cliente(String info){
HTTPClient http;   
//Especificar destino para solicitud HTTP
http.begin(ip);  
//Especificar encabezado de tipo de contenido
http.addHeader("Content-Type", "application/json");
//Enviar la solicitud POST real
int httpResponseCode = http.POST(info);    
if(httpResponseCode>0){//comprobamos la solicitud fue aceptada
 //Obtenga la respuesta a la solicitud
String response = http.getString();  
Serial.println("");
Serial.println("Codigo de verificacion de post");
Serial.println(httpResponseCode);   //Imprimir código de retorno si es posivito llego bien
//Serial.println(response);           //Imprimir solicitud de respuesta es lo que devuelvo para verificar que si llego como era
digitalWrite(ledPIN_post, HIGH);
}else{ 
Serial.print("Error al enviar POST: "); Serial.println(httpResponseCode);// si el -1 no conecto al servidor
}http.end();  //liberamos los recursos
}//fin de enviar_dato_cliente
//***************************************************************************************************************************************************************************
void re_buzzer(bool c){
   if(c){ledcWriteTone(channel, 500);}else{ledcWriteTone(channel, 0);}
  //  ledcWrite(channel, 500);  
  //delay(500);
  }
//***************************************************************************************************************************************************************************
void matrizx2(String f){
  if(temp_ma ==f){cambio=false; }else{cambio=true; temp_ma =f;  } 
  if(cambio){ledMatrix.setText(f);cambio=false;}
for(int i =0; i<300;i++){
    ledMatrix.clear();
  ledMatrix.scrollTextLeft();
  ledMatrix.drawText();
  ledMatrix.commit();
 delay(50);
  }


  
  }
