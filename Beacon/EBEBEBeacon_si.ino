#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiAP.h>
#include <EEPROM.h>
#include <SocketIoClient.h>
#include "HX711.h"
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h> 

#define debug Serial


#define RST_PIN 12
#define LED_PIN 13
#define NUMPIXELS 8
#define SCL_DAT_PIN 14
#define SCL_CLK_PIN 27
#define MOT_PIN 26
#define ALC_PIN 32
#define TILT_PIN 16
#define PWR_PIN GPIO_NUM_2
#define BAT_PIN 33

#define addr_mode 0
#define addr_iptype 10
#define addr_init 20

#define addr_weight_min 25
#define addr_weight_max 30
#define addr_alc_min 35
#define addr_alc_max 40

#define addr_uuid 50
#define addr_ssid 100
#define addr_pwd 200
#define addr_ssid_length 150
#define addr_pwd_length 250

#define addr_host_ip 300
#define addr_host_ip_length 350
#define addr_host_port 400
#define addr_host_path 420
#define addr_host_path_length 500

#define mode_station 1
#define mode_ap 0

AsyncWebServer server(80);
SocketIoClient webSocket;
WiFiClient client;
HX711 scale;
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);


// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* APssid = "EBEBEConfig";
const char* APpassword = "";

char* ssid;
char* pass;

String uuid = "";

char path[] = "/socket.io/?transport=websocket"; //?transport=websocket
char hostIP[] = "socket.example.com";
unsigned int port = 3000;

int cur_mode;
bool init_false=false;

const char* PARAM_INPUT_1 = "SSID";
const char* PARAM_INPUT_2 = "Password";

volatile float scaleFactor = 127.15;

String led_mode = "OFF";
int led_level = 0;
uint32_t led_mainColor = pixels.Color(0, 0, 0);

String motor_mode = "OFF";
String lastMotor_mode = "OFF";
int motor_duration = 0;

int frameInterval = 100;
int currentFrame = 0;
int lastFrame = 0;

int flagInterval = 2000;
int currentFlag = 0;
int lastFlag = 0;

bool isConnectedToWebSocket = 0;

int weight_min = 0;
int weight_max = 255;
int alc_min = 0;
int alc_max = 0;

StaticJsonDocument<200> outgoingJsonMsg;
StaticJsonDocument<200> incomingJsonMsg;

const char ssidConfig_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>EBEBE WiFi Config Page</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <p><b>EBEBE WiFi Config</b></p><br>
  <p><b>Please input your SSID</b></p><br>
  <form action="/password">
    SSID: <input type="text" name="SSID">
    <input type="submit" value="Submit">
  </form><br>
</body></html>)rawliteral";

const char passConfig_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>EBEBE WiFi Config Page</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <p><b>EBEBE WiFi Config</b></p><br>
  <p><b>Please input your password</b></p><br>
  <form action="/done">
    Password: <input type="text" name="Password">
    <input type="submit" value="Submit">
  </form><br>
</body></html>)rawliteral";

const char done_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>EBEBE WiFi Config Page</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <p><b>WiFi Config Done</b></p><br>
  <p><b>This device is now restarting...</b></p><br>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void wsEmit(String key, StaticJsonDocument<200> emittedJson){
  String strJson;
  serializeJson(emittedJson, strJson);
  //debug.print("WS_EMIT deserialized Json: ");debug.println(strJson);
  char wsbuffer[strJson.length()];
  strJson.toCharArray(wsbuffer, strJson.length()+1);
  char keybuffer[key.length()];
  key.toCharArray(keybuffer, key.length()+1);
  webSocket.emit(keybuffer, wsbuffer);
}

void ws_chat(const char * payload, size_t length) {
  //debug.print("got message: ");debug.println(payload);
}

void ws_connect(const char * payload, size_t length) {
  debug.print("connected to server: ");debug.println(payload);
  //setLed("ON", 50, pixels.Color(0, 255, 0));
  //handleLed();
  isConnectedToWebSocket = 1;
  //debug.println("WebSocket flag set to 1");
}

void ws_disconnect(const char * payload, size_t length){
  debug.println("disconnected from server");
  setLed("ON", 50, pixels.Color(255, 255, 0));
  
  //debug.println("WebSocket flag set to 0");
  handleLed();
  setMotor("OFF");
  handleMotor();
  isConnectedToWebSocket = 0;
  ESP.restart();
  webSocket.begin(hostIP, port, path, "/chat");
  
}
void ws_debug(const char * payload, size_t length) {
  debug.print("debug message: ");debug.println(payload);
}

void ws_command(const char * payload, size_t length) {
  //debug.print("WS_COMMAND deserialized Json: ");debug.println(payload);
  deserializeJson(incomingJsonMsg, payload);
  const char * incomingLedMode = incomingJsonMsg["ledMode"];
  //const char * incomingLedLevel = incomingJsonMsg["ledLevel"];
  int incomingLedLevel = incomingJsonMsg["ledLevel"];
  const char * incomingMotorMode = incomingJsonMsg["vibMode"];
  
  debug.print("incoming led_mode: ");debug.println(incomingLedMode);
  debug.print("incoming led_level: ");debug.println(incomingLedLevel);
  debug.print("incoming motor_mode: ");debug.println(incomingMotorMode);
  
  led_level = String(incomingLedLevel).toInt();
  setLed(String(incomingLedMode), incomingLedLevel, pixels.Color(200, 200, 200));
  setMotor(String(incomingMotorMode));
  deserializeJson(incomingJsonMsg, "");
}

void ws_report() {
  outgoingJsonMsg["uuid"] = uuid;
  outgoingJsonMsg["level"] = getScale(1);
  outgoingJsonMsg["alc"] = getAlcLevel();
  outgoingJsonMsg["isDrinking"] = digitalRead(TILT_PIN);
  wsEmit("report", outgoingJsonMsg);
  deserializeJson(outgoingJsonMsg, "");
}

void ws_login(const char * payload, size_t length) {
  //debug.print("login: ");debug.println(payload);
}

void debugMsg(){
  volatile float weight = scale.get_units(1);
  int weightOutput = (int) weight;
  outgoingJsonMsg["batLevel"] = getBatLevel();
  outgoingJsonMsg["RAWweight"] = weightOutput;
  outgoingJsonMsg["RAWalc"] = analogRead(ALC_PIN);
  outgoingJsonMsg["RAWbat"] = analogRead(BAT_PIN);
  wsEmit("debug", outgoingJsonMsg);
  deserializeJson(outgoingJsonMsg, "");
}


void setup() {
  pinMode(RST_PIN, INPUT_PULLUP);
  pinMode(MOT_PIN, OUTPUT);
  pinMode(ALC_PIN, INPUT);
  pinMode(TILT_PIN, INPUT_PULLUP);
  pinMode(BAT_PIN, INPUT);
  digitalWrite(MOT_PIN, LOW);
  //ledcSetup(0, 5000, 8);
  //ledcAttachPin(MOT_PIN, 0);
  
  Serial.begin(115200);

  pixels.setBrightness(100);
  pixels.begin();
  handleLed();
  EEPROM.begin(512);
  esp_sleep_enable_ext0_wakeup(PWR_PIN,1);

  //Reset Pin Mode Reading
  if(digitalRead(RST_PIN) == LOW){
    debug.println("Reset Button Pressed -> mode=ap_set");
    EEPROM.write(addr_mode, mode_ap);
    EEPROM.commit();
    cur_mode = mode_ap;
  } else {
    cur_mode = EEPROM.read(addr_mode);
    
  }
  if(cur_mode == mode_ap){ //if AP init mode
    debug.println("current mode is AP init");
    WiFi.softAP(APssid, APpassword);
    IPAddress myIP = WiFi.softAPIP();
    debug.print("AP IP address: ");
    debug.println(myIP);
    // Send web page with input fields to client
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", ssidConfig_html);
      setLed("ON", 80, pixels.Color(0, 255, 0));
      handleLed();
    });
  
    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/password", HTTP_GET, [] (AsyncWebServerRequest *request) {
      debug.println("HTT_GET on /password");
      request->send_P(200, "text/html", passConfig_html);
      setLed("ON", 150, pixels.Color(0, 255, 0));
      handleLed();
      String inputMessage;
      String inputParam;
      // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
      if (request->hasParam(PARAM_INPUT_1)) {
        inputMessage = request->getParam(PARAM_INPUT_1)->value();
        inputParam = PARAM_INPUT_1;

        debug.print("inputMessage: ");debug.println(inputMessage);
        debug.print("inputParam: ");debug.println(inputParam);

        int lengthSSID=inputMessage.length();
        const char* sid=inputMessage.c_str();
        
        for(int i=0;i<lengthSSID;i++){
          EEPROM.write(addr_ssid+i,sid[i]); 
            debug.print(sid[i]);   
        }
            
        EEPROM.write(addr_ssid_length,lengthSSID);
        EEPROM.commit();
        
      }
      
    });
    server.on("/done", HTTP_GET, [] (AsyncWebServerRequest *request) {
      debug.println("HTT_GET on /done");
      request->send_P(200, "text/html", done_html);
      setLed("ON", 255, pixels.Color(0, 255, 0));
      handleLed();
      String inputMessage;
      String inputParam;
      // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
      if (request->hasParam(PARAM_INPUT_2)) {
        inputMessage = request->getParam(PARAM_INPUT_2)->value();
        inputParam = PARAM_INPUT_2;

        debug.print("inputMessage: ");debug.println(inputMessage);
        debug.print("inputParam: ");debug.println(inputParam);
        
        int lengthPW=inputMessage.length();
        const char* pid=inputMessage.c_str();
        EEPROM.write(addr_mode,mode_station);
        for(int i=0;i<lengthPW;i++){
            EEPROM.write(addr_pwd+i,pid[i]);
            //debug.print(pid[i]);
        }
        EEPROM.write(addr_pwd_length,lengthPW);
        EEPROM.commit();
      }
      debug.println("settings saved, restarting...");
      ESP.restart();
    });
    server.onNotFound(notFound);
    server.begin();
  }
  if(cur_mode == mode_station){
    debug.println("current mode is station");
    debug.println("connecting to AP in DHCP");

    
      
    byte ssid_length = EEPROM.read(addr_ssid_length);
    byte pw_length = EEPROM.read(addr_pwd_length);
    String str_ssid;
    for (int i = 0; i < ssid_length; i++)
      str_ssid += (char)(EEPROM.read(addr_ssid + i));
    String str_pwd;
    for (int i = 0; i < pw_length; i++)
      str_pwd += (char)(EEPROM.read(addr_pwd + i));

    ssid = const_cast<char*>(str_ssid.c_str());
    pass = const_cast<char*>(str_pwd.c_str());
    //debug.print("ssid=");debug.println((char*)ssid);
    //debug.print("pwd=");debug.println((char*)pass);
    debug.println("WiFi settings completed");

    WiFi.begin(ssid, pass);
    int waiting_timer=0;
    while (WiFi.status() != WL_CONNECTED) { 
      setLed("ON", waiting_timer*20, pixels.Color(0, 255, 0));
      handleLed();
      delay(1000);
      debug.print(".");
      waiting_timer++;
      if(waiting_timer>10){
        debug.println("WiFi connection timeout, restart");
        setLed("ON", 255, pixels.Color(255, 0, 0));
        handleLed();
        ESP.restart();
      }
    }
    setLed("ON", 125, pixels.Color(0, 255, 0));
    handleLed();
    debug.println("");
    debug.print(F("Obtained IP: "));    //device ip
    debug.println(WiFi.localIP());   
    debug.print(F("gateway : "));   //device ip
    debug.println(WiFi.gatewayIP());   
    debug.print(F("subnet IP: "));    //device ip
    debug.println(WiFi.subnetMask());  
    debug.println("Socket.io server info loading");

    byte host_ip_length = EEPROM.read(addr_host_ip_length);
    //debug.print("Host IP Length: ");debug.println(host_ip_length);
    //hostIP[0] = '\0';
    for (int i = 0; i < host_ip_length; i++){
      hostIP[i] = EEPROM.read(addr_host_ip+i);
      //debug.print("Read byte ");debug.print(hostIP[i]);debug.print(" from addr ");debug.println(addr_host_ip+i);
    }
    hostIP[host_ip_length] = '\0';
    
    port = readIntEEPROM(addr_host_port);
    
    byte host_path_length = EEPROM.read(addr_host_path_length);
    //debug.print("Host Path Length: ");debug.println(host_path_length);
    //path[0] = '\0';
    for (int i = 0; i < host_path_length; i++){
      path[i] = EEPROM.read(addr_host_path+i);
      //debug.print("Read byte ");debug.print(path[i]);debug.print(" from addr ");debug.println(addr_host_path+i);
    }
    path[host_path_length] = '\0';
    for (int i = 0; i < 6; i++)
      uuid += (char)(EEPROM.read(addr_uuid + i));
      
    debug.print("HostIP : ");debug.println(hostIP);
    debug.print("HostPort : ");debug.println(port);
    debug.print("HostPath : ");debug.println(path);
    debug.print("UUID : ");debug.println(uuid);
    setLed("ON", 220, pixels.Color(0, 255, 0));
    handleLed();

    webSocket.on("chat message", ws_chat);
    webSocket.on("connect", ws_connect);
    webSocket.on("disconnect", ws_disconnect);
    webSocket.on("debug", ws_debug);
    webSocket.on("command", ws_command);
    webSocket.begin(hostIP, port, path, "/chat");
    
    scale.begin(SCL_DAT_PIN, SCL_CLK_PIN);
    scale.set_scale(scaleFactor);
    scale.tare();

    weight_min = readIntEEPROM(addr_weight_min);
    weight_max = readIntEEPROM(addr_weight_max);
    alc_min = readIntEEPROM(addr_alc_min);
    alc_max = readIntEEPROM(addr_alc_max);

    debug.println("ADC Compensation Value");
    debug.print("Weight MIN: ");debug.print(weight_min);debug.print(" Weight MAX: ");debug.println(weight_max);
    debug.print("Alc MIN: ");debug.print(alc_min);debug.print(" alc MAX: ");debug.println(alc_max);
    
    debug.print("Scale Begin at DATA: ");debug.print(SCL_DAT_PIN);debug.print(", CLOCK: ");debug.println(SCL_CLK_PIN);
    debug.print("Scale Factor: ");debug.println(scaleFactor);
    setLed("ON", 255, pixels.Color(0, 255, 0));
    handleLed();
    
  } 
  if(cur_mode != mode_ap && cur_mode != mode_station) {
    debug.println("mode not properly set - commit ap_set and restart");
    EEPROM.write(addr_mode, mode_ap);
    EEPROM.commit();
    ESP.restart();
  }
}
bool printStop=0;
int menuStage=0;
int option=0;

int sleepButtonTimer;
bool sleepButtonFlag;

void loop() {
  String command;
  String com1;
  String com2;
  String com3;
  String msg;

  if (digitalRead(PWR_PIN) == HIGH){
    if (sleepButtonFlag == 0){
      sleepButtonTimer = millis();
      sleepButtonFlag = 1;
    } else {
      if (millis() - sleepButtonTimer > 3000){
        debug.println("****************ESP Deep Sleep Start");
        setLed("ON", 255, pixels.Color(100, 0, 0));
        setMotor("OFF");
        handleLed();
        handleMotor();
        delay(3000);
        setLed("OFF", 0, pixels.Color(0, 0, 0));
        setMotor("OFF");
        handleLed();
        handleMotor();
        esp_deep_sleep_start();
      }
    }
  } else {
    sleepButtonFlag = 0;
  }
  if (cur_mode == mode_ap){
    if(printStop == 0){
      if(menuStage == 0){
        debug.println("EBEBE Beacon Config Page\nSelect Menu:\n1. SSID\n2. Password\n3. Host IP\n4. Host Port\n5. Host Path\n6. UUID\n7. ADC Compensation\n8. Save and Restart");
      }else if(menuStage == 1){
        if(option == 1){
          debug.println("new SSID:");
        }else if(option == 2){
          debug.println("new Password:");
        }else if(option == 3){
          debug.println("new Host IP:");
        }else if(option == 4){
          debug.println("new Host Port:");
        }else if(option == 5){
          debug.println("new Host Path:");
        }else if(option == 6){
          debug.println("new UUID:");
        }else if(option == 7){
          debug.println("ADC Compensation Value Setting\n1. Weight MIN\n2. Weight MAX\n3. Alc MIN\n4. Alc MAX");
        }else if(option == 8){
          debug.println("Set mode to station and Restarting...");
          EEPROM.write(addr_mode, mode_station);
          EEPROM.commit();
          ESP.restart();
        }
      } else if(menuStage == 2){
        if(option == 1){
          debug.println("new weight MIN:");
        }else if(option == 2){
          debug.println("new weight MAX:");
        }else if(option == 3){
          debug.println("new alc MIN:");
        }else if(option == 4){
          debug.println("new alc MAX:");
        }
      }
      printStop = 1;
    }
    if(Serial.available()>0){
      msg = debug.readString();
      if(menuStage == 0){
        if(msg.toInt() <0 || msg.toInt() > 7) debug.println("Invalid Input, Select again:");
        option = msg.toInt();
        menuStage = 1;
        printStop = 0;
      }else if(menuStage == 1){
        if(option == 1){
          debug.print("SSID set to ");debug.println(msg);
          byte lengthmsg=msg.length()-1;
          const char* sid=msg.c_str();
          for(int i=0;i<lengthmsg;i++){
            EEPROM.write(addr_ssid+i,sid[i]); 
              //debug.print(sid[i]);   
          }
          EEPROM.write(addr_ssid_length,lengthmsg);
          EEPROM.commit();
          menuStage = 0;
          printStop = 0;
        }else if(option == 2){
          debug.print("Password set to ");debug.println(msg);
          byte lengthmsg=msg.length()-1;
          const char* pid=msg.c_str();
          for(int i=0;i<lengthmsg;i++){
              EEPROM.write(addr_pwd+i,pid[i]);
              //debug.print(pid[i]);
          }
          EEPROM.write(addr_pwd_length,lengthmsg);
          EEPROM.commit();
          menuStage = 0;
          printStop = 0;
        }else if(option == 3){
          debug.print("Host IP set to ");debug.println(msg);
          byte lengthmsg=msg.length()-1;
          const char* pid=msg.c_str();
          for(int i=0;i<lengthmsg;i++){
              EEPROM.write(addr_host_ip+i,pid[i]);
              //debug.print(pid[i]);
          }
          EEPROM.write(addr_host_ip_length,lengthmsg);
          EEPROM.commit();
          menuStage = 0;
          printStop = 0;
        }else if(option == 4){
          debug.print("Host Port set to ");debug.println(msg.toInt());
          writeIntEEPROM(addr_host_port, msg.toInt());
          menuStage = 0;
          printStop = 0;
        }else if(option == 5){
          debug.print("Host Path set to ");debug.println(msg);
          byte lengthmsg=msg.length()-1;
          //debug.print("Host Path length ");debug.println(lengthmsg);
          const char* pid=msg.c_str();
          for(int i=0;i<lengthmsg;i++){
              EEPROM.write(addr_host_path+i,pid[i]);
              //debug.print(pid[i]);
          }
          EEPROM.write(addr_host_path_length,lengthmsg);
          EEPROM.commit();
          menuStage = 0;
          printStop = 0;
        }else if(option == 6){
          debug.print("UUID set to ");debug.println(msg);
          byte lengthmsg=msg.length()-1;
          const char* pid=msg.c_str();
          for(int i=0;i<lengthmsg;i++){
              EEPROM.write(addr_uuid+i,pid[i]);
              //debug.print(pid[i]);
          }
          EEPROM.commit();
          menuStage = 0;
          printStop = 0;
        }else if(option == 7){
          option = msg.toInt();
          menuStage = 2;
          printStop = 0;
        }
      }else if(menuStage == 2){
        if(option == 1){
          debug.print("weight MIN set to: ");debug.println(msg);
          writeIntEEPROM(addr_weight_min, msg.toInt());
          menuStage = 0;
          printStop = 0;
        }else if(option == 2){
          debug.print("weight MAX set to: ");debug.println(msg);
          writeIntEEPROM(addr_weight_max, msg.toInt());
          menuStage = 0;
          printStop = 0;
        }else if(option == 3){
          debug.print("alc MIN set to: ");debug.println(msg);
          writeIntEEPROM(addr_alc_min, msg.toInt());
          menuStage = 0;
          printStop = 0;
        }else if(option == 4){
          debug.print("alc MAX set to: ");debug.println(msg);
          writeIntEEPROM(addr_alc_max, msg.toInt());
          menuStage = 0;
          printStop = 0;
        }
      }
    }
  }
  if (cur_mode == mode_station){
    webSocket.loop();
    if(Serial.available()>0){
      msg = debug.readString();
      msg.remove(msg.length()-1);
      debug.print("send: ");debug.println(msg);
      outgoingJsonMsg["uuid"] = uuid;
      outgoingJsonMsg["msg"] = msg;
      wsEmit("debug", outgoingJsonMsg);
      deserializeJson(outgoingJsonMsg, "");
    }
    if(isConnectedToWebSocket == 1){
      currentFlag = millis()/flagInterval;
      if(currentFlag != lastFlag){  //execute every 1000 ms
        debug.print("Report every ");debug.print(flagInterval);debug.println("ms");
        ws_report();
        debugMsg();
        lastFlag = currentFlag;
      }
      currentFrame = millis()%(frameInterval*10);
      currentFrame = currentFrame/frameInterval;
      if(currentFrame != lastFrame){  //execute every 100ms
        //debug.print("Current frame: ");debug.println(currentFrame);
        handleLed();
        handleMotor();
        lastFrame = currentFrame;
      }
    }
  }
  
  
}

int getScale(int count){
  //scale.power_up();
  volatile float weight = scale.get_units(count);
  //scale.power_down();
  int output = (int) weight;
  //return output;
  output = map(output, weight_min, weight_max, 0, 255);
  if(output<0)output=0;
  if(output>255)output=255;
  return output;
}

int getBatLevel(){
  int output = analogRead(BAT_PIN);
  output = map(output, 2215, 2450, 0, 255); //3.8V MIN 4.2V MAX
  if(output > 255){output = 255;}
  else if(output<0){output = 0;}
  return output;
}

int ledTable_blink[10] = {1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
int ledTable_error[10] = {1, 1, 0, 0, 1, 1, 0, 0, 0, 0};

void setLed(String mode, int index, uint32_t color){
  if (led_level < 0) {led_level = 0;}
  else if (led_level > 255) {led_level = 255;}
  else {led_level = index;}
  
  if(mode == "BLINK"){
    led_mode = "BLINK";
    led_mainColor = color;
  } else if (mode == "OFF"){
    led_mode = "OFF";
    led_mainColor = pixels.Color(0, 0, 0);
  } else if (mode == "ON"){
    led_mode = "ON";
    
    led_mainColor = color;
  } else if(mode == "ERROR"){
    led_mode = "ERROR";
    led_mainColor = pixels.Color(255, 0, 0);
  }
}

void handleLed(){
  pixels.clear();
  //debug.println("handleLED();");
  if(led_mode == "OFF"){
    //debug.println("led_mode == OFF");
    for(int i = 0; i<pixels.numPixels(); i++){
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      //debug.print("ledNum: ");debug.print(i);debug.print(" pixelColor: ");debug.println(pixels.Color(0, 0, 0));
    }
  } else if(led_mode == "BLINK"){
    uint32_t ledBrightness[pixels.numPixels()];
    int segment = 255/pixels.numPixels(); //14
    if(led_level>=segment*pixels.numPixels()){
      for(int i=0; i<pixels.numPixels(); i++){
        ledBrightness[i]=led_mainColor;
      }
    }else{
      for(int i=0; i<led_level/segment; i++){
        ledBrightness[i]=led_mainColor;
      }
      ledBrightness[led_level/segment] = led_mainColor * map(led_level%segment, 0, segment-1, 0, 255) / 255;
      for(int i=led_level/segment+1; i<pixels.numPixels(); i++){
        ledBrightness[i] =0;
      }
    }
    for(int i = 0; i<pixels.numPixels(); i++){
      pixels.setPixelColor(i, ledBrightness[i]*ledTable_blink[currentFrame]);
    }
  } else if(led_mode == "ERROR"){
    for(int i = 0; i<pixels.numPixels(); i++){
      pixels.setPixelColor(i, led_mainColor*ledTable_error[currentFrame]);
      //debug.print("ledNum: ");debug.print(i);debug.print(" pixelColor: ");debug.println(led_mainColor*ledTable_blink[currentFrame]);
    }
  } else if(led_mode == "ON"){
    uint32_t ledBrightness[pixels.numPixels()];
    int segment = 255/pixels.numPixels(); //14
    if(led_level>=segment*pixels.numPixels()){
      for(int i=0; i<pixels.numPixels(); i++){
        ledBrightness[i]=led_mainColor;
      }
    }else{
      for(int i=0; i<led_level/segment; i++){
        ledBrightness[i]=led_mainColor;
      }
      ledBrightness[led_level/segment] = led_mainColor * map(led_level%segment, 0, segment-1, 0, 255) / 255;
      for(int i=led_level/segment+1; i<pixels.numPixels(); i++){
        ledBrightness[i] =0;
      }
    }
    for(int i = 0; i<pixels.numPixels(); i++){
      pixels.setPixelColor(i, ledBrightness[i]);
    }
  }
  pixels.show();
}

bool motorTable_on[10] = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
bool hasVibrated = 1;

void setMotor(String incomingMode){
  //debug.print("1. setting incoming motor mode: ");debug.println(incomingMode);
  motor_mode = incomingMode;
  //debug.print("2. set motor_mode: ");debug.println(motor_mode);
}

void handleMotor(){
  int PWM = 100;
  //debug.println("handleMotor();");
  if(motor_mode == "OFF"){
    digitalWrite(MOT_PIN, LOW);
    //ledcWrite(0, 0);
    //debug.println("MOTOR MODE OFF");
  } else if(motor_mode == "NOTIFY"){
    //debug.println("MOTOR MODE NOTIFY");
    if(hasVibrated == 0){
      //debug.println("motor vibrating");
      digitalWrite(MOT_PIN, HIGH);
      //ledcWrite(0, PWM);
      hasVibrated = 1;
    }else{
      digitalWrite(MOT_PIN, HIGH);
      //ledcWrite(0, PWM);
    }
  } else if(motor_mode == "ON"){
    //debug.println("MOTOR MODE ON");
    //debug.print("Motor State = ");debug.println(motorTable_on[currentFrame]);
    //ledcWrite(0, motorTable_on[currentFrame]*PWM);
    digitalWrite(MOT_PIN, motorTable_on[currentFrame]);
  }
}

int getAlcLevel(){
  int temp = map(analogRead(ALC_PIN), alc_min, alc_max, 0, 255);
  if (temp<0) temp=0;
  if (temp>255) temp=255;
  //debug.print("ALC_PIN ADC Value: ");debug.println(analogRead(ALC_PIN));
  return temp;
  //return analogRead(ALC_PIN);
  
}

int readIntEEPROM(int address){
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}
void writeIntEEPROM(int address, int writeInt){
  byte byte1 = writeInt >> 8;
  byte byte2 = writeInt & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
  EEPROM.commit();
}

void writeStringEEPROM(int stringAddress, int lengthAddress, String writeString){
  byte lengthmsg=writeString.length()-1;
  const char* pid=writeString.c_str();
  for(int i=0;i<lengthmsg;i++){
      EEPROM.write(stringAddress+i,pid[i]);
      //debug.print(pid[i]);
  }
  EEPROM.write(lengthAddress, lengthmsg);
  EEPROM.commit();
}
