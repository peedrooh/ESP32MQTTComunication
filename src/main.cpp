#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_wifi.h>
#include <PubSubClient.h>


// ------------------------------
// ------------ WiFi ------------
// ------------------------------

// #define UTFPR
#ifdef UTFPR
    #define EAP_SSID "UTFPR-ALUNO"
    #define EAP_IDENTITY "xxxxxxx"
    #define EAP_USERNAME "xxxxxxxx"
    #define EAP_PASSWORD "xxxxxxxx"
#else
    #define EAP_SSID "Wanderley"
    #define EAP_PASSWORD "crm15210"
#endif
uint8_t target_esp_mac [6] = {0x24 , 0x0a , 0x44 , 0x9a , 0x38 , 0x28};
void wifi_setup();
void reconnect_wifi(void* parameter);


// ------------------------------
// ------------ MQTT ------------
// ------------------------------
#define SERVO_TYPE_TOPIC "serve/type"

#define CS_VELOCITY_TOPIC "serve/continious_servo/velocity"
#define CS_LOWER_ZERO_VEL_TOPIC "serve/continious_servo/lower_zero_vel"
#define CS_UPPER_ZERO_VEL_TOPIC "serve/continious_servo/upper_zero_vel"

#define PS_SERVO_IS_ACCURATE "serve/positional_servo/servo_is_accurate"
#define PS_SERVO_EXP_HALF_POS "serve/positional_servo/exp_half_pos"
#define PS_SERVO_ACT_HALF_POS "serve/positional_servo/act_half_pos"
#define PS_SERVO_EXP_FULL_POS "serve/positional_servo/exp_full_pos"
#define PS_SERVO_ACT_FULL_POS "serve/positional_servo/act_full_pos"

#define MQTT_ID  "serve"
// const char* BROKER_MQTT = "broker.hivemq.com"; // use if local broker doesn't work's
const char* BROKER_MQTT = "192.168.0.123"; 
int BROKER_PORT = 1883;
WiFiClient espClient;
PubSubClient MQTT(espClient);
const int MQTT_MESSAGE_RATE = 3000;
void mqtt_setup();
void publish_mqtt_msg(void* parameter);
void reconnect_mqtt_broker(void* parameter);


// Global variables
float param1 = 10.0;
float max_servo_velocity_deg_s;
int lower_zero_vel_freq = 0;
int upper_zero_vel_freq = 0;
bool servo_is_accurate = false;
int expected_half_position = 0;
int expected_full_position = 0;
float actual_half_position = 0;
float actual_full_position = 0;


void setup() {
    Serial.begin(115200);
    Serial.println("Timing: ");
    Serial.println(portTICK_PERIOD_MS);
    Serial.println(configTICK_RATE_HZ);
    wifi_setup();
    mqtt_setup();

    xTaskCreate(
        reconnect_wifi,
        "WiFi Reconnector",
        30000,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        reconnect_mqtt_broker,
        "MQTT Reconnector",
        30000,
        NULL,
        2,
        NULL
    );

    xTaskCreate(
        publish_mqtt_msg,
        "MQTT Publisher",
        30000,
        NULL,
        3,
        NULL
    );
}


void loop() {
  vTaskDelete(NULL);
}


// ------------------------------
// ------------ MQTT ------------
// ------------------------------
void mqtt_setup() {
    MQTT.setKeepAlive(16960);
    MQTT.setServer(BROKER_MQTT, BROKER_PORT); 

    while (!MQTT.connected()) {
        if (MQTT.connect(MQTT_ID)) {
            Serial.println("Connected to MQTT broker");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(MQTT.state());
            Serial.println(" Retrying MQTT connection...");
        }
    }
}

void publish_mqtt_msg(void* parameter) {
    for(;;){
        vTaskDelay(MQTT_MESSAGE_RATE);
        MQTT.publish(CS_VELOCITY_TOPIC, String(param1).c_str());
        MQTT.loop();
        // Serial.println("oi"); 
        // delayMicroseconds(1000*1000);
        // Serial.println("oi"); 

  }
}

void reconnect_mqtt_broker(void* parameter) {
    for(;;) {
        vTaskDelay(4000);
        while(!MQTT.connected()) {
            String mqtt_id = MQTT_ID;
            mqtt_id += String(random(0xffff), HEX);

            if(MQTT.connect(mqtt_id.c_str())) {
                Serial.print("Conectado no broker MQTT com o id ");
                Serial.println(mqtt_id);
            } else {
                Serial.print("Failed, rc=");
                Serial.print(MQTT.state());
                Serial.println(" Retrying MQTT connection...");
            }
        }
    }
    
}


// ------------------------------
// ------------ WiFi ------------
// ------------------------------
void wifi_setup(){
  
    WiFi.mode(WIFI_STA);
    esp_wifi_set_mac(WIFI_IF_STA, target_esp_mac);
    
    #ifdef UTFPR
    WiFi.begin(EAP_SSID , WPA2_AUTH_PEAP, EAP_IDENTITY , EAP_USERNAME, EAP_PASSWORD);
    #else
    WiFi.begin(EAP_SSID , EAP_PASSWORD);
    #endif

    Serial.println(EAP_SSID) ;
    Serial.print( "Trying to connect") ;
    while(WiFi.status()!= WL_CONNECTED){
        Serial.print(".") ;
        delay(400);
    }
    Serial.println("");
    Serial.print("Connected to local Wifi as IP ");
    Serial.println(WiFi.localIP());
  
}

void reconnect_wifi(void* parameter) {
    for(;;) {
        vTaskDelay(2000);
        if(WiFi.status() != WL_CONNECTED) {
            Serial.print( "Trying to connect") ;
            while(WiFi.status()!= WL_CONNECTED){
                Serial.print(".") ;
                vTaskDelay(400);
            }
            Serial.println("");
            Serial.print("Connected to local Wifi as IP ");
            Serial.println(WiFi.localIP());
        }
    }
}