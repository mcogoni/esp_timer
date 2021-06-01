/*********
  Marco Cogoni
*********/
#include <Wire.h>
#include <WiFi.h>
#include "time.h"
#include <RTClib.h>
#include "esp_sleep.h"

const char* ssid = "linkem2";
const char* password = "";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
const char* ntpServer = "pool.ntp.org";
int old_second = -1;
RTC_DATA_ATTR int bootCount = 0;

const int max_relays = 8;
const int events = 9;
typedef struct {
    int relay_n;
    int h;
    int m;
    int duration; // if 0 just change relay state, else turn on and off after duration ms
} time_tuple;    //  creates a struct type time_tuple
time_tuple timer_time[events] = {{0,6,30,1500}, {0,7,30,1500}, {0,8,30,1500}, {0,9,30,1500}, {0,10,30,1500}, {0,16,35,1500}};
//, {1,6,32,500}, {1,6,52,500}, {2,6,52,500}, {2,7,22,500}, {3,15,25,1000}, {2,15,25,1000}, {1,15,35,1000}, {1,15,56,1000}};

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char relay_name[max_relays][10] = {"CBSM", "FRONTE", "PRATO", "FRUTTETO", "", "", "", ""};
int relay_pin[max_relays] = {27, 26, 25, 14};
int relay_status[max_relays] = {0, 0, 0, 0, 0, 0, 0, 0};

RTC_DS3231 rtc;                     // create rtc for the DS3231 RTC module, address is fixed at 0x68

void print_time_ntp()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void adjust_rtc_via_ntp(tm timeinfo) {
  int a, m, d, h, n, s;
  a = timeinfo.tm_year+1900;
  m = timeinfo.tm_mon+1;
  d = timeinfo.tm_mday;
  h = timeinfo.tm_hour;
  n = timeinfo.tm_min;
  s = timeinfo.tm_sec;
  rtc.adjust(DateTime(a, m, d, h, n, s));
  //rtc.adjust(DateTime(2021, 5, 31, 10, 43, 20));
}

void print_time_rtc() { 
  DateTime now = rtc.now();
   
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

void delay_marco(int del_ms) {
  unsigned long previousMillis = millis();
  while (millis() - previousMillis < del_ms) {
  }
}

void relay_onoff(int relay_n, int time_ms) {
  digitalWrite(relay_pin[relay_n], LOW);
  Serial.print("Enabling relay number: ");
  Serial.println(relay_name[relay_n]);
  delay_marco(time_ms);
  digitalWrite(relay_pin[relay_n], HIGH);
  Serial.print("Disabling relay number: ");
  Serial.println(relay_name[relay_n]);
}

void relay_on(int relay_n) {
  digitalWrite(relay_pin[relay_n], LOW);
  Serial.print("Enabling relay number: ");
  Serial.println(relay_name[relay_n]);
  relay_status[relay_n] = 1;
}

void relay_off(int relay_n) {
  digitalWrite(relay_pin[relay_n], HIGH);
  Serial.print("Disabling relay number: ");
  Serial.println(relay_name[relay_n]);
  relay_status[relay_n] = 0;
}

int next_timer() {
  DateTime now = rtc.now();
  uint8_t h = now.hour();
  uint8_t m = now.minute();
  uint8_t s = now.second();

  volatile int delta_min;
  delta_min = 1800;
  for (int i=0; i<events; i++) {
    int delta;
    delta = 60*60*((timer_time[i].h) - h);
    delta += 60*((timer_time[i].m) - m);
    delta -= s;
    
    if ((delta > 0) && (delta < delta_min)) {
      delta_min = delta;
    }
  }
  Serial.print("Time in seconds to next timer: ");
  Serial.println(int(delta_min));
  return delta_min; // return time to next timer in seconds
}

void setup() {
  
  Serial.begin(115200);
  delay_marco(1000); // wait for console opening

  Serial.println(bootCount);
  
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (bootCount == 0){
    //connect to WiFi
    Serial.printf("Connecting to %s ", ssid);
    WiFi.begin(ssid, password);
  
    int count = 0;
    while (WiFi.status() != WL_CONNECTED and count<20) {
        delay(500);
        count++;
        Serial.print(".");
    }
    Serial.println(WiFi.status());
    delay(1000);
  
    if (WiFi.status() == WL_CONNECTED) {
      //init and get the time
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      
      struct tm timeinfo;
      if(getLocalTime(&timeinfo)){
        if (rtc.lostPower()) {
          Serial.println("RTC lost power, lets set the time!");
          adjust_rtc_via_ntp(timeinfo);
        } else {
          Serial.println("WIFI and NTP connected, lets set the time even if RTC is OK!");
          adjust_rtc_via_ntp(timeinfo);
        }
      } else {
        Serial.println("Failed to obtain time");
      }
      //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    
      //disconnect WiFi as it's no longer needed
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    } else {
      Serial.println("Failed to connect to WIFI!");
    }
            
    Wire.begin(21, 22);
    Wire.setClock(400000L);   // set I2C clock to 400kHz
  }
  for (int relay_n=0; relay_n < 8; relay_n++) {
    pinMode(relay_pin[relay_n], OUTPUT);
    digitalWrite(relay_pin[relay_n], HIGH);
    //Serial.println(relay_pin[relay_n]);
  }

  bootCount++;
  print_time_rtc();
}

void loop() {
  DateTime now = rtc.now();
  
  if (now.second() != old_second) {
    old_second = now.second();
    print_time_rtc();

    if (now.second() == 0) {
            
      for (int event; event< events; event++) {
        if (now.hour() == timer_time[event].h) {
          if (now.minute() == timer_time[event].m) {
            if (timer_time[event].duration > 0) {
              relay_onoff(timer_time[event].relay_n, timer_time[event].duration);
            } else {
              if (not relay_status[timer_time[event].relay_n]) {
                relay_on(timer_time[event].relay_n);
              } else {
                relay_off(timer_time[event].relay_n);
              }
    
            }
          }
        }
      }
    }
    int sleep_delay = next_timer();
  
    if (sleep_delay > 60) {
      uint64_t sleeptime = (sleep_delay-30) * 1000000;
      esp_sleep_enable_timer_wakeup(sleeptime);
      //rtc_gpio_isolate(GPIO_NUM_12);
      
      Serial.println("Entering deep sleep...");
      esp_deep_sleep_start();
    }
    
  }

}
