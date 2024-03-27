/* -*- tab-width: 2; mode: c; -*-
 * 
 * Scanner for WiFi direct remote id. 
 * Handles both opendroneid and French formats.
 * 
 * Copyright (c) 2020-2021, Steve Jack.
 *
 * MIT licence.
 * 
 * Nov. '21     Added option to dump ODID frame to serial output.
 * Oct. '21     Updated for opendroneid release 1.0.
 * June '21     Added an option to log to an SD card.
 * May '21      Fixed a bug that presented when handing packed ODID data from multiple sources. 
 * April '21    Added support for EN 4709-002 WiFi beacons.
 * March '21    Added BLE scan. Doesn't work very well.
 * January '21  Added support for ANSI/CTA 2063 French IDs.
 *
 * Notes
 * 
 * May need a semaphore.
 * 
 */

#if not defined(ARDUINO_ARCH_ESP32)
#error "This program requires an ESP32"
#endif

#pragma GCC diagnostic warning "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <nvs_flash.h>
#include<mutex>

// Don't change order of includes
#include "opendroneid/libopendroneid/opendroneid.h"
#include "mavlink.h"
#include "transport.h"
extern "C" {
  #include "opendroneid/libmav2odid/mav2odid.h"  
}


#define DIAGNOSTICS        1
#define DUMP_ODID_FRAME    0

#define WIFI_SCAN          0
#define BLE_SCAN           1// Experimental, does work very well.

#define SD_LOGGER          0
#define SD_CS              5
#define SD_LOGGER_LED      2

#define LCD_DISPLAY        0 // 11 for a SH1106 128X64 OLED.
#define DISPLAY_PAGE_MS 4000


#define ID_SIZE     (ODID_ID_SIZE + 1)
#define MAX_UAVS           8
#define OP_DISPLAY_LIMIT  16

#if SD_LOGGER

#include <SD.h>
// #include <SdFat.h>

// #define SD_CONFIG       SdSpiConfig(SD_CS,DEDICATED_SPI,SD_SCK_MHZ(16))

#endif

#if BLE_SCAN

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAddress.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#endif

//

struct id_data {int       flag;
                uint8_t   mac[6];
                uint32_t  last_seen;
                char      op_id[ID_SIZE];
                char      uav_id[ID_SIZE];
                double    lat_d, long_d, base_lat_d, base_long_d;
                int       altitude_msl, height_agl, speed, rssi;
                uint16_t  heading;
                uint16_t  hor_vel;
                int16_t   ver_vel;
};

#if SD_LOGGER
struct id_log  {int8_t    flushed;
                uint32_t  last_write;
                File      sd_log;
};
#endif

//

static void               print_json(int,int,struct id_data *);
static void               write_log(uint32_t,struct id_data *,struct id_log *);
static esp_err_t          event_handler(void *,system_event_t *);
static void               callback(void *,wifi_promiscuous_pkt_type_t);
static int next_uav(uint8_t *);
static void               parse_odid(struct id_data *,ODID_UAS_Data *);
                        

static double             base_lat_d = 0.0, base_long_d = 0.0, m_deg_lat = 110000.0, m_deg_long = 110000.0;
#if SD_LOGGER
static struct id_log      logfiles[MAX_UAVS + 1];
#endif

std::mutex uavs_mutex;
char             ssid[10];
unsigned int     callback_counter = 0, french_wifi = 0, odid_wifi = 0, odid_ble = 0;
// All four array are protected by the uavs_mutex;
struct id_data   uavs[MAX_UAVS + 1];
mavlink_open_drone_id_basic_id_t uav_basic_id[MAX_UAVS + 1];
mavlink_open_drone_id_system_t uav_system[MAX_UAVS + 1];
mavlink_open_drone_id_location_t uav_location[MAX_UAVS + 1];

volatile ODID_UAS_Data    UAS_data;

//

static const char        *title = "RID Scanner", *build_date = __DATE__,
                         *blank_latlong = " ---.------";

static MAVLinkSerial mavlink1{Serial1, MAVLINK_COMM_0};
//static MAVLinkSerial mavlink2{Serial, MAVLINK_COMM_1};
#include <SPI.h>//xyz xxx ???

#if BLE_SCAN

BLEScan *BLE_scan;
BLEUUID  service_uuid;

//

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  
    void onResult(BLEAdvertisedDevice device) {

      int                   i, k, len;
      char                  text[128];
      uint8_t              *payload, *odid, *mac;
      struct id_data       *UAV;
      ODID_BasicID_data     odid_basic;
      ODID_Location_data    odid_location;
      ODID_System_data      odid_system;
      ODID_OperatorID_data  odid_operator;

      text[0] = i = k = 0;
      
      //
      
      if ((len = device.getPayloadLength()) > 0) {

        BLEAddress ble_address = device.getAddress();
        mac                    = (uint8_t *) ble_address.getNative();

//      BLEUUID BLE_UUID = device.getServiceUUID(); // crashes program

        payload = device.getPayload();
        odid    = &payload[6];
#if 0
        for (i = 0, k = 0; i < ESP_BD_ADDR_LEN; ++i, k += 3) {

          sprintf(&text[k],"%02x ",mac[i]);
        }

        Serial.printf("%s\r\n",text);

#endif

        if ((payload[1] == 0x16)&&
            (payload[2] == 0xfa)&&
            (payload[3] == 0xff)&&
            (payload[4] == 0x0d)){
          uavs_mutex.lock();
          int UAV_i            = next_uav(mac);
          UAV = &uavs[UAV_i];
          uavs[UAV_i].last_seen = millis();
          uavs[UAV_i].rssi      = device.getRSSI();
          uavs[UAV_i].flag      = 1;
          mavlink1.schedule_send_uav(UAV_i);
          //mavlink2.schedule_send_uav(UAV_i);
          // Serial.println("Setting flag 1");

          memcpy(uavs[UAV_i].mac,mac,6);

          switch (odid[0] & 0xf0) {

          case 0x00: // basic

            decodeBasicIDMessage(&odid_basic,(ODID_BasicID_encoded *) odid);
            m2o_basicId2Mavlink(&uav_basic_id[UAV_i],&odid_basic);
            break;

          case 0x10: // location
          
            decodeLocationMessage(&odid_location,(ODID_Location_encoded *) odid);
            // print lat, lon xxx
            Serial.println("Lat=");

            Serial.println(odid_location.Latitude);


            mavlink_open_drone_id_location_t location_mav;
            m2o_location2Mavlink(&uav_location[UAV_i], &odid_location);
            break;

          case 0x40: // system

            decodeSystemMessage(&odid_system,(ODID_System_encoded *) odid);
            mavlink_open_drone_id_system_t system_mav;
            m2o_system2Mavlink(&uav_system[UAV_i], &odid_system);
            break;

          case 0x50: // operator

            decodeOperatorIDMessage(&odid_operator,(ODID_OperatorID_encoded *) odid);
            strncpy((char *) UAV->op_id,(char *) odid_operator.OperatorId,ODID_ID_SIZE);
            break;
          }
          uavs_mutex.unlock();

          ++odid_ble;
        }
      }

      return;
    }
};

#endif

/*
 *
 */

void setup() {

  int         i;
  char        text[128];

  text[0] = i = 0;

  //

  memset((void *) &UAS_data,0,sizeof(ODID_UAS_Data));
  memset((void *) uavs,0,(MAX_UAVS + 1) * sizeof(struct id_data));
  memset((void *) ssid,0,10);

  strcpy((char *) uavs[MAX_UAVS].op_id,"NONE");

#if SD_LOGGER

  for (i = 0; i <= MAX_UAVS; ++i) {

    logfiles[i].flushed    = 1; 
    logfiles[i].last_write = 0;
  }

#endif
  delay(100);

  Serial.begin(57600);
//  Serial1.begin(57600, SERIAL_8N1, 17, 18);
  Serial1.begin(57600, SERIAL_8N1, 2,3);

  nvs_flash_init();
  tcpip_adapter_init();

  esp_event_loop_init(event_handler,NULL);

#if WIFI_SCAN
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&callback); 

  // The channel should be 6.
  // If the second parameter is not WIFI_SECOND_CHAN_NONE, cast it to (wifi_second_chan_t).
  // There has been a report of the ESP not scanning the first channel if the second is set.
  esp_wifi_set_channel(6,WIFI_SECOND_CHAN_NONE);
#endif

#if BLE_SCAN
  BLEDevice::init(title);

  service_uuid = BLEUUID("0000fffa-0000-1000-8000-00805f9b34fb");
  BLE_scan     = BLEDevice::getScan();

  BLE_scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  BLE_scan->setActiveScan(true); 
  BLE_scan->setInterval(100);
  BLE_scan->setWindow(99);  
#endif

#if SD_LOGGER

  File root, file;

  pinMode(SD_LOGGER_LED,OUTPUT);
  digitalWrite(SD_LOGGER_LED,0);

  if (SD.begin(SD_CS)) {

    if (root = SD.open("/")) {

      while (file = root.openNextFile()) {

        sprintf(text,"{ \"file\": \"%s\", \"size\": %u }\r\n",file.name(),file.size());
        Serial.print(text);
        
        file.close();
      }

      root.close();
    }
  }

#endif

  mavlink1.init();
  //mavlink2.init();

  return;
}

/*
 *
 */
void loop() {

  int             i, j, k, msl, agl;
  char            text[256];
  double          x_m = 0.0, y_m = 0.0;
  uint32_t        msecs, secs;
  static int      display_uav = 0;
  static uint32_t last_display_update = 0, last_page_change = 0, last_json = 0;

  text[0] = i = j = k = 0;

  //
  static uint64_t last_send = 0;
  
  msecs = millis();

  mavlink1.update();
  //mavlink2.update();
  mavlink1.update_send(uav_basic_id,uav_system,uav_location);
  //mavlink2.update_send(uav_basic_id,uav_system,uav_location);

#if BLE_SCAN

  uint32_t last_ble_scan = 0;

  if ((msecs - last_ble_scan) > 2000) {

    last_ble_scan = msecs;
  
    BLEScanResults foundDevices = BLE_scan->start(1,false);

    BLE_scan->clearResults(); 
  }
  
#endif

  msecs = millis();
  secs  = msecs / 1000;
  delay(100);

  Serial.println("Running loop");
  uavs_mutex.lock();
  for (i = 0; i < MAX_UAVS; ++i) {
    if ((uavs[i].last_seen)&&
        ((msecs - uavs[i].last_seen) > 300000L)) {
      uavs[i].last_seen = 0;
      uavs[i].mac[0]    = 0;
    }
    //print_json(i,secs,(id_data *) &uavs[i]);
    
    // Serial.printf("Flag: %d\n", uavs[i].flag);
    //Serial.println("aaa Hello world");
    if (uavs[i].flag) {
      //print_json(i,secs,(id_data *) &uavs[i]);
      //Serial.println(uav_location[i].latitude);
      Serial.println("Hello world");

      // uavs_mutex.lock();
      id_data UAV = uavs[i];
      // mavlink1.send_uav(UAV.lat_d,UAV.long_d,UAV.altitude_msl, UAV.mac, UAV.heading, UAV.hor_vel, UAV.ver_vel);
      // mavlink2.send_uav(UAV.lat_d,UAV.long_d,UAV.altitude_msl, UAV.mac, UAV.heading, UAV.hor_vel, UAV.ver_vel);
      mavlink1.send_uav(uav_basic_id[i],uav_system[i],uav_location[i]);
     //mavlink2.send_uav(uav_basic_id[i],uav_system[i],uav_location[i]);
      // uavs_mutex.unlock();
      // mavlink1.mav_printf(MAV_SEVERITY_INFO, "uav found %f,%f", uavs[i].lat_d,uavs[i].long_d);
      // mavlink2.mav_printf(MAV_SEVERITY_INFO, "uav found %f,%f", uavs[i].lat_d,uavs[i].long_d);


      uavs[i].flag = 0;
      // Serial.println("UnSetting flag 1");

      last_json = msecs;
    }
  }
  uavs_mutex.unlock();


  if ((msecs - last_json) > 60000UL) { // Keep the serial link active

      print_json(MAX_UAVS,msecs / 1000,(id_data *) &uavs[MAX_UAVS]); 

      last_json = msecs;
  }

  return;
}

/*
 *
 */

void print_json(int index,int secs,struct id_data *UAV) {

  char text[128], text1[16],text2[16], text3[16], text4[16];

  dtostrf(UAV->lat_d,11,6,text1);
  dtostrf(UAV->long_d,11,6,text2);
  dtostrf(UAV->base_lat_d,11,6,text3);
  dtostrf(UAV->base_long_d,11,6,text4);

  sprintf(text,"{ \"index\": %d, \"runtime\": %d, \"mac\": \"%02x:%02x:%02x:%02x:%02x:%02x\", ",
          index,secs,
          UAV->mac[0],UAV->mac[1],UAV->mac[2],UAV->mac[3],UAV->mac[4],UAV->mac[5]);
  Serial.print(text);
  sprintf(text,"\"id\": \"%s\", \"uav latitude\": %s, \"uav longitude\": %s, \"alitude msl\": %d, ",
          UAV->op_id,text1,text2,UAV->altitude_msl);
  Serial.print(text);
  sprintf(text,"\"height agl\": %d, \"base latitude\": %s, \"base longitude\": %s, \"speed\": %d, \"heading\": %d }\r\n",
          UAV->height_agl,text3,text4,UAV->speed,UAV->heading);
  Serial.print(text);

  return;
}


/*
 *
 */

void write_log(uint32_t msecs,struct id_data *UAV,struct id_log *logfile) {

#if SD_LOGGER

  int       secs, dsecs;
  char      text[128], filename[24], text1[16], text2[16];

  secs  = (int) (msecs / 1000);
  dsecs = ((short int) (msecs - (secs * 1000))) / 100;

  //

  if (!logfile->sd_log) {

    sprintf(filename,"/%02X%02X%02X%02X.TSV",
            UAV->mac[2],UAV->mac[3],UAV->mac[4],UAV->mac[5]);

    if (!(logfile->sd_log = SD.open(filename,FILE_APPEND))) {

      sprintf(text,"{ \"message\": \"Unable to open \'%s\'\" }\r\n",filename);
      Serial.print(text);
    }
  }

  //

  if (logfile->sd_log) {

    dtostrf(UAV->lat_d,11,6,text1);
    dtostrf(UAV->long_d,11,6,text2);

    sprintf(text,"%d.%d\t%s\t%s\t%s\t%s\t",
            secs,dsecs,UAV->op_id,UAV->uav_id,text1,text2);
    logfile->sd_log.print(text);

    sprintf(text,"%d\t%d\t%d\t",
            (int) UAV->altitude_msl,(int) UAV->speed,
            (int) UAV->heading);
    logfile->sd_log.print(text);

    logfile->sd_log.print("\r\n");
    logfile->flushed = 0;
  }
  
#endif

  return;
}

/*
 *
 */

esp_err_t event_handler(void *ctx, system_event_t *event) {
  
  return ESP_OK;
}

/*
 * This function handles WiFi packets.
 */

void callback(void* buffer,wifi_promiscuous_pkt_type_t type) {

  int                     length, typ, len, i, j, offset;
  char                    ssid_tmp[10], *a;
  uint8_t                *packet_u8, *payload, *val;
  wifi_promiscuous_pkt_t *packet;
  struct id_data         *UAV = NULL;
  static uint8_t          mac[6], nan_dest[6] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};

  a = NULL;
  
//

  ++callback_counter;

  memset(ssid_tmp,0,10);

  packet    = (wifi_promiscuous_pkt_t *) buffer;
  packet_u8 = (uint8_t *) buffer;
  
  payload   = packet->payload;
  length    = packet->rx_ctrl.sig_len;
  offset    = 36;

//

  int UAV_i = next_uav(&payload[10]);
  UAV = &UAV[UAV_i];

  memcpy(UAV->mac,&payload[10],6);

  UAV->rssi      = packet->rx_ctrl.rssi;
  UAV->last_seen = millis();

//

  if (memcmp(nan_dest,&payload[4],6) == 0) {


    if (odid_wifi_receive_message_pack_nan_action_frame((ODID_UAS_Data *) &UAS_data,(char *) mac,payload,length) == 0) {

      ++odid_wifi;

      parse_odid(UAV,(ODID_UAS_Data *) &UAS_data);
    }

  } else if (payload[0] == 0x80) { // beacon

    offset = 36;

    while (offset < length) {

      typ =  payload[offset];
      len =  payload[offset + 1];
      val = &payload[offset + 2];

      if ((typ    == 0xdd)&&
          (val[0] == 0x6a)&& // French
          (val[1] == 0x5c)&&
          (val[2] == 0x35)) {

        ++french_wifi;


      } else if ((typ      == 0xdd)&&
                 (((val[0] == 0x90)&&(val[1] == 0x3a)&&(val[2] == 0xe6))|| // Parrot
                  ((val[0] == 0xfa)&&(val[1] == 0x0b)&&(val[2] == 0xbc)))) { // ODID

        ++odid_wifi;

        if ((j = offset + 7) < length) {

          memset((void *) &UAS_data,0,sizeof(UAS_data));
          
          odid_message_process_pack((ODID_UAS_Data *) &UAS_data,&payload[j],length - j);

          parse_odid(UAV,(ODID_UAS_Data *) &UAS_data);
        }

      } else if ((typ == 0)&&(!ssid_tmp[0])) {

        for (i = 0; (i < 8)&&(i < len); ++i) {

          ssid_tmp[i] = val[i];
        }
      }

      offset += len + 2;
    }

    if (ssid_tmp[0]) {

      strncpy((char *) ssid,ssid_tmp,8);
    }
#if 0
  } else if (a = (char *) memchr(payload,'G',length)) {

    if (memcmp(a,"GBR-OP-",7) == 0) {

    }
#endif
  }

  if ((!UAV->op_id[0])&&(!UAV->lat_d)) {

    UAV->mac[0] = 0;
  }

  return;
}

/*
 * uavs_mutex must be accquired before calling this function
 */
int next_uav(uint8_t *mac) {

  int             i;
  struct id_data *UAV = NULL;

  for (i = 0; i < MAX_UAVS; ++i) {

    if (memcmp((void *) uavs[i].mac,mac,6) == 0) {

      UAV = (struct id_data *) &uavs[i];
    }
  }

  if (!UAV) {

    for (i = 0; i < MAX_UAVS; ++i) {

      if (!uavs[i].mac[0]) {

        UAV = (struct id_data *) &uavs[i];
        break;
      }
    }
  }

  if (!UAV) {

     UAV = (struct id_data *) &uavs[MAX_UAVS - 1];
  }

  return i;
}

/*
 *
 */

void parse_odid(struct id_data *UAV,ODID_UAS_Data *UAS_data2) {

  if (UAS_data2->BasicIDValid[0]) {

    UAV->flag = 1;
      // Serial.println("Setting flag 2");
    strncpy((char *) UAV->uav_id,(char *) UAS_data2->BasicID[0].UASID,ODID_ID_SIZE);
  }

  if (UAS_data2->OperatorIDValid) {

    UAV->flag = 1;
      // Serial.println("Setting flag 3");
    strncpy((char *) UAV->op_id,(char *) UAS_data2->OperatorID.OperatorId,ODID_ID_SIZE);
  }

  if (UAS_data2->LocationValid) {

    UAV->flag         = 1;
      // Serial.println("Setting flag 4");
    UAV->lat_d        = UAS_data2->Location.Latitude;
    UAV->long_d       = UAS_data2->Location.Longitude;
    UAV->altitude_msl = (int) UAS_data2->Location.AltitudeGeo;
    UAV->height_agl   = (int) UAS_data2->Location.Height;
    UAV->speed        = (int) UAS_data2->Location.SpeedHorizontal;
    UAV->heading      = (int) UAS_data2->Location.Direction;
  }

  if (UAS_data2->SystemValid) {

    UAV->flag        = 1;
      // Serial.println("Setting flag 5");
    UAV->base_lat_d  = UAS_data2->System.OperatorLatitude;
    UAV->base_long_d = UAS_data2->System.OperatorLongitude;
  }  

  return;
}

