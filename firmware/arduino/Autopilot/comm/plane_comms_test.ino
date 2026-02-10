#include <MAVLink.h> //mavlink kütüphanesi çağırılır 
#include <SoftwareSerial.h>
SoftwareSerial telSerial(10, 11);
uint8_t system_id = 1;        // Drone ID
uint8_t component_id = 1;     // Otopilot ID
float altitude_m = 0.0;       // Metre cinsinden altitude verisi oluşturulur 
float latitiude= 0;
float longtitude=0;
float speed_x=0;
float speed_y=0;
float speed_z=0;
float head= 0;
float rakip_lat, rakip_lon, rakip_alt;
void setup() {
  telSerial.begin(57600);   // Seri haberleşme başlatılır. 
}

void loop() {
  send_heartbeat();
  send_attitude(0,0,0,0,0,0);   // "Test için değerler 0 olarak girilmiştir."
  send_position(altitude_m,latitiude,longtitude,speed_x,speed_y,speed_z,head); //"Test için random değerler girilmiştir."
  rakip_inf();
  delay(1000);                 // Dongü saniyede 1 kere çalışacak şekilde ayarlanır.
}

void send_heartbeat() {
  mavlink_message_t msg;  
  uint8_t buf[MAVLINK_MAX_PACKET_LEN]; 
  //gönderilecek mesaj hazırlanır
  mavlink_msg_heartbeat_pack( 
    1,
    MAV_COMP_ID_AUTOPILOT1,
    &msg, 
    MAV_TYPE_QUADROTOR,
    MAV_AUTOPILOT_GENERIC, 
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 
    0, 
    MAV_STATE_STANDBY);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  telSerial.write(buf, len);   // Veri gönderilir.

}
// iha'nın roll pitch yaw gibi değerleri gönderilir.
void send_attitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_attitude_pack(
    system_id,
    component_id,
    &msg,
    millis(),
    roll,
    pitch,
    yaw,
    rollspeed,
    pitchspeed,
    yawspeed
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  telSerial.write(buf, len);
}


//iha'nın konumunu, yüksekliği ve hızını geönderir.
void send_position(float alt_meters, float lat,float lon,float vx, float vy,float vz,float heading){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int32_t relative_alt = (int32_t)(alt_meters * 1000); // Metreyi milimetreye çevir 
  mavlink_msg_global_position_int_pack(
    system_id, 
    component_id, 
    &msg, 
    millis(),   
    lat,        // Latitude
    lon,        // Longitude
    relative_alt, // Alt (MSL) 
    relative_alt, // Relative Alt 
    vy, vx, vz,      // Hızlar (vx, vy, vz)
    heading       // Heading 
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  telSerial.write(buf, len);




}

void rakip_inf() {
  if (Serial.available() > 0) {
    
    //  (Gönderdiğimiz paket "(*Enlem,Boylam,İrtifa)" şeklindedir)
    if (Serial.find("*")) {
      
      // Sırasıyla float değerleri okurnur ve değişkenlere atarnır
      rakip_lat  = Serial.parseFloat(); 
      rakip_lon = Serial.parseFloat(); 
      rakip_alt = Serial.parseFloat();
    }
  }
}