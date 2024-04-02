/*
  mavlink class for handling OpenDroneID messages
 */
#pragma once
#include "transport.h"
#include "parameters.h"

/*
  abstraction for MAVLink on a serial port
 */
class MAVLinkSerial : public Transport {
public:
    bool sends[9];
    using Transport::Transport;
    MAVLinkSerial(HardwareSerial &serial, mavlink_channel_t chan);
    void init(void) override;
    void update(void) override;
    void mav_printf(uint8_t severity, const char *fmt, ...);
    void send_uav(double lat, double lon, double alt, uint8_t mac[6], uint16_t heading, uint16_t hor_vel, int16_t ver_vel);
    void send_uav(mavlink_open_drone_id_basic_id_t basic_id,mavlink_open_drone_id_system_t system,mavlink_open_drone_id_location_t location);
    void send_uav_location(mavlink_open_drone_id_location_t location);
    void send_uav_basic(mavlink_open_drone_id_basic_id_t basic_id);
//    void send_uav_system(mavlink_open_drone_id_system_t system);





    void schedule_send_uav(int i);
    mavlink_channel_t chan;

    HardwareSerial &serial;
    uint32_t last_hb_ms;
    uint32_t last_hb_warn_ms;
    uint32_t param_request_last_ms;
    const Parameters::Param *param_next;

    void update_receive(void);
    void update_send(mavlink_open_drone_id_basic_id_t* basic_id,mavlink_open_drone_id_system_t* system,mavlink_open_drone_id_location_t* location);
    void process_packet(mavlink_status_t &status, mavlink_message_t &msg);
    void handle_secure_command(const mavlink_secure_command_t &pkt);

};
