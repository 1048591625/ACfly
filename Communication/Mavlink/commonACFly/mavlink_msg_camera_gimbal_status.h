#pragma once
// MESSAGE CAMERA_GIMBAL_STATUS PACKING

#define MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS 272


typedef struct __mavlink_camera_gimbal_status_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float gimBal_pitch; /*< [deg] pitch of gimBal (>360 if unknown).*/
 float gimBal_yaw; /*< [deg] yaw of gimBal (>360 if unknown).*/
 float hfov_now; /*< [deg] current horizontal field of view (>360 if unknown).*/
 float hfov_min; /*< [deg] minimal horizontal field of view (>360 if unknown).*/
 float hfov_max; /*< [deg] Maximum horizontal field of view (>360 if unknown).*/
 float vfov_now; /*< [deg] current vertical field of view (>360 if unknown).*/
 float vfov_min; /*< [deg] minimal vertical field of view (>360 if unknown).*/
 float vfov_max; /*< [deg] Maximum vertical field of view (>360 if unknown).*/
 float distance_detected; /*< [m] radar distance(-1 if unknown).*/
 int32_t target_lat; /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
 int32_t target_lon; /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/
 int32_t target_alt; /*< [m] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.*/
 int8_t currentZoomLevel; /*<  zoomLevel of camera (-1 if unknown).*/
 int8_t zoomLevel_min; /*<  minZoomLevel of camera (-1 if unknown).*/
 int8_t zoomLevel_max; /*<  maxZoomLevel of camera (-1 if unknown).*/
 int8_t yaw_mode; /*<  yaw_mode, 0: follow, 1:lock, 2:look down (-1 if unknown).*/
 int8_t track_mode; /*<  track_mode, 0: not active, 1:active(-1 if unknown).*/
 uint8_t flag; /*<  0: dynamic fov value not available; 1:dynamic fov value available.*/
} mavlink_camera_gimbal_status_t;

#define MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN 58
#define MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN 58
#define MAVLINK_MSG_ID_272_LEN 58
#define MAVLINK_MSG_ID_272_MIN_LEN 58

#define MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_CRC 206
#define MAVLINK_MSG_ID_272_CRC 206



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_GIMBAL_STATUS { \
    272, \
    "CAMERA_GIMBAL_STATUS", \
    19, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_gimbal_status_t, time_boot_ms) }, \
         { "currentZoomLevel", NULL, MAVLINK_TYPE_INT8_T, 0, 52, offsetof(mavlink_camera_gimbal_status_t, currentZoomLevel) }, \
         { "zoomLevel_min", NULL, MAVLINK_TYPE_INT8_T, 0, 53, offsetof(mavlink_camera_gimbal_status_t, zoomLevel_min) }, \
         { "zoomLevel_max", NULL, MAVLINK_TYPE_INT8_T, 0, 54, offsetof(mavlink_camera_gimbal_status_t, zoomLevel_max) }, \
         { "gimBal_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_gimbal_status_t, gimBal_pitch) }, \
         { "gimBal_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_gimbal_status_t, gimBal_yaw) }, \
         { "hfov_now", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_gimbal_status_t, hfov_now) }, \
         { "hfov_min", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_gimbal_status_t, hfov_min) }, \
         { "hfov_max", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_gimbal_status_t, hfov_max) }, \
         { "vfov_now", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_gimbal_status_t, vfov_now) }, \
         { "vfov_min", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_camera_gimbal_status_t, vfov_min) }, \
         { "vfov_max", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_camera_gimbal_status_t, vfov_max) }, \
         { "distance_detected", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_camera_gimbal_status_t, distance_detected) }, \
         { "yaw_mode", NULL, MAVLINK_TYPE_INT8_T, 0, 55, offsetof(mavlink_camera_gimbal_status_t, yaw_mode) }, \
         { "track_mode", NULL, MAVLINK_TYPE_INT8_T, 0, 56, offsetof(mavlink_camera_gimbal_status_t, track_mode) }, \
         { "target_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_camera_gimbal_status_t, target_lat) }, \
         { "target_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_camera_gimbal_status_t, target_lon) }, \
         { "target_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_camera_gimbal_status_t, target_alt) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 57, offsetof(mavlink_camera_gimbal_status_t, flag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_GIMBAL_STATUS { \
    "CAMERA_GIMBAL_STATUS", \
    19, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_gimbal_status_t, time_boot_ms) }, \
         { "currentZoomLevel", NULL, MAVLINK_TYPE_INT8_T, 0, 52, offsetof(mavlink_camera_gimbal_status_t, currentZoomLevel) }, \
         { "zoomLevel_min", NULL, MAVLINK_TYPE_INT8_T, 0, 53, offsetof(mavlink_camera_gimbal_status_t, zoomLevel_min) }, \
         { "zoomLevel_max", NULL, MAVLINK_TYPE_INT8_T, 0, 54, offsetof(mavlink_camera_gimbal_status_t, zoomLevel_max) }, \
         { "gimBal_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_gimbal_status_t, gimBal_pitch) }, \
         { "gimBal_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_gimbal_status_t, gimBal_yaw) }, \
         { "hfov_now", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_gimbal_status_t, hfov_now) }, \
         { "hfov_min", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_gimbal_status_t, hfov_min) }, \
         { "hfov_max", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_gimbal_status_t, hfov_max) }, \
         { "vfov_now", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_gimbal_status_t, vfov_now) }, \
         { "vfov_min", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_camera_gimbal_status_t, vfov_min) }, \
         { "vfov_max", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_camera_gimbal_status_t, vfov_max) }, \
         { "distance_detected", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_camera_gimbal_status_t, distance_detected) }, \
         { "yaw_mode", NULL, MAVLINK_TYPE_INT8_T, 0, 55, offsetof(mavlink_camera_gimbal_status_t, yaw_mode) }, \
         { "track_mode", NULL, MAVLINK_TYPE_INT8_T, 0, 56, offsetof(mavlink_camera_gimbal_status_t, track_mode) }, \
         { "target_lat", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_camera_gimbal_status_t, target_lat) }, \
         { "target_lon", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_camera_gimbal_status_t, target_lon) }, \
         { "target_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_camera_gimbal_status_t, target_alt) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 57, offsetof(mavlink_camera_gimbal_status_t, flag) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_gimbal_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param currentZoomLevel  zoomLevel of camera (-1 if unknown).
 * @param zoomLevel_min  minZoomLevel of camera (-1 if unknown).
 * @param zoomLevel_max  maxZoomLevel of camera (-1 if unknown).
 * @param gimBal_pitch [deg] pitch of gimBal (>360 if unknown).
 * @param gimBal_yaw [deg] yaw of gimBal (>360 if unknown).
 * @param hfov_now [deg] current horizontal field of view (>360 if unknown).
 * @param hfov_min [deg] minimal horizontal field of view (>360 if unknown).
 * @param hfov_max [deg] Maximum horizontal field of view (>360 if unknown).
 * @param vfov_now [deg] current vertical field of view (>360 if unknown).
 * @param vfov_min [deg] minimal vertical field of view (>360 if unknown).
 * @param vfov_max [deg] Maximum vertical field of view (>360 if unknown).
 * @param distance_detected [m] radar distance(-1 if unknown).
 * @param yaw_mode  yaw_mode, 0: follow, 1:lock, 2:look down (-1 if unknown).
 * @param track_mode  track_mode, 0: not active, 1:active(-1 if unknown).
 * @param target_lat [degE7] Latitude (WGS84, EGM96 ellipsoid)
 * @param target_lon [degE7] Longitude (WGS84, EGM96 ellipsoid)
 * @param target_alt [m] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
 * @param flag  0: dynamic fov value not available; 1:dynamic fov value available.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_gimbal_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, int8_t currentZoomLevel, int8_t zoomLevel_min, int8_t zoomLevel_max, float gimBal_pitch, float gimBal_yaw, float hfov_now, float hfov_min, float hfov_max, float vfov_now, float vfov_min, float vfov_max, float distance_detected, int8_t yaw_mode, int8_t track_mode, int32_t target_lat, int32_t target_lon, int32_t target_alt, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, gimBal_pitch);
    _mav_put_float(buf, 8, gimBal_yaw);
    _mav_put_float(buf, 12, hfov_now);
    _mav_put_float(buf, 16, hfov_min);
    _mav_put_float(buf, 20, hfov_max);
    _mav_put_float(buf, 24, vfov_now);
    _mav_put_float(buf, 28, vfov_min);
    _mav_put_float(buf, 32, vfov_max);
    _mav_put_float(buf, 36, distance_detected);
    _mav_put_int32_t(buf, 40, target_lat);
    _mav_put_int32_t(buf, 44, target_lon);
    _mav_put_int32_t(buf, 48, target_alt);
    _mav_put_int8_t(buf, 52, currentZoomLevel);
    _mav_put_int8_t(buf, 53, zoomLevel_min);
    _mav_put_int8_t(buf, 54, zoomLevel_max);
    _mav_put_int8_t(buf, 55, yaw_mode);
    _mav_put_int8_t(buf, 56, track_mode);
    _mav_put_uint8_t(buf, 57, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN);
#else
    mavlink_camera_gimbal_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.gimBal_pitch = gimBal_pitch;
    packet.gimBal_yaw = gimBal_yaw;
    packet.hfov_now = hfov_now;
    packet.hfov_min = hfov_min;
    packet.hfov_max = hfov_max;
    packet.vfov_now = vfov_now;
    packet.vfov_min = vfov_min;
    packet.vfov_max = vfov_max;
    packet.distance_detected = distance_detected;
    packet.target_lat = target_lat;
    packet.target_lon = target_lon;
    packet.target_alt = target_alt;
    packet.currentZoomLevel = currentZoomLevel;
    packet.zoomLevel_min = zoomLevel_min;
    packet.zoomLevel_max = zoomLevel_max;
    packet.yaw_mode = yaw_mode;
    packet.track_mode = track_mode;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_CRC);
}

/**
 * @brief Pack a camera_gimbal_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param currentZoomLevel  zoomLevel of camera (-1 if unknown).
 * @param zoomLevel_min  minZoomLevel of camera (-1 if unknown).
 * @param zoomLevel_max  maxZoomLevel of camera (-1 if unknown).
 * @param gimBal_pitch [deg] pitch of gimBal (>360 if unknown).
 * @param gimBal_yaw [deg] yaw of gimBal (>360 if unknown).
 * @param hfov_now [deg] current horizontal field of view (>360 if unknown).
 * @param hfov_min [deg] minimal horizontal field of view (>360 if unknown).
 * @param hfov_max [deg] Maximum horizontal field of view (>360 if unknown).
 * @param vfov_now [deg] current vertical field of view (>360 if unknown).
 * @param vfov_min [deg] minimal vertical field of view (>360 if unknown).
 * @param vfov_max [deg] Maximum vertical field of view (>360 if unknown).
 * @param distance_detected [m] radar distance(-1 if unknown).
 * @param yaw_mode  yaw_mode, 0: follow, 1:lock, 2:look down (-1 if unknown).
 * @param track_mode  track_mode, 0: not active, 1:active(-1 if unknown).
 * @param target_lat [degE7] Latitude (WGS84, EGM96 ellipsoid)
 * @param target_lon [degE7] Longitude (WGS84, EGM96 ellipsoid)
 * @param target_alt [m] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
 * @param flag  0: dynamic fov value not available; 1:dynamic fov value available.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_gimbal_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,int8_t currentZoomLevel,int8_t zoomLevel_min,int8_t zoomLevel_max,float gimBal_pitch,float gimBal_yaw,float hfov_now,float hfov_min,float hfov_max,float vfov_now,float vfov_min,float vfov_max,float distance_detected,int8_t yaw_mode,int8_t track_mode,int32_t target_lat,int32_t target_lon,int32_t target_alt,uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, gimBal_pitch);
    _mav_put_float(buf, 8, gimBal_yaw);
    _mav_put_float(buf, 12, hfov_now);
    _mav_put_float(buf, 16, hfov_min);
    _mav_put_float(buf, 20, hfov_max);
    _mav_put_float(buf, 24, vfov_now);
    _mav_put_float(buf, 28, vfov_min);
    _mav_put_float(buf, 32, vfov_max);
    _mav_put_float(buf, 36, distance_detected);
    _mav_put_int32_t(buf, 40, target_lat);
    _mav_put_int32_t(buf, 44, target_lon);
    _mav_put_int32_t(buf, 48, target_alt);
    _mav_put_int8_t(buf, 52, currentZoomLevel);
    _mav_put_int8_t(buf, 53, zoomLevel_min);
    _mav_put_int8_t(buf, 54, zoomLevel_max);
    _mav_put_int8_t(buf, 55, yaw_mode);
    _mav_put_int8_t(buf, 56, track_mode);
    _mav_put_uint8_t(buf, 57, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN);
#else
    mavlink_camera_gimbal_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.gimBal_pitch = gimBal_pitch;
    packet.gimBal_yaw = gimBal_yaw;
    packet.hfov_now = hfov_now;
    packet.hfov_min = hfov_min;
    packet.hfov_max = hfov_max;
    packet.vfov_now = vfov_now;
    packet.vfov_min = vfov_min;
    packet.vfov_max = vfov_max;
    packet.distance_detected = distance_detected;
    packet.target_lat = target_lat;
    packet.target_lon = target_lon;
    packet.target_alt = target_alt;
    packet.currentZoomLevel = currentZoomLevel;
    packet.zoomLevel_min = zoomLevel_min;
    packet.zoomLevel_max = zoomLevel_max;
    packet.yaw_mode = yaw_mode;
    packet.track_mode = track_mode;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_CRC);
}

/**
 * @brief Encode a camera_gimbal_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_gimbal_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_gimbal_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_gimbal_status_t* camera_gimbal_status)
{
    return mavlink_msg_camera_gimbal_status_pack(system_id, component_id, msg, camera_gimbal_status->time_boot_ms, camera_gimbal_status->currentZoomLevel, camera_gimbal_status->zoomLevel_min, camera_gimbal_status->zoomLevel_max, camera_gimbal_status->gimBal_pitch, camera_gimbal_status->gimBal_yaw, camera_gimbal_status->hfov_now, camera_gimbal_status->hfov_min, camera_gimbal_status->hfov_max, camera_gimbal_status->vfov_now, camera_gimbal_status->vfov_min, camera_gimbal_status->vfov_max, camera_gimbal_status->distance_detected, camera_gimbal_status->yaw_mode, camera_gimbal_status->track_mode, camera_gimbal_status->target_lat, camera_gimbal_status->target_lon, camera_gimbal_status->target_alt, camera_gimbal_status->flag);
}

/**
 * @brief Encode a camera_gimbal_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_gimbal_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_gimbal_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_gimbal_status_t* camera_gimbal_status)
{
    return mavlink_msg_camera_gimbal_status_pack_chan(system_id, component_id, chan, msg, camera_gimbal_status->time_boot_ms, camera_gimbal_status->currentZoomLevel, camera_gimbal_status->zoomLevel_min, camera_gimbal_status->zoomLevel_max, camera_gimbal_status->gimBal_pitch, camera_gimbal_status->gimBal_yaw, camera_gimbal_status->hfov_now, camera_gimbal_status->hfov_min, camera_gimbal_status->hfov_max, camera_gimbal_status->vfov_now, camera_gimbal_status->vfov_min, camera_gimbal_status->vfov_max, camera_gimbal_status->distance_detected, camera_gimbal_status->yaw_mode, camera_gimbal_status->track_mode, camera_gimbal_status->target_lat, camera_gimbal_status->target_lon, camera_gimbal_status->target_alt, camera_gimbal_status->flag);
}

/**
 * @brief Send a camera_gimbal_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param currentZoomLevel  zoomLevel of camera (-1 if unknown).
 * @param zoomLevel_min  minZoomLevel of camera (-1 if unknown).
 * @param zoomLevel_max  maxZoomLevel of camera (-1 if unknown).
 * @param gimBal_pitch [deg] pitch of gimBal (>360 if unknown).
 * @param gimBal_yaw [deg] yaw of gimBal (>360 if unknown).
 * @param hfov_now [deg] current horizontal field of view (>360 if unknown).
 * @param hfov_min [deg] minimal horizontal field of view (>360 if unknown).
 * @param hfov_max [deg] Maximum horizontal field of view (>360 if unknown).
 * @param vfov_now [deg] current vertical field of view (>360 if unknown).
 * @param vfov_min [deg] minimal vertical field of view (>360 if unknown).
 * @param vfov_max [deg] Maximum vertical field of view (>360 if unknown).
 * @param distance_detected [m] radar distance(-1 if unknown).
 * @param yaw_mode  yaw_mode, 0: follow, 1:lock, 2:look down (-1 if unknown).
 * @param track_mode  track_mode, 0: not active, 1:active(-1 if unknown).
 * @param target_lat [degE7] Latitude (WGS84, EGM96 ellipsoid)
 * @param target_lon [degE7] Longitude (WGS84, EGM96 ellipsoid)
 * @param target_alt [m] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
 * @param flag  0: dynamic fov value not available; 1:dynamic fov value available.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_gimbal_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, int8_t currentZoomLevel, int8_t zoomLevel_min, int8_t zoomLevel_max, float gimBal_pitch, float gimBal_yaw, float hfov_now, float hfov_min, float hfov_max, float vfov_now, float vfov_min, float vfov_max, float distance_detected, int8_t yaw_mode, int8_t track_mode, int32_t target_lat, int32_t target_lon, int32_t target_alt, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, gimBal_pitch);
    _mav_put_float(buf, 8, gimBal_yaw);
    _mav_put_float(buf, 12, hfov_now);
    _mav_put_float(buf, 16, hfov_min);
    _mav_put_float(buf, 20, hfov_max);
    _mav_put_float(buf, 24, vfov_now);
    _mav_put_float(buf, 28, vfov_min);
    _mav_put_float(buf, 32, vfov_max);
    _mav_put_float(buf, 36, distance_detected);
    _mav_put_int32_t(buf, 40, target_lat);
    _mav_put_int32_t(buf, 44, target_lon);
    _mav_put_int32_t(buf, 48, target_alt);
    _mav_put_int8_t(buf, 52, currentZoomLevel);
    _mav_put_int8_t(buf, 53, zoomLevel_min);
    _mav_put_int8_t(buf, 54, zoomLevel_max);
    _mav_put_int8_t(buf, 55, yaw_mode);
    _mav_put_int8_t(buf, 56, track_mode);
    _mav_put_uint8_t(buf, 57, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS, buf, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_CRC);
#else
    mavlink_camera_gimbal_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.gimBal_pitch = gimBal_pitch;
    packet.gimBal_yaw = gimBal_yaw;
    packet.hfov_now = hfov_now;
    packet.hfov_min = hfov_min;
    packet.hfov_max = hfov_max;
    packet.vfov_now = vfov_now;
    packet.vfov_min = vfov_min;
    packet.vfov_max = vfov_max;
    packet.distance_detected = distance_detected;
    packet.target_lat = target_lat;
    packet.target_lon = target_lon;
    packet.target_alt = target_alt;
    packet.currentZoomLevel = currentZoomLevel;
    packet.zoomLevel_min = zoomLevel_min;
    packet.zoomLevel_max = zoomLevel_max;
    packet.yaw_mode = yaw_mode;
    packet.track_mode = track_mode;
    packet.flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_CRC);
#endif
}

/**
 * @brief Send a camera_gimbal_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_gimbal_status_send_struct(mavlink_channel_t chan, const mavlink_camera_gimbal_status_t* camera_gimbal_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_gimbal_status_send(chan, camera_gimbal_status->time_boot_ms, camera_gimbal_status->currentZoomLevel, camera_gimbal_status->zoomLevel_min, camera_gimbal_status->zoomLevel_max, camera_gimbal_status->gimBal_pitch, camera_gimbal_status->gimBal_yaw, camera_gimbal_status->hfov_now, camera_gimbal_status->hfov_min, camera_gimbal_status->hfov_max, camera_gimbal_status->vfov_now, camera_gimbal_status->vfov_min, camera_gimbal_status->vfov_max, camera_gimbal_status->distance_detected, camera_gimbal_status->yaw_mode, camera_gimbal_status->track_mode, camera_gimbal_status->target_lat, camera_gimbal_status->target_lon, camera_gimbal_status->target_alt, camera_gimbal_status->flag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS, (const char *)camera_gimbal_status, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_gimbal_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int8_t currentZoomLevel, int8_t zoomLevel_min, int8_t zoomLevel_max, float gimBal_pitch, float gimBal_yaw, float hfov_now, float hfov_min, float hfov_max, float vfov_now, float vfov_min, float vfov_max, float distance_detected, int8_t yaw_mode, int8_t track_mode, int32_t target_lat, int32_t target_lon, int32_t target_alt, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, gimBal_pitch);
    _mav_put_float(buf, 8, gimBal_yaw);
    _mav_put_float(buf, 12, hfov_now);
    _mav_put_float(buf, 16, hfov_min);
    _mav_put_float(buf, 20, hfov_max);
    _mav_put_float(buf, 24, vfov_now);
    _mav_put_float(buf, 28, vfov_min);
    _mav_put_float(buf, 32, vfov_max);
    _mav_put_float(buf, 36, distance_detected);
    _mav_put_int32_t(buf, 40, target_lat);
    _mav_put_int32_t(buf, 44, target_lon);
    _mav_put_int32_t(buf, 48, target_alt);
    _mav_put_int8_t(buf, 52, currentZoomLevel);
    _mav_put_int8_t(buf, 53, zoomLevel_min);
    _mav_put_int8_t(buf, 54, zoomLevel_max);
    _mav_put_int8_t(buf, 55, yaw_mode);
    _mav_put_int8_t(buf, 56, track_mode);
    _mav_put_uint8_t(buf, 57, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS, buf, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_CRC);
#else
    mavlink_camera_gimbal_status_t *packet = (mavlink_camera_gimbal_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->gimBal_pitch = gimBal_pitch;
    packet->gimBal_yaw = gimBal_yaw;
    packet->hfov_now = hfov_now;
    packet->hfov_min = hfov_min;
    packet->hfov_max = hfov_max;
    packet->vfov_now = vfov_now;
    packet->vfov_min = vfov_min;
    packet->vfov_max = vfov_max;
    packet->distance_detected = distance_detected;
    packet->target_lat = target_lat;
    packet->target_lon = target_lon;
    packet->target_alt = target_alt;
    packet->currentZoomLevel = currentZoomLevel;
    packet->zoomLevel_min = zoomLevel_min;
    packet->zoomLevel_max = zoomLevel_max;
    packet->yaw_mode = yaw_mode;
    packet->track_mode = track_mode;
    packet->flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS, (const char *)packet, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_GIMBAL_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from camera_gimbal_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_camera_gimbal_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field currentZoomLevel from camera_gimbal_status message
 *
 * @return  zoomLevel of camera (-1 if unknown).
 */
static inline int8_t mavlink_msg_camera_gimbal_status_get_currentZoomLevel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  52);
}

/**
 * @brief Get field zoomLevel_min from camera_gimbal_status message
 *
 * @return  minZoomLevel of camera (-1 if unknown).
 */
static inline int8_t mavlink_msg_camera_gimbal_status_get_zoomLevel_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  53);
}

/**
 * @brief Get field zoomLevel_max from camera_gimbal_status message
 *
 * @return  maxZoomLevel of camera (-1 if unknown).
 */
static inline int8_t mavlink_msg_camera_gimbal_status_get_zoomLevel_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  54);
}

/**
 * @brief Get field gimBal_pitch from camera_gimbal_status message
 *
 * @return [deg] pitch of gimBal (>360 if unknown).
 */
static inline float mavlink_msg_camera_gimbal_status_get_gimBal_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field gimBal_yaw from camera_gimbal_status message
 *
 * @return [deg] yaw of gimBal (>360 if unknown).
 */
static inline float mavlink_msg_camera_gimbal_status_get_gimBal_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field hfov_now from camera_gimbal_status message
 *
 * @return [deg] current horizontal field of view (>360 if unknown).
 */
static inline float mavlink_msg_camera_gimbal_status_get_hfov_now(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field hfov_min from camera_gimbal_status message
 *
 * @return [deg] minimal horizontal field of view (>360 if unknown).
 */
static inline float mavlink_msg_camera_gimbal_status_get_hfov_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field hfov_max from camera_gimbal_status message
 *
 * @return [deg] Maximum horizontal field of view (>360 if unknown).
 */
static inline float mavlink_msg_camera_gimbal_status_get_hfov_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vfov_now from camera_gimbal_status message
 *
 * @return [deg] current vertical field of view (>360 if unknown).
 */
static inline float mavlink_msg_camera_gimbal_status_get_vfov_now(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vfov_min from camera_gimbal_status message
 *
 * @return [deg] minimal vertical field of view (>360 if unknown).
 */
static inline float mavlink_msg_camera_gimbal_status_get_vfov_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vfov_max from camera_gimbal_status message
 *
 * @return [deg] Maximum vertical field of view (>360 if unknown).
 */
static inline float mavlink_msg_camera_gimbal_status_get_vfov_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field distance_detected from camera_gimbal_status message
 *
 * @return [m] radar distance(-1 if unknown).
 */
static inline float mavlink_msg_camera_gimbal_status_get_distance_detected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field yaw_mode from camera_gimbal_status message
 *
 * @return  yaw_mode, 0: follow, 1:lock, 2:look down (-1 if unknown).
 */
static inline int8_t mavlink_msg_camera_gimbal_status_get_yaw_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  55);
}

/**
 * @brief Get field track_mode from camera_gimbal_status message
 *
 * @return  track_mode, 0: not active, 1:active(-1 if unknown).
 */
static inline int8_t mavlink_msg_camera_gimbal_status_get_track_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  56);
}

/**
 * @brief Get field target_lat from camera_gimbal_status message
 *
 * @return [degE7] Latitude (WGS84, EGM96 ellipsoid)
 */
static inline int32_t mavlink_msg_camera_gimbal_status_get_target_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  40);
}

/**
 * @brief Get field target_lon from camera_gimbal_status message
 *
 * @return [degE7] Longitude (WGS84, EGM96 ellipsoid)
 */
static inline int32_t mavlink_msg_camera_gimbal_status_get_target_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  44);
}

/**
 * @brief Get field target_alt from camera_gimbal_status message
 *
 * @return [m] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
 */
static inline int32_t mavlink_msg_camera_gimbal_status_get_target_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  48);
}

/**
 * @brief Get field flag from camera_gimbal_status message
 *
 * @return  0: dynamic fov value not available; 1:dynamic fov value available.
 */
static inline uint8_t mavlink_msg_camera_gimbal_status_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  57);
}

/**
 * @brief Decode a camera_gimbal_status message into a struct
 *
 * @param msg The message to decode
 * @param camera_gimbal_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_gimbal_status_decode(const mavlink_message_t* msg, mavlink_camera_gimbal_status_t* camera_gimbal_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_gimbal_status->time_boot_ms = mavlink_msg_camera_gimbal_status_get_time_boot_ms(msg);
    camera_gimbal_status->gimBal_pitch = mavlink_msg_camera_gimbal_status_get_gimBal_pitch(msg);
    camera_gimbal_status->gimBal_yaw = mavlink_msg_camera_gimbal_status_get_gimBal_yaw(msg);
    camera_gimbal_status->hfov_now = mavlink_msg_camera_gimbal_status_get_hfov_now(msg);
    camera_gimbal_status->hfov_min = mavlink_msg_camera_gimbal_status_get_hfov_min(msg);
    camera_gimbal_status->hfov_max = mavlink_msg_camera_gimbal_status_get_hfov_max(msg);
    camera_gimbal_status->vfov_now = mavlink_msg_camera_gimbal_status_get_vfov_now(msg);
    camera_gimbal_status->vfov_min = mavlink_msg_camera_gimbal_status_get_vfov_min(msg);
    camera_gimbal_status->vfov_max = mavlink_msg_camera_gimbal_status_get_vfov_max(msg);
    camera_gimbal_status->distance_detected = mavlink_msg_camera_gimbal_status_get_distance_detected(msg);
    camera_gimbal_status->target_lat = mavlink_msg_camera_gimbal_status_get_target_lat(msg);
    camera_gimbal_status->target_lon = mavlink_msg_camera_gimbal_status_get_target_lon(msg);
    camera_gimbal_status->target_alt = mavlink_msg_camera_gimbal_status_get_target_alt(msg);
    camera_gimbal_status->currentZoomLevel = mavlink_msg_camera_gimbal_status_get_currentZoomLevel(msg);
    camera_gimbal_status->zoomLevel_min = mavlink_msg_camera_gimbal_status_get_zoomLevel_min(msg);
    camera_gimbal_status->zoomLevel_max = mavlink_msg_camera_gimbal_status_get_zoomLevel_max(msg);
    camera_gimbal_status->yaw_mode = mavlink_msg_camera_gimbal_status_get_yaw_mode(msg);
    camera_gimbal_status->track_mode = mavlink_msg_camera_gimbal_status_get_track_mode(msg);
    camera_gimbal_status->flag = mavlink_msg_camera_gimbal_status_get_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN;
        memset(camera_gimbal_status, 0, MAVLINK_MSG_ID_CAMERA_GIMBAL_STATUS_LEN);
    memcpy(camera_gimbal_status, _MAV_PAYLOAD(msg), len);
#endif
}
