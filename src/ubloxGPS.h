/*
 * Copyright (c) 2020 Particle Industries, Inc. 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __UBLOXGPS_H
#define __UBLOXGPS_H

#include "Particle.h"
#include "gps/gps.h" // the nmea parser

const uint16_t UBX_RX_MSG_MAX_LEN = 512;
const uint16_t UBX_LOG_STRING_MAX_LEN = 256;
const size_t UBX_MGA_FLASH_DATA_MAX_LEN = 512;

typedef enum {
    UBX_CLASS_NAV      = 0x01,  // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
    UBX_CLASS_RXM      = 0x02,  // Receiver Manager Messages: Satellite Status, RTC Status
    UBX_CLASS_INF      = 0x04,  // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
    UBX_CLASS_ACK      = 0x05,  // Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages
    UBX_CLASS_CFG      = 0x06,  // Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
    UBX_CLASS_UPD      = 0x09,  // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
    UBX_CLASS_MON      = 0x0A,  // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
    UBX_CLASS_AID      = 0x0B,  // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
    UBX_CLASS_TIM      = 0x0D,  // Timing Messages: Time Pulse Output, Time Mark Results
    UBX_CLASS_ESF      = 0x10,  // External Sensor Fusion Messages: External Sensor Measurements and Status Information
    UBX_CLASS_MGA      = 0x13,  // Multiple GNSS Assistance Messages: Assistance data for various GNSS
    UBX_CLASS_LOG      = 0x21,  // Logging Messages: Log creation, deletion, info and retrieval
    UBX_CLASS_SEC      = 0x27,  // Security Feature Messages
    UBX_CLASS_HNR      = 0x28,  // High Rate Navigation Results Messages: High rate time, position, speed, heading
    UBX_CLASS_NMEA     = 0xF0,  // NMEA standard messages
    UBX_CLASS_PUBX     = 0xF1,  // Proprietary Messages: i.e. Messages defined by u-blox
    UBX_CLASS_INVALID  = 0xFF
} ubx_msg_class_t;

typedef enum {
    UBX_ACK_ACK              = 0x01, // Message Acknowledged
    UBX_ACK_NAK              = 0x00, // Message Not-Acknowledged
    UBX_CFG_ANT              = 0x13, // Antenna Control Settings
    UBX_CFG_CFG              = 0x09, // Clear, Save and Load configurations
    UBX_CFG_DAT              = 0x06, // The currently defined Datum
    UBX_CFG_DOSC             = 0x61, // Disciplined oscillator configuration
    UBX_CFG_ESFALG           = 0x56, // IMU-mount misalignment configuration
    UBX_CFG_DYNSEED          = 0x85, // Programming the dynamic seed for the host
    UBX_CFG_ESRC             = 0x60, // External synchronization source configuration
    UBX_CFG_FIXSEED          = 0x84, // Programming the fixed seed for host
    UBX_CFG_GEOFENCE         = 0x69, // Geofencing configuration
    UBX_CFG_GNSS             = 0x3E, // GNSS system configuration
    UBX_CFG_HNR              = 0x5C, // High Navigation Rate Settings
    UBX_CFG_ITFM             = 0x39, // Jamming/Interference Monitor configuration
    UBX_CFG_INF              = 0x02, // Information message configuration
    UBX_CFG_LOGFILTER        = 0x47, // Data Logger Configuration
    UBX_CFG_MSG              = 0x01, // Set Message Rate
    UBX_CFG_NAV5             = 0x24, // Navigation Engine Settings
    UBX_CFG_NAVX5            = 0x23, // Navigation Engine Expert Settings
    UBX_CFG_NMEA             = 0x17, // NMEA protocol configuration
    UBX_CFG_ODO              = 0x1E, // Odometer, Low-speed COG Engine Settings
    UBX_CFG_PM2              = 0x3B, // Extended Power Management configuration
    UBX_CFG_PMS              = 0x86, // Power Mode Setup
    UBX_CFG_PRT              = 0x00, // Port Configuration
    UBX_CFG_PWR              = 0x57, // Put receiver in a defined power state
    UBX_CFG_RATE             = 0x08, // Navigation/Measurement Rate Settings
    UBX_CFG_RINV             = 0x34, // Contents of Remote Inventory
    UBX_CFG_RST              = 0x04, // Reset Receiver / Clear Backup Data Structures
    UBX_CFG_RXM              = 0x11, // RXM configuration
    UBX_CFG_SBAS             = 0x16, // SBAS Configuration
    UBX_CFG_SMGR             = 0x62, // Synchronization manager configuration
    UBX_CFG_TMODE2           = 0x3D, // Time Mode Settings 2
    UBX_CFG_TP5              = 0x31, // Time Pulse Parameters
    UBX_CFG_TXSLOT           = 0x53, // TX buffer time slots configuration
    UBX_CFG_USB              = 0x1B, // USB Configuration
    UBX_ESF_ALG              = 0x14, // IMU alignment information
    UBX_ESF_INS              = 0x15, // Vehicle dynamics information
    UBX_ESF_MEAS             = 0x02, // External Sensor Fusion Measurements
    UBX_ESF_RAW              = 0x03, // Raw sensor measurements
    UBX_ESF_STATUS           = 0x10, // External Sensor Fusion status information
    UBX_LOG_ERASE            = 0x03, // Erase Logged Data
    UBX_LOG_STRING           = 0x04, // Store arbitrary string in on-board flash
    UBX_LOG_CREATE           = 0x07, // Create Log File
    UBX_LOG_INFO             = 0x08, // Log information
    UBX_LOG_RETRIEVE         = 0x09, // Request log data
    UBX_LOG_RETRIEVEPOS      = 0x0B, // Position fix log entry
    UBX_LOG_RETRIEVESTRING   = 0x0D, // Byte string log entry
    UBX_LOG_FINDTIME         = 0x0E, // Find index of a log entry based on a given time
    UBX_LOG_RETRIEVEPOSEXTRA = 0x0F, // Odometer log entry
    UBX_MGA_FLASH_DATA       = 0x21, // Transfer MGA-ANO data block to flash
    UBX_MON_VER              = 0x04, // Receiver/Software Version
    UBX_NAV_AOPSTATUS        = 0x60, // AssistNow Autonomous Status
    UBX_NAV_ATT              = 0x05, // Attitude Solution
    UBX_NAV_CLOCK            = 0x22, // Clock Solution
    UBX_NAV_DGPS             = 0x31, // DGPS Data Used for NAV
    UBX_NAV_DOP              = 0x04, // Dilution of Precision
    UBX_NAV_EOE              = 0x62, // End Of Epoch
    UBX_NAV_GEOFENCE         = 0x39, // Geofencing status
    UBX_NAV_ODO              = 0x09, // Odometer Solution
    UBX_NAV_ORB              = 0x34, // GNSS Orbit Database Info
    UBX_NAV_POSECEF          = 0x01, // Position Solution in ECEF
    UBX_NAV_POSLLH           = 0x02, // Geodetic Position Solution
    UBX_NAV_PVT              = 0x07, // Navigation Position Velocity Time Solution
    UBX_NAV_RESETODO         = 0x10, // Reset Odometer
    UBX_NAV_SAT              = 0x35, // Satellite Information
    UBX_NAV_SBAS             = 0x32, // SBAS Status Data
    UBX_NAV_SOL              = 0x06, // Navigation Solution Information
    UBX_NAV_STATUS           = 0x03, // Receiver Navigation Status
    UBX_NAV_SVINFO           = 0x30, // Space Vehicle Information
    UBX_NAV_TIMEBDS          = 0x24, // BDS Time Solution
    UBX_NAV_TIMEGAL          = 0x25, // Galileo Time Solution
    UBX_NAV_TIMEGLO          = 0x23, // GLO Time Solution
    UBX_NAV_TIMEGPS          = 0x20, // GPS Time Solution
    UBX_NAV_TIMELS           = 0x26, // Leap second event information
    UBX_NAV_VELECEF          = 0x11, // Velocity Solution in ECEF
    UBX_NAV_TIMEUTC          = 0x21, // UTC Time Solution
    UBX_NAV_VELNED           = 0x12, // Velocity Solution in NED    
    UBX_PUBX_CONFIG          = 0x41, // Set protocols and baudrate
    UBX_PUBX_POSITION        = 0x00, // Lat/Long Position Data
    UBX_PUBX_RATE            = 0x40, // Set NMEA message output rate
    UBX_PUBX_SVSTATUS        = 0x03, // Satellite Status
    UBX_PUBX_TIME            = 0x04, // Time of Day and Clock information
    UBX_ID_INVALID           = 0xFF
} ubx_msg_id_t;

typedef enum { // NMEA message types
    UBX_NEMA_DTM                   = 0x0a, // Datum reference
    UBX_NEMA_GBS                   = 0x09, // GNSS Satellite Fault Detection
    UBX_NEMA_GGA                   = 0x00, // Global positioning system fix data
    UBX_NEMA_GLL                   = 0x01, // Latitude and longitude, with time of position fix and status
    UBX_NEMA_GSA                   = 0x02, // GNSS DOP and Active Satellites
    UBX_NEMA_GST                   = 0x07, // GNSS Pseudo Range Error Statistics
    UBX_NEMA_GSV                   = 0x03, // GNSS Satellites in View
    UBX_NEMA_RMC                   = 0x04, // Recommended Minimum data
    UBX_NEMA_VTG                   = 0x05, // Course over ground and Ground speed
    UBX_NEMA_ZDA                   = 0x08  // Time and Date
} ubx_nmea_msg_t;

typedef enum { // message types for UBX_MGA_FLASH_DATA
    UBX_MGA_FLASH_DATA_WRITE       = 0x01,
    UBX_MGA_FLASH_DATA_STOP        = 0x02,
    UBX_MGA_FLASH_DATA_ACK         = 0x03,
} ubx_mga_flash_msg_type_t;

typedef enum { // ack types for UBX_MGA_FLASH_DATA_ACK
    UBX_MGA_FLASH_DATA_ACK_OK      = 0x00,
    UBX_MGA_FLASH_DATA_ACK_RETRY   = 0x01,
    UBX_MGA_FLASH_DATA_ACK_ABORT   = 0x02,
    // internal errors
    UBX_MGA_FLASH_DATA_ACK_TIMEOUT = -1,
    UBX_MGA_FLASH_DATA_ACK_INVALID = -2,
} ubx_mga_flash_ack_type_t;

typedef enum { // antenna type
    UBX_INTERNAL_ANT = 0x00,
    UBX_EXTERNAL_ANT = 0x01
} ubx_antenna_t;

typedef enum {
    UBX_GNSSID_GPS      = 0x00,
    UBX_GNSSID_SBAS     = 0x01,
    UBX_GNSSID_Galileo  = 0x02,
    UBX_GNSSID_BeiDou   = 0x03,
    UBX_GNSSID_IMES     = 0x04,
    UBX_GNSSID_QZSS     = 0x05,
    UBX_GNSSID_GLONASS  = 0x06
} ubx_gnssid_t;

typedef enum {
    UBX_GNSS_TYPE_GPS      = 0x01,
    UBX_GNSS_TYPE_SBAS     = 0x02,
    UBX_GNSS_TYPE_Galileo  = 0x04,
    UBX_GNSS_TYPE_BeiDou   = 0x08,
    UBX_GNSS_TYPE_IMES     = 0x10,
    UBX_GNSS_TYPE_QZSS     = 0x20,
    UBX_GNSS_TYPE_GLONASS  = 0x40
} ubx_gnss_type_t;

typedef enum {
    UBX_GPS_UNIT_DEGREE = 0x00,
    UBX_GPS_UNIT_DMS    = 0x01
} ubx_gps_unit_t;


typedef enum  {
    DECODE_RESULT_INVALID,
    DECODE_RESULT_OK,
    DECODE_RESULT_END,
    DECODE_RESULT_IS_ACK_NAK_RSP
} decode_result_t;

typedef enum {
    UBX_POWER_MODE_FULL_POWER = 0,
    UBX_POWER_MODE_BALANCED,
    UBX_POWER_MODE_INTERVAL,
    UBX_POWER_MODE_AGGRESSIVE_WITH_1_HZ,
    UBX_POWER_MODE_AGGRESSIVE_WITH_2_HZ,
    UBX_POWER_MODE_AGGRESSIVE_WITH_4_HZ,
    UBX_POWER_MODE_INVALID = 0xFF,
} ubx_power_mode_t;

typedef enum {
    UBX_DYNAMIC_MODEL_PORTABLE     = 0,
    UBX_DYNAMIC_MODEL_STATIONARY   = 2,
    UBX_DYNAMIC_MODEL_PEDESTRIAN   = 3,
    UBX_DYNAMIC_MODEL_AUTOMOTIVE   = 4,
    UBX_DYNAMIC_MODEL_AT_SEA       = 5,
    UBX_DYNAMIC_MODEL_AIRBORNE_1G  = 6,
    UBX_DYNAMIC_MODEL_AIRBORNE_2G  = 7,
    UBX_DYNAMIC_MODEL_AIRBORNE_4G  = 8,
    UBX_DYNAMIC_MODEL_WRIST        = 9
} ubx_dynamic_model_t;

typedef enum {
    UBX_BAUDRATE_DEFAULT = 9600,
    UBX_BAUDRATE_4800    = 4800,
    UBX_BAUDRATE_9600    = 9600,
    UBX_BAUDRATE_19200   = 19200,
    UBX_BAUDRATE_38400   = 38400,
    UBX_BAUDRATE_57600   = 57600,
    UBX_BAUDRATE_115200  = 115200,
    UBX_BAUDRATE_230400  = 230400,
    UBX_BAUDRATE_460800  = 460800
} ubx_baudrate_t;

typedef enum {
    GPS_STATUS_OFF,
    GPS_STATUS_FIXING,
    GPS_STATUS_LOCK
} gps_led_status_t;

typedef enum {
    GPS_SPEED_UNIT_MPS = 0, // m/s
    GPS_SPEED_UNIT_MPH,     // m/h
    GPS_SPEED_UNIT_KMPH     // km/h
} gps_speed_unit_t;

enum {
	HEADER_CLASS_OFFSET = 0,
	HEADER_MSG_ID_OFFSET, // 1
	HEADER_LENGTH_LSB_OFFSET, // 2
	HEADER_LENGTH_MSB_OFFSET // 3
};

struct ubx_geofence_t {
    uint8_t  numFences   = 0;   // limit to 4, must be <=4
    uint8_t  confLvl     = 0;   // 0=no confidence required, 1=68%, 2=95%, 3=99.7% etc.
    uint8_t  pioEnabled  = 0;   // 1 = Enable PIO combined fence state output, 0 = disable
    uint8_t  pinPolarity = 0;   // PIO pin polarity. 0 = Low means inside, 1 = Low means outside. Unknown state is always high
    uint8_t  pinNumber   = 0;   // PIO pin number
    int32_t  lat[4]      = {0}; // deg, scaling 1e-7, Latitude of the geofence circle center
    int32_t  lon[4]      = {0}; // deg, scaling 1e-7, Longitude of the geofence circle center
    uint32_t rad[4]      = {0}; // m,   scaling 1e-2, Radius of the geofence circle

    void init()
    {
        numFences   = 0;
        confLvl     = 0;
        pioEnabled  = 0;
        pinPolarity = 0;
        pinNumber   = 0;
        lat[4]      = {0};
        lon[4]      = {0};
        rad[4]      = {0};
    }

    bool addGeofence(float latitude, float longitude, float radius)
    {
        if (numFences < 4) {
            lat[numFences] = (int32_t)(latitude * 1e7);
            lon[numFences] = (int32_t)(longitude * 1e7);
            rad[numFences] = (uint32_t)(radius * 1e2);
            numFences++;
            return true;
        } else {
            return false;
        }
    }

    void enablePIO(uint8_t pin, uint8_t polarity)
    {
        pioEnabled = 1;
        pinPolarity = polarity;
        pinNumber = pin;
    }

    void disablePIO()
    {
        pioEnabled = 0;
        pinPolarity = 0;
        pinNumber = 0;
    }

} __attribute__((packed)) ;

struct ubx_msg_header_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} __attribute__((packed));

// Includes fixed storage, intended for internal use when accumulating a new
// RX frame. crc_a and crc_b are updated as the message is received so there
// is no function to calculate it. 
struct ubx_rx_msg_t {
    uint8_t         msg_class;
    uint8_t         msg_id;
    uint16_t        length;
    uint8_t         ubx_msg[UBX_RX_MSG_MAX_LEN];
    uint8_t         crc_a;
    uint8_t         crc_b;

    ubx_rx_msg_t()
    {
        init();
    }

    void init()
    {
        msg_class = UBX_CLASS_INVALID;
        msg_id = UBX_ID_INVALID;
        length = 0;
        crc_a = crc_b = 0;
    }

    
} __attribute__((packed)) ;

struct ubx_msg_t {
    uint8_t         msg_class;
    uint8_t         msg_id;
    uint16_t        length;
    uint8_t         payload[];
} __attribute__((packed));

struct ubx_esf_alg_t {
    uint32_t iTow;
    uint8_t version;
    struct
    {
        uint8_t autoMntAlgOn : 1;
        uint8_t status : 3;
        uint8_t reserved : 4;
    }flags;
    struct
    {
        uint8_t titleAlg : 1;
        uint8_t yawAlg : 1;
        uint8_t angleAlg : 1;
        uint8_t reserved : 5;
    }error;    
    uint8_t reserved1;
    uint32_t yaw;
    int16_t pitch;
    int16_t roll;
};

struct ubx_esf_status_t {
    uint8_t fusionMode;
    uint8_t numSens;
    uint8_t sensStatus1[12];
    uint8_t sensStatus2[12];
    uint8_t freq[12];
    uint8_t faults[12];
    bool    valid;
};

struct  ubx_nav_odo_t {
    uint32_t iTOW;          // ms, GPS time of week of the navigation epoch.
    uint32_t distance;      // meter, Ground distance since last reset
    uint32_t totalDistance; // meter, Total cumulative ground distance
    uint32_t distanceStd;   // meter, Ground distance accuracy (1-sigma)
    bool     valid;
};

struct ubx_ack_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t req_class;
    uint8_t req_id;
} __attribute__((packed));

struct ubx_nak_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t req_class;
    uint8_t req_id;
} __attribute__((packed));

struct ubx_log_create_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t version;
        #define UBX_LOG_CREATE_CONFIG_CIRCULAR_BIT (0x01)
    uint8_t config;
    uint8_t reserved;
        #define UBX_LOG_CREATE_SIZE_MAX (0x00)
        #define UBX_LOG_CREATE_SIZE_MIN (0x01)
        #define UBX_LOG_CREATE_SIZE_USER (0x02)
    uint8_t size;
    uint32_t user_size;
} __attribute__((packed));

struct ubx_cfg_port_tx_ready_t {
    uint16_t enable : 1 ;
        #define UBX_CFG_PRT_TX_READY_ACTIVE_HIGH (0)
        #define UBX_CFG_PRT_TX_READY_ACTIVE_LOW (1)
    uint16_t polarity : 1;
    uint16_t pin : 5;
    uint16_t threshold : 9;
};

union ubx_cfg_port_mode_t {
    struct {
        uint32_t reserved_1 : 1;
            #define UBX_CFG_PRT_MODE_SPI_MODE_0 (0x00)
            #define UBX_CFG_PRT_MODE_SPI_MODE_1 (0x01)
            #define UBX_CFG_PRT_MODE_SPI_MODE_2 (0x02)
            #define UBX_CFG_PRT_MODE_SPI_MODE_3 (0x03)
        uint32_t mode : 2;
        uint32_t reserved_2 : 5;
        uint32_t ff_count : 6;
    } spi;
};


struct ubx_cfg_port_t {
    ubx_msg_header_t header;
        #define UBX_CFG_PRT_USB (3)
        #define UBX_CFG_PRT_SPI (4)
    uint8_t port;
    uint8_t reserved_1;
    ubx_cfg_port_tx_ready_t tx_ready;
    ubx_cfg_port_mode_t mode;
    uint32_t reserved_2;
        #define UBX_CFG_PRT_PROTO_UBX (0x0001)
        #define UBX_CFG_PRT_PROTO_NMEA (0x0002)
        #define UBX_CFG_PRT_PROTO_RTCM (0x0004)
        #define UBX_CFG_PRT_PROTO_RTCM3 (0x0020)
    uint16_t in_proto_mask;
    uint16_t out_proto_mask;
        #define UBX_CFG_PRT_FLAGS_EXTENDED_TIMEOUT (0x0002)
    uint16_t flags;
    uint16_t reserved_3;
} __attribute__((packed));

struct ubx_cfg_log_filter_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t version;
        #define UBX_CFG_LOG_FILTER_FLAGS_RECORD_BIT (0x01)
        #define UBX_CFG_LOG_FILTER_FLAGS_ONCE_PER_WAKE_BIT (0x02)
        #define UBX_CFG_LOG_FILTER_FLAGS_APPLY_ALL_BIT (0x04)
    uint8_t flags;
    uint16_t min_interval; // seconds
    uint16_t time_threshold; // seconds
    uint16_t speed_threshold; // meters per second
    uint32_t position_threshold; // meters
} __attribute__((packed));

struct ubx_log_retrieve_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint32_t start_number;
        #define UBX_LOG_RETRIEVE_ENTRY_COUNT_MAX (256)
    uint32_t entry_count; // uint32_t but max is 256
    uint8_t version;
    uint8_t reserved[3];
} __attribute__((packed));

struct ubx_log_erase_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} __attribute__((packed));

struct ubx_log_pos_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint32_t entry_index;
    int32_t lon; // mult 1e-7 to decimal
    int32_t lat; // mult 1e-7 to decimal
    int32_t alt; // mm above mean sea level
    uint32_t horizontal_accuracy; // mm
    uint32_t ground_speed; // mm/s
    uint32_t heading; // deg, mult 1e-5
    uint8_t version;
        #define UBX_LOG_POS_FIX_DEAD_RECKON (0x01)
        #define UBX_LOG_POS_FIX_2D (0x02)
        #define UBX_LOG_POS_FIX_3D (0x03)
        #define UBX_LOG_POS_FIX_GNNS_DEAD (0x04)
    uint8_t fix;
    uint16_t year;
    uint8_t month; // 1-12
    uint8_t day; // 1-31
    uint8_t hour; // 0-23
    uint8_t minute; // 0-59
    uint8_t second; // 0-60???
    uint8_t reserved1;
    uint8_t num_satellites;
    uint8_t reserved2;
} __attribute__((packed));

struct ubx_log_rx_string_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint32_t entry_index;
    uint8_t version;
    uint8_t reserved1;
    uint16_t year;
    uint8_t month; // 1-12
    uint8_t day; // 1-31
    uint8_t hour; // 0-23
    uint8_t minute; // 0-59
    uint8_t second; // 0-60???
    uint8_t reserved2;
    uint16_t byte_count;
    uint8_t bytes[];
} __attribute__((packed));

struct ubx_log_info_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} __attribute__((packed));

struct ubx_log_info_rsp_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t version;
    uint8_t reserved1[3];
    uint32_t capacity;
    uint8_t reserved2[8];
    uint32_t current_max_size;
    uint32_t current_size;
    uint32_t entry_count;
    uint16_t oldest_year;
    uint8_t oldest_month; // 1-12
    uint8_t oldest_day; // 1-31
    uint8_t oldest_hour; // 0-23
    uint8_t oldest_minute; // 0-59
    uint8_t oldest_second; // 0-60???
    uint8_t reserved3;
    uint16_t newest_year;
    uint8_t newest_month; // 1-12
    uint8_t newest_day; // 1-31
    uint8_t newest_hour; // 0-23
    uint8_t newest_minute; // 0-59
    uint8_t newest_second; // 0-60???
    uint8_t reserved4;
        #define UBX_LOG_INFO_STATUS_RECORDING_BIT (0x08)
        #define UBX_LOG_INFO_STATUS_INACTIVE_BIT (0x10)
        #define UBX_LOG_INFO_STATUS_CIRCULAR_BIT (0x20)
    uint8_t status;
    uint8_t reserved5[3];

    bool enabled() { return !(status & UBX_LOG_INFO_STATUS_INACTIVE_BIT); }
    bool recording() { return (status & UBX_LOG_INFO_STATUS_RECORDING_BIT); }
    bool circular() { return (status & UBX_LOG_INFO_STATUS_CIRCULAR_BIT); }
} __attribute__((packed));

struct ubx_log_string_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t bytes[];
} __attribute((packed));

struct ubx_mga_flash_write_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t msg_type;
    uint8_t version;
    uint16_t sequence;
    uint16_t size;
    uint8_t data[]; // size bytes
} __attribute((packed));

struct ubx_mga_flash_stop_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t msg_type;
    uint8_t version;
} __attribute((packed));

struct ubx_mga_flash_ack_t {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t msg_type;
    uint8_t version;
    uint8_t ack; // ubx_mga_flash_ack_type_t
    uint8_t reserved1;
    uint16_t sequence; // of the message for this ACK or 0xFFFF for UBX-MGA-FLASH-STOP
} __attribute((packed));

struct ubx_mon_ver_t {
    bool    valid;
    uint8_t *sw_version;
    uint8_t *hw_version;
    uint8_t *extension;
};

class ubloxGPS
{

public:
    ubloxGPS(USARTSerial &serial,
        std::function<bool(bool)> pwr_enable);
    ubloxGPS(SPIClass &spi,
        std::function<bool(bool)> spi_select,
        std::function<bool(bool)> pwr_enable,
        int tx_ready_mcu_pin = PIN_INVALID,
        int tx_ready_gps_pin = PIN_INVALID);
    gps_t nmea_gps;

    // Acquires the gps lock, useful for multiple gps operations at a time
    void lock();
    // Releases the gps lock
    void unlock();
    // interrupt handler for tx ready signal from gps (if enabled)
    void txReadyHandler();

    void on(void);
    void off(void);
    bool is_active(void);

    void enable_log(bool en)
    {
        log_enabled = en;
    }

    void setOutputNMEA(void);
    void setOutputPUBX(void);
    uint8_t  getGpsStatus();

    void setCal(void);
    void setUnit(ubx_gps_unit_t unit);

    double   getLatitude(void);
    double   getLongitude(void);
    float    getAltitude(void);
    uint8_t  getFixQuality(void);
    bool     getLock(void);
    uint32_t getLockTime(void);
    unsigned int getLockDuration(void);
    bool     isLockStable();
    float    getSpeed(uint8_t unit);
    float    getHeading(void);
    uint32_t getDate(void);
    uint32_t getTime(void);
    uint32_t getUTCTime();
    uint8_t  getSatellites(void);
    int8_t   getGeoIdHeight(void);
    uint8_t  getHDOP(void);
    uint8_t  getSignalStrength(void);
    double   getHorizontalAccuracy(void);
    double   getVerticalAccuracy(void);
    float    getDistance(double lat1, double long1, double lat2, double long2);

    /**
     * set new baudrate for ubloxGPS UART port
     *
     * NOTE: please pay attention that set baudrate can't get ACK/NAK
     *       so it's better to test the NMEA output by log print after
     *       you change baudrate, make sure config is successful.
     */
    bool  setBaudrate(ubx_baudrate_t baudrate);
    bool  setAntanna(ubx_antenna_t ant);
    bool  setRate(uint16_t measRateHz);
    bool  updateEsfStatus(void);
    bool  updateEsfAlg(void);
    bool  getEsfStatus(ubx_esf_status_t &esf);
    bool  setReset(void);
    bool  resetOdometer(void);
    bool  updateOdometer(void);
    bool  getOdometer(ubx_nav_odo_t &odo);
    bool  updateVersion(void);
    bool  getVersion(String& swVersion, String& hwVersion, String& extVersion);
    bool  setGNSS(uint8_t gnssMask);
    bool  setMode(ubx_dynamic_model_t dynModel);
    bool  setPower(ubx_power_mode_t power_mode);
    bool  setPower(ubx_power_mode_t power_mode, uint16_t period, uint16_t onTime);
    bool  setGeofence(ubx_geofence_t geofence);
    bool  configMsg(ubx_msg_class_t msgClass, uint8_t msgId, uint8_t rate);
    bool  disableNMEA(void);
    bool  enableNMEA(uint8_t intervalSec, uint8_t slowIntervalSec);
    bool  disableUBX(void);
    bool  enablePUBX(uint8_t intervalSec, uint8_t slowIntervalSec);
    bool  disablePUBX(void);

    bool  set_auto_imu_alignment(bool enable);
    bool  is_auto_imu_alignment_ready(void);

    bool  createLog(void);
    bool  eraseLog(void);
    bool  configLog(uint16_t min_interval, uint16_t time_threshold, uint16_t speed_threshold, uint32_t position_threshold, bool start);
    bool  startLog(void);
    bool  pauseLog(void);
    bool  getLogInfo(ubx_log_info_rsp_t *info);
    bool  addLogString(uint8_t *bytes, uint16_t length);
    const ubx_msg_t  *getLogEntry(uint32_t start);

    bool startWriteMGA();
    bool stopWriteMGA();
    void abortWriteMGA();
    ubx_mga_flash_ack_type_t writeMGA(uint8_t *bytes, uint16_t length);

    String deg2DMS(float degree);
    float  DMS2deg(String DMS);


    void enableDebugNMEA(bool en);
    void hex_dump(LogLevel level, uint8_t *data, int len, Logger *logger=NULL);
protected:
    bool checkWaitingForAckOrRspFlags() const;
    
private:
    SPIClass *spi = NULL;
    __SPISettings spi_settings;
    USARTSerial *serial = NULL;
    std::function<bool(bool)> spi_select = NULL;
    std::function<bool(bool)> pwr_enable = NULL;
    int tx_ready_mcu_pin = PIN_INVALID;
    int tx_ready_gps_pin = PIN_INVALID;
    os_queue_t tx_ready_queue = NULL; // to signal tx ready updates from interrupt
    bool log_enabled = true;
    bool debugNMEA;
    Thread *gpsThread;
    uint32_t lastLockTime;
    ubx_rx_msg_t  ubx_rx_msg;

    bool write_mga_active = false;
    uint16_t write_mga_sequence = 0;
    
    uint8_t gpsStatus;
    uint8_t gpsUnit;
    uint32_t last_receive_time = 0;
    ubx_esf_status_t esf_status = {0};
    ubx_esf_alg_t    alg_info = {0};
    ubx_nav_odo_t    nav_odo = {0};
    ubx_mon_ver_t    mon_ver = {0};

    bool  ackExpected;
    bool  rspExpected;
    bool  ackReceived;
    bool  nakReceived;
    bool  rspReceived;
    bool  rxTimeout;

    // UBX class/id of the current request waiting for response
    // used as extra validation for ACK/NACK detection
    uint8_t waitForReqClass;
    uint8_t waitForReqId;

    // UBX class/id of response expected for current request (if applicable)
    uint8_t waitForRspClass;
    uint8_t waitForRspId;

    typedef struct {
        bool     output_pubx = true;        // true - output PUBX,  false - output NMEA
        uint8_t  power_enable = HIGH;       // pin state when GPS power on 
        uint8_t  power_mode = UBX_POWER_MODE_FULL_POWER;
        uint8_t  dynamic_model = UBX_DYNAMIC_MODEL_AUTOMOTIVE;
        uint8_t  support_gnss = UBX_GNSS_TYPE_GPS | UBX_GNSS_TYPE_BeiDou | UBX_GNSS_TYPE_GLONASS;
        uint32_t baudrate = UBX_BAUDRATE_115200; 
        uint32_t fastIntervalSec = 1;       // fast interval for output position message
        uint32_t slowIntervalSec = 30;      // slow interval for output satellite message

        void resetDefault(void){
            output_pubx = true;
            power_enable = HIGH;
            power_mode = UBX_POWER_MODE_FULL_POWER;
            dynamic_model = UBX_DYNAMIC_MODEL_AUTOMOTIVE;
            support_gnss = UBX_GNSS_TYPE_GPS | UBX_GNSS_TYPE_BeiDou | UBX_GNSS_TYPE_GLONASS;
            baudrate = UBX_BAUDRATE_115200; 
            fastIntervalSec = 1;
            slowIntervalSec = 30;
        }
    } lib_config_t;
    lib_config_t lib_config;

    void setOn(lib_config_t &config);
    void updateGPS(void);
    void processLockStability();
    void processGPSByte(uint8_t c);
#define UBX_REQ_FLAGS_EXPECT_ACK 0x01
    bool requestSendUBX(const uint8_t *sentences, uint16_t len);
    bool requestSendUBX(const ubx_msg_t *request,
        uint16_t len,
        uint8_t flags,
        uint8_t req_class=UBX_CLASS_INVALID,
        uint8_t req_id=UBX_ID_INVALID);
    size_t writeBytes(const uint8_t *buf, size_t len);
    bool sendUBX(const uint8_t *sentences, uint16_t len);
    void processUBX();
    void processBytes();
    bool yieldThread(uint32_t timeout);
    void waitForAckOrRsp();
    const ubx_msg_t *waitForAck(uint8_t req_class=UBX_CLASS_INVALID,
        uint8_t req_id=UBX_ID_INVALID);
    const ubx_msg_t *waitForResponse(uint8_t rsp_class=UBX_CLASS_INVALID,
        uint8_t rsp_id=UBX_ID_INVALID,
        uint8_t req_class=UBX_CLASS_INVALID,
        uint8_t req_id=UBX_ID_INVALID);
    const ubx_msg_t *getResponse();
    bool isACK(const ubx_msg_t *rsp);
    bool isNAK(const ubx_msg_t *rsp);
    void initRxMsg();
    bool parseRxMsg();
    std::function<decode_result_t(ubloxGPS *,uint8_t chr)> decodeStateHandler = &ubloxGPS::stateSync1;
    decode_result_t decodeUbx(uint8_t chr);
    decode_result_t stateSync1(uint8_t chr); 
    decode_result_t stateSync2(uint8_t chr); 
    decode_result_t stateHeader(uint8_t chr); 
    decode_result_t stateData(uint8_t chr); 
    decode_result_t stateCrcA(uint8_t chr); 
    decode_result_t stateCrcB(uint8_t chr);

    // number of points to look at when determining if locked location is stable
    static constexpr unsigned int STABILITY_WINDOW_LENGTH = 5;
    // std deviation in window must be under this percentage of mean to be
    // classified as stable
    static constexpr float STABILITY_WINDOW_THRESHOLD = 0.05;

    // track accuracy of previous points to determine if locked location is stable
    float stabilityWindow[STABILITY_WINDOW_LENGTH];
    unsigned int stabilityWindowLength;
    unsigned int stabilityWindowNext;
    time_t stabilityWindowLastTimestamp;
    bool isStable;
    uint32_t startLockUptime;
};

#endif /* __UBLOXGPS_H */
