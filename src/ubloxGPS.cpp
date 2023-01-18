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
#include "Particle.h"
#include "check.h"
#include "scope_guard.h"
#include "ubloxGPS.h"
#include <mutex>
#include <cmath>
#include <time.h>

static Logger Loglib("app.gps.ubx");
static Logger nmea_log("app.gps.nmea");
RecursiveMutex gps_mutex;

/* Parse NMEA item define */
#define PARSE_NMEA_GPGGA
// #define PARSE_NMEA_GPGLL
// #define PARSE_NMEA_GPGSA
#define PARSE_NMEA_GPGSV
#define PARSE_NMEA_GPRMC
// #define PARSE_NMEA_GPVTG
// #define PARSE_NMEA_GPGST
// #define PARSE_NMEA_GPZDA

/* Parse PUBX item define */
#define PARSE_PUBX_POSITION
#define PARSE_PUBX_SVSTATUS
#define PARSE_PUBX_TIME

/* Parse UBX item define */
//#define PARSE_UBX_XXX

#define GPS_HEX_LOGGING (0)

#define LOCK()      std::lock_guard<RecursiveMutex> __gps_guard(gps_mutex);

static const int MAX_GPS_AGE_MS = 10000; // GPS location must be newer than this to be considered valid

static const uint8_t SYNC_1 = 0xB5;
static const uint8_t SYNC_2 = 0x62;

static const uint8_t MAX_BUF_SIZE = 100;

static uint32_t   byteCounter = 0;

static uint32_t lastUbxMsgSent = 0;
static const uint32_t UBX_MSG_TIMEOUT = 3000;
static const uint32_t UBX_MAX_POLL_INTERVAL_MS = 1000;

static int nmea_uptime_wrapper()
{
    return System.uptime();
}

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

static void nmea_event_log_cb(gps_t* gh, gps_statement_t res)
{
    if(res != STAT_CHECKSUM_FAIL)
    {
        nmea_log.trace("RX: %.*s", gh->sentence_len, gh->sentence);
    }
}

ubloxGPS::ubloxGPS(USARTSerial &serial,
    std::function<bool(bool)> pwr_enable) :
    interface(ubloxGpsInterface::Uart),
    spi(nullptr),
    spi_settings(5*MHZ, MSBFIRST, SPI_MODE0),
    spi_select(nullptr),
    tx_ready_mcu_pin(PIN_INVALID),
    tx_ready_gps_pin(PIN_INVALID),

    serial(&serial),

    pwr_enable(pwr_enable),
    tx_ready_queue(nullptr),
    log_enabled(true),
    debugNMEA(false),
    gpsThread(nullptr),
    lastLockTime(0),
    ubx_rx_msg({}),
    write_mga_active(false),
    write_mga_sequence(0),

    initializing(false),
    gpsStatus(GPS_STATUS_OFF),
    powerOn(false),

    lockMethod(ubloxGpsLockMethod::HorizontalAccuracy),
    hdopStability(STABILITY_HDOP_THRESHOLD),

    gpsUnit(0),
    last_receive_time(0),
    esf_status({}),
    nav_odo({}),
    nav_aopstatus({}),
    mon_ver({}),
    cfg_dyn_model(UBX_DEFAULT_MODEL),

    stabilityWindowLength(0),
    stabilityWindowNext(0),
    isStable(false),
    startLockUptime(0)
{
    enablePower(false);
    gps_init(&nmea_gps, nmea_uptime_wrapper);
}

ubloxGPS::ubloxGPS(SPIClass &spi,
    std::function<bool(bool)> spi_select,
    std::function<bool(bool)> pwr_enable,
    int tx_ready_mcu_pin,
    int tx_ready_gps_pin) :
    interface(ubloxGpsInterface::Spi),
    spi(&spi),
    spi_settings(5*MHZ, MSBFIRST, SPI_MODE0),
    spi_select(spi_select),
    tx_ready_mcu_pin(tx_ready_mcu_pin),
    tx_ready_gps_pin(tx_ready_gps_pin),

    serial(nullptr),

    pwr_enable(pwr_enable),
    tx_ready_queue(nullptr),
    log_enabled(true),
    debugNMEA(false),
    gpsThread(nullptr),
    lastLockTime(0),
    ubx_rx_msg({}),
    write_mga_active(false),
    write_mga_sequence(0),

    initializing(false),
    gpsStatus(GPS_STATUS_OFF),
    powerOn(false),
    lockMethod(ubloxGpsLockMethod::HorizontalAccuracy),
    hdopStability(STABILITY_HDOP_THRESHOLD),

    gpsUnit(0),
    last_receive_time(0),
    esf_status({}),
    nav_odo({}),
    nav_aopstatus({}),
    mon_ver({}),
    cfg_dyn_model(UBX_DEFAULT_MODEL),

    stabilityWindowLength(0),
    stabilityWindowNext(0),
    isStable(false),
    startLockUptime(0)
{
    enablePower(false);
    spi_select(false);
    gps_init(&nmea_gps, nmea_uptime_wrapper);
}

void ubloxGPS::lock()
{
    gps_mutex.lock();
}

void ubloxGPS::unlock()
{
    gps_mutex.unlock();
}

int ubloxGPS::enablePower(bool enable)
{
    auto ret = pwr_enable(enable);
    if (!ret) {
        return SYSTEM_ERROR_IO;
    }
    powerOn = enable;
    return SYSTEM_ERROR_NONE;
}

void ubloxGPS::txReadyHandler()
{
    uint8_t dummy = 0x00;

    // if queue is full than the consumer will already be notified that there
    // was a tx ready event so doesn't matter if the insert fails
    os_queue_put(tx_ready_queue, &dummy, 0, nullptr);
}

bool ubloxGPS::isLockStable()
{
    LOCK();

    return getLock() && isStable;
}

int ubloxGPS::setLockHdopThreshold(double threshold)
{
    if (0.0 < threshold) {
        return SYSTEM_ERROR_INVALID_ARGUMENT;
    }

    hdopStability = threshold;
    return SYSTEM_ERROR_NONE;
}

unsigned int ubloxGPS::getLockDuration()
{
    LOCK();

    if(!getLock())
    {
        return 0;
    }

    return System.uptime() - startLockUptime;
}

// MUST BE CALLED WITH GPS LOCK ALREADY HELD
void ubloxGPS::processLockStability()
{
    if(!getLock())
    {
        isStable = false;
        stabilityWindowLength = 0;
        startLockUptime = 0;
        return;
    }

    if(!startLockUptime)
    {
        startLockUptime = System.uptime();
    }

    uint32_t last_timestamp = getTime();
    if(stabilityWindowLastTimestamp == last_timestamp)
    {
        return;
    }
    if (ubloxGpsLockMethod::HorizontalAccuracy == lockMethod) {
        if(stabilityWindowLength < STABILITY_WINDOW_LENGTH)
        {
            stabilityWindowLength++;
        }
        stabilityWindow[stabilityWindowNext] = getHorizontalAccuracy();
        stabilityWindowNext = (stabilityWindowNext + 1) % STABILITY_WINDOW_LENGTH;

        float avg = 0.0;
        for(unsigned int i=0; i < stabilityWindowLength; i++)
        {
            avg += stabilityWindow[i];
        }
        avg /= stabilityWindowLength;

        float std_dev = 0.0;
        for(unsigned int i=0; i < stabilityWindowLength; i++)
        {
            float temp = (avg - stabilityWindow[i]);
            std_dev += temp * temp;
        }
        std_dev /= stabilityWindowLength;
        std_dev = sqrt(std_dev);

        isStable = ((stabilityWindowLength == STABILITY_WINDOW_LENGTH) && (std_dev < (avg * STABILITY_WINDOW_THRESHOLD)));
    } else if (ubloxGpsLockMethod::HorizontalDop == lockMethod) {
        isStable = (getHDOP() < (double)hdopStability);
    } else {
        isStable = true;
    }
    stabilityWindowLastTimestamp = last_timestamp;
}

void ubloxGPS::processGPSByte(uint8_t c)
{
    // in SPI mode there are too many null 0xFF reads to reasonably log raw
    // traffic on the SPI port as we don't know at this layer if the byte is
    // part of an actual message or not
    if (debugNMEA && log_enabled && isInterfaceUart()) {
    #if (GPS_HEX_LOGGING == 1)
        const char d = ',';
        Loglib.dump(LOG_LEVEL_TRACE, &c, 1);
        Loglib.write(LOG_LEVEL_TRACE, &d, 1);
    #else
        Loglib.write(LOG_LEVEL_TRACE, (char *) &c, 1);
    #endif
    }
    // decode UBX command
    if (decodeUbx((uint8_t)c) == DECODE_RESULT_IS_ACK_NAK_RSP) {
        // Loglib.info("UBX DECODE_RESULT_COMPLETE");
    }

    int pos_timestamp_prev = nmea_gps.pos_timestamp;
    int date_timestamp_prev = nmea_gps.date_timestamp;

    // parse NMEA message
    gps_process(&nmea_gps, &c, 1, log_enabled ? nmea_event_log_cb :  nullptr);

    if (pos_timestamp_prev != nmea_gps.pos_timestamp)
    {
        perf_counts.pos_report_count++;
    }
    if (date_timestamp_prev != nmea_gps.date_timestamp)
    {
        perf_counts.time_report_count++;
    }

    if (!initializing) {
        if (getLock())
        {
            lastLockTime = Time.now() - (System.uptime() - nmea_gps.time_timestamp);
            gpsStatus = GPS_STATUS_LOCK;
        }
        else
        {
            gpsStatus = GPS_STATUS_FIXING;
        }
    }

    processLockStability();

    last_receive_time = millis();
}

// gps reading Serial1 thread
void ubloxGPS::updateGPS(void)
{
    while (true) {
        bool available =yieldThread(UBX_MAX_POLL_INTERVAL_MS);

        if ((gpsStatus != GPS_STATUS_OFF) && (gpsStatus != GPS_STATUS_ERROR))
        {
            // try and acquire the lock in a non-blocking fashion as another
            // thread may have already acquired the lock and also be pending
            // on the tx ready event that was just consumed, want to be able
            // to put it back in such a case
            if(!gps_mutex.try_lock())
            {
                if(tx_ready_queue && available)
                {
                    // put the tx ready event back
                    txReadyHandler();
                    // lock to force wait on whatever task may have been
                    // holding the lock before coming back to try again
                    gps_mutex.lock();
                    gps_mutex.unlock();
                }
            }
            else
            {
                processBytes();
                gps_mutex.unlock();
            }
        }
    }
}

int ubloxGPS::setOn(lib_config_t &config)
{
    LOCK();

    if(isInterfaceUart())
    {
        serial->begin((uint32_t)UBX_BAUDRATE_DEFAULT);
    }

    enablePower(true);
    initializing = true;
    gps_init(&nmea_gps, nmea_uptime_wrapper);

    NAMED_SCOPE_GUARD(exitScope, {
        gpsStatus = GPS_STATUS_ERROR;
        enablePower(false);
        if (log_enabled) {
            Loglib.error("Initialization failed");
        }
    });
    delay(1000); //delay 1000ms to wait for ubloxGPS get ready for receive UBX command (tested value)

    if(isInterfaceUart())
    {
        // set high baudrate to improve performance
        CHECK_TRUE(setBaudrate((ubx_baudrate_t)config.baudrate), SYSTEM_ERROR_IO);
        if(log_enabled)
            Loglib.info("UBX baudrate is %lu", config.baudrate);
    } else if (isInterfaceSpi()) {
        CHECK_FALSE((tx_ready_mcu_pin == PIN_INVALID) || (tx_ready_gps_pin == PIN_INVALID), SYSTEM_ERROR_INVALID_ARGUMENT);
        // TODO: break this conditional up to exit on queue create error
        if(!tx_ready_queue && !os_queue_create(&tx_ready_queue, sizeof(uint8_t), 1, nullptr))
        {
            pinMode(tx_ready_mcu_pin, INPUT);
            if(!attachInterrupt(tx_ready_mcu_pin, &ubloxGPS::txReadyHandler, this, FALLING))
            {
                os_queue_destroy(tx_ready_queue, nullptr);
                tx_ready_queue = nullptr;
                return SYSTEM_ERROR_NO_MEMORY;
            }
        }

        // first process all bytes off of the port, reconfiguring seems to drop
        // some startup frames that are useful debug indications
        processBytes();

        CHECK_TRUE(setSpiMode(tx_ready_gps_pin, 0, UBX_CFG_PRT_MODE_SPI_MODE_0), SYSTEM_ERROR_IO);
    } else {
        // Interface not defined
        return SYSTEM_ERROR_INVALID_STATE;
    }

    if (config.output_pubx) {
        CHECK_TRUE(enablePUBX(config.fastIntervalSec, config.slowIntervalSec), SYSTEM_ERROR_IO);
        CHECK_TRUE(disableNMEA(), SYSTEM_ERROR_IO);
    } else {
        CHECK_TRUE(enableNMEA(config.fastIntervalSec, config.slowIntervalSec), SYSTEM_ERROR_IO);
        CHECK_TRUE(disablePUBX(), SYSTEM_ERROR_IO);
    }

    CHECK_TRUE(configMsg(UBX_CLASS_ESF, UBX_ESF_STATUS, 10), SYSTEM_ERROR_IO); // TODO: Once ESF is good maybe we can slow this down.
    CHECK_TRUE(configMsg(UBX_CLASS_NAV, UBX_NAV_AOPSTATUS, 30), SYSTEM_ERROR_IO);
    CHECK_TRUE(configMsg(UBX_CLASS_NAV, UBX_NAV_ODO, 5), SYSTEM_ERROR_IO);
    CHECK_TRUE(configMsg(UBX_CLASS_NAV, UBX_NAV_SAT, 5), SYSTEM_ERROR_IO);
    CHECK_TRUE(configMsg(UBX_CLASS_NAV, UBX_NAV_ORB, 5), SYSTEM_ERROR_IO);

    CHECK_TRUE(setGNSS(config.support_gnss), SYSTEM_ERROR_IO);
    CHECK_TRUE(setPower((ubx_power_mode_t)config.power_mode), SYSTEM_ERROR_IO);
    CHECK_TRUE(setMode((ubx_dynamic_model_t)config.dynamic_model), SYSTEM_ERROR_IO);
    last_receive_time = 0;

    // TODO: Move this segment earlier and check for success
    if (!gpsThread) {
        gpsThread = new Thread("gps", [this]() { updateGPS(); }, OS_THREAD_PRIORITY_DEFAULT);
    }

    exitScope.dismiss();
    initializing = false;
    gpsStatus = GPS_STATUS_FIXING;
    return SYSTEM_ERROR_NONE;
}

int ubloxGPS::on(ubx_dynamic_model_t model)
{
    lib_config.resetDefault();
    lib_config.dynamic_model = model;
    return setOn(lib_config);
}

int ubloxGPS::off(void)
{
    initializing = false;
    gpsStatus = GPS_STATUS_OFF;
    return enablePower(false);
}

bool ubloxGPS::is_active(void)
{
    switch (gpsStatus) {
        case GPS_STATUS_OFF:
            // Fall through
        case GPS_STATUS_ERROR:
            return false;
    }

    return  (millis() -last_receive_time < (lib_config.fastIntervalSec * 3000));
}

void ubloxGPS::setOutputNMEA(void)
{
    enableNMEA(lib_config.fastIntervalSec, lib_config.slowIntervalSec);
    disablePUBX();
}

void ubloxGPS::setOutputPUBX(void)
{
    enablePUBX(lib_config.fastIntervalSec, lib_config.slowIntervalSec);
    disableNMEA();
}


uint8_t ubloxGPS::getGpsStatus()
{
    return gpsStatus;
}

void ubloxGPS::setCal(void)
{
    LOCK();
    //TODO: add automotive dead rockoning calibration procedure
}

void ubloxGPS::setUnit(ubx_gps_unit_t unit)
{
    // degree or DMS is save to RAM, but not use it now
    gpsUnit = unit;
}

double ubloxGPS::getLatitude(void)
{
    return nmea_gps.latitude;
}

double ubloxGPS::getLongitude(void)
{
    return nmea_gps.longitude;
}

float ubloxGPS::getAltitude(void)
{
    return nmea_gps.altitude;
}

uint8_t ubloxGPS::getFixQuality(void)
{
    //  0 = No fix,
    //  1 = Autonomous GNSS fix,
    //  2 = Differential GNSS fix,
    //  4 = RTK fixed,
    //  5 = RTK float,
    //  6 = Estimated/Dead reckoning fix
    return nmea_gps.fix;
}

bool ubloxGPS::getLock(void)
{
    LOCK();
    return (nmea_gps.fix
        && nmea_gps.pos_valid
        && nmea_gps.pos_timestamp &&
        System.uptime() - nmea_gps.pos_timestamp < (MAX_GPS_AGE_MS / 1000));
}

uint32_t ubloxGPS::getLockTime(void)
{
    return lastLockTime;
}

float ubloxGPS::getSpeed(uint8_t unit)
{
    constexpr float KNOTS_TO_MPS = 0.51444444444444;
    constexpr float KNOTS_TO_MPH = 1.1507794480235;
    constexpr float KNOTS_TO_KMPH = 1.852;
    float speed = nmea_gps.speed; // in knots

    switch (unit) {
    default:
    case GPS_SPEED_UNIT_MPS:
        speed *= KNOTS_TO_MPS;
        break;
    case GPS_SPEED_UNIT_MPH:
        speed *= KNOTS_TO_MPH;
        break;
    case GPS_SPEED_UNIT_KMPH:
        speed *= KNOTS_TO_KMPH;
        break;
    }
    return speed;
}

float ubloxGPS::getHeading(void)
{
    return nmea_gps.course;
}

uint32_t ubloxGPS::getDate(void)
{
    LOCK();
    return nmea_gps.year * 10000 + nmea_gps.month * 100 + nmea_gps.date;
}

uint32_t ubloxGPS::getTime(void)
{
    LOCK();
    return nmea_gps.hours * 10000 + nmea_gps.minutes * 100 + nmea_gps.seconds;
}

uint32_t ubloxGPS::getUTCTime()
{
    struct tm t;

    memset(&t, 0, sizeof(t));
    WITH_LOCK(*this) {
        if (nmea_gps.time_valid && nmea_gps.date_valid) {
            t.tm_hour = nmea_gps.hours;
            t.tm_min = nmea_gps.minutes;
            t.tm_sec = nmea_gps.seconds;
            t.tm_mday = nmea_gps.date;
            // struct tm expects 0-11 mon, nmea parser gives 1-12
            t.tm_mon = nmea_gps.month - 1;
            // struct tm expect years since 1900, nmea parser gives year since 2000
            t.tm_year = nmea_gps.year + 100;
        }
    }

    return mktime(&t);
}

uint8_t ubloxGPS::getSatellites(void)
{
    // Return the number of satellites in use.
    return nmea_gps.sats_in_use;
}

uint8_t ubloxGPS::getSatellitesDesc(gps_sat_t sat_arr[])
{
    if (sat_arr != NULL) {
        memcpy(sat_arr, nmea_gps.sats_in_view_desc, sizeof(nmea_gps.sats_in_view_desc));
    }

    // Return the number of satellites in view.
    return nmea_gps.sats_in_view;
}

double ubloxGPS::getGeoIdHeight(void)
{
    //the geoid's variation ranges from +85 m (Iceland) to −106 m (southern India)
    return (double) nmea_gps.geo_sep;
}

double ubloxGPS::getHDOP(void)
{
    return (double) nmea_gps.dop_h;
}

double ubloxGPS::getVDOP(void)
{
    return (double) nmea_gps.dop_v;
}

uint8_t ubloxGPS::getSignalStrength(void)
{
    // take the initial satellites average cno as the signal strength
    LOCK();
    constexpr int MAX_SAT_COUNT = 4;
    uint16_t avgSignal = 0;
    int count = std::min((int)nmea_gps.sats_in_view, MAX_SAT_COUNT);
    int counted = 0;

    for (int i = 0; i < nmea_gps.sats_in_view && counted < count; i++)
    {
        if(nmea_gps.sats_in_view_desc[i].used)
        {
            avgSignal += nmea_gps.sats_in_view_desc[i].snr;
            counted++;
        }
    }
    if(counted)
    {
        avgSignal /= counted;
    }

    return (uint8_t)avgSignal;
}

double ubloxGPS::getHorizontalAccuracy(void)
{
    return nmea_gps.h_accuracy;
}

double ubloxGPS::getVerticalAccuracy(void)
{
    return nmea_gps.v_accuracy;
}

float ubloxGPS::getDistance(double lat1, double long1, double lat2, double long2)
{
    //default unit is m(meter)
    gps_float_t distance, bearing;
    gps_distance_bearing(lat1, long1, lat2, long2, &distance, &bearing);
    return distance;
}

void ubloxGPS::enableDebugNMEA(bool en)
{
    debugNMEA = en;
}

void ubloxGPS::initRxMsg()
{
    ubx_rx_msg.init();
    byteCounter = 0;
}

bool ubloxGPS::parseRxMsg()
{
    if (ubx_rx_msg.msg_class == UBX_CLASS_ACK) {
        if (ackExpected || rspExpected) {
            if (ubx_rx_msg.msg_id == UBX_ACK_ACK) {
                ubx_ack_t *ack = (ubx_ack_t *) &ubx_rx_msg;
                // allow wildcarding for ACK matches
                if((waitForReqClass == UBX_CLASS_INVALID || ack->req_class == waitForReqClass) &&
                    ((waitForReqId == UBX_ID_INVALID || ack->req_id == waitForReqId)))
                {
                    ackReceived = true;
                }
            } else if (ubx_rx_msg.msg_id == UBX_ACK_NAK) {
                ubx_nak_t *nak = (ubx_nak_t *) &ubx_rx_msg;
                // allow wildcarding for NAK matches
                if((waitForReqClass == UBX_CLASS_INVALID || nak->req_class == waitForReqClass) &&
                    ((waitForReqId == UBX_ID_INVALID || nak->req_id == waitForReqId)))
                {
                    nakReceived = true;
                }
            }
        }
    } else {
        processUBX();

        // allow wildcarding for response matches (match any response, match any
        // response from a specified class)
        if(rspExpected &&
            ((waitForRspClass == UBX_CLASS_INVALID || ubx_rx_msg.msg_class == waitForRspClass) &&
            (waitForRspId == UBX_ID_INVALID || ubx_rx_msg.msg_id == waitForRspId)))
        {
            rspReceived = true;
        }
    }

    return (ackReceived || nakReceived || rspReceived);
}

// TODO: Should implement a mechanism allowing registration of listeners for
// desired messages.
void ubloxGPS::processUBX()
{
    // Check for ESF Status
    if (ubx_rx_msg.msg_class == UBX_CLASS_ESF && ubx_rx_msg.msg_id == UBX_ESF_STATUS ) {
        esf_status.valid = true;
        esf_status.fusionMode = ubx_rx_msg.ubx_msg[12];
        esf_status.numSens = ubx_rx_msg.ubx_msg[15];
        /*
        Loglib.write("\r\n");
        Loglib.info("UBX_ESF_STATUS[fusionMode]:%d", esf_status.fusionMode);
        Loglib.info("UBX_ESF_STATUS[sensors]   :%d", esf_status.numSens);
        */
        for (int x = 0; x < esf_status.numSens; x++) {
            esf_status.sensStatus1[x] = ubx_rx_msg.ubx_msg[16 + (x * 4)];
            esf_status.sensStatus2[x] = ubx_rx_msg.ubx_msg[17 + (x * 4)];
            esf_status.freq[x]        = ubx_rx_msg.ubx_msg[18 + (x * 4)];
            esf_status.faults[x]      = ubx_rx_msg.ubx_msg[19 + (x * 4)];
            /*
            Loglib.info("UBX_ESF_STATUS[sensor%d]   :r:%d u:%d t:%d ts:%d cs:%d fq:%d bf:%01x", x + 1,
                                    (ubx_rx_msg.ubx_msg[16 + (x * 4)] & 0x80) ? 1 : 0, // ready
                                    (ubx_rx_msg.ubx_msg[16 + (x * 4)] & 0x40) ? 1 : 0, // used
                                    ubx_rx_msg.ubx_msg[16 + (x * 4)] & 0x1f, // type
                                    (ubx_rx_msg.ubx_msg[17 + (x * 4)] & 0x0c) >> 2, // timeStatus
                                    ubx_rx_msg.ubx_msg[17 + (x * 4)] & 0x03, // calibStatus
                                    ubx_rx_msg.ubx_msg[18 + (x * 4)], // freq
                                    ubx_rx_msg.ubx_msg[19 + (x * 4)]); // bitfield faults
            */
        }
    } else if (ubx_rx_msg.msg_class == UBX_CLASS_NAV && ubx_rx_msg.msg_id == UBX_NAV_ODO ) {
        nav_odo.valid = true;
        nav_odo.iTOW = 0;
        nav_odo.distance = 0;
        nav_odo.totalDistance = 0;
        nav_odo.distanceStd = 0;
        for (int i = 0; i < 4; i++) {
            nav_odo.iTOW          |= ubx_rx_msg.ubx_msg[4  + i] << (8 * i);
            nav_odo.distance      |= ubx_rx_msg.ubx_msg[8  + i] << (8 * i);
            nav_odo.totalDistance |= ubx_rx_msg.ubx_msg[12 + i] << (8 * i);
            nav_odo.distanceStd   |= ubx_rx_msg.ubx_msg[16 + i] << (8 * i);
        }
        // Loglib.info("ODO: iTOW:%lums dis:%lum total:%lum std:%lum", nav_odo.iTOW, nav_odo.distance, nav_odo.totalDistance, nav_odo.distanceStd);
    } else if (ubx_rx_msg.msg_class == UBX_CLASS_NAV && ubx_rx_msg.msg_id == UBX_NAV_AOPSTATUS ) {
        nav_aopstatus.valid = true;
        nav_aopstatus.iTOW = 0;
        nav_aopstatus.aopCfg = 0;
        nav_aopstatus.aopStatus = 0;
        for (int i = 0; i < 4; i++) {
            nav_aopstatus.iTOW |= ubx_rx_msg.ubx_msg[i] << (8 * i);
        }
        nav_aopstatus.aopCfg    = ubx_rx_msg.ubx_msg[4];
        nav_aopstatus.aopStatus = ubx_rx_msg.ubx_msg[5];
        Loglib.info("AOPSTATUS: iTOW:%lums, aopCfg:%u, aopStatus:%u", nav_aopstatus.iTOW, nav_aopstatus.aopCfg, nav_aopstatus.aopStatus);
    } else if (ubx_rx_msg.msg_class == UBX_CLASS_NAV && ubx_rx_msg.msg_id == UBX_NAV_SAT ) {
        memcpy(nav_sat.bytes, ubx_rx_msg.ubx_msg, sizeof(ubx_nav_sat_t));
        Loglib.info("UBX_NAV_SAT: %u sats in view", nav_sat.regs.numSvs);
        for (auto sv : nav_sat.regs.sats) {
            Loglib.trace("\tSatellite: {used: %c, num: %*u, snr: %*u, qualInd: %u, health: %u, ephAv: %c, almAv: %c, anoAv: %c, aopAv: %c}", 
                sv.flags.fields.svUsed ? 'Y' : 'N', 3, sv.svID, 3, sv.cno, sv.flags.fields.qualityInd, sv.flags.fields.health, 
                sv.flags.fields.ephAvail ? 'Y' : 'N', sv.flags.fields.almAvail ? 'Y' : 'N', sv.flags.fields.anoAvail ? 'Y' : 'N',
                sv.flags.fields.aopAvail ? 'Y' : 'N');
        }
    } else if (ubx_rx_msg.msg_class == UBX_CLASS_NAV && ubx_rx_msg.msg_id == UBX_NAV_ORB ) {
        memcpy(nav_orb.bytes, ubx_rx_msg.ubx_msg, sizeof(ubx_nav_orb_t));
        Loglib.info("UBX_NAV_ORB: %u sats in almanac", nav_orb.regs.numSv);
        for (auto sv : nav_orb.regs.sats) {
            Loglib.trace("\tOrbit: {num: %*u, health: %u, viz: %u, ephUse: %u, ephSrc: %u, almUse: %u, almSrc: %u, aopUse: %u, orbTyp: %u}", 
                3, sv.svId, 
                sv.svFlag.flags.health, sv.svFlag.flags.visibility, 
                sv.eph.flags.ephUseability, sv.eph.flags.ephSource,
                sv.alm.flags.almUsability, sv.alm.flags.almSource,
                sv.otherOrb.flags.anoAopUsability, sv.otherOrb.flags.type);
        }
    } else if (ubx_rx_msg.msg_class == UBX_CLASS_MON && ubx_rx_msg.msg_id == UBX_MON_VER ) {
        free(mon_ver.sw_version);
        free(mon_ver.hw_version);
        free(mon_ver.extension);
        mon_ver.sw_version = (uint8_t *)malloc(30);
        mon_ver.hw_version = (uint8_t *)malloc(10);
        memcpy(mon_ver.sw_version, &ubx_rx_msg.ubx_msg[0],  30);
        memcpy(mon_ver.hw_version, &ubx_rx_msg.ubx_msg[30], 10);

        Log.info("==== GPS MON VER ====");
        Log.info("length :%d", ubx_rx_msg.length);
        Log.info("swVer  :%s", &ubx_rx_msg.ubx_msg[0]);
        Log.info("hwVer  :%s", &ubx_rx_msg.ubx_msg[30]);

        if(ubx_rx_msg.length > 40)
        {
            int index = 0;
            String str_ext = "";
            for(int i = 40; i < ubx_rx_msg.length; i += 30)
            {
                Log.info("ext[%d]: %s", index++, &ubx_rx_msg.ubx_msg[i]);
                str_ext += String::format("%s,", &ubx_rx_msg.ubx_msg[i]);
            }
            mon_ver.extension = (uint8_t *)malloc(str_ext.length());
            memcpy(mon_ver.extension, str_ext.c_str(), str_ext.length());
            mon_ver.extension[str_ext.length()-1] = '\0';
        }
        mon_ver.valid = true;
    } else if (ubx_rx_msg.msg_class == UBX_CLASS_CFG && ubx_rx_msg.msg_id == UBX_CFG_NAV5 ) {
        cfg_dyn_model = static_cast<ubx_dynamic_model_t>(ubx_rx_msg.ubx_msg[2] + ubx_rx_msg.ubx_msg[3] * 256);

        Log.info("==== UBX CFG NAV5 ====");
        Log.info("dynModel :%d", (int)cfg_dyn_model);
    } else if (ubx_rx_msg.msg_class == UBX_CLASS_UPD && ubx_rx_msg.msg_id == UBX_UPD_SOS ) {
        // Loglib.info("=== UBX UPD SOS ===");
        switch(ubx_rx_msg.ubx_msg[0]) {
            // Backup creation acknowledge message
            case 2: {
                switch((ubx_upd_sos_create_resp_t)ubx_rx_msg.ubx_msg[4]) {
                    case UBX_UPD_SOS_CREATE_ACK: {
                        Loglib.info("Save on Shutdown: Backup creation SUCCESS");
                        // TOOD: now safe to sleep!
                        saveOnShutdownACK = true;
                        break;
                    }
                    case UBX_UPD_SOS_CREATE_NAK: {
                        Loglib.warn("Save on Shutdown: Backup creation FAILED");
                        // TODO: unsafe to sleep!
                        saveOnShutdownACK = false;
                        break;
                    }
                }
                break;
            }
            
            // System restored from backup message
            case 3: {
                restoreStatus = (ubx_upd_sos_restore_resp_t)ubx_rx_msg.ubx_msg[4];
                static const char * sos_restore_msgs[] = {
                    "UNKNOWN",
                    "FAILURE",
                    "SUCCESS",
                    "No backups"
                };
                static const char * sos_restore_prefix = "Save on Shutdown backup restore: ";
                switch(restoreStatus) {
                    case UBX_UPD_SOS_RESTORE_UNKNOWN:
                    case UBX_UPD_SOS_RESTORE_FAILED:
                    {
                        Loglib.warn("%s%s",sos_restore_prefix, sos_restore_msgs[restoreStatus]);
                        break;
                    }
                    case UBX_UPD_SOS_RESTORE_SUCCESS:
                    case UBX_UPD_SOS_RESTORE_NONE: {
                        Loglib.info("%s%s",sos_restore_prefix, sos_restore_msgs[restoreStatus]);
                        break;
                    }
                }
                break;
            }

            default: break;
        }
    }
}


decode_result_t ubloxGPS::decodeUbx(uint8_t byte)
{
    return (decodeStateHandler)(this, byte);
}

decode_result_t ubloxGPS::stateSync1(uint8_t byte)
{
    if (byte == SYNC_1) {
        decodeStateHandler = &ubloxGPS::stateSync2;
        return DECODE_RESULT_OK;
    }
    else {
        return DECODE_RESULT_INVALID;
    }
}

decode_result_t ubloxGPS::stateSync2(uint8_t byte)
{
    if (byte == SYNC_2) {
        initRxMsg();
        decodeStateHandler = &ubloxGPS::stateHeader;
        return DECODE_RESULT_OK;
    }
    else {
        decodeStateHandler = &ubloxGPS::stateSync1;
        return stateSync1(byte);
    }
}



decode_result_t ubloxGPS::stateHeader(uint8_t byte)
{
    ubx_rx_msg.crc_a += byte;
    ubx_rx_msg.crc_b += ubx_rx_msg.crc_a;

    switch (byteCounter++) {
    case HEADER_CLASS_OFFSET:
        ubx_rx_msg.msg_class = (ubx_msg_class_t) byte;
        break;

    case HEADER_MSG_ID_OFFSET:
        ubx_rx_msg.msg_id = (ubx_msg_id_t) byte;
        break;

    case HEADER_LENGTH_LSB_OFFSET:
        ubx_rx_msg.length = byte;
        break;

    case HEADER_LENGTH_MSB_OFFSET:
        ubx_rx_msg.length += byte << 8;
        if (ubx_rx_msg.length > UBX_RX_MSG_MAX_LEN) {
            decodeStateHandler = &ubloxGPS::stateSync1;
            return DECODE_RESULT_INVALID;
        }

        byteCounter = 0;
        decodeStateHandler = &ubloxGPS::stateData;
        break;
    }
    return DECODE_RESULT_OK;
}

decode_result_t ubloxGPS::stateData(uint8_t byte)
{
    ubx_rx_msg.crc_a += byte;
    ubx_rx_msg.crc_b += ubx_rx_msg.crc_a;

    if ((byteCounter < ubx_rx_msg.length) && (byteCounter < sizeof(ubx_rx_msg.ubx_msg))) {
        ubx_rx_msg.ubx_msg[byteCounter] = byte;
    }

    if (++byteCounter >= ubx_rx_msg.length) {
        decodeStateHandler = &ubloxGPS::stateCrcA;
    }
    return DECODE_RESULT_OK;
}

decode_result_t ubloxGPS::stateCrcA(uint8_t byte)
{
    if (byte == ubx_rx_msg.crc_a) {
        decodeStateHandler = &ubloxGPS::stateCrcB;
        return DECODE_RESULT_OK;
    } else {
        decodeStateHandler = &ubloxGPS::stateSync1;
        return DECODE_RESULT_INVALID;
    }
}
decode_result_t ubloxGPS::stateCrcB(uint8_t byte)
{
    decode_result_t res = DECODE_RESULT_OK;

    if (byte == ubx_rx_msg.crc_b) {
        if (parseRxMsg()) {
            // Has an ACK or NACK
            res = DECODE_RESULT_IS_ACK_NAK_RSP;
        } else {
            res = DECODE_RESULT_END;
        }
        if(log_enabled)
        {
            Loglib.trace("RX: ");
            hex_dump(LOG_LEVEL_TRACE, (uint8_t *) &ubx_rx_msg, byteCounter + 4);
        }
    }
    else {
        res = DECODE_RESULT_INVALID;
    }

    decodeStateHandler = &ubloxGPS::stateSync1;

    return res;
}


/**
 * set new baudrate for ubloxGPS UART port
 *
 * NOTE: please pay attention that set baudrate can't get ACK/NAK
 *       so it's better to test the NMEA output by log print after
 *       you change baudrate, make sure config is successful.
 */
bool ubloxGPS::setBaudrate(ubx_baudrate_t baudrate)
{
    LOCK();

    CHECK_TRUE(isInterfaceUart(), false);

    uint8_t sentences[24] = {0};
    uint32_t br = (uint32_t)baudrate;
    sentences[0]  = (uint8_t)UBX_CLASS_CFG;
    sentences[1]  = (uint8_t)UBX_CFG_PRT;
    sentences[2]  = 0x14;
    sentences[3]  = 0x00;
    sentences[4]  = 0x01;   // portID
    sentences[5]  = 0x00;   // reserved
    sentences[6]  = 0x00;   // txReady, disable
    sentences[7]  = 0x00;
    sentences[8]  = 0xC0;   // mode 7-0,  8bit data
    sentences[9]  = 0x08;   // mode 15-8, no parity, 1bit stop
    sentences[10] = 0x00;   // mode 23-16
    sentences[11] = 0x00;   // mode 31-24
    sentences[12] = br & 0xFF;          // baudrate 7-0
    sentences[13] = (br >> 8) & 0xFF;   // baudrate 15-8
    sentences[14] = (br >> 16) & 0xFF;  // baudrate 23-16
    sentences[15] = (br >> 24) & 0xFF;  // baudrate 31-24
    sentences[16] = 0x07;   // inProtoMask,  bit0-inUBX,  bit1-inNMEA, bit2-inRTCM
    sentences[17] = 0x00;
    sentences[18] = 0x03;   // outProtoMask, bit0-outUBX, bit1-outNMEA
    sentences[19] = 0x00;
    sentences[20] = 0x00;   // flags
    sentences[21] = 0x00;
    sentences[22] = 0x00;   // reserved
    sentences[23] = 0x00;

    requestSendUBX((const ubx_msg_t *)sentences, 24, 0x00);

    // close and re-open the serial port on the new baud-rate
    serial->flush();
    serial->end();
    serial->begin((uint32_t)baudrate);
    return (bool) (waitForAck(UBX_CLASS_CFG, UBX_CFG_PRT) != NULL);
}

/**
 * Set configuration for ubloxGPS SPI port
 *
 */
bool ubloxGPS::setSpiMode(int txReady, int threshold, int spiMode) {
    ubx_cfg_port_t msg = {
        .header = {.msg_class = UBX_CLASS_CFG, .msg_id = UBX_CFG_PRT},
    };

    msg.port = UBX_CFG_PRT_SPI;
    msg.tx_ready.enable = 1;
    msg.tx_ready.polarity = UBX_CFG_PRT_TX_READY_ACTIVE_LOW;
    msg.tx_ready.pin = txReady;
    msg.tx_ready.threshold = threshold;
    msg.mode.spi.mode = spiMode;
    msg.mode.spi.ff_count = 50;
    msg.in_proto_mask = UBX_CFG_PRT_PROTO_UBX | UBX_CFG_PRT_PROTO_NMEA;
    msg.out_proto_mask = msg.in_proto_mask;
    msg.flags = UBX_CFG_PRT_FLAGS_EXTENDED_TIMEOUT;
    msg.header.length = sizeof(msg) - sizeof(msg.header);

    return requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg), UBX_REQ_FLAGS_EXPECT_ACK);
}

bool ubloxGPS::setAntanna(ubx_antenna_t ant)
{
    LOCK();
    uint8_t sentences[8] = { (uint8_t)UBX_CLASS_CFG, (uint8_t)UBX_CFG_ANT, 0x04, 0x00, 0x00, 0x00, 0xF0, 0x7D };
    sentences[4] = (uint8_t)ant;
    return requestSendUBX(sentences, 8);
}

bool ubloxGPS::setRate(uint16_t measRateHz)
{
    LOCK();
    uint8_t sentences[10] = { (uint8_t)UBX_CLASS_CFG, (uint8_t)UBX_CFG_RATE, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00 };
    uint16_t measTime = 1000 / measRateHz;
    sentences[4] = (uint8_t)(measTime & 0xFF);
    sentences[5] = (uint8_t)(measTime >> 8);
    return requestSendUBX(sentences, 10);
}

bool ubloxGPS::updateEsfStatus(void)
{
    LOCK();
    // invalidate ESF status local storage when forcing a poll request to know when data has updated from decode()
    esf_status.valid = false;
    uint8_t sentences[4] = { (uint8_t)UBX_CLASS_ESF, (uint8_t)UBX_ESF_STATUS, 0x00, 0x00 };
    return requestSendUBX(sentences, 4);
}

bool ubloxGPS::getEsfStatus(ubx_esf_status_t &esf)
{
    LOCK();
    memcpy(&esf, &esf_status, sizeof(ubx_esf_status_t));
    return esf_status.valid;
}

bool ubloxGPS::getSatelliteInfo(ubx_nav_sat_t &sats) {
    LOCK();
    memcpy(&sats, &nav_sat, sizeof(ubx_nav_sat_t));
    return true;
}

bool ubloxGPS::getOrbitDB(ubx_nav_orb_t &orb) {
    LOCK();
    memcpy(&orb, &nav_orb, sizeof(ubx_nav_orb_t));
    return true;
}

bool ubloxGPS::resetOdometer(void)
{
    // Reset the traveled distance computed by the odometer
    LOCK();
    nav_odo.valid = false;
    uint8_t sentences[4] = { (uint8_t)UBX_CLASS_NAV, (uint8_t)UBX_NAV_RESETODO, 0x00, 0x00 };
    return requestSendUBX(sentences, 4);
}

bool ubloxGPS::updateOdometer(void)
{
    // Polled output odometer data
    LOCK();
    nav_odo.valid = false;
    uint8_t sentences[4] = { (uint8_t)UBX_CLASS_NAV, (uint8_t)UBX_NAV_ODO, 0x00, 0x00 };
    return requestSendUBX(sentences, 4);
}

bool ubloxGPS::getOdometer(ubx_nav_odo_t &odo)
{
    LOCK();
    memcpy(&odo, &nav_odo, sizeof(ubx_nav_odo_t));
    return nav_odo.valid;
}

bool ubloxGPS::updateAopStatus(void)
{
    LOCK();
    // invalidate AOP status local storage when forcing a poll request to know when data has updated from decode()
    nav_aopstatus.valid = false;
    uint8_t sentences[4] = { (uint8_t)UBX_CLASS_NAV, (uint8_t)UBX_NAV_AOPSTATUS, 0x00, 0x00 };
    return requestSendUBX(sentences, 4);
}

bool ubloxGPS::updateVersion(void)
{
    LOCK();
    const ubx_msg_t *rsp = NULL;
    uint8_t sentences[4] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_MON;
    sentences[1] = (uint8_t)UBX_MON_VER;
    sentences[2] = 0x00;
    sentences[3] = 0x00;


    if(requestSendUBX((ubx_msg_t *) sentences, sizeof(sentences), 0x00, UBX_CLASS_MON, UBX_MON_VER))
    {
        rsp = getResponse();
    }

    return (bool) rsp;
}

bool ubloxGPS::getVersion(String& swVersion, String& hwVersion, String& extVersion)
{
    LOCK();
    if(mon_ver.valid)
    {
        swVersion = String::format("%s", mon_ver.sw_version);
        hwVersion = String::format("%s", mon_ver.hw_version);
        if(mon_ver.extension)
        {
            extVersion = String::format("%s", mon_ver.extension);
        }
    }
    return mon_ver.valid;
}

bool ubloxGPS::setGNSS(uint8_t gnssMask)
{
    LOCK();
    uint8_t sentences[64] = {0};
    uint8_t index = 9;
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_GNSS;
    sentences[2] = 0x04;
    sentences[3] = 0x00;
    sentences[4] = 0x00;
    sentences[5] = 0xFF;
    sentences[6] = 0xFF;
    sentences[7] = 0xFF;
    if (gnssMask == 0 || (gnssMask & (uint8_t)UBX_GNSS_TYPE_GPS) != 0 || (gnssMask & (uint8_t)UBX_GNSS_TYPE_QZSS) != 0) {
        sentences[2] += 8;
        sentences[index++] = (uint8_t)UBX_GNSSID_GPS;   /* gnssid */
        sentences[index++] = 4;     /* resTrkCh */
        sentences[index++] = 16;    /* maxTrkCh */
        sentences[index++] = 0;     /* reserved */
        sentences[index++] = 0x01;  /* flags */
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        if(log_enabled) Loglib.info("enable GPS");

        sentences[2] += 8;
        sentences[index++] = (uint8_t)UBX_GNSSID_QZSS;
        sentences[index++] = 4;
        sentences[index++] = 8;
        sentences[index++] = 0;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        if(log_enabled) Loglib.info("enable QZSS");
    }
    if ((gnssMask & (uint8_t)UBX_GNSS_TYPE_SBAS) != 0) {
        sentences[2] += 8;
        sentences[index++] = (uint8_t)UBX_GNSSID_SBAS;
        sentences[index++] = 4;
        sentences[index++] = 8;
        sentences[index++] = 0;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        if(log_enabled) Loglib.info("enable SBAS");
    }
    if ((gnssMask & (uint8_t)UBX_GNSS_TYPE_Galileo) != 0) {
        sentences[2] += 8;
        sentences[index++] = (uint8_t)UBX_GNSSID_Galileo;
        sentences[index++] = 4;
        sentences[index++] = 8;
        sentences[index++] = 0;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        if(log_enabled) Loglib.info("enable Galileo");
    }
    if ((gnssMask & (uint8_t)UBX_GNSS_TYPE_BeiDou) != 0) {
        sentences[2] += 8;
        sentences[index++] = (uint8_t)UBX_GNSSID_BeiDou;
        sentences[index++] = 6;
        sentences[index++] = 20;
        sentences[index++] = 0;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        if(log_enabled) Loglib.info("enable BeiDou");
    }
    if ((gnssMask & (uint8_t)UBX_GNSS_TYPE_IMES) != 0) {
        sentences[2] += 8;
        sentences[index++] = (uint8_t)UBX_GNSSID_IMES;
        sentences[index++] = 4;
        sentences[index++] = 8;
        sentences[index++] = 0;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        if(log_enabled) Loglib.info("enable IMES");
    }
    if ((gnssMask & (uint8_t)UBX_GNSS_TYPE_GLONASS) != 0) {
        sentences[2] += 8;
        sentences[index++] = (uint8_t)UBX_GNSSID_GLONASS;
        sentences[index++] = 4;
        sentences[index++] = 12;
        sentences[index++] = 0;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        sentences[index++] = 0x01;
        if(log_enabled) Loglib.info("enable GLONASS");
    }

    return requestSendUBX(sentences, sentences[2] + 4);
}
bool ubloxGPS::setUDREnable(bool useUDR)
{
    LOCK();
    uint8_t sentences[44] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_NAVX5;
    sentences[2] = 40;
    sentences[3] = 0x00;
    
    // Payload
    sentences[4] = 0x02; // version LSB = 0x02
    sentences[5] = 0x00; // version MSB = 0x00
    
    uint32_t mask2 = (1 << 6);  // Update ADR/UDR setting only
    sentences[8]  = (mask2 & 0xFF);         // mask2 LLSB
    sentences[9]  = ((mask2 >> 8) & 0xFF);  // mask2 LMSB
    sentences[10] = ((mask2 >> 16) & 0xFF); // mask2 MLSB
    sentences[11] = ((mask2 >> 24) & 0xFF); // mask2 MMSB

    sentences[43] = useUDR ? 0x01 : 0x00; // useUdr

    // other item not apply, so keep 0
    if(log_enabled) Loglib.info("set UDR Enable {useUDR: %s}", 
        useUDR ? "true" : "false");
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setIMUAutoAlignment(bool enable)
{
    LOCK();
    uint8_t sentences[16] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_ESFALG;
    sentences[2] = 12;
    sentences[3] = 0x00;
    
    // uint32_t bitfield
    sentences[4] = 0x00; // bit7 - version (0x0)
    sentences[5] = enable ? 0x01: 0x00; // bit8 - 0=use manual mount angles, 1=use automatic calibration
    sentences[6] = 0x00;
    sentences[7] = 0x00;

    // other item not apply, so keep 0
    if(log_enabled) Loglib.info("set IMU Automatic Alignment = %s", enable ? "true" : "false");
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setIMUAlignmentAngles(double yaw_angle_deg, double pitch_angle_deg, double roll_angle_deg)
{
    // Clip the inputs to the range the registers can take
    yaw_angle_deg   = clip(yaw_angle_deg,     0.0, 360.0);
    pitch_angle_deg = clip(pitch_angle_deg, -90.0,  90.0);
    roll_angle_deg  = clip(roll_angle_deg, -180.0, 180.0);

    LOCK();
    uint8_t sentences[16] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_ESFALG;
    sentences[2] = 12;
    sentences[3] = 0x00;
    
    // uint32_t bitfield
    sentences[4] = 0x00; // bit7 - version (0x0)
    sentences[5] = 0x00; // bit8 - 0=use manual mount angles, 1=use automatic calibration
    sentences[6] = 0x00;
    sentences[7] = 0x00;

    // yaw [0, 36000], 1e-2 scaling
    modf(yaw_angle_deg * 100, &yaw_angle_deg);
    uint32_t yaw_bytes = (uint32_t)yaw_angle_deg;
    sentences[8]  = (yaw_bytes & 0xFF);
    sentences[9]  = ((yaw_bytes >> 8)  & 0xFF);
    sentences[10] = ((yaw_bytes >> 16) & 0xFF);
    sentences[11] = ((yaw_bytes >> 24) & 0xFF);

    // pitch [-9000, 9000], 1e-2 scaling
    int16_t pitch_bytes;
    modf(pitch_angle_deg * 100, &pitch_angle_deg);
    pitch_bytes = (int16_t)pitch_angle_deg;
    sentences[12]  = (pitch_bytes & 0xFF);
    sentences[13]  = ((pitch_bytes >> 8) & 0xFF);

    // roll [-18000, 18000], 1e-2 scaling
    int16_t roll_bytes;
    modf(roll_angle_deg * 100, &roll_angle_deg);
    roll_bytes = (int16_t)roll_angle_deg;
    sentences[14]  = (roll_bytes & 0xFF);
    sentences[15]  = ((roll_bytes >> 8) & 0xFF);

    // other item not apply, so keep 0
    if(log_enabled) Loglib.info("set IMU alignment angles {yaw: %0.0f, pitch: %0.0f, roll: %0.0f}", yaw_angle_deg, pitch_angle_deg, roll_angle_deg);
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setIMUtoVRP(int16_t x, int16_t y, int16_t z)
{
    LOCK();
    uint8_t sentences[16] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_ESFLA;
    sentences[2] = 12;  // 4 + 8*numConfigs (which is 1 in this case)
    sentences[3] = 0x00;
    
    // uint32_t bitfield
    sentences[4] = 0x00; // message version (0x0)
    sentences[5] = 0x01; // numConfigs (1)
    sentences[6] = 0x00; // reserved1
    sentences[7] = 0x00; // reserved1

    // Lever arm configuration 1(
    sentences[8]  = static_cast<uint8_t>(UBX_LEVER_ARM_IMU_TO_VRP); // Lever arm type
    sentences[9]  = 0x00;               // reserved2
    sentences[10] = (x & 0xFF);         // Lever arm X LSB
    sentences[11] = ((x >> 8) & 0xFF);  // Lever arm X MSB
    sentences[12] = (y & 0xFF);         // Lever arm Y LSB
    sentences[13] = ((y >> 8) & 0xFF);  // Lever arm Y MSB
    sentences[14] = (z & 0xFF);         // Lever arm Z LSB
    sentences[15] = ((z >> 8) & 0xFF);  // Lever arm Z MSB

    if(log_enabled) Loglib.info("set IMU to VRP Lever Arm (cm) {x: %i, y: %i, z: %i}", x, y, z);
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setAOPSettings(bool useAop, uint16_t aopOrbMaxErr)
{
    LOCK();
    uint8_t sentences[48] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_NAVX5;
    sentences[2] = 44;
    sentences[3] = 0x00;
    
    // Payload
    sentences[4] = 0x03; // version LSB = 0x02
    sentences[5] = 0x00; // version MSB = 0x00
    
    uint16_t mask1 = (1 << 14); // Update assist now settings (aopCfg, aopOrbMaxErr) only
    sentences[6] = (mask1 & 0xFF);          // mask1 LSB
    sentences[7] = ((mask1 >> 8) & 0xFF);   // mask1 MSB

    sentences[31] = useAop ? 0x01 : 0x00;   // aopCfg: Assist Now Autonomous enable

    if (aopOrbMaxErr != 0) {
        // If we're not using the FW default value...
        if      (aopOrbMaxErr < 5)      aopOrbMaxErr = 5;
        else if (aopOrbMaxErr > 1000)   aopOrbMaxErr = 1000;
        sentences[34] = (aopOrbMaxErr & 0xFF);          // aopOrbMaxErr LSB
        sentences[35] = ((aopOrbMaxErr >> 8) & 0xFF);   // aopOrbMaxErr MSB
    } // else '0' means use FW default (100)

    // other item not apply, so keep 0
    if(log_enabled) Loglib.info("set AOP Settings {aopCfg: %s, aopOrbMaxErr: %u}", 
        useAop ? "true" : "false", 
        aopOrbMaxErr);
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setConfigClearSaveLoad(uint32_t clear, uint32_t save, uint32_t load)
{
    // Proper masking for registers
    clear &= 0x00001F1F;
    save  &= 0x00001F1F;
    load  &= 0x00001F1F;

    LOCK();
    uint8_t sentences[16] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_CFG;
    sentences[2] = 12;
    sentences[3] = 0x00;
    
    // Payload, all uint32_t's
    *(uint32_t*)(&sentences[4])  = clear;
    *(uint32_t*)(&sentences[8])  =  save;
    *(uint32_t*)(&sentences[12]) =  load;

    if(log_enabled) Loglib.info("setConfig: {clear: %04X, save: %04X, load: %04X}", (uint16_t)clear, (uint16_t)save, (uint16_t)load);
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setReset(ubx_reset_nav_bbr_mask_t navBbrMask, ubx_reset_mode_t resetMode)
{
    LOCK();
    uint8_t sentences[8] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_RST;
    sentences[2] = 8;
    sentences[3] = 0x00;
    
    // Payload
    sentences[4] = (uint8_t)(navBbrMask & 0xFF);
    sentences[5] = (uint8_t)((navBbrMask >> 8) & 0xFF);
    *(uint8_t*)(&sentences[6])  = (uint8_t)resetMode;

    if(log_enabled) Loglib.trace("setReset: {navBbrMask: %04X, resetMode: %02X}", (uint16_t)navBbrMask, (uint8_t)resetMode);
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::createBackup(void)
{
    LOCK();
    uint8_t sentences[8] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_UPD;
    sentences[1] = (uint8_t)UBX_UPD_SOS;
    sentences[2] = 4;
    sentences[3] = 0x00;

    sentences[4] = 0;   // command to save backup
    // sentences [5:7] is reserved

    if(log_enabled) Loglib.trace("createBackup()");
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::saveOnShutdown(void)
{
    saveOnShutdownACK = false;
    if (!setReset(UBX_RESET_MASK_HOT_START, UBX_RESET_MODE_CONTROLLED_GNSS_STOP)) {
        Loglib.warn("setReset() FAILED!");
        return false;
    }

    // Write UBX_UPD_SOS_BACKUP
    if (!createBackup()) {
        Loglib.warn("createBackup() FAILED!");
        return false;
    }
    
    // saveOnShutdown boolean will be set when UBX_UPD_SOS message is received!
    long unsigned int start = millis();
    while (!saveOnShutdownACK) {
        if (millis() - start > UBX_MSG_TIMEOUT) {   // TODO: check to make sure this timeout period is appropriate. Emprically 400ms is enough
            // Timeout
            Loglib.warn("saveOnShutdown ACK TIMEOUT!");
            return false;
        }
    }
    return true;
}

bool ubloxGPS::setTime(ubx_mga_init_time_utc_t timeAssist) {
    LOCK();
    uint8_t sentences[28] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_MGA;
    sentences[1] = (uint8_t)UBX_MGA_INI_TIME_UTC;
    sentences[2] = 24;
    sentences[3] = 0x00;

    sentences[4] = 0x10; // type, fixed
    sentences[5] = 0x00; // version, fixed
    sentences[6] = 0x00; // source = none (time valid when message sent, inaccurate)
    sentences[7] = 0x80; // leap seconds since 1980 (0x80 = unknown)
    sentences[8] = timeAssist.year & 0xFF;
    sentences[9] = (timeAssist.year >> 8) & 0xFF;
    sentences[10] = timeAssist.month;
    sentences[11] = timeAssist.day;
    sentences[12] = timeAssist.hour;
    sentences[13] = timeAssist.minute;
    sentences[14] = timeAssist.second;
    // sentences[15] = reserved1
    if (timeAssist.ns > 0) {
        sentences[16] = (uint8_t)timeAssist.ns         & 0xFF;
        sentences[17] = (uint8_t)(timeAssist.ns >> 8)  & 0xFF;
        sentences[18] = (uint8_t)(timeAssist.ns >> 16) & 0xFF;
        sentences[19] = (uint8_t)(timeAssist.ns >> 24) & 0xFF;
    }
    sentences[20] = timeAssist.tAccS & 0xFF;
    sentences[21] = (timeAssist.tAccS >> 8) & 0xFF;
    // sentences[22] = reserved2
    // sentences[23] = reserved2
    if (timeAssist.tAccNs > 0) {
        sentences[24] = (uint8_t)timeAssist.tAccNs         & 0xFF;
        sentences[25] = (uint8_t)(timeAssist.tAccNs >> 8)  & 0xFF;
        sentences[26] = (uint8_t)(timeAssist.tAccNs >> 16) & 0xFF;
        sentences[27] = (uint8_t)(timeAssist.tAccNs >> 24) & 0xFF;
    }
    // other item not apply, so keep 0
    if(log_enabled) Loglib.info("Set UTC time");
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setMode(ubx_dynamic_model_t dynModel)
{
    //TODO: The firmware shall initialize the GPS with automotive mode at default
    LOCK();
    uint8_t sentences[40] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_NAV5;
    sentences[2] = 36;
    sentences[3] = 0x00;
    sentences[4] = 0x05; // bit0 - apply dynamic model settings, bit2 - posFixMode
    sentences[5] = 0x00;
    sentences[6] = (uint8_t)dynModel;
    sentences[7] = 3;   // 1-2d only, 2-3d only, 3-auto 2d/3d
    // other item not apply, so keep 0
    if(log_enabled) Loglib.info("set dynamic platform model to %d", (uint8_t)dynModel);
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::getMode(ubx_dynamic_model_t& dynModel)
{
    dynModel = cfg_dyn_model;
    return true;
}

bool ubloxGPS::updateMode()
{
    //TODO: The firmware shall initialize the GPS with automotive mode at default
    LOCK();
    uint8_t sentences[40] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_NAV5;
    sentences[2] = 0x00;
    sentences[3] = 0x00;
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setPower(ubx_power_mode_t power_mode)
{
    //TODO: The firmware shall initialize the GPS with XXXX power management mode at default
    LOCK();
    uint8_t sentences[12] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_PMS;
    sentences[2] = 0x08;
    sentences[3] = 0x00;
    sentences[4] = 0x00;
    sentences[5] = (uint8_t)power_mode;
    if (power_mode == UBX_POWER_MODE_INTERVAL) {
        // period valid only when power mode set to UBX_POWER_MODE_INTERVAL, recommend to larger than 10s
        sentences[6] = 60;
        sentences[7] = 0;
        // onTime must smaller than period
        sentences[8] = 10;
        sentences[9] = 0;
        if(log_enabled) Loglib.info("set to power management mode %d-interval, period 60s, onTime 10s", (uint8_t)power_mode);
    } else {
        if(log_enabled) Loglib.info("set to power management mode %d", (uint8_t)power_mode);
    }
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setPower(ubx_power_mode_t power_mode, uint16_t period, uint16_t onTime)
{
    //TODO: The firmware shall initialize the GPS with XXXX power management mode at default
    LOCK();
    uint8_t sentences[12] = {0};
    sentences[0] = (uint8_t)UBX_CLASS_CFG;
    sentences[1] = (uint8_t)UBX_CFG_PMS;
    sentences[2] = 0x08;
    sentences[3] = 0x00;
    sentences[4] = 0x00;
    sentences[5] = (uint8_t)power_mode;
    if (power_mode == UBX_POWER_MODE_INTERVAL) {
        sentences[6] = (uint8_t)(period & 0xFF);
        sentences[7] = (uint8_t)((period >> 8) & 0xFF);
        sentences[8] = (uint8_t)(onTime & 0xFF);
        sentences[9] = (uint8_t)((onTime >> 8) & 0xFF);
        if(log_enabled) Loglib.info("set to power management mode %d-interval, period %ds, onTime %ds", (uint8_t)power_mode, period, onTime);
    } else {
        if(log_enabled) Loglib.info("set to power management mode %d", (uint8_t)power_mode);
    }
    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::setGeofence(ubx_geofence_t geofence)
{
    LOCK();
    uint8_t sentences[60] = {0};
    if (geofence.numFences > 4)
        geofence.numFences = 4;

    sentences[0]  = (uint8_t)UBX_CLASS_CFG;
    sentences[1]  = (uint8_t)UBX_CFG_GEOFENCE;
    sentences[2]  = 8 + 12 * geofence.numFences;
    sentences[3]  = 0x00;
    sentences[4]  = 0x00;
    sentences[5]  = geofence.numFences;
    sentences[6]  = geofence.confLvl;
    sentences[7]  = 0x00;
    sentences[8]  = geofence.pioEnabled;
    sentences[9]  = geofence.pinPolarity;
    sentences[10] = geofence.pinNumber;
    sentences[11] = 0x00;
    for (int i = 0; i < geofence.numFences; i++) {
        sentences[12 + i] = geofence.lat[i] & 0xFF;
        sentences[13 + i] = (geofence.lat[i] >> 8) & 0xFF;
        sentences[14 + i] = (geofence.lat[i] >> 16) & 0xFF;
        sentences[15 + i] = (geofence.lat[i] >> 24) & 0xFF;
        sentences[16 + i] = geofence.lon[i] & 0xFF;
        sentences[17 + i] = (geofence.lon[i] >> 8) & 0xFF;
        sentences[18 + i] = (geofence.lon[i] >> 16) & 0xFF;
        sentences[19 + i] = (geofence.lon[i] >> 24) & 0xFF;
        sentences[20 + i] = geofence.rad[i] & 0xFF;
        sentences[21 + i] = (geofence.rad[i] >> 8) & 0xFF;
        sentences[22 + i] = (geofence.rad[i] >> 16) & 0xFF;
        sentences[23 + i] = (geofence.rad[i] >> 24) & 0xFF;
        if(log_enabled) Loglib.info("geofence[%d] : (Lat,Lon,Radius) - (%.8f,%.8f,%.2f)", i + 1, geofence.lat[i] * 1e-7, geofence.lon[i] * 1e-7, geofence.rad[i] * 1e-2);
    }

    return requestSendUBX(sentences, sentences[2] + 4);
}

bool ubloxGPS::configMsg(ubx_msg_class_t msgClass, uint8_t msgId, uint8_t rate)
{
    uint8_t sentences[12] = { (uint8_t)UBX_CLASS_CFG, (uint8_t)UBX_CFG_MSG, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    sentences[4] = (uint8_t)msgClass;
    sentences[5] = (uint8_t)msgId;
    for (int i = 6; i < 12; i++) {
        sentences[i] = rate;
    }
    return requestSendUBX(sentences, 12);
}

bool ubloxGPS::disableNMEA(void)
{
    LOCK();
    uint8_t err = 0;
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GGA, 0) ? 0 : 0x01;
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GLL, 0) ? 0 : 0x02;
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GSA, 0) ? 0 : 0x04;
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GSV, 0) ? 0 : 0x08;
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_RMC, 0) ? 0 : 0x10;
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_VTG, 0) ? 0 : 0x20;
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GST, 0) ? 0 : 0x40;
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_ZDA, 0) ? 0 : 0x80;
    if (err != 0) {
        perf_counts.disable_nmea_error_count++;
        if(log_enabled) Loglib.info("disableNMEA error: 0x%02X", err);
    }
    return err == 0 ? true : false;
}

// TODO: It would be helpful to have configurable intervals for specific
// sentences. For example satellite data (GSV) generates a lot of NMEA
// messages but doesn't need to be happening at the same speed as position
// data. For now we will just define a quick/slow interval.
bool ubloxGPS::enableNMEA(uint8_t intervalSec, uint8_t slowIntervalSec)
{
    LOCK();
    uint8_t err = 0;
#ifdef PARSE_NMEA_GPGGA
    if(log_enabled) Loglib.info("enable GGA");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GGA, intervalSec) ? 0 : 0x01;
#else
    if(log_enabled) Loglib.info("disable GGA");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GGA, 0) ? 0 : 0x01;
#endif
#ifdef PARSE_NMEA_GPGLL
    if(log_enabled) Loglib.info("enable GLL");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GLL, intervalSec) ? 0 : 0x02;
#else
    if(log_enabled) Loglib.info("disable GLL");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GLL, 0) ? 0 : 0x02;
#endif
#ifdef PARSE_NMEA_GPGSA
    if(log_enabled) Loglib.info("enable GSA");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GSA, intervalSec) ? 0 : 0x04;
#else
    if(log_enabled) Loglib.info("disable GSA");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GSA, 0) ? 0 : 0x04;
#endif
#ifdef PARSE_NMEA_GPGSV
    if(log_enabled) Loglib.info("enable GSV");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GSV, slowIntervalSec) ? 0 : 0x08;
#else
    if(log_enabled) Loglib.info("disable GSV");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GSV, 0) ? 0 : 0x08;
#endif
#ifdef PARSE_NMEA_GPRMC
    if(log_enabled) Loglib.info("enable RMC");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_RMC, intervalSec) ? 0 : 0x10;
#else
    if(log_enabled) Loglib.info("disable RMC");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_RMC, 0) ? 0 : 0x10;
#endif
#ifdef PARSE_NMEA_GPVTG
    if(log_enabled) Loglib.info("enable VTG");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_VTG, intervalSec) ? 0 : 0x20;
#else
    if(log_enabled) Loglib.info("disable VTG");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_VTG, 0) ? 0 : 0x20;
#endif
#ifdef PARSE_NMEA_GPGST
    if(log_enabled) Loglib.info("enable GST");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GST, intervalSec) ? 0 : 0x40;
#else
    if(log_enabled) Loglib.info("disable GST");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_GST, 0) ? 0 : 0x40;
#endif
#ifdef PARSE_NMEA_GPZDA
    if(log_enabled) Loglib.info("enable ZDA");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_ZDA, intervalSec) ? 0 : 0x80;
#else
    if(log_enabled) Loglib.info("disable ZDA");
    err |= configMsg(UBX_CLASS_NMEA, UBX_NEMA_ZDA, 0) ? 0 : 0x80;
#endif
    if (err != 0) {
        perf_counts.enable_nmea_error_count++;
        if(log_enabled) Loglib.info("enableNMEA error: 0x%02X", err);
    }
    return err == 0 ? true : false;
}

bool ubloxGPS::disablePUBX(void)
{
    LOCK();
    uint8_t err = 0;
    err |= configMsg(UBX_CLASS_PUBX, UBX_PUBX_POSITION, 0) ? 0 : 0x01;
    err |= configMsg(UBX_CLASS_PUBX, UBX_PUBX_SVSTATUS, 0) ? 0 : 0x02;
    err |= configMsg(UBX_CLASS_PUBX, UBX_PUBX_TIME,     0) ? 0 : 0x04;
    if (err != 0) {
        perf_counts.disable_pubx_error_count++;
        if(log_enabled) Loglib.info("disablePUBX error: 0x%02X", err);
    }
    return err == 0 ? true : false;
}

bool ubloxGPS::enablePUBX(uint8_t intervalSec, uint8_t slowIntervalSec)
{
    LOCK();
    uint8_t err = 0;
#ifdef PARSE_PUBX_POSITION
    if(log_enabled) Loglib.info("enable PUBX-POSITION");
    err |= configMsg(UBX_CLASS_PUBX, UBX_PUBX_POSITION, intervalSec) ? 0 : 0x01;
#else
    if(log_enabled) Loglib.info("disable PUBX-POSITION");
    err |= configMsg(UBX_CLASS_PUBX, UBX_PUBX_POSITION, 0) ? 0 : 0x01;
#endif
#ifdef PARSE_PUBX_SVSTATUS
    if(log_enabled) Loglib.info("enable PUBX-SVSTATUS");
    err |= configMsg(UBX_CLASS_PUBX, UBX_PUBX_SVSTATUS, slowIntervalSec) ? 0 : 0x02;
#else
    if(log_enabled) Loglib.info("disable PUBX-SVSTATUS");
    err |= configMsg(UBX_CLASS_PUBX, UBX_PUBX_SVSTATUS, 0) ? 0 : 0x02;
#endif
#ifdef PARSE_PUBX_TIME
    if(log_enabled) Loglib.info("enable PUBX-TIME");
    err |= configMsg(UBX_CLASS_PUBX, UBX_PUBX_TIME, slowIntervalSec) ? 0 : 0x04;
#else
    if(log_enabled) Loglib.info("disable PUBX-TIME");
    err |= configMsg(UBX_CLASS_PUBX, UBX_PUBX_TIME, 0) ? 0 : 0x04;
#endif
    if (err != 0) {
        perf_counts.enable_pubx_error_count++;
        if(log_enabled) Loglib.info("enablePUBX error: 0x%02X", err);
    }
    return err == 0 ? true : false;
}

bool ubloxGPS::disableUBX(void)
{
    LOCK();
    uint8_t err = 0;
    if(log_enabled) Loglib.info("disable NAV TIMEGPS");
    err |= configMsg(UBX_CLASS_NAV, UBX_NAV_TIMEGPS, 0) ? 0 : 0x01;
    if(log_enabled) Loglib.info("disable NAV TIMEUTC");
    err |= configMsg(UBX_CLASS_NAV, UBX_NAV_TIMEUTC, 0) ? 0 : 0x02;
    if(log_enabled) Loglib.info("disable NAV VELNED");
    err |= configMsg(UBX_CLASS_NAV, UBX_NAV_VELNED, 0) ? 0 : 0x04;
    if(log_enabled) Loglib.info("disable NAV POSLLH");
    err |= configMsg(UBX_CLASS_NAV, UBX_NAV_POSLLH, 0) ? 0 : 0x08;
    if(log_enabled) Loglib.info("disable NAV DOP");
    err |= configMsg(UBX_CLASS_NAV, UBX_NAV_DOP, 0) ? 0 : 0x10;
    if (err != 0) {
        perf_counts.disable_ubx_error_count++;
        if(log_enabled) Loglib.info("disableUBX error: 0x%02X", err);
    }
    return err == 0 ? true : false;
}

bool ubloxGPS::requestSendUBX(const uint8_t *sentences, uint16_t len)
{
    uint8_t flags = 0x00;

    if(!sentences || len < 2)
    {
        // minimum len is 2-bytes for the request class/id
        return false;
    }

    // Configuration messages will ack, except for the RST message
    if( (sentences[0] == (uint8_t)UBX_CLASS_CFG) && (sentences[1] != (uint8_t)UBX_CFG_RST) ) {
        flags = UBX_REQ_FLAGS_EXPECT_ACK;
    }
    return requestSendUBX((const ubx_msg_t *) sentences, len, flags);
}

bool ubloxGPS::requestSendUBX(const ubx_msg_t *request,
    uint16_t len,
    uint8_t flags,
    uint8_t rsp_class,
    uint8_t rsp_id)
{
    bool ok = false;

    if(!request || len < 2)
    {
        // minimum len is 2-bytes for the request class/id
        return false;
    }

    ackExpected = false;
    rspExpected = false;
    ackReceived = false;
    nakReceived = false;
    rspReceived = false;
    rxTimeout   = false;

    waitForReqClass = request->msg_class;
    waitForReqId = request->msg_id;

    if(flags & UBX_REQ_FLAGS_EXPECT_ACK)
    {
        ackExpected = true;
    }

    if(rsp_class != UBX_CLASS_INVALID)
    {
        rspExpected = true;
        waitForRspClass = rsp_class;
        waitForRspId = rsp_id;
    }

    // send out ubx command
    if(isInterfaceUart())
    {
        serial->flush();
    }
    if (sendUBX((const uint8_t *) request, len)) {
        lastUbxMsgSent = millis();
        waitForAckOrRsp();
        ok = !rxTimeout;
    }

    return ok;
}

void ubloxGPS::processBytes()
{
    // track if waiting on expected frame on entry
    // will immediately break out on receiving expected frame during parse
    bool _waiting = checkWaitingForAckOrRspFlags();
    bool bytes_available = false;

    if(isInterfaceUart())
    {
        while (serial->available() > 0)
        {
            processGPSByte(serial->read());
            if(_waiting && !checkWaitingForAckOrRspFlags())
            {
                // break out if got an an expected frame to prevent overwrite
                break;
            }
        }

        bytes_available = serial->available();
    }
    else
    {
        // when receiving SPI bytes process in small chunks via a small static
        // buffer to speed up operation and allow resuming parse after breaking
        // out on expected frame
        static uint8_t rx_buf[32];
        static unsigned int rx_buf_offset = sizeof(rx_buf);

        // for each chunk track if received any non 0xFF null bytes to decide
        // if should immediately continue parsing next chunk or break out
        static bool got_data = false;

        do
        {
            if(rx_buf_offset >= sizeof(rx_buf))
            {
                // fully parsed last chunk, receive a new one
                spi->beginTransaction(spi_settings);
                spi_select(true);
                spi->transfer(NULL, rx_buf, sizeof(rx_buf), NULL);
                spi_select(false);
                spi->endTransaction();
                rx_buf_offset = 0;
                got_data = false;
            }

            for(;rx_buf_offset < sizeof(rx_buf); rx_buf_offset++)
            {
                if(rx_buf[rx_buf_offset] != 0xFF)
                {
                    got_data = true;
                }

                processGPSByte(rx_buf[rx_buf_offset]);

                if(_waiting && !checkWaitingForAckOrRspFlags())
                {
                    // break out if got an an expected frame to prevent overwrite
                    goto break_spi;
                }
            }
        } while(got_data);
break_spi: // break out of nested SPI receive
        // possible to break out with data still remaining so assume if we got
        // any bytes off of the SPI port then there might be more
        bytes_available = got_data;
    }

    if(tx_ready_queue && bytes_available)
    {
         // treat as a tx ready event to notify listeners
        txReadyHandler();
    }
}

// wait for additional data either by capturing a tx ready event or fixed delay
// for polling
// will return TRUE if returned based on tx ready event or FALSE otherwise
bool ubloxGPS::yieldThread(uint32_t timeout)
{
    if(tx_ready_queue)
    {
        uint8_t dummy;
        // os_queue_take returns 0 on success
        return !os_queue_take(tx_ready_queue, &dummy, std::min(timeout, UBX_MAX_POLL_INTERVAL_MS), nullptr);
    }
    else
    {
        // short polling delay to play nicely with rest of the system
        // serial uses a very short delay as checking for available bytes is
        // a very quick operation
        // SPI is more resource intensive on a shared bus so use a longer delay
        // (tx ready is highly recommended for SPI usage)
        if(isInterfaceUart())
        {
            delay(1);
        }
        else
        {
            delay(50);
        }
        return false;
    }
}

bool ubloxGPS::checkWaitingForAckOrRspFlags() const
{
    // Return true if ACK or RSP is expected
    // UNLESS we have received an ACK, NAK, RSP, or have timed out (then we return false)
    //
    // In other words, return true until an ACK, NAK, or RSP is received or a timeout occurs,
    // if we are waiting for an ACK or RSP.
    //
    // Returns false if we are not waiting for anything, have received an ACK, NAK, or RSP, or timed out.
    return ((ackExpected || rspExpected) && !(rxTimeout || ackReceived || nakReceived || rspReceived));
}

void ubloxGPS::waitForAckOrRsp()
{
    uint32_t t0 = millis();
    LOCK();

    while(checkWaitingForAckOrRspFlags())
    {
        if(millis() - t0 > UBX_MSG_TIMEOUT)
        {
            rxTimeout = true;
            perf_counts.timeouts++;
            decodeStateHandler = &ubloxGPS::stateSync1;
            if(log_enabled)
            {
                Loglib.info("UBX response timeout");
            }
            break;
        }
        else
        {
            processBytes();

            if(checkWaitingForAckOrRspFlags())
            {
                uint32_t duration = millis() - t0;
                uint32_t timeout = (duration < UBX_MSG_TIMEOUT) ? (UBX_MSG_TIMEOUT - duration) : 0;

                // Note: May be yielded for less than timeout when there is no
                // data ready hardware support
                yieldThread(timeout);
            }
        }

    }
}

const ubx_msg_t *ubloxGPS::waitForAck(uint8_t req_class, uint8_t req_id)
{
    LOCK();

    ackExpected = true;
    rspExpected = false;
    ackReceived = false;
    nakReceived = false;
    rspReceived = false;
    rxTimeout   = false;

    waitForReqClass = req_class;
    waitForReqId = req_id;

    waitForAckOrRsp();

    return rxTimeout ? NULL : (ubx_msg_t *) &ubx_rx_msg;
}

const ubx_msg_t *ubloxGPS::waitForResponse(uint8_t rsp_class, uint8_t rsp_id, uint8_t req_class, uint8_t req_id)
{
    LOCK();

    if(rsp_class == UBX_CLASS_INVALID || rsp_id == UBX_ID_INVALID)
    {
        return NULL;
    }

    ackExpected = false;
    rspExpected = true;
    ackReceived = false;
    nakReceived = false;
    rspReceived = false;
    rxTimeout   = false;

    waitForReqClass = req_class;
    waitForReqId = req_id;
    waitForRspClass = rsp_class;
    waitForRspId = rsp_id;

    waitForAckOrRsp();

    return rxTimeout ? NULL : (ubx_msg_t *) &ubx_rx_msg;
}

const ubx_msg_t *ubloxGPS::getResponse()
{
    if (ackReceived || nakReceived || rspReceived)
    {
        ackReceived = nakReceived = rspReceived = false;
        return (ubx_msg_t *) &ubx_rx_msg;
    }
    else
    {
        return NULL;
    }
}

size_t ubloxGPS::writeBytes(const uint8_t *tx_buf, size_t len)
{
    if(isInterfaceUart())
    {
        return serial->write(tx_buf, len);
    }
    else
    {
        // synchronous SPI means we need an rx buffer to capture bytes from
        // gps module and process as they could be actual data bytes
        // process in chunks
        uint8_t rx_buf[128];
        memset(rx_buf, 0xFF, sizeof(rx_buf));
        for(unsigned int i=0; i < len; i += sizeof(rx_buf))
        {
            int _len = min(len-i, sizeof(rx_buf));
            spi->beginTransaction(spi_settings);
            spi_select(true);
            spi->transfer((void *) (tx_buf + i), rx_buf, _len, NULL);
            spi_select(false);
            spi->endTransaction();
            for(int j=0; j < _len; j++)
            {
                processGPSByte(rx_buf[j]);
            }
        }
        return len;
    }
}

bool ubloxGPS::sendUBX(const uint8_t *sentences, uint16_t len)
{
    uint16_t tx_len = 0;

    if(log_enabled)
    {
        Loglib.trace("TX: ");
        hex_dump(LOG_LEVEL_TRACE, (uint8_t *)sentences, len);
    }
    uint8_t sync[] = {SYNC_1, SYNC_2};
    tx_len += writeBytes(sync, sizeof(sync));

    uint8_t a = 0, b = 0;

    tx_len += writeBytes(sentences, len);

    for(unsigned int i=0; i < len; i++)
    {
        a += sentences[i];
        b += a;
    }

    uint8_t checksum[] = {a, b};
    tx_len += writeBytes(checksum, sizeof(checksum));

    return tx_len == (len + 4); // + 4 for sync and checksum
}

bool ubloxGPS::isACK(const ubx_msg_t *rsp)
{
    return (rsp && rsp->msg_class == UBX_CLASS_ACK && rsp->msg_id == UBX_ACK_ACK);
}

bool ubloxGPS::isNAK(const ubx_msg_t *rsp)
{
    return (rsp && rsp->msg_class == UBX_CLASS_ACK && rsp->msg_id == UBX_ACK_NAK);
}

String ubloxGPS::deg2DMS(float degree)
{
    int   D = int(degree);
    int   M = int((degree - D) * 60);
    float S = (((degree - D) * 60) - M) * 60;
    if (M < 0)
        M = -M;
    if (S < 0)
        S = -S;

    String DMS = String::format("%d°%02d\'%0.4f\"", D, M, S);
    return DMS;
}

float ubloxGPS::DMS2deg(String DMS)
{
    String ARG[3];
    float d, m, s, degree = 0;
    bool err = true;

    // parse String into CMD, ARG1, ARG2
    if (DMS.indexOf("°") != -1) {
        ARG[0] = DMS.substring(0, DMS.indexOf("°"));
        String MS = DMS.remove(0, ARG[0].length() + 2);
        if (MS.indexOf('\'') != -1) {
            ARG[1] = MS.substring(0, MS.indexOf('\''));
            String S = MS.remove(0, ARG[1].length() + 1);
            if (S.indexOf('\"') != -1) {
                ARG[2] = S.substring(0, S.indexOf('\"'));
                err = false;
            }
        }
    }

    if (!err) {
        d = ARG[0].trim().toInt();
        m = ARG[1].trim().toInt();
        if (ARG[2].indexOf('.') != -1)
            s = ARG[2].trim().toFloat();
        else
            s = ARG[2].trim().toInt();

        if (d < 0) {
            degree = d - m / 60.0 - s / 3600.0;
        } else {
            degree = d + m / 60.0 + s / 3600.0;
        }
    }

    return degree;
}

void ubloxGPS::hex_dump(LogLevel level, uint8_t *data, int len, Logger *logger)
{
    if(!logger)
    {
        logger = &Loglib;
    }
    logger->print(level, "\t");
    logger->dump(level, data, len);
    logger->print(level, "\n\r");
}

bool ubloxGPS::createLog(void)
{
    ubx_log_create_t msg = {
        UBX_CLASS_LOG,
        UBX_LOG_CREATE,
        sizeof(ubx_log_create_t) - offsetof(ubx_msg_t, payload),
        0,
    };
    const ubx_msg_t *rsp = NULL;
    LOCK();

    // TODO: Hardcode some log parameters for now, make configurable later?
    msg.config = UBX_LOG_CREATE_CONFIG_CIRCULAR_BIT;
    msg.size = UBX_LOG_CREATE_SIZE_MAX;
    msg.user_size = 0; // not used with size max
    if(requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg), UBX_REQ_FLAGS_EXPECT_ACK))
    {
        rsp = getResponse();
    }

    return isACK(rsp);
}

bool ubloxGPS::eraseLog(void)
{
    ubx_log_erase_t msg = {
        UBX_CLASS_LOG,
        UBX_LOG_ERASE,
        0,
    };
    const ubx_msg_t *rsp = NULL;
    LOCK();

    if(requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg), UBX_REQ_FLAGS_EXPECT_ACK))
    {
        rsp = getResponse();
    }

    return isACK(rsp);
}

bool ubloxGPS::configLog(uint16_t min_interval, uint16_t time_threshold, uint16_t speed_threshold, uint32_t position_threshold, bool start)
{
    ubx_cfg_log_filter_t msg = {
        UBX_CLASS_CFG,
        UBX_CFG_LOGFILTER,
        sizeof(ubx_cfg_log_filter_t) - offsetof(ubx_msg_t, payload),
    };
    const ubx_msg_t *rsp = NULL;
    LOCK();

    // TODO: Hardcode some log parameters for now, make configurable later?
    msg.version = 1;
    msg.min_interval = min_interval;
    msg.time_threshold = time_threshold;
    msg.speed_threshold = speed_threshold;
    msg.position_threshold = position_threshold;
    msg.flags = UBX_CFG_LOG_FILTER_FLAGS_APPLY_ALL_BIT;
    if(start)
    {
        msg.flags |= UBX_CFG_LOG_FILTER_FLAGS_RECORD_BIT;
    }
    if(requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg), UBX_REQ_FLAGS_EXPECT_ACK))
    {
        rsp = getResponse();
    }

    return isACK(rsp);
}

bool ubloxGPS::startLog(void)
{
    ubx_cfg_log_filter_t msg = {
        UBX_CLASS_CFG,
        UBX_CFG_LOGFILTER,
        sizeof(ubx_cfg_log_filter_t) - offsetof(ubx_msg_t, payload),
    };
    const ubx_msg_t *rsp = NULL;
    LOCK();

    // TODO: Hardcode some log parameters for now, make configurable later?
    msg.version = 1;
    msg.flags = UBX_CFG_LOG_FILTER_FLAGS_RECORD_BIT;

    if(requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg), UBX_REQ_FLAGS_EXPECT_ACK))
    {
        rsp = getResponse();
    }

    return isACK(rsp);
}

bool ubloxGPS::pauseLog(void)
{
    ubx_cfg_log_filter_t msg = {
        UBX_CLASS_CFG,
        UBX_CFG_LOGFILTER,
        sizeof(ubx_cfg_log_filter_t) - offsetof(ubx_msg_t, payload),
    };
    const ubx_msg_t *rsp = NULL;
    LOCK();

    // TODO: Hardcode some log parameters for now, make configurable later?
    msg.version = 1;
    msg.flags = 0x00;

    if(requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg), UBX_REQ_FLAGS_EXPECT_ACK))
    {
        rsp = getResponse();
    }

    return isACK(rsp);
}

bool ubloxGPS::getLogInfo(ubx_log_info_rsp_t *info)
{
    ubx_log_info_t msg = {
        UBX_CLASS_LOG,
        UBX_LOG_INFO,
        sizeof(ubx_log_info_t) - offsetof(ubx_msg_t, payload),
    };
    const ubx_msg_t *rsp = NULL;
    LOCK();

    if(!info)
    {
        return false;
    }

    if(requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg), 0x00, UBX_CLASS_LOG, UBX_LOG_INFO))
    {
        rsp = getResponse();
    }

    if(rsp && rsp->msg_class == UBX_CLASS_LOG && rsp->msg_id == UBX_LOG_INFO)
    {
        memcpy(info, rsp, sizeof(*info));
        return true;
    }
    else
    {
        return false;
    }
}

bool ubloxGPS::addLogString(uint8_t *bytes, uint16_t length)
{
    union {
        ubx_log_string_t header = {
            UBX_CLASS_LOG,
            UBX_LOG_STRING,
        };
        struct {
            uint8_t dummy[sizeof(ubx_log_string_t)];
            char payload[UBX_LOG_STRING_MAX_LEN];
        };
    } __attribute__((packed)) msg;
    LOCK();

    if(length > UBX_LOG_STRING_MAX_LEN)
    {
        return false;
    }

    msg.header.length = length;
    memcpy(msg.payload, bytes, length);
    requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg.header) + length, 0x00);
    return (bool) (waitForAck(UBX_CLASS_LOG, UBX_LOG_STRING) != NULL);
}

const ubx_msg_t *ubloxGPS::getLogEntry(uint32_t start)
{
    ubx_log_retrieve_t msg = {
        UBX_CLASS_LOG,
        UBX_LOG_RETRIEVE,
        sizeof(ubx_log_retrieve_t) - offsetof(ubx_msg_t, payload),
    };
    const ubx_msg_t *rsp = NULL;
    LOCK();

    msg.start_number = start;
    msg.entry_count = 1;
    msg.version = 0;

    // we can get multiple types of log entries so accept ANY message from the
    // logging class
    // TODO: Possible a non-entry logging message might show up, maybe from a
    // previous command that failed to retrieve its response. Should continue
    // trying to get a log entry until an overall timeout occurs even if the
    // first response isn't an entry;
    if(requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg), 0x00, UBX_CLASS_LOG))
    {
        rsp = getResponse();
    }

    if(rsp && rsp->msg_class == UBX_CLASS_LOG &&
        (rsp->msg_id == UBX_LOG_RETRIEVEPOS ||
        rsp->msg_id == UBX_LOG_RETRIEVESTRING ||
        rsp->msg_id == UBX_LOG_RETRIEVEPOSEXTRA))
    {
        return rsp;
    }
    else
    {
        return NULL;
    }
}

// prepares for a new write MGA sequence OR returns an appropriate error
bool ubloxGPS::startWriteMGA()
{
    LOCK();
    if(write_mga_active)
    {
        return false;
    }
    write_mga_active = true;
    write_mga_sequence = 0;
    return true;
}

// finishes an active write MGA sequence OR returns an appropriate error
bool ubloxGPS::stopWriteMGA()
{
    ubx_mga_flash_stop_t msg {
        UBX_CLASS_MGA,
        UBX_MGA_FLASH_DATA,
        sizeof(ubx_mga_flash_stop_t) - offsetof(ubx_msg_t, payload),
    };
    const ubx_mga_flash_ack_t *rsp = NULL;
    LOCK();

    msg.msg_type = UBX_MGA_FLASH_DATA_STOP;
    msg.version = 0x00;

    if(requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg), 0x00, UBX_CLASS_MGA, UBX_MGA_FLASH_DATA))
    {
        rsp = (const ubx_mga_flash_ack_t *) getResponse();
    }

    if(rsp &&
        rsp->msg_class == UBX_CLASS_MGA &&
        rsp->msg_id == UBX_MGA_FLASH_DATA &&
        rsp->msg_type == UBX_MGA_FLASH_DATA_ACK &&
        rsp->ack == UBX_MGA_FLASH_DATA_ACK_OK)
    {
        write_mga_active = false;
        write_mga_sequence = 0;
        return true;
    }
    else
    {
        return false;
    }
}

// unconditionally aborts an active write MGA sequence (no ubx messaging)
void ubloxGPS::abortWriteMGA()
{
    LOCK();
    write_mga_active = false;
    write_mga_sequence = 0;
}

// write an MGA flash block
// max size is 512 bytes
ubx_mga_flash_ack_type_t ubloxGPS::writeMGA(uint8_t *bytes, uint16_t length)
{
    union {
        ubx_mga_flash_write_t header = {
            UBX_CLASS_MGA,
            UBX_MGA_FLASH_DATA,
        };
        struct {
            uint8_t dummy[sizeof(ubx_mga_flash_write_t)];
            char payload[UBX_MGA_FLASH_DATA_MAX_LEN];
        };
    } __attribute__((packed)) msg;
    const ubx_mga_flash_ack_t *rsp = NULL;
    LOCK();

    if(length > sizeof(msg.payload))
    {
        return UBX_MGA_FLASH_DATA_ACK_INVALID;
    }

    // overall message length
    msg.header.length = length + offsetof(ubx_mga_flash_write_t, data) - offsetof(ubx_msg_t, payload);
    msg.header.msg_type = UBX_MGA_FLASH_DATA_WRITE;
    msg.header.version = 0x00;
    msg.header.sequence = write_mga_sequence;
    // payload size
    msg.header.size = length;
    memcpy(msg.payload, bytes, length);
    if(requestSendUBX((const ubx_msg_t *) &msg, sizeof(msg.header) + length, 0x00, UBX_CLASS_MGA, UBX_MGA_FLASH_DATA))
    {
        rsp = (const ubx_mga_flash_ack_t *) getResponse();
    }

    if(rsp &&
        rsp->msg_class == UBX_CLASS_MGA &&
        rsp->msg_id == UBX_MGA_FLASH_DATA &&
        rsp->msg_type == UBX_MGA_FLASH_DATA_ACK)
    {
        if(rsp->ack == UBX_MGA_FLASH_DATA_ACK_OK)
        {
            write_mga_sequence += 1;
        }
        return (ubx_mga_flash_ack_type_t) rsp->ack;
    }

    return UBX_MGA_FLASH_DATA_ACK_TIMEOUT;
}
