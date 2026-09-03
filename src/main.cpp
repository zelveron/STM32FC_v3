/**
 * STM32F407VET6 - BMP581 + BMI323 real-time streamer
 *
 *   BMP581 : I2C1  PB6=SCL, PB7=SDA @0x47  (pressure / temp / altitude)
 *   BMI323 : SPI1  PA5=SCK, PA6=MISO, PA7=MOSI, PA4=CS (accel / gyro, bit-bang)
 *   uBlox  : USART1 PA9=TX, PA10=RX (NMEA, crossed wiring)
 *
 * No sensor has an enable/power pin -- all sensors are always on.
 *
 * Output over USB CDC ("SerialUSB"), tagged CSV lines:
 *   BMP,<pressure_hPa>,<temperature_C>,<altitude_m>
 *   BMI,<acc_x>,<acc_y>,<acc_z>,<gyr_x>,<gyr_y>,<gyr_z>   (g, deg/s)
 *   GPS,<lat>,<lon>,<alt_m>,<sats>,<fix>
 *   ATT,<roll_deg>,<pitch_deg>,<yaw_deg>
 *   SD_STATUS,1|<logfile>  (0 = SD init/open failed)
 *   BMI_STATUS,0|1       (0 = not detected, 1 = detected)
 *
 * Sensor data is also logged to the SD card (SDIO, FatFs) as FLTxxxxx.CSV.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "bmp5.h"
#include "bmp5_defs.h"
#include "bmi323.h"
#include <STM32SD.h>

#define BMI_CS     PA4
#define BMI_SCK    PA5
#define BMI_MISO   PA6
#define BMI_MOSI   PA7

static const uint8_t BMP5_ADDR = 0x47;

/* ------------------------------------------------------------------------- */
/* Non-blocking line logger over USB CDC                                     */
/*                                                                          */
/* SerialUSB.write() busy-waits up to USB_CDC_TRANSMIT_TIMEOUT (3 ms) per    */
/* call when the host is connected but not draining, and can emit half      */
/* lines. This buffers one line and pushes it only if it fits the CDC TX    */
/* queue whole; otherwise the line is dropped and counted. Never blocks,    */
/* never emits a partial line.                                              */
/*                                                                          */
/* Bench scaffolding only -- deleted when the USB path is removed. The CDC  */
/* TX queue is enlarged via -D CDC_TRANSMIT_QUEUE_BUFFER_PACKET_NUMBER so   */
/* the longest debug line still fits in one atomic push.                    */
/* ------------------------------------------------------------------------- */
class NbLog : public Print
{
public:
    size_t write(uint8_t c) override
    {
        if (c == '\n') { flush_line(); return 1; }
        if (_len < sizeof(_buf)) _buf[_len++] = (char)c;
        else                     _overflow = true;
        return 1;
    }
    size_t write(const uint8_t *b, size_t n) override
    {
        for (size_t i = 0; i < n; i++) write(b[i]);
        return n;
    }
    uint32_t drops() const { return _drops; }

private:
    void flush_line(void)
    {
        if (!_overflow && SerialUSB.availableForWrite() >= (int)(_len + 1))
        {
            SerialUSB.write((const uint8_t *)_buf, _len);
            SerialUSB.write((uint8_t)'\n');
        }
        else
        {
            _drops++;
        }
        _len = 0;
        _overflow = false;
    }

    char     _buf[288];
    uint16_t _len = 0;
    bool     _overflow = false;
    uint32_t _drops = 0;
};

static NbLog Log;

/* ------------------------------------------------------------------------- */
/* BMP581 (I2C)                                                              */
/* ------------------------------------------------------------------------- */

static int8_t bmp5_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;
    Wire.beginTransmission(BMP5_ADDR);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) return BMP5_E_COM_FAIL;
    if (Wire.requestFrom((uint8_t)BMP5_ADDR, (uint8_t)len) != len) return BMP5_E_COM_FAIL;
    for (uint32_t i = 0; i < len; i++) read_data[i] = Wire.read();
    return BMP5_INTF_RET_SUCCESS;
}

static int8_t bmp5_i2c_write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;
    Wire.beginTransmission(BMP5_ADDR);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) Wire.write(write_data[i]);
    if (Wire.endTransmission() != 0) return BMP5_E_COM_FAIL;
    return BMP5_INTF_RET_SUCCESS;
}

static void bmp5_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    delayMicroseconds(period);
}

static struct bmp5_dev bmp_dev;
static struct bmp5_osr_odr_press_config bmp_cfg;
static bool bmp_ok = false;

/* Worst-case BMP581 conversion time for 1x temp + 16x pressure oversampling,
   with margin. Used to space the forced-mode trigger from the result read so
   neither blocks the loop. */
static const uint32_t BMP_CONV_US = 40000;

static bool bmp5_begin(void)
{
    memset(&bmp_dev, 0, sizeof(bmp_dev));
    bmp_dev.intf     = BMP5_I2C_INTF;
    bmp_dev.read     = bmp5_i2c_read;
    bmp_dev.write    = bmp5_i2c_write;
    bmp_dev.delay_us = bmp5_delay_us;

    if (bmp5_init(&bmp_dev) != BMP5_OK) return false;

    bmp_cfg.osr_t    = BMP5_OVERSAMPLING_1X;
    bmp_cfg.osr_p    = BMP5_OVERSAMPLING_16X;
    bmp_cfg.press_en = BMP5_ENABLE;
    bmp_cfg.odr      = BMP5_ODR_100_2_HZ;   /* unused in forced mode */

    if (bmp5_set_osr_odr_press_config(&bmp_cfg, &bmp_dev) != BMP5_OK) return false;
    return true;
}

/* ------------------------------------------------------------------------- */
/* BMI323 (SPI)                                                              */
/* ------------------------------------------------------------------------- */

/* Bit-bang one byte over SPI, mode 0 (sample MISO on rising SCK). */
static uint8_t bmi_bb_xfer(uint8_t out)
{
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--)
    {
        digitalWrite(BMI_MOSI, (out >> i) & 1);
        digitalWrite(BMI_SCK, HIGH);
        delayMicroseconds(2);
        if (digitalRead(BMI_MISO)) in |= (1 << i);
        digitalWrite(BMI_SCK, LOW);
        delayMicroseconds(2);
    }
    return in;
}

static int8_t bmi3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;
    if (length == 0) return BMI3_INTF_RET_SUCCESS;

    digitalWrite(BMI_CS, LOW);
    delayMicroseconds(1);
    bmi_bb_xfer(reg_addr);          /* driver already set the 0x80 read bit */
    for (uint32_t i = 0; i < length; i++) reg_data[i] = bmi_bb_xfer(0x00);
    delayMicroseconds(1);
    digitalWrite(BMI_CS, HIGH);
    return BMI3_INTF_RET_SUCCESS;
}

static int8_t bmi3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;
    if (length == 0) return BMI3_INTF_RET_SUCCESS;

    digitalWrite(BMI_CS, LOW);
    delayMicroseconds(1);
    bmi_bb_xfer(reg_addr);          /* driver already applied the 0x7F write mask */
    for (uint32_t i = 0; i < length; i++) bmi_bb_xfer(reg_data[i]);
    delayMicroseconds(1);
    digitalWrite(BMI_CS, HIGH);
    return BMI3_INTF_RET_SUCCESS;
}

static void bmi3_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    delayMicroseconds(period);
}

static struct bmi3_dev bmi_dev;
static bool bmi_ready = false;

/* Raw single-transaction CHIP_ID read (no config upload) for diagnostics. */
static uint8_t bmi_raw_chip_id(void)
{
    uint8_t b[2] = { 0, 0 };
    digitalWrite(BMI_CS, LOW);
    delayMicroseconds(1);
    bmi_bb_xfer(0x80);          /* read register 0x00 */
    b[0] = bmi_bb_xfer(0x00);   /* dummy byte */
    b[1] = bmi_bb_xfer(0x00);   /* chip_id (0x43 for BMI323) */
    delayMicroseconds(1);
    digitalWrite(BMI_CS, HIGH);
    return b[1];
}

/* Configured full-scale ranges. These two constants are the single source of
   truth: the raw->physical scale factors below are derived from them, so the
   streaming-loop conversion can never fall out of sync with the sensor config
   (e.g. when the accel range moves to +/-8 g per the Phase 0 target).
     accel FS = 2^(range+1) g            (2G=0 .. 16G=3)
     gyro  FS = 2000 / 2^(2000dps-range) (125dps=0 .. 2000dps=4) */
static constexpr uint8_t BMI_ACC_RANGE = BMI3_ACC_RANGE_4G;
static constexpr uint8_t BMI_GYR_RANGE = BMI3_GYR_RANGE_2000DPS;

static constexpr float BMI_ACC_LSB_G =
    (float)(2u << BMI_ACC_RANGE) / 32768.0f;
static constexpr float BMI_GYR_LSB_DPS =
    (2000.0f / (float)(1u << (BMI3_GYR_RANGE_2000DPS - BMI_GYR_RANGE))) / 32768.0f;

static bool bmi323_begin(void)
{
    memset(&bmi_dev, 0, sizeof(bmi_dev));
    bmi_dev.intf           = BMI3_SPI_INTF;
    bmi_dev.read           = bmi3_spi_read;
    bmi_dev.write          = bmi3_spi_write;
    bmi_dev.delay_us       = bmi3_delay_us;
    bmi_dev.read_write_len = 8;

    if (bmi323_init(&bmi_dev) != BMI3_OK) return false;

    struct bmi3_sens_config cfg[2];
    cfg[0].type = BMI323_ACCEL;
    cfg[1].type = BMI323_GYRO;
    if (bmi323_get_sensor_config(cfg, 2, &bmi_dev) != BMI3_OK) return false;

    cfg[0].cfg.acc.odr      = BMI3_ACC_ODR_200HZ;
    cfg[0].cfg.acc.range    = BMI_ACC_RANGE;
    cfg[0].cfg.acc.bwp      = BMI3_ACC_BW_ODR_QUARTER;
    cfg[0].cfg.acc.avg_num  = BMI3_ACC_AVG4;
    cfg[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    cfg[1].cfg.gyr.odr      = BMI3_GYR_ODR_200HZ;
    cfg[1].cfg.gyr.range    = BMI_GYR_RANGE;
    cfg[1].cfg.gyr.bwp      = BMI3_GYR_BW_ODR_HALF;
    cfg[1].cfg.gyr.avg_num  = BMI3_GYR_AVG1;
    cfg[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

    if (bmi323_set_sensor_config(cfg, 2, &bmi_dev) != BMI3_OK) return false;

    return true;
}

/* ------------------------------------------------------------------------- */
/* Attitude estimate (complementary filter — ArduPilot DCM/EKF foundation)   */
/* ------------------------------------------------------------------------- */

/* Attitude state, radians, body FRD (see CLAUDE.md conventions).
   NOT flight-grade: fixed blend gain, no accel gating, no gyro bias
   calibration, unreferenced drifting yaw. Phase 1 replaces this. */
static float att_roll_rad = 0.0f, att_pitch_rad = 0.0f, att_yaw_rad = 0.0f;
static bool  att_init = false;

static inline float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* Complementary filter. Inputs: accel in g, gyro in deg/s, timestep in seconds
   (the caller owns the clock). State is radians. */
static void att_update(float ax_g, float ay_g, float az_g,
                       float gx_dps, float gy_dps, float gz_dps,
                       float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f;

    /* Roll / pitch from the gravity vector (valid in non-accelerating flight). */
    float acc_roll  = atan2f(ay_g, az_g);
    float acc_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

    if (!att_init)
    {
        att_roll_rad  = acc_roll;
        att_pitch_rad = acc_pitch;
        att_yaw_rad   = 0.0f;
        att_init      = true;
        return;
    }

    /* Gyro body rates -> Euler angle rates (rad/s). */
    float p = gx_dps * DEG_TO_RAD;
    float q = gy_dps * DEG_TO_RAD;
    float r = gz_dps * DEG_TO_RAD;

    float sp = sinf(att_roll_rad);
    float cp = cosf(att_roll_rad);
    float tt = tanf(att_pitch_rad);
    float ct = cosf(att_pitch_rad);
    if (fabsf(ct) < 0.1f) ct = (ct < 0.0f) ? -0.1f : 0.1f;

    float phi_dot   = p + sp * tt * q + cp * tt * r;
    float theta_dot = cp * q - sp * r;
    float psi_dot   = (sp / ct) * q + (cp / ct) * r;

    /* Blend gyro integration (fast, drifts) with the accel reference (noisy,
       no drift). alpha = 0.98 -> ~0.5 s time constant at 100 Hz. */
    const float alpha = 0.98f;
    att_roll_rad  = alpha * (att_roll_rad  + phi_dot   * dt_s) + (1.0f - alpha) * acc_roll;
    att_pitch_rad = alpha * (att_pitch_rad + theta_dot * dt_s) + (1.0f - alpha) * acc_pitch;
    att_yaw_rad   = wrap_pi(att_yaw_rad + psi_dot * dt_s);
}

/* ------------------------------------------------------------------------- */
/* uBlox GNSS (USART1, NMEA)                                                 */
/* ------------------------------------------------------------------------- */

#define GPS_BAUD 9600

static const uint32_t GPS_BAUDS[] = { 9600, 38400, 115200, 4800, 57600, 19200, 230400, 460800, 921600 };
static uint8_t  gps_baud_idx   = 0;
static uint32_t gps_last_switch = 0;
static uint32_t gps_rx_bytes   = 0;
static bool     gps_locked     = false;
static bool     gps_nmea_valid = false;
static uint8_t  gps_first[160];
static uint16_t gps_first_len  = 0;
static int   gps_sats      = 0;
static int   gps_fix       = 0;
static char  gps_time_str[12] = "";
static float gps_speed_kmh = 0.0f;

/* --- UBX (u-blox binary) helpers for active probing ---------------------- */

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t ck_a = 0, ck_b = 0;
    Serial1.write(0xB5);
    Serial1.write(0x62);
    Serial1.write(cls);  ck_a += cls;              ck_b += ck_a;
    Serial1.write(id);   ck_a += id;               ck_b += ck_a;
    Serial1.write(len & 0xFF);        ck_a += (uint8_t)(len & 0xFF);         ck_b += ck_a;
    Serial1.write((len >> 8) & 0xFF); ck_a += (uint8_t)((len >> 8) & 0xFF);  ck_b += ck_a;
    for (uint16_t i = 0; i < len; i++)
    {
        Serial1.write(payload[i]);
        ck_a += payload[i];
        ck_b += ck_a;
    }
    Serial1.write(ck_a);
    Serial1.write(ck_b);
}

/* UBX-MON-VER poll: any u-blox module answers regardless of output config. */
static void ubx_poll_monver(void)
{
    ubx_send(0x0A, 0x04, NULL, 0);
}

/* UBX-CFG-PRT: configure UART1 -> NMEA + UBX out at 9600 8N1. */
static void ubx_enable_nmea(void)
{
    static const uint8_t prt[20] = {
        0x01, 0x00, 0x00, 0x00,       /* portID = UART1, reserved, txReady */
        0xD0, 0x08, 0x00, 0x00,       /* mode: 8N1 */
        0x80, 0x25, 0x00, 0x00,       /* baudRate: 9600 */
        0x03, 0x00,                   /* inProtoMask: UBX | NMEA */
        0x03, 0x00,                   /* outProtoMask: UBX | NMEA */
        0x00, 0x00, 0x00, 0x00        /* flags, reserved */
    };
    ubx_send(0x06, 0x00, prt, sizeof(prt));
}

/* Drain USART1 RX into the boot-burst capture buffer. */
static void gps_drain(void)
{
    while (Serial1.available())
    {
        uint8_t c = (uint8_t)Serial1.read();
        gps_rx_bytes++;
        if (gps_first_len < sizeof(gps_first)) gps_first[gps_first_len++] = c;
    }
}

/* Start USART1 with the correct (crossed) uBlox wiring:
   PA9 = TX (MCU -> uBlox RX), PA10 = RX (uBlox TX -> MCU). */
static void gps_uart_begin(uint32_t baud)
{
    Serial1.end();
    Serial1.setTx(PA9);
    Serial1.setRx(PA10);
    Serial1.begin(baud);
}

static char gps_line[128];
static uint8_t gps_len = 0;
static char gps_last_nmea[96];

/* Split a comma-separated NMEA field list, preserving empty fields. */
static int gps_split(char *s, char *f[], int max)
{
    int n = 0;
    f[n++] = s;
    for (char *p = s; *p && n < max; p++)
    {
        if (*p == ',')
        {
            *p = '\0';
            f[n++] = p + 1;
        }
    }
    return n;
}

/* Validate the trailing *XX checksum of any NMEA sentence. */
static bool gps_valid_nmea(const char *line)
{
    int len = (int)strlen(line);
    if (line[0] != '$' || len < 8 || line[len - 3] != '*') return false;
    uint8_t cs = 0;
    for (int i = 1; i < len - 3; i++) cs ^= (uint8_t)line[i];
    uint8_t cs_hex = (uint8_t)strtol(line + len - 2, NULL, 16);
    return cs == cs_hex;
}

/* Convert NMEA time "HHMMSS.ss" into "HH:MM:SS". */
static void gps_store_time(const char *nmea_time)
{
    if (nmea_time[0] == '\0' || strlen(nmea_time) < 6) return;
    gps_time_str[0] = nmea_time[0];
    gps_time_str[1] = nmea_time[1];
    gps_time_str[2] = ':';
    gps_time_str[3] = nmea_time[2];
    gps_time_str[4] = nmea_time[3];
    gps_time_str[5] = ':';
    gps_time_str[6] = nmea_time[4];
    gps_time_str[7] = nmea_time[5];
    gps_time_str[8] = '\0';
}

/* Parse $GxRMC: UTC time + speed over ground (knots -> km/h). */
static bool gps_process_rmc(char *line)
{
    int len = (int)strlen(line);
    if (line[0] != '$' || line[3] != 'R' || line[4] != 'M' || line[5] != 'C')
        return false;
    if (len < 10 || line[len - 3] != '*') return false;
    uint8_t cs = 0;
    for (int i = 1; i < len - 3; i++) cs ^= (uint8_t)line[i];
    uint8_t cs_hex = (uint8_t)strtol(line + len - 2, NULL, 16);
    if (cs != cs_hex) return false;

    line[len - 3] = '\0';
    char *f[13];
    int n = gps_split(line, f, 13);
    if (n < 8) return false;

    gps_store_time(f[1]);
    if (f[7][0] != '\0') gps_speed_kmh = atof(f[7]) * 1.852f;  /* knots -> km/h */
    return true;
}

static bool gps_process_line(char *line)
{
    int len = (int)strlen(line);

    /* Match NMEA GGA sentences ($GPGGA / $GNGGA / $GLGGA / ...). */
    if (line[0] != '$' || line[3] != 'G' || line[4] != 'G' || line[5] != 'A')
        return false;

    /* Validate the trailing *XX checksum. */
    if (len < 10 || line[len - 3] != '*') return false;
    uint8_t cs = 0;
    for (int i = 1; i < len - 3; i++) cs ^= (uint8_t)line[i];
    uint8_t cs_hex = (uint8_t)strtol(line + len - 2, NULL, 16);
    if (cs != cs_hex) return false;

    line[len - 3] = '\0';            /* drop checksum before splitting */

    char *f[15];
    int n = gps_split(line, f, 15);

    /* Need fields through altitude unit (index 10). */
    if (n < 11) return false;

    int sats = atoi(f[7]);
    int fix  = atoi(f[6]);
    gps_sats = sats;
    gps_fix  = fix;
    gps_store_time(f[1]);

    /* Report fix quality + satellite count once per second, even with no
       position (lets us watch satellite acquisition). */
    static uint32_t last_gps_stat = 0;
    if ((millis() - last_gps_stat) >= 1000)
    {
        last_gps_stat = millis();
        Log.print(F("GPS_STAT,"));
        Log.print(fix);
        Log.print(',');
        Log.print(sats);
        Log.print(',');
        Log.print(gps_time_str);
        Log.print(',');
        Log.println(gps_speed_kmh, 1);
    }

    if (f[2][0] == '\0' || f[4][0] == '\0') return false;

    float lat_raw = atof(f[2]);
    float lat = (int)(lat_raw / 100.0f) + (lat_raw - (int)(lat_raw / 100.0f) * 100.0f) / 60.0f;
    if (f[3][0] == 'S') lat = -lat;

    float lon_raw = atof(f[4]);
    float lon = (int)(lon_raw / 100.0f) + (lon_raw - (int)(lon_raw / 100.0f) * 100.0f) / 60.0f;
    if (f[5][0] == 'W') lon = -lon;

    float gps_alt = atof(f[9]);

    Log.print(F("GPS,"));
    Log.print(lat, 6);
    Log.print(',');
    Log.print(lon, 6);
    Log.print(',');
    Log.print(gps_alt, 1);
    Log.print(',');
    Log.print(sats);
    Log.print(',');
    Log.print(fix);
    Log.print(',');
    Log.print(gps_time_str);
    Log.print(',');
    Log.println(gps_speed_kmh, 1);
    return true;
}

/* ------------------------------------------------------------------------- */
/* Data logging (SD card over SDIO, FatFs)                                   */
/* ------------------------------------------------------------------------- */

static float g_ax = 0, g_ay = 0, g_az = 0, g_gx = 0, g_gy = 0, g_gz = 0;
static float g_press = 0, g_temp = 0, g_alt = 0;

static File sd_file;
static bool sd_ok = false;
static char sd_log_name[16];

/* ------------------------------------------------------------------------- */
/* setup / loop                                                              */
/* ------------------------------------------------------------------------- */

void setup(void)
{
    pinMode(BMI_CS, OUTPUT);   digitalWrite(BMI_CS, HIGH);
    pinMode(BMI_SCK, OUTPUT);  digitalWrite(BMI_SCK, LOW);
    pinMode(BMI_MOSI, OUTPUT); digitalWrite(BMI_MOSI, LOW);
    pinMode(BMI_MISO, INPUT_PULLUP);

    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();

    SerialUSB.begin();
    gps_uart_begin(GPS_BAUD);

    /* Wait (bounded) for the USB host, draining uBlox boot burst meanwhile. */
    for (int i = 0; i < 300 && !SerialUSB; i++) { gps_drain(); delay(10); }
    delay(100);
    gps_drain();
    Log.println(F("boot: BMP581 + BMI323 + uBlox + SD streamer"));

    /* BMP581 is initialised once here, not retried from loop(). */
    bmp_ok = bmp5_begin();
    Log.println(bmp_ok ? F("BMP_STATUS,1") : F("BMP_STATUS,0"));
    gps_drain();

    /* --- SD card (SDIO 4-bit) + log file -------------------------------- */
    sd_ok = SD.begin();
    if (sd_ok)
    {
        int n = 0;
        do { snprintf(sd_log_name, sizeof(sd_log_name), "FLT%05d.CSV", n++); }
        while (SD.exists(sd_log_name));

        sd_file = SD.open(sd_log_name, FILE_WRITE);
        if (sd_file)
        {
            sd_file.println(F("t_ms,ax,ay,az,gx,gy,gz,roll,pitch,yaw,press_hPa,temp_c,alt_m,gps_time,gps_sats,gps_speed_kmh"));
            sd_file.flush();
            Log.print(F("SD_STATUS,1,"));
            Log.println(sd_log_name);
        }
        else
        {
            sd_ok = false;
            Log.println(F("SD_STATUS,0,open_failed"));
        }
    }
    else
    {
        Log.println(F("SD_STATUS,0,begin_failed"));
    }
}

void loop(void)
{
    /* --- uBlox GNSS: read NMEA over USART1 ------------------------------- */
    while (Serial1.available())
    {
        char c = (char)Serial1.read();
        gps_rx_bytes++;
        if (gps_first_len < sizeof(gps_first)) gps_first[gps_first_len++] = (uint8_t)c;
        if (c == '\n')
        {
            gps_line[gps_len] = '\0';
            if (gps_valid_nmea(gps_line))
            {
                gps_nmea_valid = true;
                strncpy(gps_last_nmea, gps_line, sizeof(gps_last_nmea) - 1);
                gps_last_nmea[sizeof(gps_last_nmea) - 1] = '\0';
            }
            if (gps_process_line(gps_line)) gps_locked = true;
            gps_process_rmc(gps_line);
            gps_len = 0;
        }
        else if (c != '\r' && gps_len < (sizeof(gps_line) - 1))
        {
            gps_line[gps_len++] = c;
        }
    }

    /* Baud auto-detect: cycle until we see a valid NMEA sentence. */
    if (!gps_nmea_valid && (millis() - gps_last_switch) >= 2000)
    {
        gps_last_switch = millis();
        gps_baud_idx = (uint8_t)((gps_baud_idx + 1) % (sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0])));
        gps_uart_begin(GPS_BAUDS[gps_baud_idx]);
    }

    /* Active u-blox probe: poll version + enable NMEA every ~2 s. */
    static uint32_t last_ubx = 0;
    if ((millis() - last_ubx) >= 2000)
    {
        last_ubx = millis();
        ubx_poll_monver();
        ubx_enable_nmea();
    }

    /* Debug: report rx byte count / baud / lock state every 2 s. */
    static uint32_t last_gps_dbg = 0;
    if ((millis() - last_gps_dbg) >= 2000)
    {
        last_gps_dbg = millis();
        Log.print(F("GPS_DBG,"));
        Log.print(gps_rx_bytes);
        Log.print(',');
        Log.print(GPS_BAUDS[gps_baud_idx]);
        Log.print(',');
        Log.println(gps_locked ? 1 : 0);
    }

    /* Show captured uBlox boot bytes (hex) every 3 s. */
    static uint32_t last_gps_first = 0;
    if ((millis() - last_gps_first) >= 3000)
    {
        last_gps_first = millis();
        Log.print(F("GPS_FIRST,"));
        Log.print(gps_first_len);
        Log.print(',');
        for (uint16_t i = 0; i < gps_first_len && i < 64; i++)
        {
            if (gps_first[i] < 0x10) Log.print('0');
            Log.print(gps_first[i], HEX);
        }
        Log.println();
    }

    /* Echo the most recent valid NMEA sentence (1 Hz). */
    static uint32_t last_nmea = 0;
    if (gps_nmea_valid && (millis() - last_nmea) >= 1000)
    {
        last_nmea = millis();
        Log.print(F("GPS_RAW,"));
        Log.println(gps_last_nmea);
    }

    /* --- BMI323 presence detection (retry every 1 s until found) --------- */
    static uint32_t last_bmi_retry = 0;
    if (!bmi_ready && (millis() - last_bmi_retry) >= 1000)
    {
        last_bmi_retry = millis();
        if (bmi323_begin())
        {
            bmi_ready = true;
            Log.println(F("BMI_STATUS,1"));
        }
        else
        {
            Log.println(F("BMI_STATUS,0"));
            uint8_t raw = bmi_raw_chip_id();
            Log.print(F("BMI_RAW,0x"));
            Log.println(raw, HEX);
        }
    }

    /* --- BMI323 streaming at ~100 Hz ------------------------------------- */
    static uint32_t last_bmi = 0;
    static uint32_t att_last_us = 0;
    if (bmi_ready && (millis() - last_bmi) >= 10)
    {
        last_bmi = millis();

        struct bmi3_sensor_data data[2];
        data[0].type = BMI323_ACCEL;
        data[1].type = BMI323_GYRO;
        if (bmi323_get_sensor_data(data, 2, &bmi_dev) == BMI3_OK)
        {
            float ax = (float)data[0].sens_data.acc.x * BMI_ACC_LSB_G;
            float ay = (float)data[0].sens_data.acc.y * BMI_ACC_LSB_G;
            float az = (float)data[0].sens_data.acc.z * BMI_ACC_LSB_G;
            float gx = (float)data[1].sens_data.gyr.x * BMI_GYR_LSB_DPS;
            float gy = (float)data[1].sens_data.gyr.y * BMI_GYR_LSB_DPS;
            float gz = (float)data[1].sens_data.gyr.z * BMI_GYR_LSB_DPS;

            uint32_t now_us = micros();
            float dt_s = (att_last_us == 0) ? 0.01f
                                            : (float)(now_us - att_last_us) * 1e-6f;
            att_last_us = now_us;
            att_update(ax, ay, az, gx, gy, gz, dt_s);

            g_ax = ax; g_ay = ay; g_az = az;
            g_gx = gx; g_gy = gy; g_gz = gz;

            Log.print(F("BMI,"));
            Log.print(ax, 4);
            Log.print(',');
            Log.print(ay, 4);
            Log.print(',');
            Log.print(az, 4);
            Log.print(',');
            Log.print(gx, 2);
            Log.print(',');
            Log.print(gy, 2);
            Log.print(',');
            Log.println(gz, 2);
        }
        else
        {
            /* Read failed - drop out of ready so we re-initialize. */
            bmi_ready = false;
            Log.println(F("BMI_STATUS,0"));
        }
    }

    /* --- Attitude output at ~20 Hz --------------------------------------- */
    static uint32_t last_att_out = 0;
    if (bmi_ready && (millis() - last_att_out) >= 50)
    {
        last_att_out = millis();
        Log.print(F("ATT,"));
        Log.print(att_roll_rad  * RAD_TO_DEG, 1);
        Log.print(',');
        Log.print(att_pitch_rad * RAD_TO_DEG, 1);
        Log.print(',');
        Log.println(att_yaw_rad * RAD_TO_DEG, 1);
    }

    /* --- BMP581 forced mode, non-blocking: trigger at ~10 Hz, collect the
       result on a later pass once the conversion has had time to finish.
       No delay() -- the old code blocked the whole loop for 30 ms here. ----- */
    static uint32_t last_bmp   = 0;
    static uint32_t bmp_trig_us = 0;
    static bool     bmp_pending = false;

    if (bmp_ok && !bmp_pending && (millis() - last_bmp) >= 100)
    {
        last_bmp = millis();
        if (bmp5_set_power_mode(BMP5_POWERMODE_FORCED, &bmp_dev) == BMP5_OK)
        {
            bmp_trig_us = micros();
            bmp_pending = true;
        }
    }
    else if (bmp_pending && (micros() - bmp_trig_us) >= BMP_CONV_US)
    {
        bmp_pending = false;

        struct bmp5_sensor_data data;
        if (bmp5_get_sensor_data(&data, &bmp_cfg, &bmp_dev) == BMP5_OK)
        {
            float p = data.pressure;
            float t = data.temperature;
            float alt = 44330.0f * (1.0f - powf(p / 101325.0f, 0.1902632f));

            g_press = p / 100.0f;
            g_temp  = t;
            g_alt   = alt;

            Log.print(F("BMP,"));
            Log.print(p / 100.0f, 3);
            Log.print(',');
            Log.print(t, 2);
            Log.print(',');
            Log.println(alt, 2);
        }
    }

    /* --- SD data log at ~50 Hz ------------------------------------------- */
    static uint32_t last_sd_log = 0;
    if (sd_ok && bmi_ready && (millis() - last_sd_log) >= 20)
    {
        last_sd_log = millis();

        sd_file.print(millis());
        sd_file.print(',');
        sd_file.print(g_ax, 4); sd_file.print(',');
        sd_file.print(g_ay, 4); sd_file.print(',');
        sd_file.print(g_az, 4); sd_file.print(',');
        sd_file.print(g_gx, 2); sd_file.print(',');
        sd_file.print(g_gy, 2); sd_file.print(',');
        sd_file.print(g_gz, 2); sd_file.print(',');
        sd_file.print(att_roll_rad  * RAD_TO_DEG, 1); sd_file.print(',');
        sd_file.print(att_pitch_rad * RAD_TO_DEG, 1); sd_file.print(',');
        sd_file.print(att_yaw_rad   * RAD_TO_DEG, 1); sd_file.print(',');
        sd_file.print(g_press, 3); sd_file.print(',');
        sd_file.print(g_temp, 2); sd_file.print(',');
        sd_file.print(g_alt, 2); sd_file.print(',');
        sd_file.print(gps_time_str); sd_file.print(',');
        sd_file.print(gps_sats); sd_file.print(',');
        sd_file.println(gps_speed_kmh, 1);

        static uint32_t last_sd_sync = 0;
        if ((millis() - last_sd_sync) >= 1000)
        {
            last_sd_sync = millis();
            sd_file.flush();
        }
    }

    /* --- SD status debug every 5 s --------------------------------------- */
    static uint32_t last_sd_dbg = 0;
    if ((millis() - last_sd_dbg) >= 5000)
    {
        last_sd_dbg = millis();
        Log.print(F("SD_DBG,"));
        Log.print(sd_ok ? 1 : 0);
        Log.print(F(","));
        Log.print(sd_log_name);
        Log.print(F(","));
        Log.print(Log.drops());          /* USB log lines dropped (host too slow) */
        Log.println();
    }
}
