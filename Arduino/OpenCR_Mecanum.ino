/*******************************************************************************************
 * OpenCR Dynamixel Velocity Bridge for ROS 2 (via USB)
 *
 * - Controls 4 Dynamixels in velocity mode (rad/s) for a mecanum base.
 * - Communicates with a Raspberry Pi over USB serial using a small binary protocol.
 * - Periodically sends measured wheel velocities back to the Pi.
 * - Includes watchdog safety and a wheel identification test.
 *
 * -----------------------------------------------------------------------------------------
 * Protocol Summary (USB serial)
 * -----------------------------------------------------------------------------------------
 * All packets are binary, little-endian.
 *
 * Frame format:
 *   [0]      0xAA    // HEADER_1
 *   [1]      0x55    // HEADER_2
 *   [2]      TYPE    // Message type
 *   [3]      LEN     // Payload length in bytes
 *   [4]      SEQ     // Sequence number (host-chosen; echoed in status)
 *   [5..N]   PAYLOAD // LEN bytes
 *   [N+1..N+2] CRC16 // CCITT-FALSE, init 0xFFFF, poly 0x1021 (low byte first)
 *
 * Host -> OpenCR:
 *   TYPE = 0x01 (MSG_TYPE_CMD_VEL)
 *     LEN = 16 bytes
 *     PAYLOAD:
 *       float32 v_fl_rad_s;  // Front-left wheel desired speed (rad/s)
 *       float32 v_fr_rad_s;  // Front-right wheel desired speed (rad/s)
 *       float32 v_rl_rad_s;  // Rear-left wheel desired speed (rad/s)
 *       float32 v_rr_rad_s;  // Rear-right wheel desired speed (rad/s)
 *
 * OpenCR -> Host:
 *   TYPE = 0x10 (MSG_TYPE_STATUS)
 *     LEN = 1 + 4*4 = 17 bytes
 *     PAYLOAD:
 *       uint8  system_status;   // bit 0: watchdog tripped, bits 1-7: reserved
 *       float32 m_fl_rad_s;     // measured front-left speed  (rad/s)
 *       float32 m_fr_rad_s;     // measured front-right speed (rad/s)
 *       float32 m_rl_rad_s;     // measured rear-left speed   (rad/s)
 *       float32 m_rr_rad_s;     // measured rear-right speed  (rad/s)
 *
 * Safety:
 *   - If no valid CMD_VEL is received for CMD_TIMEOUT_MS (default: 200 ms),
 *     the watchdog trips: desired velocities are set to 0 and the status bit is set.
 *
 * Wheel identification test:
 *   - Set RUN_WHEEL_ID_TEST_ON_BOOT to true, flash, and power the robot.
 *   - The sketch will rotate each motor (ID list) one by one with text prompts
 *     over Serial, so you can see which physical wheel corresponds to which ID.
 *   - After you know which ID is FL/FR/RL/RR, update WHEEL_ID[] and WHEEL_DIR[]
 *     in the configuration section and set RUN_WHEEL_ID_TEST_ON_BOOT back to false.
 *
 *******************************************************************************************/

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <stdint.h>
#include <string.h>

// -------------------------------------------------------------------------------------------------
// User Configuration Section
// -------------------------------------------------------------------------------------------------

// ----- USB Serial (to Raspberry Pi) -----
static const uint32_t USB_BAUD = 115200;  // You can increase later if needed

// ----- Dynamixel Bus (OpenCR TTL port) -----
#define DXL_SERIAL   Serial3
#define DXL_DIR_PIN  84
static const uint32_t DXL_BAUD            = 57600;  // From your scan
static const float    DXL_PROTOCOL_VER    = 2.0f;

// ----- Number of wheels and IDs -----
// 4-wheel mecanum: logical order = [0] FL, [1] FR, [2] RL, [3] RR
enum WheelIndex {
  WHEEL_FL = 0,
  WHEEL_FR = 1,
  WHEEL_RL = 2,
  WHEEL_RR = 3,
  WHEEL_COUNT = 4
};

// Initial ID mapping (edit after running wheel identification test)
uint8_t WHEEL_ID[WHEEL_COUNT] = {
  6,  // index 0 -> currently ID 1 (later set this to actual FL)
  3,  // index 1 -> currently ID 3 (later set this to actual FR)
  4,  // index 2 -> currently ID 4 (later set this to actual RL)
  1   // index 3 -> currently ID 6 (later set this to actual RR)
};

// Direction sign for each wheel (+1 or -1).
// Adjust after you know which way each motor spins for "forward" robot motion.
float WHEEL_DIR[WHEEL_COUNT] = {
  1.0f,  // FL
  1.0f,  // FR
  -1.0f,  // RL
  -1.0f   // RR
};

// Run the wheel identification test automatically at boot?
// - true: runs test, then halts (no protocol).
// - false: runs normal protocol mode.
static const bool RUN_WHEEL_ID_TEST_ON_BOOT = true;

// ----- Control timing / safety -----
static const uint32_t DXL_UPDATE_HZ        = 25;      // DXL read/write + status send (~25 Hz)
static const uint32_t DXL_UPDATE_PERIOD_US = 1000000UL / DXL_UPDATE_HZ;  // ~40 ms
static const uint32_t CMD_TIMEOUT_MS       = 200;     // Watchdog timeout
static const uint32_t CMD_TIMEOUT_US       = CMD_TIMEOUT_MS * 1000UL;

// ----- Dynamixel velocity unit (for most X-series; adjust if needed) -----
//
// 1 unit = 0.229 rpm ≈ 0.0239808 rad/s
//
static const float DXL_VEL_UNIT_RAD_PER_SEC = 0.229f * 2.0f * PI / 60.0f;
static const float INV_DXL_VEL_UNIT         = 1.0f / DXL_VEL_UNIT_RAD_PER_SEC;

// -------------------------------------------------------------------------------------------------
// Protocol constants
// -------------------------------------------------------------------------------------------------
static const uint8_t MSG_HEADER_1       = 0xAA;
static const uint8_t MSG_HEADER_2       = 0x55;

// Message types
static const uint8_t MSG_TYPE_CMD_VEL   = 0x01;  // Host -> OpenCR: desired wheel velocities
static const uint8_t MSG_TYPE_PING      = 0x02;  // Host <-> OpenCR: protocol ping (no payload)
static const uint8_t MSG_TYPE_ECHO      = 0x03;  // Host <-> OpenCR: echo arbitrary payload
static const uint8_t MSG_TYPE_STATUS    = 0x10;  // OpenCR -> Host: measured wheel velocities

// Status flag bits
static const uint8_t STATUS_WATCHDOG    = 0x01;  // bit 0: watchdog tripped

// Maximum payload size (small, only 16 or 17 needed)
static const uint8_t MAX_PAYLOAD_SIZE   = 32;

// -------------------------------------------------------------------------------------------------
// Global objects and state
// -------------------------------------------------------------------------------------------------

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;  // For GOAL_VELOCITY, PRESENT_VELOCITY, VELOCITY_LIMIT, etc.

// Desired wheel velocities (rad/s), command from host:
static float desired_vel_rad_s[WHEEL_COUNT]  = {0.0f, 0.0f, 0.0f, 0.0f};
// Measured wheel velocities (rad/s), from Dynamixel:
static float measured_vel_rad_s[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
// Per-wheel raw velocity limits in Dynamixel units (read from VELOCITY_LIMIT):
static int32_t vel_limit_raw[WHEEL_COUNT]    = {0, 0, 0, 0};
static float   vel_limit_rad_s[WHEEL_COUNT]  = {0.0f, 0.0f, 0.0f, 0.0f};

// Watchdog / command timing:
static bool     cmd_received      = false;
static bool     watchdog_tripped  = false;
static uint8_t  last_cmd_seq      = 0;
static uint32_t last_cmd_time_us  = 0;

// DXL update timing:
static uint32_t last_dxl_update_us = 0;

// -------------------------------------------------------------------------------------------------
// CRC16-CCITT implementation (poly 0x1021, init 0xFFFF, no reflection, no final xor)
// -------------------------------------------------------------------------------------------------
static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data)
{
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
    } else {
      crc <<= 1;
    }
  }
  return crc;
}

// -------------------------------------------------------------------------------------------------
// RX state machine for protocol
// -------------------------------------------------------------------------------------------------
enum RxState {
  RX_WAIT_HEADER_1,
  RX_WAIT_HEADER_2,
  RX_READ_TYPE,
  RX_READ_LEN,
  RX_READ_SEQ,
  RX_READ_PAYLOAD,
  RX_READ_CRC_LO,
  RX_READ_CRC_HI
};

static RxState  rx_state        = RX_WAIT_HEADER_1;
static uint8_t  rx_type         = 0;
static uint8_t  rx_len          = 0;
static uint8_t  rx_seq          = 0;
static uint8_t  rx_payload[MAX_PAYLOAD_SIZE];
static uint8_t  rx_payload_idx  = 0;
static uint16_t rx_crc_calc     = 0;
static uint16_t rx_crc_recv     = 0;

// Forward declarations
void handlePacket(uint8_t type, uint8_t seq, uint8_t len, uint8_t *payload);

// Reset RX state after a complete packet or on error
static void resetRxState()
{
  rx_state       = RX_WAIT_HEADER_1;
  rx_type        = 0;
  rx_len         = 0;
  rx_seq         = 0;
  rx_payload_idx = 0;
  rx_crc_calc    = 0;
  rx_crc_recv    = 0;
}

// Feed one incoming byte into the state machine
static void processIncomingByte(uint8_t b)
{
  switch (rx_state) {
    case RX_WAIT_HEADER_1:
      if (b == MSG_HEADER_1) {
        rx_state = RX_WAIT_HEADER_2;
      }
      break;

    case RX_WAIT_HEADER_2:
      if (b == MSG_HEADER_2) {
        rx_state = RX_READ_TYPE;
      } else if (b == MSG_HEADER_1) {
        // Stay in potential header start
        rx_state = RX_WAIT_HEADER_2;
      } else {
        rx_state = RX_WAIT_HEADER_1;
      }
      break;

    case RX_READ_TYPE:
      rx_type     = b;
      rx_crc_calc = crc16_ccitt_update(0xFFFF, b);
      rx_state    = RX_READ_LEN;
      break;

    case RX_READ_LEN:
      rx_len = b;
      rx_crc_calc = crc16_ccitt_update(rx_crc_calc, b);
      if (rx_len > MAX_PAYLOAD_SIZE) {
        // Invalid length, reset
        resetRxState();
      } else {
        rx_state       = RX_READ_SEQ;
      }
      break;

    case RX_READ_SEQ:
      rx_seq     = b;
      rx_crc_calc = crc16_ccitt_update(rx_crc_calc, b);
      rx_payload_idx = 0;
      if (rx_len == 0) {
        rx_state = RX_READ_CRC_LO;
      } else {
        rx_state = RX_READ_PAYLOAD;
      }
      break;

    case RX_READ_PAYLOAD:
      rx_payload[rx_payload_idx++] = b;
      rx_crc_calc = crc16_ccitt_update(rx_crc_calc, b);
      if (rx_payload_idx >= rx_len) {
        rx_state = RX_READ_CRC_LO;
      }
      break;

    case RX_READ_CRC_LO:
      rx_crc_recv = (uint16_t)b;
      rx_state    = RX_READ_CRC_HI;
      break;

    case RX_READ_CRC_HI:
      rx_crc_recv |= ((uint16_t)b << 8);
      // Check CRC
      if (rx_crc_recv == rx_crc_calc) {
        // Valid packet
        handlePacket(rx_type, rx_seq, rx_len, rx_payload);
      }
      // Regardless of validity, reset state for next packet
      resetRxState();
      break;
  }
}

// -------------------------------------------------------------------------------------------------
// Helpers for float <-> byte (little-endian; both Pi and OpenCR are little-endian)
// -------------------------------------------------------------------------------------------------
static void writeFloatLE(uint8_t *dst, float value)
{
  memcpy(dst, &value, sizeof(float));
}

static float readFloatLE(const uint8_t *src)
{
  float value;
  memcpy(&value, src, sizeof(float));
  return value;
}

// -------------------------------------------------------------------------------------------------
// Packet sending
// -------------------------------------------------------------------------------------------------
static void sendPacket(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len)
{
  uint16_t crc = 0xFFFF;

  Serial.write(MSG_HEADER_1);
  Serial.write(MSG_HEADER_2);

  Serial.write(type);
  crc = crc16_ccitt_update(crc, type);

  Serial.write(len);
  crc = crc16_ccitt_update(crc, len);

  Serial.write(seq);
  crc = crc16_ccitt_update(crc, seq);

  for (uint8_t i = 0; i < len; i++) {
    uint8_t b = payload[i];
    Serial.write(b);
    crc = crc16_ccitt_update(crc, b);
  }

  // CRC low byte, then high byte
  Serial.write((uint8_t)(crc & 0xFF));
  Serial.write((uint8_t)((crc >> 8) & 0xFF));
}

// Send periodic status packet with measured velocities
static void sendStatusPacket()
{
  uint8_t payload[1 + WHEEL_COUNT * sizeof(float)];
  uint8_t idx = 0;

  uint8_t status_flags = 0;
  if (watchdog_tripped) {
    status_flags |= STATUS_WATCHDOG;
  }

  payload[idx++] = status_flags;

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    writeFloatLE(&payload[idx], measured_vel_rad_s[i]);
    idx += sizeof(float);
  }

  sendPacket(MSG_TYPE_STATUS, last_cmd_seq, payload, idx);
}

// -------------------------------------------------------------------------------------------------
// Command handling
// -------------------------------------------------------------------------------------------------

// Clamp value between minVal and maxVal
static float clampFloat(float x, float minVal, float maxVal)
{
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}

// Handle a complete, CRC-valid packet
void handlePacket(uint8_t type, uint8_t seq, uint8_t len, uint8_t *payload)
{
  // --------------------------------------------------------------------------
  // 1) PING: no payload. Just echo back the same TYPE and SEQ, no payload.
  // --------------------------------------------------------------------------
  if (type == MSG_TYPE_PING) {
    if (len != 0) {
      // For robustness, if someone accidentally adds payload, ignore it.
      return;
    }

    // Reply: same type, same seq, empty payload
    sendSimpleReply(MSG_TYPE_PING, seq, nullptr, 0);
    return;
  }

  // --------------------------------------------------------------------------
  // 2) ECHO: any payload. Echo back same TYPE, SEQ, and PAYLOAD.
  // --------------------------------------------------------------------------
  if (type == MSG_TYPE_ECHO) {
    if (len > 0 && payload != nullptr) {
      sendSimpleReply(MSG_TYPE_ECHO, seq, payload, len);
    } else {
      // Empty payload is allowed; just echo empty.
      sendSimpleReply(MSG_TYPE_ECHO, seq, nullptr, 0);
    }
    return;
  }

  // --------------------------------------------------------------------------
  // 3) CMD_VEL: Host -> OpenCR wheel velocity commands (rad/s)
  // --------------------------------------------------------------------------
  if (type == MSG_TYPE_CMD_VEL) {
    // Expect 4 float32 values = 16 bytes
    if (len != (WHEEL_COUNT * sizeof(float))) {
      return;  // Ignore malformed payload
    }

    // Parse desired wheel velocities (rad/s) in order FL, FR, RL, RR
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
      float v = readFloatLE(&payload[i * sizeof(float)]);

      // Apply sign for each wheel and clamp to that wheel's velocity limit
      float v_signed = v * WHEEL_DIR[i];

      // If limit not read correctly, vel_limit_rad_s[i] may be 0; fallback to some default.
      float max_rad = vel_limit_rad_s[i];
      if (max_rad <= 0.0f) {
        // Conservative default: roughly 10 rad/s
        max_rad = 10.0f;
      }

      desired_vel_rad_s[i] = clampFloat(v_signed, -max_rad, max_rad);
    }

    // Update command timing and seq
    cmd_received     = true;
    watchdog_tripped = false;
    last_cmd_seq     = seq;
    last_cmd_time_us = micros();

    return;
  }

  // --------------------------------------------------------------------------
  // (You can add more message types later if needed)
  // Unrecognized type: ignore
  // --------------------------------------------------------------------------
}

// -------------------------------------------------------------------------------------------------
// Dynamixel helpers
// -------------------------------------------------------------------------------------------------

static void stopAllMotors()
{
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    uint8_t id = WHEEL_ID[i];
    dxl.writeControlTableItem(GOAL_VELOCITY, id, 0);
  }
}

// Convert rad/s -> DXL raw velocity (int32)
static int32_t radPerSecToDxlRaw(float rad_s)
{
  float raw_f = rad_s * INV_DXL_VEL_UNIT;
  // Round to nearest integer
  if (raw_f >= 0.0f) {
    raw_f += 0.5f;
  } else {
    raw_f -= 0.5f;
  }
  return (int32_t)raw_f;
}

// Convert DXL raw velocity -> rad/s
static float dxlRawToRadPerSec(int32_t raw)
{
  return (float)raw * DXL_VEL_UNIT_RAD_PER_SEC;
}

// Apply desired_vel_rad_s[] to Dynamixels (GOAL_VELOCITY)
static void applyDesiredVelocities()
{
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    uint8_t id = WHEEL_ID[i];

    // Clamp again (belt and suspenders) based on per-wheel limit
    float v = desired_vel_rad_s[i];
    float max_rad = vel_limit_rad_s[i];
    if (max_rad <= 0.0f) {
      max_rad = 10.0f;  // fallback
    }
    float v_clamped = clampFloat(v, -max_rad, max_rad);

    int32_t raw = radPerSecToDxlRaw(v_clamped);
    dxl.writeControlTableItem(GOAL_VELOCITY, id, raw);
  }
}

// Read PRESENT_VELOCITY from each Dynamixel and update measured_vel_rad_s[]
static void readPresentVelocities()
{
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    uint8_t id = WHEEL_ID[i];
    int32_t raw = dxl.readControlTableItem(PRESENT_VELOCITY, id);
    float   rad_s = dxlRawToRadPerSec(raw);
    // Undo WHEEL_DIR so that host sees velocities in its logical direction convention
    measured_vel_rad_s[i] = rad_s * ((WHEEL_DIR[i] != 0.0f) ? (1.0f / WHEEL_DIR[i]) : 1.0f);
  }
}

// Initialize Dynamixels: velocity mode, read limits, torque on
static void initDynamixels()
{
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER);

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    uint8_t id = WHEEL_ID[i];

    dxl.ping(id);  // Optional: ignore failure for now

    // Set to velocity control mode
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_VELOCITY);
    dxl.torqueOn(id);

    // Read VELOCITY_LIMIT to know max safe speed
    int32_t limit = dxl.readControlTableItem(VELOCITY_LIMIT, id);
    if (limit <= 0 || limit > 3000) {
      // Fallback / sanity: typical X-series has around 1023 as default
      limit = 1023;
    }

    vel_limit_raw[i]   = limit;
    vel_limit_rad_s[i] = dxlRawToRadPerSec(limit);
  }

  // Ensure all motors start stopped
  stopAllMotors();
}

// -------------------------------------------------------------------------------------------------
// Wheel identification test
// -------------------------------------------------------------------------------------------------
//
// Run once at boot if RUN_WHEEL_ID_TEST_ON_BOOT is true.
// It will:
//   - Sequentially spin each motor (based on WHEEL_ID list) forward then backward.
//   - Print messages over Serial (115200).
//   - After seeing which wheel moves, you can decide which ID is FL, FR, RL, RR.
//   - Then update WHEEL_ID[] and WHEEL_DIR[] in code and reflash with RUN_WHEEL_ID_TEST_ON_BOOT=false.
//

static void runWheelIdentificationTest()
{
  Serial.println();
  Serial.println("========================================================");
  Serial.println("WHEEL IDENTIFICATION TEST (WITH WHEEL_DIR)");
  Serial.println("========================================================");
  Serial.println("This will rotate each Dynamixel in WHEEL_ID[] one by one.");
  Serial.println("For each wheel, it will use WHEEL_DIR[i] to determine the");
  Serial.println("direction that is considered POSITIVE robot command (+rad/s).");
  Serial.println();
  Serial.println("Current WHEEL_ID and WHEEL_DIR:");
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    Serial.print("  index ");
    Serial.print(i);
    Serial.print(" -> ID ");
    Serial.print(WHEEL_ID[i]);
    Serial.print("  WHEEL_DIR = ");
    Serial.println(WHEEL_DIR[i], 1);
  }
  Serial.println();
  Serial.println("Interpretation:");
  Serial.println("  - During the \"Forward\" phase, the motor is commanded with");
  Serial.println("      +test_speed_rad_s * WHEEL_DIR[i]");
  Serial.println("  - During the \"Reverse\" phase, it is commanded with");
  Serial.println("      -test_speed_rad_s * WHEEL_DIR[i]");
  Serial.println("So if WHEEL_DIR[i] = +1, \"Forward\" is +rad/s on the DXL.");
  Serial.println("If WHEEL_DIR[i] = -1, \"Forward\" is -rad/s on the DXL.");
  Serial.println();
  Serial.println("Make sure wheels are free and robot is safely supported.");
  Serial.println("Press any key to start...");
  Serial.println("========================================================");

  // Wait for user key
  while (!Serial.available()) {
    // Do nothing
  }
  while (Serial.available()) {
    Serial.read();  // flush
  }

  const float   test_speed_rad_s    = 1.0f;     // approx slow spin (robot command)
  const uint32_t test_duration_ms   = 2000;     // 2 seconds each direction
  const uint32_t pause_between_ms   = 1000;     // 1 second pause between motors

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    uint8_t id = WHEEL_ID[i];

    Serial.println();
    Serial.println("--------------------------------------------------------");
    Serial.print("Testing wheel index ");
    Serial.print(i);
    Serial.print("  (Dynamixel ID ");
    Serial.print(id);
    Serial.println(")");
    Serial.print("Using WHEEL_DIR[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(WHEEL_DIR[i], 1);
    Serial.println();
    Serial.println("Forward spin (robot +rad/s command):");
    Serial.print("  Commanded rad/s = +");
    Serial.print(test_speed_rad_s, 3);
    Serial.print(" * WHEEL_DIR = ");
    Serial.println(test_speed_rad_s * WHEEL_DIR[i], 3);
    Serial.println("--------------------------------------------------------");

    // Forward: positive robot command * WHEEL_DIR
    float   cmd_forward_rad_s = test_speed_rad_s * WHEEL_DIR[i];
    int32_t raw_fwd = radPerSecToDxlRaw(cmd_forward_rad_s);
    dxl.writeControlTableItem(GOAL_VELOCITY, id, raw_fwd);
    delay(test_duration_ms);

    Serial.println("Reverse spin (robot -rad/s command):");
    Serial.print("  Commanded rad/s = -");
    Serial.print(test_speed_rad_s, 3);
    Serial.print(" * WHEEL_DIR = ");
    Serial.println(-test_speed_rad_s * WHEEL_DIR[i], 3);

    // Reverse: negative robot command * WHEEL_DIR
    float   cmd_reverse_rad_s = -test_speed_rad_s * WHEEL_DIR[i];
    int32_t raw_rev = radPerSecToDxlRaw(cmd_reverse_rad_s);
    dxl.writeControlTableItem(GOAL_VELOCITY, id, raw_rev);
    delay(test_duration_ms);

    Serial.println("Stopping...");
    dxl.writeControlTableItem(GOAL_VELOCITY, id, 0);
    delay(pause_between_ms);
  }

  Serial.println();
  Serial.println("========================================================");
  Serial.println("Wheel identification test complete.");
  Serial.println();
  Serial.println("Use this test to verify:");
  Serial.println("  1) Which physical wheel corresponds to each index/ID.");
  Serial.println("  2) That when you mentally think \"robot forward +rad/s\",");
  Serial.println("     the \"Forward\" phase above actually spins in that");
  Serial.println("     direction for each wheel.");
  Serial.println();
  Serial.println("If a wheel is backwards, just flip its WHEEL_DIR[i] sign");
  Serial.println("in the code and rerun this test.");
  Serial.println("========================================================");
}

// -------------------------------------------------------------------------------------------------
// Arduino setup() and loop()
// -------------------------------------------------------------------------------------------------

void setup()
{
  // USB serial to Raspberry Pi or PC
  Serial.begin(USB_BAUD);
  while (!Serial) {
    // Wait for USB enumerated (optional)
  }

  // Initialize Dynamixel bus and motors
  initDynamixels();

  // Optional wheel ID test at boot
  if (RUN_WHEEL_ID_TEST_ON_BOOT) {
    runWheelIdentificationTest();
    // Stop motors and halt afterwards
    stopAllMotors();
    while (true) {
      delay(1000);
    }
  }

  // Initialize timing / state
  last_dxl_update_us = micros();
  last_cmd_time_us   = micros();
  cmd_received       = false;
  watchdog_tripped   = false;

  resetRxState();
}

void loop()
{
  // 1) Process all incoming bytes from USB serial (from Raspberry Pi)
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    processIncomingByte(b);
  }

  uint32_t now_us = micros();

  // 2) Watchdog: if no command for CMD_TIMEOUT_US, force desired speeds to 0
  if (cmd_received) {
    uint32_t dt_us = now_us - last_cmd_time_us;
    if (dt_us > CMD_TIMEOUT_US) {
      // Clear desired velocities
      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        desired_vel_rad_s[i] = 0.0f;
      }
      cmd_received      = false;
      watchdog_tripped  = true;
    }
  }

  // 3) Periodic DXL update and status feedback
  if ((now_us - last_dxl_update_us) >= DXL_UPDATE_PERIOD_US) {
    last_dxl_update_us = now_us;

    // Apply desired velocities to Dynamixels
    applyDesiredVelocities();

    // Read back present velocities
    readPresentVelocities();

    // Send status packet to host
    sendStatusPacket();
  }
}
