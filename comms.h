#include <SimpleFOC.h>
#include <stdbool.h>

#define ADDR 0x08 //0x00-0x07 Reserved

#define COMM_STOP 0      // Stops and disables the motor
#define COMM_ENABLE 1    // Enables the motor to respond to future commands
#define COMM_VEL 2       // Retreives the current motor velocity
#define COMM_POS 3       // Retreives the current motor position
#define COMM_TELEMETRY 4 // Retreives both the current motor position and velocity from a single request
#define COMM_TORQUE 5    // Sets the motor FOC torque set point

#define COMMAND_MAX_INDX 5 // Used for packet validation

// === PACKAGE STRUCTURE ====
//  unsigned char command;
//  unsigned char parameter;
//  unsigned char crc;

#define PACKET_SIZE 3 // Number of bytes in a motor message packet


typedef struct {
  BLDCMotor* motor;
  bool initialised;
} comms_state_t;

static comms_state_t comms_state = {.initialised = false};

void initialise_i2c(BLDCMotor* motor);
void recv_message(int numBytes);
void send_message(byte *packet);

void comm_stop_callback();
void comm_enable_callback();
void comm_vel_callback();
void comm_pos_callback();
void comm_telemetry_callback();
void comm_torque_callback(byte param);
byte compute_checksum(byte comm, byte param);
