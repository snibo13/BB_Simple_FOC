#include <comms.h>

void initComms(BLDCMotor* motor)
{
  comms_state.motor = motor;
}

void initialisei2c()
{
  Wire.begin(ADDR);
  Wire.onReceive(recv_message);
}

bool valid_packet(byte *packet)
{
  // Check if the command is valid
  if (packet[0] > COMMAND_MAX_INDX)
    return false;

  // Verify the checksum
  // For num using a simple XOR checksum could use CRC if reliability is an issue
  if (compute_checksum(packet[0], packet[1]) != packet[2])
    return false;

  // Default case return true unless specific flaw with the packet
  return true;
}

void recv_message(int numBytes)
{
  if (numBytes == PACKET_SIZE)
  {
    // byte startByte = Wire.read();
    byte command = Wire.read();
    byte param = Wire.read();
    byte checksum = Wire.read();
    byte packet[PACKET_SIZE] = {command, param, checksum};
    if (!valid_packet(&packet[0]))
      return;
    if (!comms_state.initialised) return;
    // TODO: Setup better error response than simply nothing

    switch (command)
    {
    case COMM_STOP:
      comm_stop_callback();
      break;
    case COMM_ENABLE:
      comm_enable_callback();
      break;
    case COMM_VEL:
      comm_vel_callback();
      break;
    case COMM_POS:
      comm_pos_callback();
      break;
    case COMM_TELEMETRY:
      comm_telemetry_callback();
      break;
    case COMM_TORQUE:
      comm_torque_callback(param);
      break;
    default:
      break;
    }
  }
}

void send_message(byte *packet)
{
  Wire.beginTransmission(0);
  for (int i = 0; i < PACKET_SIZE; ++i)
  {
    Wire.write(packet[i]);
  }
  Wire.endTransmission();
  // delay(100); // Unnecessary?
}

/* Callbacks */
void comm_stop_callback()
{
  // Sets the target torque to zero and disables further listening
  comms_state.motor->target = 0;
  comms_state.motor->disable();
}

void comm_enable_callback()
{
  // Enables communication to listen to commands
  comms_state.motor->enable();
}

void comm_vel_callback()
{
  byte comm = COMM_VEL;
  byte param = comms_state.motor->shaftVelocity();
  byte checksum = compute_checksum(comm, param); // To allow for easy checksum replacement if necessary

  byte packet[3] = {comm, param, checksum};
  send_message(packet);
}

void comm_pos_callback()
{
  byte comm = COMM_POS;
  byte param = comms_state.motor->shaftAngle();
  byte checksum = compute_checksum(comm, param); // To allow for easy checksum replacement if necessary

  byte packet[3] = {comm, param, checksum};
  send_message(packet);
}

void comm_telemetry_callback()
{
  comm_pos_callback();
  delay(100);
  comm_vel_callback();
}



/* Helpers */
byte compute_checksum(byte comm, byte param)
{
  return comm ^ param; // Simple XOR checksum
}