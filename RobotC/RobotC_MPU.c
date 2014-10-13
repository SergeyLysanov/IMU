#include "RobotC_MPU.h"

#pragma config(Sensor, S1,TIR,sensorI2CCustomFastSkipStates)
#define MPU_PORT S1
#define ACC_RANGE_2G_DEVISOR 64
#define GYRO_RANGE_250_DEVISOR 114.28571

ubyte I2Cmessage[14];
ubyte I2Creply[14];

bool waitForI2CBus()
{
  //TI2CStatus i2cstatus;
  while (true)
  {
    //i2cstatus = nI2CStatus[link];
    switch (nI2CStatus[MPU_PORT])
    //switch(i2cstatus)
    {
	    case NO_ERR:
	      return true;

	    case STAT_COMM_PENDING:
	      break;

	    case ERR_COMM_CHAN_NOT_READY:
	      break;

	    case ERR_COMM_BUS_ERR:
        return false;
    }
    EndTimeSlice();
  }
}

void clearI2CError(tSensors link, ubyte address) {
  ubyte error_array[2];
  error_array[0] = 1;           // Message size
  error_array[1] = address; // I2C Address

  for (int i = 0; i < 5; i++) {
    sendI2CMsg(MPU_PORT, &error_array[0], 0);
    wait1Msec(5);
  }
}

void i2c_read_registers(ubyte register_2_read, int message_size, int return_size)
{
  memset(I2Creply, 0, sizeof(I2Creply));
	message_size = message_size+3;

  I2Cmessage[0] = message_size;    // Messsage Size
  I2Cmessage[1] = MPU6050_ADDRESS;
  I2Cmessage[2] = register_2_read; // Register

  if (!waitForI2CBus()) {
    clearI2CError(MPU_PORT, MPU6050_ADDRESS);

    if (!waitForI2CBus())
      return;
  }

  sendI2CMsg(MPU_PORT, &I2Cmessage[0], return_size);

  //try more
 	if (!waitForI2CBus()) {
    clearI2CError(MPU_PORT, MPU6050_ADDRESS);
    sendI2CMsg(MPU_PORT, &I2Cmessage[0], return_size);
    if (!waitForI2CBus())
      return;
  }

  readI2CReply(MPU_PORT, &I2Creply[0], return_size);
}

byte i2c_read_register(ubyte register_2_read)
{
  I2Cmessage[0] = 3;    // Messsage Size
  I2Cmessage[1] = MPU6050_ADDRESS;
  I2Cmessage[2] = register_2_read; // Register

  if (!waitForI2CBus()) {
    clearI2CError(MPU_PORT, MPU6050_ADDRESS);

    if (!waitForI2CBus())
      return 0;
  }

  sendI2CMsg(MPU_PORT, &I2Cmessage[0], 1);

  //try more
 	if (!waitForI2CBus()) {
    clearI2CError(MPU_PORT, MPU6050_ADDRESS);
    sendI2CMsg(MPU_PORT, &I2Cmessage[0], 1);
    if (!waitForI2CBus())
      return 0;
  }

  readI2CReply(MPU_PORT, &I2Creply[0], 1);

  return I2Creply[0];
}

bool i2c_write_registers(ubyte register_2_write, int message_size, int return_size, ubyte byte1, ubyte byte2 = 0, ubyte byte3 = 0, ubyte byte4= 0)
{
  memset(I2Creply, 0, sizeof(I2Creply));

  message_size = message_size+3;

  I2Cmessage[0] = message_size;    // Messsage Size
  I2Cmessage[1] = MPU6050_ADDRESS;
  I2Cmessage[2] = register_2_write; // Register

  I2Cmessage[3] = byte1;
  I2Cmessage[4] = byte2;
  I2Cmessage[5] = byte3;
  I2Cmessage[6] = byte4;

	waitForI2CBus();
  sendI2CMsg(MPU_PORT, &I2Cmessage[0], 6);

  return waitForI2CBus();
}

bool i2c_write_bits(ubyte address, byte bit_start, byte length, byte data)
{
     //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value

     byte b = i2c_read_register(address);

     byte mask = ((1 << length) - 1) << (bit_start - length + 1);
     data <<= (bit_start - length + 1); // shift data into correct position
     data &= mask; // zero all non-important bits in data
     b &= ~(mask); // zero all important bits in existing byte
     b |= data; // combine data with existing byte

     return i2c_write_registers(address, 0, 0, b);
}

int GyY_offset = -65;  //-68 -:back +forward
int AcZ_offset = 885; //810
void getMotion(float& AcX, float& AcY, float& AcZ, float& GyX, float& GyY, float& GyZ)
{
		i2c_read_registers(0x3B - 2, 0, 14);

		AcX = (I2Creply[0]+((long)(I2Creply[1]<<8))) / ACC_RANGE_2G_DEVISOR;
    AcY = (I2Creply[2]+((long)(I2Creply[3]<<8))) / ACC_RANGE_2G_DEVISOR;
    AcZ = ((I2Creply[4]+((long)(I2Creply[5]<<8))) + AcZ_offset) / ACC_RANGE_2G_DEVISOR;

    GyX = (I2Creply[8]+((long)(I2Creply[9]<<8))) / GYRO_RANGE_250_DEVISOR;
    GyY = ((I2Creply[10]+((long)(I2Creply[11]<<8)))+ GyY_offset) / GYRO_RANGE_250_DEVISOR;
    GyZ = (I2Creply[12]+((long)(I2Creply[13]<<8))) / GYRO_RANGE_250_DEVISOR;

    //writeDebugStream("AcX=%d AcY=%d AcZ=%d\n", AcX, AcY, AcZ);
    //writeDebugStream("GyX=%d GyY=%d GyZ=%d\n", GyX, GyY, GyZ);
}

void setClockSource(byte source)
{
     i2c_write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void setFullScaleAccelRange(byte range)
{
     i2c_write_bits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void setFullScaleGyroRange(byte range)
{
     i2c_write_bits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void setSleepEnabled(bool enabled)
{
     i2c_write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1, enabled);
}

void configMPU()
{
		setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
}

//test
/*
float AcX, AcY, AcZ, GyX, GyY, GyY_prev, GyZ, pitch, pitch_accel;
void complimentary()
{
     pitch += 90.0*(GyY + GyY_prev)/25000.0;
     GyY_prev = GyY;

     //y angle
     pitch_accel = atan(AcZ/AcX) * 180.0 / PI;
     pitch = 0.98*pitch + 0.02*pitch_accel;
}

float total = 0;
long count = 0;
task main()
{
	configMPU();
	while(true)
	{
		getMotion(AcX, AcY, AcZ, GyX, GyY, GyZ);
		//complimentary();
		total+=AcZ;
		++count;
		writeDebugStream("AcZ=%f total=%d count=%d\n", AcZ, total, count);

		//total+=GyY;
		//++count;
		//writeDebugStream("GyY=%f total=%f count=%d\n", GyY, total, count);
		//writeDebugStream("Pitch=%f\n", pitch);
	}
}*/
