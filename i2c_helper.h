
#define USE_AND_OR	// To enable AND_OR mask setting for I2C. 
//#include <i2c.h>
#include "plib.h"
#include "tft_master.h"
#define SLAVE_ADDRESS 0xd0
#define ACCEL_XOUT_H 0x3b
#define ACCEL_XOUT_L 0x3c
#define ACCEL_YOUT_H 0x3d
#define ACCEL_YOUT_L 0x3e
#define ACCEL_ZOUT_H 0x3f
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define GYRO_SCALE 131.0 // lsb/(degrees/second)

float X_GYRO_OFF, Y_GYRO_OFF, Z_GYRO_OFF;

// Wait by executing nops
void i2c_wait(unsigned int cnt)
{
	while(--cnt)
	{
		asm( "nop" );
		asm( "nop" );
	}
}

// Write a number of chars from data specified by num to the specified address
void i2c_write(char address, char *data, int num)
{
    char i2c_header[2];
    i2c_header[0] = SLAVE_ADDRESS | 0;	//device address & WR
	i2c_header[1] = address;            //register address

    StartI2C1();	//Send the Start Bit
	IdleI2C1();		//Wait to complete

    int i;
	for(i = 0; i < num + 2; i++)
	{
        if(i < 2)
            MasterWriteI2C1( i2c_header[i] );
        else
            MasterWriteI2C1( data[i - 2] );
		IdleI2C1();		//Wait to complete

		//ACKSTAT is 0 when slave acknowledge, 
		//if 1 then slave has not acknowledge the data.
		if( I2C1STATbits.ACKSTAT )
			break;
	}
    
    StopI2C1();	//Send the Stop condition
	IdleI2C1();	//Wait to complete
}

// Read a char from the register specified by address
char i2c_read(char address)
{
    char i2c_header[2];
    i2c_header[0] = ( SLAVE_ADDRESS | 0 );	//device address & WR
	i2c_header[1] = address;                //register address

    StartI2C1();	//Send the Start Bit
	IdleI2C1();		//Wait to complete

    int i;
	for(i = 0; i < 2; i++)
	{
        MasterWriteI2C1( i2c_header[i] );
		IdleI2C1();		//Wait to complete

		//ACKSTAT is 0 when slave acknowledge, 
		//if 1 then slave has not acknowledge the data.
		if( I2C1STATbits.ACKSTAT )
        {
			break;
        }
	}
    
    //now send a start sequence again
	RestartI2C1();	//Send the Restart condition
	i2c_wait(10);
	//wait for this bit to go back to zero
	IdleI2C1();	//Wait to complete

	MasterWriteI2C1( SLAVE_ADDRESS | 1 ); //transmit read command
	IdleI2C1();		//Wait to complete

	// read some bytes back
//	MastergetsI2C1(num, dataBuf, 20);
    char data = MasterReadI2C1();
	
	IdleI2C1();	//Wait to complete
    
    StopI2C1();	//Send the Stop condition
	IdleI2C1();	//Wait to complete
    
    return data;
}

// Read three-axis accelerometer and three-axis gyroscope from MPU 6050
// Return values in array pointed to by values
void readImuValues(float* values)
{
    int xAccelH = (int) i2c_read(ACCEL_XOUT_H);
    int xAccelL = (int) i2c_read(ACCEL_XOUT_L);
    int yAccelH = (int) i2c_read(ACCEL_YOUT_H);
    int yAccelL = (int) i2c_read(ACCEL_YOUT_L);
    int zAccelH = (int) i2c_read(ACCEL_ZOUT_H);
    int zAccelL = (int) i2c_read(ACCEL_ZOUT_L);
    int xGyroH  = (int) i2c_read(GYRO_XOUT_H);
    int xGyroL  = (int) i2c_read(GYRO_XOUT_L);
    int yGyroH  = (int) i2c_read(GYRO_YOUT_H);
    int yGyroL  = (int) i2c_read(GYRO_YOUT_L);
    int zGyroH  = (int) i2c_read(GYRO_ZOUT_H);
    int zGyroL  = (int) i2c_read(GYRO_ZOUT_L);

    values[0] = (float)((xAccelH  << 8) + xAccelL);
    values[1] = (float)((yAccelH << 8) + yAccelL);
    values[2] = (float)((zAccelH << 8) + zAccelL);
    values[3] = ((xGyroH << 8) + xGyroL)/GYRO_SCALE - X_GYRO_OFF;
    values[4] = ((yGyroH << 8) + yGyroL)/GYRO_SCALE - Y_GYRO_OFF;
    values[5] = ((zGyroH << 8) + zGyroL)/GYRO_SCALE - Z_GYRO_OFF;
}

// Sample the gyroscope over an extended period and average
// in order to find and subtract the bias
void calibrateGyros()
{
    int numSamples = 100;
    int i = 0;
    float xGyroSum, yGyroSum, zGyroSum;
    float values[6];

    for (i = 0; i < numSamples; i++)
    {
        readImuValues(values);

        // Parse the IMU data
        float xGyro  = values[3];
        float yGyro  = values[4];
        float zGyro  = values[5];

        xGyroSum += xGyro;
        yGyroSum += yGyro;
        zGyroSum += zGyro;
    }
    
    X_GYRO_OFF += xGyroSum/numSamples;
    Y_GYRO_OFF += yGyroSum/numSamples;
    Z_GYRO_OFF += zGyroSum/numSamples;
}
