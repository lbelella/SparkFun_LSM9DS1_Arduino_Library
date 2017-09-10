#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "SparkFunLSM9DS1.h"

#define DECLINATION 10.5

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
	float roll = atan2(ay, az);
	float pitch = atan2(-ax, sqrt(ay * ay + az * az));
	float heading = 0;

	if(my == 0)
		heading = (mx < 0) ? M_PI : 0;
	else
		heading = atan2(mx, my);

	heading -= DECLINATION * M_PI/ 180;

	if (heading > M_PI) heading -= (2 * M_PI);
	else if (heading < -M_PI) heading += (2 * M_PI);
	else if (heading < 0) heading += (2 * M_PI);

	heading *= 180.0 / M_PI;
	pitch *= 180.0 / M_PI;
	roll *= 180.0 / M_PI;

	printf("%f %f %f\n", pitch, roll, heading);
}

int main(int argc, char** argv)
{
	LSM9DS1 imu(IMU_MODE_I2C, 0x6B, 0x1E);
	imu.settings.device.commInterface = IMU_MODE_I2C;
	imu.settings.device.mAddress = 0x1E;
	imu.settings.device.agAddress = 0x6B;
	int status = imu.begin();
	printf("0x%X\n", status);

	while(1)
	{
		if(imu.gyroAvailable())
		{
			imu.readGyro();
		}

		if(imu.accelAvailable())
		{
			imu.readAccel();
		}

		if(imu.magAvailable())
		{
			imu.readMag();
		}

		printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
		sleep(1);
	}

	return 0;
}
