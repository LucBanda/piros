#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include <sstream>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>


#include "minimu9-ahrs.h"

bool enableMuxer(char mask) {
  int adapter_nr = 1;
  int fd;

  char filename[20];
  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
  fd = open(filename, O_RDWR);
	printf("Enabling Mutiplexer\n");
  printf("ioctl = %d\n", ioctl(fd, I2C_SLAVE, 0x70));
  printf("buf = %02x\n", mask);
  printf("status = %d\n", write(fd, &mask, 1));
}

int main(int argc, char **argv)
{
//	enableMuxer(0b00000011);

	if(argc < 5) {
		printf("Please supply address for lsm, lis, topic, name\n");
		for (int i=0; i< argc; i++)
			printf("%s ",argv[i]);
		printf(" count = %d\n", argc);
		exit(0);
	}
	int lsmaddr, lisaddr;
	lsmaddr = (int) strtol(argv[1], NULL, 16);
	lisaddr = (int) strtol(argv[2], NULL, 16);

  ros::init(argc, argv, argv[3]);
  ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>(argv[4], 1000);
	std::string frame = "1";

	// Initializa IMU
	MinIMU9 imu("/dev/i2c-1", lsmaddr, lisaddr);
	imu.loadCalibration();
	imu.enable();
	imu.measureOffsets();

	int count = 0;
  ros::Rate loop_rate(100);
	quaternion rotation = quaternion::Identity();

	int start = millis();
  while (ros::ok())
  {
				int last_start = start;
				start = millis();
				float dt = (start-last_start)/1000.0;
				if (dt < 0){ throw std::runtime_error("Time went backwards."); }

				vector angular_velocity = imu.readGyro();
				vector acceleration = imu.readAcc();
				vector magnetic_field = imu.readMag();

				fuse_default(rotation, dt, angular_velocity, acceleration, magnetic_field);

				sensor_msgs::Imu msg;
				msg.header.frame_id = frame;
				msg.header.stamp = ros::Time::now();

				output_quaternion(rotation);
				std::cout << "  " << acceleration << "  " << magnetic_field << std::endl << std::flush;

				write_quaternion(msg, rotation);

				msg.linear_acceleration.x = acceleration[0];
				msg.linear_acceleration.y = acceleration[1];
				msg.linear_acceleration.z = acceleration[2];

				msg.angular_velocity.x = angular_velocity[0];
				msg.angular_velocity.y = angular_velocity[1];
				msg.angular_velocity.z = angular_velocity[2];

				chatter_pub.publish(msg);
      	// Ensure that each iteration of the loop takes at least 20 ms.
        /* while(millis() - start < 20)
        {
            usleep(1000);
        }*/
	}

  return 0;
}
