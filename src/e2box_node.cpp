#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16MultiArray.h"

#include <vector>
#include <stdlib.h>
#include "serial.h"

#define SIZE_SOP	2

typedef union
{
	struct
	{
		int8_t id;					// 1byte
		int8_t channel;				// 1byte
		int16_t roll, pitch, yaw;	// 6byte
		int16_t acc_x, acc_y, acc_z;// 6byte
		int16_t batt;				// 2byte
		int16_t checksum;			// 2byte
	} f;

	char bytes[18];

} DataFormat;

void RevEndian(char *bytes, int size)
{
	int8_t buf;
	for(int i = 0; i < size; i+=2)
	{
		buf = bytes[i];
		bytes[i] = bytes[i+1];
		bytes[i+1] = buf;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"e2box_node");
	ros::NodeHandle nh;

	ros::Rate loop_rate(250);

	DataFormat dataRead;

	char port_name[30] = "/dev/ttyUSB0";
	int count=0;

	if( argc > 1 )
		strcpy(port_name, argv[1]);

	uint16_t SOP_HEX = 0x5555;
	Serial comm( SIZE_SOP, &SOP_HEX ); 

	if( comm.Open(port_name, 460800, 10, 1) )
		ROS_INFO("E2BOX Device Connected! (%s)",port_name);
	else
		ROS_INFO("E2BOX Device Connection Failed! (%s)",port_name);

	while( ros::ok() )
	{
		if( !comm.readPacket(dataRead.bytes, sizeof(DataFormat)) )
			ROS_INFO("ERROR : Packet Read Failed!");

		RevEndian( dataRead.bytes );

		if( count > 249 )			
		{
			ROS_INFO("Data receving freqency : %ld Hz", comm.getFreq());
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

