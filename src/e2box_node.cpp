#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16MultiArray.h"

#include <vector>
#include <stdlib.h>
#include "serial.h"

#define SIZE_SOP	2
#define SOP_HEX		0x5555

typedef union
{
	struct
	{
		int8_t channel;				// 1byte
		int8_t id;					// 1byte
		int16_t roll, pitch, yaw;	// 6byte
		int16_t acc_x, acc_y, acc_z;// 6byte
		int16_t batt;				// 2byte
		int16_t checksum;			// 2byte
	} f;

	char bytes[18];

} DataFormat;
int main(int argc, char **argv)
{
	ros::init(argc,argv,"e2box_node");
	ros::NodeHandle nh;

	ros::Rate loop_rate(250);

	std::vector<char> vecBuf;
	std::vector<char> vecRead; 
	DataFormat dataRead;

	vecRead.assign( 1000, 0 );

	char port_name[30] = "/dev/ttyUSB0";
	int comm_fd;
	int dwRead;
	int16_t sop_check; 
	char temp;
	int data_count=0, sec_count=0;
	int i;

	if( argc > 1 )
		strcpy(port_name, argv[1]);

	comm_fd = open_serial( port_name, 460800, 10, 1 );

	if( comm_fd <= 0 )
		goto EXIT;

	while( ros::ok() )
	{
		dwRead = read( comm_fd, &vecRead[0], SIZE_SOP + sizeof(DataFormat) ); 

		if( dwRead <= 0 )
		{
//			ROS_INFO("[ERROR] Time out!");
		}
		else if( dwRead == 1 )
		{
			vecBuf.push_back( vecRead[0] );			
		}
		else
		{
			vecBuf.insert( vecBuf.end() , vecRead.begin() , vecRead.begin() + dwRead ); 
		}


		while( vecBuf.size() >= sizeof(DataFormat) + SIZE_SOP )
		{
			memcpy(&sop_check, &vecBuf[0], SIZE_SOP); 					

			if( sop_check == SOP_HEX )
			{
				dataRead.bytes[0] = vecBuf[SIZE_SOP];
				dataRead.bytes[1] = vecBuf[SIZE_SOP+1];

				for(i=2; i < sizeof(DataFormat); i+=2)
				{
					dataRead.bytes[i] = vecBuf[ SIZE_SOP + i + 1 ];	
					dataRead.bytes[i+1] = vecBuf[ SIZE_SOP + i ];	
				}
				
				vecBuf.erase( vecBuf.begin(), vecBuf.begin() + sizeof(DataFormat) + SIZE_SOP ); 
				
				data_count++;
			}
			else
			{
				ROS_INFO("[ERROR] Packet Error!"); 
				vecBuf.erase( vecBuf.begin(), vecBuf.begin() + SIZE_SOP );
			}
		}

		sec_count++;

		if( sec_count >= 249 )
		{
			ROS_INFO("Reading Frequency : %d", data_count); 	

			data_count = 0;
			sec_count = 0;
		}
		
		loop_rate.sleep();
		ros::spinOnce();
	}

EXIT:

	close_serial( comm_fd );	
	
	return 0;
}

