#include "ros/ros.h"
#include "clothoid_msgs/clothoid_CAN.h"
#include <boost/thread.hpp>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <fstream>
#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*
/* A simple SocketCAN example */

const char *byte_to_binary(int x)
{
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

class CANNode{
    ros::NodeHandle nh;
    ros::Publisher can_pub;

    int soc;
    int read_can_port;
    int nbytes;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;
public:

    CANNode(ros::NodeHandle _nh):nh(_nh){

    can_pub = nh.advertise<clothoid_msgs::clothoid_CAN>("vehicle_status", 1000);

    struct ifreq ifr;
    struct sockaddr_can addr;

    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(soc < 0)
    {
        return;
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, "can0");

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {

        return;
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        return;
    }
    boost::thread grab_thread = boost::thread(boost::bind(&CANNode::publishCAN, this));
}
    ~CANNode(){close(soc);}
    void publishCAN()
    {
        clothoid_msgs::clothoid_CAN can_msg;
    std::ofstream fout;
	fout.open("steer.txt");
        struct can_frame frame_rd;
        int recvbytes = 0;
    
        read_can_port = 1;
        while(read_can_port)
        {
            struct timeval timeout = {1, 0};
            fd_set readSet;
            FD_ZERO(&readSet);
            FD_SET(soc, &readSet);
    
            if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
            {
                if (!read_can_port)
                {
                    break;
                }
                if (FD_ISSET(soc, &readSet))
                {
                    recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
                    if(recvbytes)
                    {
                        //printf("ID = %x, dlc = %d, data = %s\n", frame_rd.can_id, frame_rd.can_dlc, byte_to_binary(frame_rd.data[1]));
                        switch( frame_rd.can_id ){
                        case 0x100:
              can_msg.Gway_Wheel_Velocity_FR = ((frame_rd.data[1]<<8)|frame_rd.data[0])*0.03125;
              can_msg.Gway_Wheel_Velocity_RL = ((frame_rd.data[3]<<8)|frame_rd.data[2])*0.03125;
              can_msg.Gway_Wheel_Velocity_RR = ((frame_rd.data[5]<<8)|frame_rd.data[4])*0.03125;
              can_msg.Gway_Wheel_Velocity_FL = ((frame_rd.data[7]<<8)|frame_rd.data[6])*0.03125;
                            can_msg.header.stamp = ros::Time::now();
                            can_pub.publish(can_msg);

                            break;
                        case 0x101:
                            can_msg.Gway_Lateral_Accel_Speed = ((frame_rd.data[1]<<8)|frame_rd.data[0])*0.01 - 10.23;
                            can_msg.Gway_Parking_Brake_Active = frame_rd.data[2]&0x0F;
                            can_msg.Gway_AirConditioner_On = (frame_rd.data[2]&0xF0)>>4;
                            can_msg.Gway_Steering_Angle = ((int16_t)(frame_rd.data[4]<<8)|frame_rd.data[3])*0.1;
                            can_msg.Gway_Steering_Speed = frame_rd.data[5]*4;
                            can_msg.Gway_Steering_Tq = (((frame_rd.data[7]<<8)|frame_rd.data[6])-0x800)*0.01;
                            can_msg.header.stamp = ros::Time::now();
                            can_pub.publish(can_msg);
				fout << can_msg.Gway_Steering_Angle << ",";

                            break;
                        case 0x102:
                            can_msg.Gway_Accel_Pedal_Position = frame_rd.data[0]*0.3906;
                            can_msg.Gway_Brake_Active = frame_rd.data[1]&0x0F;
                            can_msg.Gway_BrakeMasterCylinder_Pressure = (((frame_rd.data[3]&0x0F)<<12)|(frame_rd.data[2]<<8)|(frame_rd.data[1]&0xF0)>>4)*0.1;
                            can_msg.Gway_Engine_Speed = (((frame_rd.data[5]&0x0F)<<12)|(frame_rd.data[4]<<4)|(frame_rd.data[3]&0xF0)>>4)*0.25;
                            can_msg.Gway_Gear_Target_Change = ((frame_rd.data[5]&0xF0)>>4);
                            can_msg.Gway_GearSelDisp = (frame_rd.data[6]&0x0F);
                            can_msg.Gway_Throttle_Position = ((((frame_rd.data[7]&0x0F)<<4)|((frame_rd.data[6]&0xF0)>>4))-0x20)*0.46948357;
                            can_msg.header.stamp = ros::Time::now();
                            can_pub.publish(can_msg);

                            break;
                        case 0x103:
                            can_msg.Gway_Cluster_Odometer = ((frame_rd.data[2]<<16)|(frame_rd.data[1]<<8)|frame_rd.data[0])*0.1;
                            can_msg.Gway_Longitudinal_Accel_Speed = ((frame_rd.data[4]<<8)|frame_rd.data[3])*0.01-10.23;
                            can_msg.Gway_Vehicle_Speed_Engine = frame_rd.data[5];
                            can_msg.Gway_Yaw_Rate_Sensor = ((frame_rd.data[7]<<8)|frame_rd.data[6])*0.01-40.95;
                            can_msg.header.stamp = ros::Time::now();
                            can_pub.publish(can_msg);

                            break;
                        default:
                            break;
                        }
                    }
                }
            }
    
        }
	fout.close();    
	}
};
int main(int argc, char* argv[]) 
{
    ros::init(argc, argv, "gateway_can_node");
    ros::NodeHandle nh;
    CANNode kn(nh);
    ros::spin();
	return 0;
}

