// ROS headers
#include <ros/ros.h>
#include <servo_server/ServoControl.h>

// C library headers
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

// Linux headers
#include <fcntl.h>	// Contains file controls like O_RDWR
#include <errno.h>	// Error integer and strerror() function
#include <termios.h>// Contains POSIX terminal control definitions
#include <unistd.h>	// write(), read(), close()

// Open the port
int serial_port = open("/dev/ttyTHS0", O_RDWR);

unsigned int position;

//======================= SUBROUTINES ================================================

void wait ( int seconds )
{
	clock_t endwait;
	endwait = clock () + seconds * CLOCKS_PER_SEC ;
	while (clock() < endwait) {}
}

// subroutine for UART ===================================================================

void uart_send(unsigned char data)
{
	// Write to serial port
	write(serial_port, &data, sizeof(unsigned char));
	
}

// subroutines for commands ==============================================================

// command Servo positioning with speed 
void position_speed_cmd(unsigned char channel, unsigned int position, unsigned char speed)
{
	unsigned char first_byte=0, higher_byte=0, lower_byte=0;

	first_byte = 0b11100000 | channel;

	higher_byte = (position >> 6) & 0b01111111;
	lower_byte  = position & 0b00111111;

	// send commands
	uart_send(first_byte);
	uart_send(higher_byte);
	uart_send(lower_byte);
	uart_send(speed);
}

// Command Enable/Disable the servo channel
void on_off_cmd(unsigned char channel, unsigned char on_off)
{
	unsigned char first_byte = 0;

	first_byte = 0b11000000 | channel;

	uart_send(first_byte);
	uart_send(on_off);
}

//=== Service CallBack ===
bool servo_control(servo_server::ServoControl::Request  &req,
		   servo_server::ServoControl::Response &res)
{
	// on_off command callback
	if (req.Command == 'o')
	{		
		on_off_cmd(req.Channel, req.On_Off);
		res.Ret = true;
	}
	// Position_speed command callback
	else if (req.Command == 'p')
	{
		ROS_INFO("%d, %d, %d\n", req.Channel, req.Position, req.Speed);
		
		on_off_cmd((unsigned char)req.Channel, (unsigned char)req.On_Off);
			
		position_speed_cmd((unsigned char)req.Channel, (unsigned int)req.Position, (unsigned char)req.Speed);

		ROS_INFO("Position Command Sent: %d, %d, %d\n", req.Channel, req.Position, req.Speed);	

		res.Ret = true;
	}
	else
		res.Ret = false;
	
	return true;
}

//============ MAIN ===============================================================================
int main(int argc, char** argv)
{

	//=== Setting up the UART port (/dev/ttyTHS0) ===

	// check for errors
	if (serial_port < 0)
	{
		ROS_INFO("Error %i from open: %s\n", errno, strerror(errno));
	}

	// Create new termios struct, we call it 'tty' for convention
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Read in existing settings, and handle any error
	if (tcgetattr(serial_port, &tty) != 0)
	{
		ROS_INFO("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	tty.c_cflag &= ~PARENB;			// Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB;			// Clear stop field, only one stop bit used in common case
	tty.c_cflag |= CS8;				// 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS;		// Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL;	// Turn on the READ & ignore ctrl lines
	
	tty.c_lflag &= ~ICANON;			
	tty.c_lflag &= ~ECHO;			// Disable the echo
	tty.c_lflag &= ~ECHOE;			// Disable the erasure
	tty.c_lflag &= ~ECHONL;			// Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
	{
		ROS_INFO("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}	

	//=== Starting the ROS Node ===

	ros::init(argc, argv, "servo_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("servo_control", servo_control);
	ROS_INFO("Ready to control servo");
	ros::spin();

	return 0;
}

































