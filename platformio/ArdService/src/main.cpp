#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/ColorRGBA.h>
#include <rosserial_ard_service_test/ButtonCheck.h>


using rosserial_ard_service_test::ButtonCheck;

/*
example of using arduino ros service 
https://github.com/OpenAgricultureFoundation/rosserial_arduino_libs/blob/master/examples/ServiceServer/ServiceServer.pde 
*/

// declare arduino hal abstractions
Servo servo;
int available_pins [] = {22, 23, 24, 25}; 
int servo_pin = 44;

// declare msgs
std_msgs::Float32MultiArray f_array_msg;
std_msgs::Bool toggle_msg;
std_msgs::Int32 servo_msg;
std_msgs::String echo_msg;

// char arr stubs to generate info messages to echo pub
const char * corr_msg = "we got correct angle: ";
const char * incorr_msg = "we got incorrect angle: ";
char integer_string[32];
char copy_str[32];

// pub
ros::Publisher servo_cmd_pub("servo_cmd_echo", &echo_msg);

// sub callback
void servo_cmd_callback(const std_msgs::Int32& servo_msg)
{
	if(servo_msg.data <= 180 && servo_msg.data >= 0)
	{
		servo.write(servo_msg.data);
		// manually create message to echo pub
		strcpy(copy_str, corr_msg);
		sprintf(integer_string, "%d", servo_msg.data);
		echo_msg.data = strcat(copy_str, integer_string);
		servo_cmd_pub.publish( &echo_msg );
	}
	else
	{
		// manually create message to echo pub
		strcpy(copy_str, incorr_msg);
		sprintf(integer_string, "%d", servo_msg.data);
		echo_msg.data = strcat(copy_str, integer_string);
		servo_cmd_pub.publish( &echo_msg );
	}
	
}

// service callback
void service_callback(const ButtonCheck::Request & req, ButtonCheck::Response & res)
{
	int i;
    for(i = 0; i <  (int) (sizeof(available_pins) / sizeof(int)); i++)
	{
        if(available_pins[i] == req.pin)
		{
			int state = digitalRead(available_pins[i]);
			if(state == LOW)
			{
				res.state = false;
			}
			else
			{
				res.state = true;
			}
		}
    }

}


// sub
ros::Subscriber<std_msgs::Int32> servo_cmd_sub("servo_cmd", &servo_cmd_callback);
// service server
ros::ServiceServer<ButtonCheck::Request, ButtonCheck::Response> server("button_check_srv", &service_callback);


class NewHardware : public ArduinoHardware
{
	public:
	// especially for using in turtlebro`s mainboard arduino
	NewHardware():ArduinoHardware(&Serial1, 115200){};
};

// declare node
ros::NodeHandle_<NewHardware>  nh;


void setup()
{
	// init pins and servos
	for(int i = 0; i <  (int) (sizeof(available_pins) / sizeof(int)); i++)
	{
		pinMode(available_pins[i], INPUT);
	}
	servo.attach(servo_pin);
	// init node and advertise topics and services
	nh.initNode();
	nh.advertiseService(server);
    nh.advertise(servo_cmd_pub);
    nh.subscribe(servo_cmd_sub);

}

void loop()
{
	// there must be nothing more then spin
	nh.spinOnce();
  	delay(10);
}
