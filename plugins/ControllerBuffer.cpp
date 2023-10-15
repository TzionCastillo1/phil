/*
    TODO: Add license
*/

// Gazebo library headers
#include "ControllerBuffer.hpp"
#include <ignition/plugin/Register.hh>
#include <ignition/msgs/imu.pb.h>

// Cpp library Headers
#include <thread>

// Linux Headers
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

using namespace controller_buffer;


IGNITION_ADD_PLUGIN(
    controller_buffer::ControllerBuffer,
    ignition::gazebo::System,
    controller_buffer::ControllerBuffer::ISystemConfigure,    
    controller_buffer::ControllerBuffer::ISystemPreUpdate,
    controller_buffer::ControllerBuffer::ISystemUpdate,
    controller_buffer::ControllerBuffer::ISystemPostUpdate
)

ControllerBuffer::ControllerBuffer()
{
}

ControllerBuffer::~ControllerBuffer()
{
}


void ControllerBuffer::Configure(const ignition::gazebo::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &,
                                ignition::gazebo::EntityComponentManager & _ecm,
                                ignition::gazebo::EventManager &)
{
    this->model = ignition::gazebo::Model(_entity);
    this->motorCommands.mutable_velocity()->resize(4, 0);

    _ecm.CreateComponent(this->model.Entity(),
                        ignition::gazebo::components::Actuators(this->motorCommands));

    std::string topic{"/imu"};

    this->node.Subscribe(topic, &ControllerBuffer::imu_cb, this);
    /*
    //Maybe not needed: std::thread worker()
    if(flock(fd, LOCK_EX | LOCK_NB) == -1) {
        throw std::runtime_error("Serial port with file descriptor " + 
            std::to_string(fd) + " is already locked with another process.");
    }

    // Open the serial port. Change the device path as needed.
    this->serialPort = open("/dev/ttyUSB0", 0_RDWR);

    // Create new termios struct
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B1152000);
    cfsetospeed(&tty, B1152000);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }
    */

}

void ControllerBuffer:imu_cb(const ignition::msgs::Imu &_msg)
{
    this->imu = _msg;

    /*
    After an imu message is received, this should be put into a buffer to be sent
    out over the serial port to the hardware. Once in this buffer, this program will
    wait to be polled (synchronously), at which point it will send the simulated sensor
    readings to the hardware

    The different values can be fed into an array of doubles
    */

   this->outputBuffer[0] = this->imu.orientation.x;
   this->outputBuffer[1] = this->imu.orientation.y;
   this->outputBuffer[2] = this->imu.orientation.z;
   this->outputBuffer[3] = this->imu.orientation.w;
   this->outputBuffer[4] = this->imu.angular_velocity.x;
   this->outputBuffer[5] = this->imu.angular_velocity.y;
   this->outputBuffer[6] = this->imu.angular_velocity.z;
   this->outputBuffer[7] = this->imu.linear_acceleration.x;
   this->outputBuffer[8] = this->imu.linear_acceleration.y;
   this->outputBuffer[9] = this->imu.linear_acceleration.z;




}

/* From the Gazebo Documentation:
    PreUpdate can be used to modify state before physics runs,
    for example for applying control signals or performing network synchronization.
    This is executed with simulation time at t0, and has read-write access to world
    entities and components.

    PostUpdate can be used to read out results at the end of a simulation step to
    be used for sensor or controller updates.
    This is executed with simulation time at t0 + dt, and has only read access to be
    used for sensor or controller updates
 */

void ControllerBuffer:PreUpdate(const ignition::gazebo::UpdateInfo& _info, 
                                const ignition::gazebo::EntityComponentManager &_ecm)
{

    /*
    Here will be the logic to listen to the serial port. 
    The program will check if there is anything in the buffer.
    If that is the case the information will be pulled from the buffer and sent as actuation
    commands to GZ.
    If not the program will not send any actuation commands. For now, this seems to be the 
    most representative of what would happen in real life (thruster dynamics will dictate how 
    the system will behave if there is no applied torque)
    */
   /*
    int bytes;
    ioctl(fd, FIONREAD, &bytes);
    if(bytes > 0){
        //Do something now that we have actuator information waiting for us
        bytes = read(this->serialPort, &this->inputBuffer, sizeof(this->inputBuffer));
        if (bytes < 0) {
           ignmsg << "Error reading:" << strerror(errno); 
        }
        else {
            ignmsg << "Read bytes" << std::endl;
        }


    }
    */
}