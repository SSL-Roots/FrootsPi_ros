#include <ros/ros.h>
#include <pigpiod_if2.h>
#include <frootspi_msg/FrootsCommand.h>

class Core{
    public:
        Core();
        ~Core();

        void callbackCommand(const frootspi_msg::FrootsCommandConstPtr&);

    private:
        int mGPIO_CHARGING;
        int mPi;

};

Core::Core(){
    mGPIO_CHARGING = 26;
    mPi = pigpio_start(NULL, NULL);

    set_mode(mPi, mGPIO_CHARGING, PI_OUTPUT);

}

Core::~Core(){
    pigpio_stop(mPi);
}


void Core::callbackCommand(const frootspi_msg::FrootsCommandConstPtr& command){
    if(command->charge_flag){
        gpio_write(mPi, mGPIO_CHARGING, PI_HIGH);
    }else{
        gpio_write(mPi, mGPIO_CHARGING, PI_LOW);
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "kicker");
    ros::NodeHandle nh;

    Core core;

    ros::Subscriber subCommand = nh.subscribe("froots_command", 1, 
            &Core::callbackCommand, &core);

    ros::spin();

}
