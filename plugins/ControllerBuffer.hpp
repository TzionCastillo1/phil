#include <ignition/gazbeo/System.hh>
#include <ignition/msgs.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>
#include "ignition/gazbeo/Model.hh"

#ifndef SYSTEM_PLUGIN_CONTROLLERBUFFER_HH
#define SYSTEM_PLUGIN_CONRTOLLERBUFFER_HH

namespace controller_buffer
{
    class ControllerBuffer:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemUpdate,
    public ignition::gazebo::ISystemPostUpdate
    {
        public: ControllerBuffer();
        public: ~ControllerBuffer();
        public: void Configure(const ignition::gazebo::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &,
                                ignition::gazebo::EntityComponentManager &_ecm,
                                ignition::gazebo::EventManager &) override;

        public: void Update(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm) override;
        public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm) override;
        public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm) override;
        
        private: void 
        private: ignition::msgs::Actuators motorCommands;
        private: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
        private: ignition::msgs::Imu imu;
        private: ignition::transport::Node node;

        private: double outputBuffer [10];
        private: char inputBuffer [256];
        private: double pastTime;
        private: int serialPort

    }
}

#endif