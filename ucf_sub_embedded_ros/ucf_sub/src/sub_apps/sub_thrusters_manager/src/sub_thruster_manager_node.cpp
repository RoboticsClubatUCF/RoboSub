#include "t200_thruster.h"
//#include "sub_thruster_library/seabotix_thruster.h"
#include "generic_thruster.h"
#include "sub_trajectory/ThrusterCmd.h"
#include "sub_trajectory/ThrusterStatus.h"
#include <json/json.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/SelfTest.h> //TODO: implement self tests
#include <self_test/self_test.h>

#include <iostream>
#include <fstream>
#include <memory>

inline const std::string BoolToString(const bool b); //http://stackoverflow.com/a/29798

class ThrusterManager {
    ros::NodeHandle nh_;
    ros::Subscriber command_subscriber;
    ros::Publisher diagnostics_output;
    ros::Publisher status_output;
    self_test::TestRunner self_test_;
    std::string configPath;

    sub_trajectory::ThrusterCmd savedMsg;

    std::map<int, std::unique_ptr<GenericThruster>> thrusterMap;

    int updateRate;
    int expectedThrusters;

    ros::ServiceServer initServer;

public:
    ThrusterManager() : self_test_()
    {
        for(int i = 0; i < 8; i++) {
            savedMsg.cmd.push_back(0.0);
        }

        command_subscriber = nh_.subscribe("/thrusters/cmd_vel", 1000, &ThrusterManager::thrusterCb, this);

        diagnostics_output = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000);
        status_output = nh_.advertise<sub_trajectory::ThrusterStatus>("/thrusterStatus", 1000);

        self_test_.add("Test connections", this, &ThrusterManager::testThrusterConnections);

        initServer = nh_.advertiseService("initThrusters", &ThrusterManager::initService, this);

        nh_.param("/updateRate", updateRate, 30);
        nh_.param("/thrusterConfigPath", configPath, std::string("config.json"));
    }

    Json::Value loadConfig(std::string filename)
    {
        ifstream configFile(filename);
        if(!configFile.is_open()){
            ROS_ERROR("Thruster controller couldn't open config file");
            //status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Config file didn't load");
            Json::Value obj;
            return obj;
        }

        ROS_INFO("Thruster controller config loading");
        Json::Value obj;
        configFile >> obj;
        ROS_INFO("Thruster controller config loaded");
        return obj;
    }
    bool initService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
    {
        init();
        return true;
    }

    void init()
    {
        thrusterMap.clear();
        Json::Value thrustersJson = loadConfig(configPath)["COMPUTE"];

        savedMsg = sub_trajectory::ThrusterCmd();
        savedMsg.cmd.resize(thrustersJson.size(), 0.0);
        expectedThrusters = thrustersJson.size();

        diagnostic_msgs::DiagnosticArray diag;

        for(int i = 0; i < thrustersJson.size(); i++) {
            int thrusterID = thrustersJson[i]["ID"].asInt();
            int thrusterType = thrustersJson[i]["Type"].asInt(); //TODO: support for multiple thruster types
            int thrusterAddress = thrustersJson[i]["Address"].asInt();
            ROS_INFO("Initializing thruster %d", thrusterID);
            try{
                  thrusterMap[i] = std::unique_ptr<GenericThruster>(new T200Thruster(1,thrusterAddress));
            }
            catch(I2CException e){
                //If we get here there's a bus problem
                ROS_ERROR("I2C error while connecting to thruster address %x", thrusterAddress);
                thrusterMap.erase(i);
                //Publish an error message for the diagnostic system to do something about
                diagnostic_msgs::DiagnosticStatus status;
                status.name = "Thruster_"+thrustersJson[i]["Address"].asString();
                status.hardware_id = "Thruster_"+thrustersJson[i]["Address"].asString();
                status.message = std::string("Couldn't connect");
                status.level = status.ERROR;
                diag.status.push_back(status);
            }
        }
        diagnostics_output.publish(diag);
        ros::spinOnce();
        ROS_INFO("Done initializing thrusters");
    }

    void spin()
    {
        ros::Rate rate(updateRate);
        int loopCount;
        sub_trajectory::ThrusterStatus thrusterStatusMsg = sub_trajectory::ThrusterStatus();
        while(ros::ok()) {
            diagnostic_msgs::DiagnosticArray diag;
            ROS_DEBUG("Updating thrusters");
            if(loopCount++ > updateRate*2 && thrusterMap.size() < expectedThrusters)
            {
                init();
                loopCount = 0;
            }

            if(thrusterMap.size() == 0)
            {
                diagnostic_msgs::DiagnosticStatus status;
                status.name = "Thrusters";
                status.hardware_id = "Thrusters";
                status.level = status.WARN;
                diag.status.push_back(status);
                ROS_WARN("No thrusters initialized");
            }
            else if(savedMsg.cmd.size() == 0)
            {
                diagnostic_msgs::DiagnosticStatus status;
                status.name = "Thrusters";
                status.hardware_id = "Thrusters";
                status.level = status.WARN;
                diag.status.push_back(status);
                ROS_WARN("No comand message");
            }
            else if(thrusterMap.size() != savedMsg.cmd.size())
            {
                diagnostic_msgs::DiagnosticStatus status;
                status.name = "Thrusters";
                status.hardware_id = "Thrusters";
                status.level = status.ERROR;
                diag.status.push_back(status);
                ROS_ERROR_THROTTLE(10,"Thrusters command has wrong number of values");
            }
            else
            {
                for(auto& iter:thrusterMap)
                {
                    diagnostic_msgs::DiagnosticStatus status;
                    status.name = "Thruster_"+std::to_string(iter.first);
                    status.hardware_id = "Thruster_"+std::to_string(iter.first);
                    thrusterStatusMsg.thrusterChannel = iter.first;
                    try {
                        iter.second->updateStatus();
                        iter.second->setVelocityRatio(savedMsg.cmd.at(iter.first));
                        thrusterStatusMsg.thrusterOk = true;
                    } catch(I2CException e) {
                        //Publish an error message for the diagnostic system to do something about
                        status.level = status.ERROR;
                        thrusterStatusMsg.thrusterOk = false;
                    } catch (std::out_of_range e) {
                        ROS_ERROR("Thrusters command has not enough values");
                        thrusterStatusMsg.thrusterOk = false;
                    }

                    if (thrusterOk(iter.second) && status.level != status.ERROR)
                        status.level = status.OK;
                    else {
                        status.level = status.ERROR;
                        thrusterStatusMsg.thrusterOk = false;
                    }
                    PushDiagData(status, iter.second, std::to_string(iter.first));
                    diag.status.push_back(status);
                    thrusterStatusMsg.header.stamp = ros::Time::now();
                    status_output.publish(thrusterStatusMsg);
                }
            }

            diagnostics_output.publish(diag);
            ros::spinOnce();
            self_test_.checkTest();
            rate.sleep();
        }
    }

    bool thrusterOk (std::unique_ptr<GenericThruster> & thruster)
    {
        return thruster->isAlive() && thruster->inLimits();
    }

    void PushDiagData(diagnostic_msgs::DiagnosticStatus & statusmsg, std::unique_ptr<GenericThruster> & thruster, std::string thrusterName)
    {
        diagnostic_msgs::KeyValue thrusterValue;

        thrusterValue.key = "Thruster Type";
        thrusterValue.value = thruster->getType();
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Alive";
        thrusterValue.value = BoolToString(thruster->isAlive());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Voltage";
        thrusterValue.value = std::to_string(thruster->getVoltage());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Current";
        thrusterValue.value = std::to_string(thruster->getCurrent());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Temperature";
        thrusterValue.value = std::to_string(thruster->getTemperature());
        statusmsg.values.push_back(thrusterValue);
    }

    void thrusterCb(const sub_trajectory::ThrusterCmd &msg)
    {
        savedMsg = msg;
    }

    //Self test function
    void testThrusterConnections(diagnostic_updater::DiagnosticStatusWrapper& status)
    {
        self_test_.setID("thrusterController");
        std::stringstream failedThrusters;
        Json::Value& thrustersJson = loadConfig(configPath)["COMPUTE"];
        for(int i = 0; i < thrustersJson.size(); i++)
        {
            int thrusterID = thrustersJson[i]["ID"].asInt();
            int thrusterType = thrustersJson[i]["Type"].asInt(); //TODO: support for multiple thruster types
            int thrusterAddress = thrustersJson[i]["Address"].asInt();

            try{
                T200Thruster(1, thrusterAddress).updateStatus();
            }
            catch(I2CException e){
                failedThrusters << thrusterAddress << " ";
            }
        }

        if(failedThrusters.str().length() > 0){
            status.add("Failures", failedThrusters.str());
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Thruster couldn't connect");
        } else {
            status.summary(diagnostic_msgs::DiagnosticStatus::OK, "All thrusters OK");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thruster_driver");
    ThrusterManager tc;
    tc.init();
    ROS_INFO("Thruster controller initialized, spinning");
    tc.spin();
    return 0;
}

inline const std::string BoolToString(const bool b)
{
  return b ? "true" : "false";
}
