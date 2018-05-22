#include <robosub_gazebo_interface.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gate_finder");
    GazeboInterface gi;
    ros::spin();
    return 0;
}

template <typename T> int sgn(T val) {  //http://stackoverflow.com/a/4609795
    return (T(0) < val) - (val < T(0));
}

GazeboInterface::GazeboInterface()
{
    commandSub = nh_.subscribe("/sub/dualStickControl", 1000, &GazeboInterface::commandCb, this);
    modelUpdate = nh_.subscribe("/gazebo/model_states", 1000, &GazeboInterface::stateCb, this);
    gazeboWrenchCaller = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    linear = tf::Vector3(0.0,0.0,0.0);
    angular = tf::Vector3(0.0,0.0,0.0);
}

GazeboInterface::~GazeboInterface()
{

}
void GazeboInterface::stateCb(const gazebo_msgs::ModelStates &msg)
{
    int subIndex = 0;
    if(std::find(msg.name.begin(),msg.name.end(), "ucf_submarine_simple") != msg.name.end()){
        subIndex = std::find(msg.name.begin(),msg.name.end(), "ucf_submarine_simple") - msg.name.begin();
    }
    else {
        return;
    }

    geometry_msgs::Quaternion orientation = msg.pose[subIndex].orientation;
    tf::Quaternion rotWtL(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Quaternion rotLtW = rotWtL.inverse();

    tf::Vector3 gravityLocal = tf::quatRotate(rotWtL, tf::Vector3(0,0,-311)); //Transform world space forces to local
    tf::Vector3 bouyancyLocal = tf::quatRotate(rotWtL, tf::Vector3(0,0,311));
    tf::Vector3 gravityOffset = tf::Vector3(0,0,-0.2); //Local space offsets
    tf::Vector3 bouyancyOffset = tf::Vector3(0,0,0.2);

    tf::Vector3 rotationVelocity(msg.twist[subIndex].angular.x,msg.twist[subIndex].angular.y,msg.twist[subIndex].angular.z);

    tf::Vector3 stabilityTorque = gravityOffset.cross(gravityLocal) + bouyancyOffset.cross(bouyancyLocal) - tf::quatRotate(rotWtL,rotationVelocity) * 50;
    tf::Vector3 floatiness = gravityLocal + bouyancyLocal;

    gazebo_msgs::ApplyBodyWrench wrench;
    wrench.request.body_name = "ucf_submarine_simple::body";
    wrench.request.reference_frame = "world";

    //You can't deliver maximum thrust and maximum torque on the same axis, so mix things to represent this
    //+x forward, +z down, +y right
    float tFrontUp, tRearUp, tLeftForward, tRightForward, tTopStrafe, tBottomStrafe;

    tFrontUp = std::max(-1.0, std::min(1.0, linear.getZ() + angular.getY()));
    tRearUp = std::max(-1.0, std::min(1.0, linear.getZ()  - angular.getY()));

    tLeftForward = std::max(-1.0, std::min(1.0, linear.getX() - angular.getZ()));
    tRightForward = std::max(-1.0, std::min(1.0, linear.getX() + angular.getZ()));

    tTopStrafe = std::max(-1.0, std::min(1.0, linear.getY() - angular.getX()));
    tBottomStrafe = std::max(-1.0, std::min(1.0, linear.getY() + angular.getX()));

    tf::Vector3 force;
    tf::Vector3 torque;

    force.setX((tLeftForward + tRightForward) * MAX_THRUST + floatiness.x());
    force.setY((tTopStrafe + tBottomStrafe) * MAX_THRUST + floatiness.y());
    force.setZ((tFrontUp + tRearUp) * MAX_THRUST + floatiness.z());

    tf::Vector3 forceWorld = tf::quatRotate(rotLtW, force);

    torque.setX((tBottomStrafe - tTopStrafe) * MAX_TORQUE_ROLL + stabilityTorque.x());
    torque.setY((tFrontUp - tRearUp) * MAX_TORQUE_PITCH + stabilityTorque.y());
    torque.setZ((tRightForward - tLeftForward) * MAX_TORQUE_YAW + stabilityTorque.z());

    tf::Vector3 torqueWorld = tf::quatRotate(rotLtW, torque);

    wrench.request.wrench.force.x = forceWorld.getX();
    wrench.request.wrench.force.y = forceWorld.getY();
    wrench.request.wrench.force.z = forceWorld.getZ();

    wrench.request.wrench.torque.x = torqueWorld.getX();
    wrench.request.wrench.torque.y = torqueWorld.getY();
    wrench.request.wrench.torque.z = torqueWorld.getZ();

    std::ostringstream oss;
    printf(oss.str().c_str());


    wrench.request.duration.sec = 0.00001;
    wrench.request.start_time.sec = 0;

    gazeboWrenchCaller.call(wrench);
}

void GazeboInterface::commandCb(const geometry_msgs::Twist &msg)
{
    //std::ostringstream oss;
    //double x = msg.linear.x;
    //double y = msg.linear.y;
    //double z = msg.linear.z;
    //oss << "Translation input: (" << x << ", " << y << ", " << z << ")\n";
    //printf(oss.str().c_str());

    angular.setX(msg.angular.x);
    angular.setY(msg.angular.y);
    angular.setZ(msg.angular.z);

    linear.setX(msg.linear.x);
    linear.setY(msg.linear.y);
    linear.setZ(msg.linear.z);
}
