#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class ModelJointControler : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelJointControler::OnUpdate, this));

      this->old_secs =ros::Time::now().toSec();



      if (_sdf->HasElement("wheel_kp"))
          this->wheel_kp = _sdf->Get<double>("wheel_kp");
      if (_sdf->HasElement("wheel_ki"))
          this->wheel_ki = _sdf->Get<double>("wheel_ki");
      if (_sdf->HasElement("wheel_kd"))
          this->wheel_kd = _sdf->Get<double>("wheel_kd");
      if (_sdf->HasElement("namespace_model"))
          this->namespace_model = _sdf->Get<std::string>("namespace_model");
      if (_sdf->HasElement("activate_pid_control"))
          this->activate_pid_control = (_sdf->Get<std::string>("activate_pid_control") == "yes");


      // Create a topic name
      std::string leg1_speed = "/"+this->namespace_model + "/leg1_speed";
      std::string leg2_speed = "/"+this->namespace_model + "/leg2_speed";
      std::string leg3_speed = "/"+this->namespace_model + "/leg3_speed";
      std::string leg4_speed = "/"+this->namespace_model + "/leg4_speed";
      std::string leg5_speed = "/"+this->namespace_model + "/leg5_speed";
      std::string leg6_speed = "/"+this->namespace_model + "/leg6_speed";








      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "set_wheelSpeed_rosnode",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("earthquake_rosnode"));









      if(this->activate_pid_control)
      {
          // Activated PID Speed Control
	  const auto &jointController = this->model->GetJointController();
          jointController->Reset();






          jointController->AddJoint(model->GetJoint("leg1"));
          jointController->AddJoint(model->GetJoint("leg2"));
          jointController->AddJoint(model->GetJoint("leg3"));
          jointController->AddJoint(model->GetJoint("leg4"));
          jointController->AddJoint(model->GetJoint("leg5"));
          jointController->AddJoint(model->GetJoint("leg6"));





          this->leg1_name = model->GetJoint("leg1")->GetScopedName();
          this->leg2_name = model->GetJoint("leg2")->GetScopedName();
          this->leg3_name = model->GetJoint("leg3")->GetScopedName();
          this->leg4_name = model->GetJoint("leg4")->GetScopedName();
          this->leg5_name = model->GetJoint("leg5")->GetScopedName();
          this->leg6_name = model->GetJoint("leg6")->GetScopedName();





          jointController->SetVelocityPID(this->leg1_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          jointController->SetVelocityPID(this->leg2_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          jointController->SetVelocityPID(this->leg3_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          jointController->SetVelocityPID(this->leg4_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          jointController->SetVelocityPID(this->leg5_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
          jointController->SetVelocityPID(this->leg6_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));






          jointController->SetVelocityTarget(this->leg1_name, 0.0);
          jointController->SetVelocityTarget(this->leg2_name, 0.0);
          jointController->SetVelocityTarget(this->leg3_name, 0.0);
          jointController->SetVelocityTarget(this->leg4_name, 0.0);
          jointController->SetVelocityTarget(this->leg5_name, 0.0);
          jointController->SetVelocityTarget(this->leg6_name, 0.0);


      }










      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            leg1_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_leg1_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ModelJointControler::QueueThread, this));









      ros::SubscribeOptions so2 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            leg2_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_leg2_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue2);
      this->rosSub2 = this->rosNode->subscribe(so2);

      // Spin up the queue helper thread.
      this->rosQueueThread2 =
        std::thread(std::bind(&ModelJointControler::QueueThread2, this));







      ros::SubscribeOptions so3 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            leg3_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_leg3_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue3);
      this->rosSub3 = this->rosNode->subscribe(so3);

      // Spin up the queue helper thread.
      this->rosQueueThread3 =
        std::thread(std::bind(&ModelJointControler::QueueThread3, this));












      ros::SubscribeOptions so4 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            leg4_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_leg4_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue4);
      this->rosSub4 = this->rosNode->subscribe(so4);

      // Spin up the queue helper thread.
      this->rosQueueThread4 =
        std::thread(std::bind(&ModelJointControler::QueueThread4, this));






      ros::SubscribeOptions so5 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            leg5_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_leg5_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue5);
      this->rosSub5 = this->rosNode->subscribe(so5);

      // Spin up the queue helper thread.
      this->rosQueueThread5 =
        std::thread(std::bind(&ModelJointControler::QueueThread5, this));




      ros::SubscribeOptions so6 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            leg6_speed,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_leg6_speed, this, _1),
            ros::VoidPtr(), &this->rosQueue6);
      this->rosSub6 = this->rosNode->subscribe(so6);

      // Spin up the queue helper thread.
      this->rosQueueThread6 =
        std::thread(std::bind(&ModelJointControler::QueueThread6, this));





      ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());

    }













    // Called by the world update start event
    public: void OnUpdate()
    {
      double new_secs =ros::Time::now().toSec();
      double delta = new_secs - this->old_secs;

      double max_delta = 0.0;

      if (this->freq_update != 0.0)
      {
        max_delta = 1.0 / this->freq_update;
      }

      if (delta > max_delta && delta != 0.0)
      {
        this->old_secs = new_secs;






	if(this->activate_pid_control)
        {
          ROS_DEBUG("Update Wheel Speed PID...");
	  const auto &jointController = this->model->GetJointController();




	        jointController->SetVelocityTarget(this->leg1_name, this->leg1_speed_magn);
          jointController->SetVelocityTarget(this->leg2_name, this->leg2_speed_magn);
          jointController->SetVelocityTarget(this->leg3_name, this->leg3_speed_magn);
          jointController->SetVelocityTarget(this->leg4_name, this->leg4_speed_magn);
          jointController->SetVelocityTarget(this->leg5_name, this->leg5_speed_magn);
          jointController->SetVelocityTarget(this->leg6_name, this->leg6_speed_magn);


        }else
        {

            // Apply a small linear velocity to the model.
            ROS_DEBUG("Update Wheel Speed BASIC...");




      	    this->model->GetJoint("leg1")->SetVelocity(0, this->leg1_speed_magn);
            this->model->GetJoint("leg2")->SetVelocity(0, this->leg2_speed_magn);
            this->model->GetJoint("leg3")->SetVelocity(0, this->leg3_speed_magn);
            this->model->GetJoint("leg4")->SetVelocity(0, this->leg4_speed_magn);
            this->model->GetJoint("leg5")->SetVelocity(0, this->leg5_speed_magn);
            this->model->GetJoint("leg6")->SetVelocity(0, this->leg6_speed_magn);




        }

      }

    }













    public: void SetLeg1Speed(const double &_freq)
    {
      this->leg1_speed_magn = _freq;
      ROS_WARN("leg1_speed_magn >> %f", this->leg1_speed_magn);
    }





    public: void SetLeg2Speed(const double &_magn)
    {
      this->leg2_speed_magn = _magn;
      ROS_WARN("leg2_speed_magn >> %f", this->leg2_speed_magn);
    }






    public: void SetLeg3Speed(const double &_magn)
    {
      this->leg3_speed_magn = _magn;
      ROS_WARN("leg3_speed_magn >> %f", this->leg3_speed_magn);
    }






    public: void SetLeg4Speed(const double &_magn)
    {
      this->leg4_speed_magn = _magn;
      ROS_WARN("leg4_speed_magn >> %f", this->leg4_speed_magn);
    }





    public: void SetLeg5Speed(const double &_magn)
    {
      this->leg5_speed_magn = _magn;
      ROS_WARN("leg5_speed_magn >> %f", this->leg5_speed_magn);
    }






    public: void SetLeg6Speed(const double &_magn)
    {
      this->leg6_speed_magn = _magn;
      ROS_WARN("leg6_speed_magn >> %f", this->leg6_speed_magn);
    }













    public: void OnRosMsg_leg1_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetLeg1Speed(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }











    public: void OnRosMsg_leg2_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetLeg2Speed(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread2()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue2.callAvailable(ros::WallDuration(timeout));
      }
    }



    public: void OnRosMsg_leg3_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetLeg3Speed(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread3()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue3.callAvailable(ros::WallDuration(timeout));
      }
    }








    public: void OnRosMsg_leg4_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetLeg4Speed(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread4()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue4.callAvailable(ros::WallDuration(timeout));
      }
    }








    public: void OnRosMsg_leg5_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetLeg5Speed(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread5()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue5.callAvailable(ros::WallDuration(timeout));
      }
    }






    public: void OnRosMsg_leg6_speed(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetLeg6Speed(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread6()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue6.callAvailable(ros::WallDuration(timeout));
      }
    }
























    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Time Memory
    double old_secs;





    // Frequency of earthquake
    double freq_update = 10.0;

    double leg1_speed_magn = 0.0;
    // Magnitude of the Oscilations
    double leg2_speed_magn = 0.0;
    double leg3_speed_magn = 0.0;
    double leg4_speed_magn = 0.0;
    double leg5_speed_magn = 0.0;
    double leg6_speed_magn = 0.0;














    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;






    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;




    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub2;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue2;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread2;




     /// \brief A ROS subscriber
    private: ros::Subscriber rosSub3;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue3;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread3;




    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub4;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue4;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread4;




    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub5;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue5;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread5;




    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub6;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue6;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread6;








    std::string leg1_name;
    std::string leg2_name;
    std::string leg3_name;
    std::string leg4_name;
    std::string leg5_name;
    std::string leg6_name;





    std::string namespace_model = "";
    bool activate_pid_control;

    double wheel_kp = 0.1;
    double wheel_ki = 0.0;
    double wheel_kd = 0.0;


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelJointControler)
}

