/*
 * Copyright 2016 James Jackson, MAGICC Lab, Brigham Young University, Provo, UT
 * Copyright 2016 Gary Ellingson, MAGICC Lab, Brigham Young University, Provo, UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "rosplane_sim/aircraft_forces_and_moments.h"

namespace gazebo
{

AircraftForcesAndMoments::AircraftForcesAndMoments() {}


AircraftForcesAndMoments::~AircraftForcesAndMoments()
{
  updateConnection_.reset();
  // event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_)
  {
    nh_->shutdown();
    delete nh_;
  }
}

bool AircraftForcesAndMoments::retrieveParameter(std::string name, double& destination)
{
  bool success = nh_->getParam(name, destination);
  if (!success)
  {
    double tmp = nh_->param<double>(name, 10101.0);
    //if hasParam returns true and getParam returns false then there is a problem with your yaml file, possibly commas
    gzerr << "Unable to find parameter " << nh_->getNamespace() << "/" << name << " on server and has param returns: " << nh_->hasParam(name) << " and param returns " << tmp << "\n";
  }
  return success;
}

void AircraftForcesAndMoments::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various elements in the simulation
   */
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[gazebo_aircraft_forces_and_moments] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_aircraft_forces_and_moments] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_aircraft_forces_and_moments] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  wind_speed_topic_ = nh_->param<std::string>("windSpeedTopic", "wind");
  command_topic_ = nh_->param<std::string>("commandTopic", "command");

  // The following parameters are aircraft-specific, most of these can be found using AVL
  // The rest are more geometry-based and can be found in conventional methods
  // For the moments of inertia, look into using the BiFilar pendulum method

  // physical parameters
  retrieveParameter("mass", mass_);
  retrieveParameter("Jx", Jx_);
  retrieveParameter("Jy", Jy_);
  retrieveParameter("Jz", Jz_);
  retrieveParameter("Jxz", Jxz_);
  retrieveParameter("rho", rho_);

  // Wing Geometry
  retrieveParameter("wing_s", wing_.S);
  retrieveParameter("wing_b", wing_.b);
  retrieveParameter("wing_c", wing_.c);
  retrieveParameter("wing_M", wing_.M);
  retrieveParameter("wing_epsilon", wing_.epsilon);
  retrieveParameter("wing_alpha0", wing_.alpha0);

  // Propeller Coefficients
  retrieveParameter("k_motor", prop_.k_motor);
  retrieveParameter("k_T_P", prop_.k_T_P);
  retrieveParameter("k_Omega", prop_.k_Omega);
  retrieveParameter("prop_e", prop_.e);
  retrieveParameter("prop_S", prop_.S);
  retrieveParameter("prop_C", prop_.C);

  // Lift Params
  retrieveParameter("C_L_O", CL_.O);
  retrieveParameter("C_L_alpha", CL_.alpha);
  retrieveParameter("C_L_beta", CL_.beta);
  retrieveParameter("C_L_p", CL_.p);
  retrieveParameter("C_L_q", CL_.q);
  retrieveParameter("C_L_r", CL_.r);
  retrieveParameter("C_L_delta_a", CL_.delta_a);
  retrieveParameter("C_L_delta_e", CL_.delta_e);
  retrieveParameter("C_L_delta_r", CL_.delta_r);

  // Drag Params
  retrieveParameter("C_D_O", CD_.O);
  retrieveParameter("C_D_alpha", CD_.alpha);
  retrieveParameter("C_D_beta", CD_.beta);
  retrieveParameter("C_D_p", CD_.p);
  retrieveParameter("C_D_q", CD_.q);
  retrieveParameter("C_D_r", CD_.r);
  retrieveParameter("C_D_delta_a", CD_.delta_a);
  retrieveParameter("C_D_delta_e", CD_.delta_e);
  retrieveParameter("C_D_delta_r", CD_.delta_r);

  // ell Params (x axis moment)
  retrieveParameter("C_ell_O", Cell_.O);
  retrieveParameter("C_ell_alpha", Cell_.alpha);
  retrieveParameter("C_ell_beta", Cell_.beta);
  retrieveParameter("C_ell_p", Cell_.p);
  retrieveParameter("C_ell_q", Cell_.q);
  retrieveParameter("C_ell_r", Cell_.r);
  retrieveParameter("C_ell_delta_a", Cell_.delta_a);
  retrieveParameter("C_ell_delta_e", Cell_.delta_e);
  retrieveParameter("C_ell_delta_r", Cell_.delta_r);

  // m Params (y axis moment)
  retrieveParameter("C_m_O", Cm_.O);
  retrieveParameter("C_m_alpha", Cm_.alpha);
  retrieveParameter("C_m_beta", Cm_.beta);
  retrieveParameter("C_m_p", Cm_.p);
  retrieveParameter("C_m_q", Cm_.q);
  retrieveParameter("C_m_r", Cm_.r);
  retrieveParameter("C_m_delta_a", Cm_.delta_a);
  retrieveParameter("C_m_delta_e", Cm_.delta_e);
  retrieveParameter("C_m_delta_r", Cm_.delta_r);

  // n Params (z axis moment)
  retrieveParameter("C_n_O", Cn_.O);
  retrieveParameter("C_n_alpha", Cn_.alpha);
  retrieveParameter("C_n_beta", Cn_.beta);
  retrieveParameter("C_n_p", Cn_.p);
  retrieveParameter("C_n_q", Cn_.q);
  retrieveParameter("C_n_r", Cn_.r);
  retrieveParameter("C_n_delta_a", Cn_.delta_a);
  retrieveParameter("C_n_delta_e", Cn_.delta_e);
  retrieveParameter("C_n_delta_r", Cn_.delta_r);

  // Y Params (Sideslip Forces)
  retrieveParameter("C_Y_O", CY_.O);
  retrieveParameter("C_Y_alpha", CY_.alpha);
  retrieveParameter("C_Y_beta", CY_.beta);
  retrieveParameter("C_Y_p", CY_.p);
  retrieveParameter("C_Y_q", CY_.q);
  retrieveParameter("C_Y_r", CY_.r);
  retrieveParameter("C_Y_delta_a", CY_.delta_a);
  retrieveParameter("C_Y_delta_e", CY_.delta_e);
  retrieveParameter("C_Y_delta_r", CY_.delta_r);

  // Initialize Wind
  wind_.N = 0.0;
  wind_.E = 0.0;
  wind_.D = 0.0;

  //initialize deltas
  delta_.t = 0.0;
  delta_.e = 0.0;
  delta_.a = 0.0;
  delta_.r = 0.0;

  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AircraftForcesAndMoments::OnUpdate, this, _1));

  // Connect Subscribers
  command_sub_ = nh_->subscribe(command_topic_, 1, &AircraftForcesAndMoments::CommandCallback, this);
  wind_speed_sub_ = nh_->subscribe(wind_speed_topic_, 1, &AircraftForcesAndMoments::WindSpeedCallback, this);

  // Pull off initial pose so we can reset to it
  initial_pose_ = link_->WorldCoGPose();
}

// This gets called by the world update event.
void AircraftForcesAndMoments::OnUpdate(const common::UpdateInfo &_info)
{
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  SendForces();
}

void AircraftForcesAndMoments::Reset()
{
  forces_.Fx = 0.0;
  forces_.Fy = 0.0;
  forces_.Fz = 0.0;
  forces_.l = 0.0;
  forces_.m = 0.0;
  forces_.n = 0.0;

  link_->SetWorldPose(initial_pose_);
  link_->ResetPhysicsStates();
}

void AircraftForcesAndMoments::WindSpeedCallback(const geometry_msgs::Vector3 &wind)
{
  wind_.N = wind.x;
  wind_.E = wind.y;
  wind_.D = wind.z;
}

void AircraftForcesAndMoments::CommandCallback(const rosflight_msgs::CommandConstPtr &msg)
{
  // This is a little bit weird.  We need to nail down why these are negative
  delta_.t = msg->F;
  delta_.e = -msg->y;
  delta_.a = msg->x;
  delta_.r = -msg->z;
}


void AircraftForcesAndMoments::UpdateForcesAndMoments()
{
  /* Get state information from Gazebo (in NED)                 *
   * C denotes child frame, P parent frame, and W world frame.  *
  //   * Further C_pose_W_P denotes pose of P wrt. W expressed in C.*/
  ignition::math::Vector3d C_linear_velocity_W_C = link_->RelativeLinearVel();
  double u = C_linear_velocity_W_C.X();
  double v = -C_linear_velocity_W_C.Y();
  double w = -C_linear_velocity_W_C.Z();
  ignition::math::Vector3d C_angular_velocity_W_C = link_->RelativeAngularVel();
  double p = C_angular_velocity_W_C.X();
  double q = -C_angular_velocity_W_C.Y();
  double r = -C_angular_velocity_W_C.Z();

  // wind info is available in the wind_ struct
  /// TODO: This is wrong. Wind is being applied in the body frame, not inertial frame
  double ur = u - wind_.N;
  double vr = v - wind_.E;
  double wr = w - wind_.D;

  double Va = sqrt(pow(ur, 2.0) + pow(vr, 2.0) + pow(wr, 2.0));

  // Don't divide by zero, and don't let NaN's get through (sometimes RelativeLinearVel returns NaNs)
  if (Va > 0.000001 && std::isfinite(Va))
  {
    /*
     * The following math follows the method described in chapter 4 of
     * Small Unmanned Aircraft: Theory and Practice
     * By Randy Beard and Tim McLain.
     * Look there for a detailed explanation of each line in the rest of this function
     */
    double alpha = atan2(wr , ur);
    double beta = asin(vr/Va);

    double sign = (alpha >= 0 ? 1 : -1); //Sigmoid function
    double sigma_a = (1 + exp(-(wing_.M*(alpha - wing_.alpha0))) + exp((wing_.M*(alpha + wing_.alpha0))))/((1 + exp(-
                     (wing_.M*(alpha - wing_.alpha0))))*(1 + exp((wing_.M*(alpha + wing_.alpha0)))));
    double CL_a = (1 - sigma_a)*(CL_.O + CL_.alpha*alpha) + sigma_a*(2.0*sign*pow(sin(alpha), 2.0)*cos(alpha));
    double AR = (pow(wing_.b, 2.0))/wing_.S;
    double CD_a = CD_.p + ((pow((CL_.O + CL_.alpha*(alpha)),
                                2.0))/(3.14159*0.9 *
                                         AR)); //the const 0.9 in this equation replaces the e (Oswald Factor) variable and may be inaccurate

    double CX_a = -CD_a*cos(alpha) + CL_a*sin(alpha);
    double CX_q_a = -CD_.q*cos(alpha) + CL_.q*sin(alpha);
    double CX_deltaE_a = -CD_.delta_e*cos(alpha) + CL_.delta_e*sin(alpha);

    double CZ_a = -CD_a*sin(alpha) - CL_a*cos(alpha);
    double CZ_q_a = -CD_.q*sin(alpha) - CL_.q*cos(alpha);
    double CZ_deltaE_a = -CD_.delta_e*sin(alpha) - CL_.delta_e*cos(alpha);

    forces_.Fx = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*(CX_a + (CX_q_a*wing_.c*q) /
                 (2.0*Va) + CX_deltaE_a*delta_.e) + 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t), 2.0) - pow(Va,
                     2.0));
    forces_.Fy = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*(CY_.O + CY_.beta*beta + ((CY_.p*wing_.b*p) /
                 (2.0*Va)) + ((CY_.r*wing_.b*r)/(2.0*Va)) + CY_.delta_a*delta_.a + CY_.delta_r*delta_.r);
    forces_.Fz = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*(CZ_a + (CZ_q_a*wing_.c*q) /
                 (2.0*Va) + CZ_deltaE_a*delta_.e);

    forces_.l = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*wing_.b*(Cell_.O + Cell_.beta*beta + (Cell_.p*wing_.b*p) /
                (2.0*Va) + (Cell_.r*wing_.b*r)/(2.0*Va) + Cell_.delta_a*delta_.a + Cell_.delta_r*delta_.r) - prop_.k_T_P *
                pow((prop_.k_Omega*delta_.t), 2.0);
    forces_.m = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*wing_.c*(Cm_.O + Cm_.alpha*alpha + (Cm_.q*wing_.c*q) /
                (2.0*Va) + Cm_.delta_e*delta_.e);
    forces_.n = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*wing_.b*(Cn_.O + Cn_.beta*beta + (Cn_.p*wing_.b*p) /
                (2.0*Va) + (Cn_.r*wing_.b*r)/(2.0*Va) + Cn_.delta_a*delta_.a + Cn_.delta_r*delta_.r);
  }
  else
  {
    if (!std::isfinite(Va))
    {
      gzerr << "u = " << u << "\n";
      gzerr << "v = " << v << "\n";
      gzerr << "w = " << w << "\n";
      gzerr << "p = " << p << "\n";
      gzerr << "q = " << q << "\n";
      gzerr << "r = " << r << "\n";
      gzerr << "ur = " << ur << "\n";
      gzerr << "vr = " << vr << "\n";
      gzerr << "wr = " << wr << "\n";
      gzthrow("we have a NaN or an infinity:\n");
    }
    else
    {
      forces_.Fx = 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t), 2.0));
      forces_.Fy = 0.0;
      forces_.Fz = 0.0;
      forces_.l = 0.0;
      forces_.m = 0.0;
      forces_.n = 0.0;
    }
  }
}


void AircraftForcesAndMoments::SendForces()
{
  // Make sure we are applying reasonable forces
  if (std::isfinite(forces_.Fx + forces_.Fy + forces_.Fz + forces_.l + forces_.m + forces_.n))
  {
    // apply the forces and torques to the joint
    link_->AddRelativeForce(ignition::math::Vector3d(forces_.Fx, -forces_.Fy, -forces_.Fz));
    link_->AddRelativeTorque(ignition::math::Vector3d(forces_.l, -forces_.m, -forces_.n));
  }
}

GZ_REGISTER_MODEL_PLUGIN(AircraftForcesAndMoments);
}
