#include "kinova_controller/robot/kinova_gen3_simulation.hpp"

KinovaGen3Simulation::KinovaGen3Simulation(mj::Simulate* sim_arg):
KinovaGen3Base()
{ 
  sim = sim_arg;
}

KinovaGen3Simulation::~KinovaGen3Simulation()
{

}


bool KinovaGen3Simulation::init(std::shared_ptr<RobotState> robot_state_arg)
{
    robot_state = robot_state_arg;

    YAML::Node config = robot_state->config;
    std::string mujoco_xml_path_string;
    try
    {   
        mujoco_xml_path_string = config["mujoco_xml_path"].as<std::string>();
    }
    catch(YAML::Exception &e)
    {   
        std::cout<<"ERROR: mujoco_xml_path are poorly defined in the yaml file"<<std::endl;
        std::cout<<"YAML Exception : "<<e.what()<<std::endl;
    }

    char* mujoco_xml_path = nullptr;
    mujoco_xml_path = const_cast<char*>(mujoco_xml_path_string.c_str());

    // request loadmodel if file given (otherwise drag-and-drop)
    if (mujoco_xml_path != nullptr) {
        m = LoadModel(mujoco_xml_path, *sim);
        if (m) d = mj_makeData(m);
        if (d) { 
        sim->load(mujoco_xml_path, m, d);
        mj_forward(m, d);
        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
        mju_zero(ctrlnoise, m->nu);
        }
    }

    try
    {
      const YAML::Node& simulation_config = config["simulation"];

      if (!simulation_config.IsDefined() || simulation_config.IsNull())
      {
          std::cout<<"simulation_config is not defined or null" << std::endl;
      }
      else
      {
        //Initial position
        Eigen::VectorXd theta_init(robot_state->nv);
        std::vector<double> theta_init_vec = simulation_config["theta_init"].as<std::vector<double>>();
        if(theta_init_vec.size() != robot_state->nv)
        {
            std::cout<<"theta_init size is not equal to nv"<<std::endl;
        }
        theta_init = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(theta_init_vec.data(), theta_init_vec.size());

        if(robot_state->is_rigid)
        {
            for(int i=0; i<theta_init.size(); i++)
            {
                d->qpos[i] = theta_init(i);
            }
        }
        else
        {
            for(int i=0; i<theta_init.size(); i++)
            {
                d->qpos[2*i] = theta_init(i);
            }
        }
      }

      //initialize member variable
      joint_axis = simulation_config["joint_axis"].as<std::vector<int>>();
    }
    catch(YAML::Exception &e)
    { 
      std::cout<<"ERROR: simulation_config are poorly defined in the yaml file"<<std::endl;
        std::cout<<"YAML Exception : "<<e.what()<<std::endl;
    }
      

    
    mj_forward(m,d);
    
    

    std::cout<<"LoadModel Done"<<std::endl;
    return true;
}

void KinovaGen3Simulation::destroy()
{
    std::cout<<"KinovaGen3Simulation destroy"<<std::endl;

    // delete everything we allocated
    free(ctrlnoise);
    mj_deleteData(d);
    mj_deleteModel(m);
}

void KinovaGen3Simulation::update_state()
{ 
  const std::lock_guard<std::mutex> lock(sim->mtx);
  
  if (m)
  {
    if(sim->run)
    {
      // // inject noise
      // if(sim->ctrlnoisestd) {
      //   // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
      //   mjtNum rate = mju_exp(-m->opt.timestep / sim->ctrlnoiserate);
      //   mjtNum scale = sim->ctrlnoisestd * mju_sqrt(1-rate*rate);

      //   for (int i=0; i<m->nu; i++) {
      //     // update noise
      //     ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

      //     // apply noise
      //     d->ctrl[i] = ctrlnoise[i];
      //   }
      // }

      mju_zero(d->xfrc_applied, 6*m->nbody);

      sim->applyposepertubations(0);  // move mocap bodies only
      sim->applyforceperturbations();


        if(robot_state->is_rigid)
        {
          for(unsigned int i=0; i<robot_state->nv; i++)
          {
              gen3_state.joint_positions(i) = d->qpos[i]; //motor side angle
              gen3_state.joint_velocities(i) = d->qvel[i];
              gen3_state.joint_torques(i) = joint_axis[i]/std::abs(joint_axis[i])*d->sensordata[3*i+std::abs(joint_axis[i])]; // from JTS
          }
        }
        else
        {
          for(unsigned int i=0; i<robot_state->nv; i++)
          {
              gen3_state.joint_positions(i) = d->qpos[2*i]; //motor side angle
              gen3_state.joint_velocities(i) = d->qvel[2*i];
              gen3_state.joint_torques(i) = joint_axis[i]/std::abs(joint_axis[i])*d->sensordata[3*i+std::abs(joint_axis[i])]; // from JTS
          }
        }
      
    
      // continuous joint position
      robot_state->convert_q(gen3_state);

      // update robot state
      robot_state->update_robot_state(gen3_state);
    }
  }
}

bool KinovaGen3Simulation::set_joint_torque(const Eigen::VectorXd &u)
{   
  const std::lock_guard<std::mutex> lock(sim->mtx);

  if (m)
  { 
    if(sim->run)
    {
      //apply control input u
      for (int i=0; i<robot_state->nv; i++)
      {
          d->ctrl[i] = u(i);
      }

      mj_step(m, d);
    }
    else
    {
      // apply pose perturbation
      sim->applyposepertubations(1);  // move mocap and dynamic bodies

      // run mj_forward, to update rendering and joint sliders
      mj_forward(m, d);
    }
  }

  return true;
}

mjModel* KinovaGen3Simulation::LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      // mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, mj::Simulate::kMaxFilenameLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      // int error_length = mju::strlen_arr(loadError);
      // if (loadError[error_length-1] == '\n') {
        // loadError[error_length-1] = '\0';
      // }
    }
  }

  // mju::strcpy_arr(sim.loadError, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }
  return mnew;
}
