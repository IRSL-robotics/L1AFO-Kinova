#include <kinova_controller/controller/controller_gain.hpp>
ControllerGain::ControllerGain()
{

}

ControllerGain::~ControllerGain()
{

}

void ControllerGain::init(const YAML::Node& node)
{
    read_JOINT_PD_Gains(node);
    read_JOINT_NRIC_Gains(node);
    read_TASK_PD_Gains(node);
    read_TASK_NRIC_Gains(node);
    read_JOINT_FRIC_Gains(node);
    read_TASK_FRIC_Gains(node);
    read_FIRST_FRIC_Gains(node);
    read_SECOND_FRIC_Gains(node);
    read_THIRD_FRIC_Gains(node);
    read_INERTIA_RESHAPING_Gains(node);
    read_L1_FRIC_Gains(node);
    read_IMPLICIT_L1_FRIC_Gains(node);
    read_COULOMB_OBSERVER_Gains(node);
    read_ASYM_FRICTION_Gains(node);
    read_STATIC_FRICTION_Gains(node);
}

void ControllerGain::print_matrix(const Eigen::MatrixXd& matrix, const std::string& name)
{
    std::cout<<name <<" ("<< matrix.cols() << "," << matrix.rows() << ")" << " : " << std::endl;
    std::cout<<matrix << std::endl;
}

TORQUE_INTERFACE ControllerGain::select_torque_interface(const std::string& torque_interface_str)
{
    if(torque_interface_str == "FIRST_ORDER_FRIC")
    {
        return FIRST_ORDER_FRIC;
    }
    else if(torque_interface_str == "SECOND_ORDER_FRIC")
    {
        return SECOND_ORDER_FRIC;
    }
    else if(torque_interface_str == "THIRD_ORDER_FRIC")
    {
        return THIRD_ORDER_FRIC;
    }
    else if(torque_interface_str == "INERTIA_RESHAPING")
    {
        return INERTIA_RESHAPING;
    }
    else if(torque_interface_str == "L1_FRIC")
    {
        return L1_FRIC;
    }
    else if(torque_interface_str == "IMPLICIT_L1_FRIC")
    {
        return IMPLICIT_L1_FRIC;
    }
    else if(torque_interface_str == "COULOMB_OBSERVER")
    {
        return COULOMB_OBSERVER;
    }
    else if(torque_interface_str == "ASYM_FRICTION")
    {
        return ASYM_FRICTION;
    }
    else if(torque_interface_str == "STATIC_FRICTION")
    {
        return STATIC_FRICTION;
    }
    else
    {
        std::cout<<"ERROR: torque_interface_str is not defined in the yaml file" << std::endl;
        return INERTIA_RESHAPING;
    }
}


void ControllerGain::read_JOINT_PD_Gains(const YAML::Node& node)
{
    //JOINT_PD_Gains
    try
    {
        const YAML::Node& joint_pd_gains_node = node["joint_pd_gains"];

        if (!joint_pd_gains_node.IsDefined() || joint_pd_gains_node.IsNull())
        {
            std::cout<<"joint_pd_gains is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> Kp_vec = joint_pd_gains_node["Kp"].as<std::vector<double>>();
            std::vector<double> Kd_vec = joint_pd_gains_node["Kd"].as<std::vector<double>>();
            joint_pd_gains_.Kp.resize(Kp_vec.size(),Kp_vec.size()); joint_pd_gains_.Kp.setZero();
            joint_pd_gains_.Kd.resize(Kd_vec.size(),Kd_vec.size()); joint_pd_gains_.Kd.setZero();
            joint_pd_gains_.Kp.diagonal() = Eigen::Map<Eigen::VectorXd>(Kp_vec.data(),Kp_vec.size());
            joint_pd_gains_.Kd.diagonal() = Eigen::Map<Eigen::VectorXd>(Kd_vec.data(),Kd_vec.size());

            const bool verbose = joint_pd_gains_node["verbose"].as<bool>();
            if(verbose)
            {   
                print_matrix(joint_pd_gains_.Kp,"joint_pd_gains_.Kp");
                print_matrix(joint_pd_gains_.Kd,"joint_pd_gains_.Kd");
            }            
        }
    }
    catch(const std::exception& e)
    {   
        std::cout<<"ERROR: joint_pd_gains are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';
    }
}

void ControllerGain::read_JOINT_NRIC_Gains(const YAML::Node& node)
{
    //JOINT_PD_Gains
    try
    {
        const YAML::Node& joint_nric_gains_node = node["joint_nric_gains"];

        if (!joint_nric_gains_node.IsDefined() || joint_nric_gains_node.IsNull())
        {
            std::cout<<"joint_nric_gains_node is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> k1_vec = joint_nric_gains_node["k1"].as<std::vector<double>>();
            std::vector<double> k2_vec = joint_nric_gains_node["k2"].as<std::vector<double>>();
            std::vector<double> K_vec = joint_nric_gains_node["K"].as<std::vector<double>>();
            std::vector<double> gamma_vec = joint_nric_gains_node["gamma"].as<std::vector<double>>();
            std::vector<double> reflected_inertia_vec = joint_nric_gains_node["reflected_inertia"].as<std::vector<double>>();
            std::vector<double> Kp_vec = joint_nric_gains_node["Kp"].as<std::vector<double>>();            
            std::vector<double> Ki_vec = joint_nric_gains_node["Ki"].as<std::vector<double>>();

            joint_nric_gains_.k1.resize(k1_vec.size(),k1_vec.size()); joint_nric_gains_.k1.setZero();
            joint_nric_gains_.k2.resize(k2_vec.size(),k2_vec.size()); joint_nric_gains_.k2.setZero();
            joint_nric_gains_.K.resize(K_vec.size(),K_vec.size()); joint_nric_gains_.K.setZero();
            joint_nric_gains_.gamma.resize(gamma_vec.size(),gamma_vec.size()); joint_nric_gains_.gamma.setZero();
            joint_nric_gains_.reflected_inertia.resize(reflected_inertia_vec.size(),reflected_inertia_vec.size()); joint_nric_gains_.reflected_inertia.setZero();
            joint_nric_gains_.Kp.resize(Kp_vec.size(),Kp_vec.size()); joint_nric_gains_.Kp.setZero();            
            joint_nric_gains_.Ki.resize(Ki_vec.size(),Ki_vec.size()); joint_nric_gains_.Ki.setZero();

            joint_nric_gains_.k1.diagonal() = Eigen::Map<Eigen::VectorXd>(k1_vec.data(),k1_vec.size());
            joint_nric_gains_.k2.diagonal() = Eigen::Map<Eigen::VectorXd>(k2_vec.data(),k2_vec.size());
            joint_nric_gains_.K.diagonal() = Eigen::Map<Eigen::VectorXd>(K_vec.data(),K_vec.size());
            joint_nric_gains_.gamma.diagonal() = Eigen::Map<Eigen::VectorXd>(gamma_vec.data(),gamma_vec.size());
            joint_nric_gains_.reflected_inertia.diagonal() = Eigen::Map<Eigen::VectorXd>(reflected_inertia_vec.data(),reflected_inertia_vec.size());
            joint_nric_gains_.Kp.diagonal() = Eigen::Map<Eigen::VectorXd>(Kp_vec.data(),Kp_vec.size());            
            joint_nric_gains_.Ki.diagonal() = Eigen::Map<Eigen::VectorXd>(Ki_vec.data(),Ki_vec.size());


            const bool verbose = joint_nric_gains_node["verbose"].as<bool>();
            if(verbose)
            {
                print_matrix(joint_nric_gains_.k1,"joint_nric_gains_.k1");
                print_matrix(joint_nric_gains_.k2,"joint_nric_gains_.k2");
                print_matrix(joint_nric_gains_.K,"joint_nric_gains_.K");
                print_matrix(joint_nric_gains_.gamma,"joint_nric_gains_.gamma");
                print_matrix(joint_nric_gains_.reflected_inertia,"joint_nric_gains_.reflected_inertia");
                print_matrix(joint_nric_gains_.Kp,"joint_nric_gains_.Kp");
                print_matrix(joint_nric_gains_.Ki,"joint_nric_gains_.Ki");
            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: joint_nric_gains are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}

void ControllerGain::read_TASK_PD_Gains(const YAML::Node& node)
{
    //JOINT_PD_Gains
    try
    {
        const YAML::Node& task_pd_gains_node = node["task_pd_gains"];

        if (!task_pd_gains_node.IsDefined() || task_pd_gains_node.IsNull())
        {
            std::cout<<"task_pd_gains is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> Kp_vec = task_pd_gains_node["Kp"].as<std::vector<double>>();
            std::vector<double> Kd_vec = task_pd_gains_node["Kd"].as<std::vector<double>>();
            task_pd_gains_.Kp.resize(Kp_vec.size(),Kp_vec.size()); task_pd_gains_.Kp.setZero();
            task_pd_gains_.Kd.resize(Kd_vec.size(),Kd_vec.size()); task_pd_gains_.Kd.setZero();
            task_pd_gains_.Kp.diagonal() = Eigen::Map<Eigen::VectorXd>(Kp_vec.data(),Kp_vec.size());
            task_pd_gains_.Kd.diagonal() = Eigen::Map<Eigen::VectorXd>(Kd_vec.data(),Kd_vec.size());

            const bool verbose = task_pd_gains_node["verbose"].as<bool>();
            if(verbose)
            {
                print_matrix(task_pd_gains_.Kp,"task_pd_gains_.Kp");
                print_matrix(task_pd_gains_.Kd,"task_pd_gains_.Kd");
            }            
        }
    }
    catch(const std::exception& e)
    {   
        std::cout<<"ERROR: task_pd_gains are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';
    }
}

void ControllerGain::read_TASK_NRIC_Gains(const YAML::Node& node)
{
    //JOINT_PD_Gains
    try
    {
        const YAML::Node& task_nric_gains_node = node["task_nric_gains"];

        if (!task_nric_gains_node.IsDefined() || task_nric_gains_node.IsNull())
        {
            std::cout<<"task_nric_gains_node is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> k1_vec = task_nric_gains_node["k1"].as<std::vector<double>>();
            std::vector<double> k2_vec = task_nric_gains_node["k2"].as<std::vector<double>>();
            std::vector<double> K_vec = task_nric_gains_node["K"].as<std::vector<double>>();
            std::vector<double> gamma_vec = task_nric_gains_node["gamma"].as<std::vector<double>>();
            std::vector<double> reflected_inertia_vec = task_nric_gains_node["reflected_inertia"].as<std::vector<double>>();
            std::vector<double> Kp_vec = task_nric_gains_node["Kp"].as<std::vector<double>>();            
            std::vector<double> Ki_vec = task_nric_gains_node["Ki"].as<std::vector<double>>();

            task_nric_gains_.k1.resize(k1_vec.size(),k1_vec.size()); task_nric_gains_.k1.setZero();
            task_nric_gains_.k2.resize(k2_vec.size(),k2_vec.size()); task_nric_gains_.k2.setZero();
            task_nric_gains_.K.resize(K_vec.size(),K_vec.size()); task_nric_gains_.K.setZero();
            task_nric_gains_.gamma.resize(gamma_vec.size(),gamma_vec.size()); task_nric_gains_.gamma.setZero();
            task_nric_gains_.reflected_inertia.resize(reflected_inertia_vec.size(),reflected_inertia_vec.size()); task_nric_gains_.reflected_inertia.setZero();
            task_nric_gains_.Kp.resize(Kp_vec.size(),Kp_vec.size()); task_nric_gains_.Kp.setZero();            
            task_nric_gains_.Ki.resize(Ki_vec.size(),Ki_vec.size()); task_nric_gains_.Ki.setZero();

            task_nric_gains_.k1.diagonal() = Eigen::Map<Eigen::VectorXd>(k1_vec.data(),k1_vec.size());
            task_nric_gains_.k2.diagonal() = Eigen::Map<Eigen::VectorXd>(k2_vec.data(),k2_vec.size());
            task_nric_gains_.K.diagonal() = Eigen::Map<Eigen::VectorXd>(K_vec.data(),K_vec.size());
            task_nric_gains_.gamma.diagonal() = Eigen::Map<Eigen::VectorXd>(gamma_vec.data(),gamma_vec.size());
            task_nric_gains_.reflected_inertia.diagonal() = Eigen::Map<Eigen::VectorXd>(reflected_inertia_vec.data(),reflected_inertia_vec.size());
            task_nric_gains_.Kp.diagonal() = Eigen::Map<Eigen::VectorXd>(Kp_vec.data(),Kp_vec.size());            
            task_nric_gains_.Ki.diagonal() = Eigen::Map<Eigen::VectorXd>(Ki_vec.data(),Ki_vec.size());


            const bool verbose = task_nric_gains_node["verbose"].as<bool>();
            if(verbose)
            {
                print_matrix(task_nric_gains_.k1,"task_nric_gains_.k1");
                print_matrix(task_nric_gains_.k2,"task_nric_gains_.k2");
                print_matrix(task_nric_gains_.K,"task_nric_gains_.K");
                print_matrix(task_nric_gains_.gamma,"task_nric_gains_.gamma");
                print_matrix(task_nric_gains_.reflected_inertia,"task_nric_gains_.reflected_inertia");
                print_matrix(task_nric_gains_.Kp,"task_nric_gains_.Kp");
                print_matrix(task_nric_gains_.Ki,"task_nric_gains_.Ki");

            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: joint_nric_gains are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}

void ControllerGain::read_JOINT_FRIC_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& joint_fric_gains_node = node["joint_fric_gains"];

        if (!joint_fric_gains_node.IsDefined() || joint_fric_gains_node.IsNull())
        {
            std::cout<<"joint_fric_gains_node is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> Kp_vec = joint_fric_gains_node["Kp"].as<std::vector<double>>();
            std::vector<double> Kd_vec = joint_fric_gains_node["Kd"].as<std::vector<double>>();
            std::vector<double> joint_stiffness_vec = joint_fric_gains_node["joint_stiffness_matrix"].as<std::vector<double>>();
            std::string torque_interface_str = joint_fric_gains_node["torque_interface"].as<std::string>();

            joint_fric_gains_.Kp.resize(Kp_vec.size(),Kp_vec.size()); 
            joint_fric_gains_.Kp.setZero();

            joint_fric_gains_.Kd.resize(Kd_vec.size(),Kd_vec.size()); 
            joint_fric_gains_.Kd.setZero();

            joint_fric_gains_.joint_stiffness_matrix.resize(joint_stiffness_vec.size(),joint_stiffness_vec.size()); 
            joint_fric_gains_.joint_stiffness_matrix.setZero();

            joint_fric_gains_.Kp.diagonal() = Eigen::Map<Eigen::VectorXd>(Kp_vec.data(),Kp_vec.size());
            joint_fric_gains_.Kd.diagonal() = Eigen::Map<Eigen::VectorXd>(Kd_vec.data(),Kd_vec.size());
            joint_fric_gains_.joint_stiffness_matrix.diagonal() = Eigen::Map<Eigen::VectorXd>(joint_stiffness_vec.data(),joint_stiffness_vec.size());

            joint_fric_gains_.torque_interface = this->select_torque_interface(torque_interface_str);
            const bool verbose = joint_fric_gains_node["verbose"].as<bool>();
            if(verbose)
            {   
                std::cout<<"controller interface : " << torque_interface_str << std::endl;
                print_matrix(joint_fric_gains_.Kp,"joint_fric_gains_.Kp");
                print_matrix(joint_fric_gains_.Kd,"joint_fric_gains_.Kd");
                print_matrix(joint_fric_gains_.joint_stiffness_matrix,"joint_fric_gains_.joint_stiffness_matrix");
            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: joint_fric_gains are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}

void ControllerGain::read_TASK_FRIC_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& task_fric_gains_node = node["task_fric_gains"];

        if (!task_fric_gains_node.IsDefined() || task_fric_gains_node.IsNull())
        {
            std::cout<<"task_fric_gains_node is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> Kp_vec = task_fric_gains_node["Kp"].as<std::vector<double>>();
            std::vector<double> Kd_vec = task_fric_gains_node["Kd"].as<std::vector<double>>();
            std::vector<double> joint_stiffness_vec = task_fric_gains_node["joint_stiffness_matrix"].as<std::vector<double>>();
            std::string torque_interface_str = task_fric_gains_node["torque_interface"].as<std::string>();

            task_fric_gains_.Kp.resize(Kp_vec.size(),Kp_vec.size()); 
            task_fric_gains_.Kp.setZero();

            task_fric_gains_.Kd.resize(Kd_vec.size(),Kd_vec.size()); 
            task_fric_gains_.Kd.setZero();

            task_fric_gains_.joint_stiffness_matrix.resize(joint_stiffness_vec.size(),joint_stiffness_vec.size()); 
            task_fric_gains_.joint_stiffness_matrix.setZero();

            task_fric_gains_.Kp.diagonal() = Eigen::Map<Eigen::VectorXd>(Kp_vec.data(),Kp_vec.size());
            task_fric_gains_.Kd.diagonal() = Eigen::Map<Eigen::VectorXd>(Kd_vec.data(),Kd_vec.size());
            task_fric_gains_.joint_stiffness_matrix.diagonal() = Eigen::Map<Eigen::VectorXd>(joint_stiffness_vec.data(),joint_stiffness_vec.size());

            task_fric_gains_.torque_interface = this->select_torque_interface(torque_interface_str);
            const bool verbose = task_fric_gains_node["verbose"].as<bool>();
            if(verbose)
            {   
                std::cout<<"controller interface : " << torque_interface_str << std::endl;
                print_matrix(task_fric_gains_.Kp,"task_fric_gains_.Kp");
                print_matrix(task_fric_gains_.Kd,"task_fric_gains_.Kd");
                print_matrix(task_fric_gains_.joint_stiffness_matrix,"task_fric_gains_.joint_stiffness_matrix");
            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: task_fric_gains are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}


void ControllerGain::read_FIRST_FRIC_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& gain_node = node["first_order_fric_gain"];

        if (!gain_node.IsDefined() || gain_node.IsNull())
        {
            std::cout<<"first_order_fric_gain is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> L_vec = gain_node["L"].as<std::vector<double>>();
            std::vector<double> motor_inertia_vec = gain_node["motor_inertia_matrix"].as<std::vector<double>>();

            first_fric_gains_.L.resize(L_vec.size(),L_vec.size()); 
            first_fric_gains_.L.setZero();

            first_fric_gains_.motor_inertia_matrix.resize(motor_inertia_vec.size(),motor_inertia_vec.size()); 
            first_fric_gains_.motor_inertia_matrix.setZero();    

            first_fric_gains_.L.diagonal() = Eigen::Map<Eigen::VectorXd>(L_vec.data(),L_vec.size());
            first_fric_gains_.motor_inertia_matrix.diagonal() = Eigen::Map<Eigen::VectorXd>(motor_inertia_vec.data(),motor_inertia_vec.size());


            const bool verbose = gain_node["verbose"].as<bool>();
            if(verbose)
            {
                print_matrix(first_fric_gains_.L,"first_order_fric_gain.L");
                print_matrix(first_fric_gains_.motor_inertia_matrix,"first_order_fric_gain.motor_inertia_matrix");
            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: first_order_fric_gain are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}

void ControllerGain::read_SECOND_FRIC_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& gain_node = node["second_order_fric_gain"];

        if (!gain_node.IsDefined() || gain_node.IsNull())
        {
            std::cout<<"second_order_fric_gain is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> L_vec = gain_node["L"].as<std::vector<double>>();
            std::vector<double> Lp_vec = gain_node["Lp"].as<std::vector<double>>();
            std::vector<double> motor_inertia_vec = gain_node["motor_inertia_matrix"].as<std::vector<double>>();

            second_fric_gains_.L.resize(L_vec.size(),L_vec.size()); 
            second_fric_gains_.L.setZero();

            second_fric_gains_.Lp.resize(Lp_vec.size(),Lp_vec.size()); 
            second_fric_gains_.Lp.setZero();

            second_fric_gains_.motor_inertia_matrix.resize(motor_inertia_vec.size(),motor_inertia_vec.size()); 
            second_fric_gains_.motor_inertia_matrix.setZero();
     

            second_fric_gains_.L.diagonal() = Eigen::Map<Eigen::VectorXd>(L_vec.data(),L_vec.size());
            second_fric_gains_.Lp.diagonal() = Eigen::Map<Eigen::VectorXd>(Lp_vec.data(),Lp_vec.size());
            second_fric_gains_.motor_inertia_matrix.diagonal() = Eigen::Map<Eigen::VectorXd>(motor_inertia_vec.data(),motor_inertia_vec.size());

            const bool verbose = gain_node["verbose"].as<bool>();
            if(verbose)
            {
                print_matrix(second_fric_gains_.L,"second_fric_gains_.L");
                print_matrix(second_fric_gains_.Lp,"second_fric_gains_.Lp");
                print_matrix(second_fric_gains_.motor_inertia_matrix,"second_fric_gains_.motor_inertia_matrix");
            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: second_fric_gains_ are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}

void ControllerGain::read_THIRD_FRIC_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& gain_node = node["third_order_fric_gain"];

        if (!gain_node.IsDefined() || gain_node.IsNull())
        {
            std::cout<<"thrid_fric_gains_ is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> L_vec = gain_node["L"].as<std::vector<double>>();
            std::vector<double> Lp_vec = gain_node["Lp"].as<std::vector<double>>();
            std::vector<double> Li_vec = gain_node["Li"].as<std::vector<double>>();
            std::vector<double> motor_inertia_vec = gain_node["motor_inertia_matrix"].as<std::vector<double>>();

            thrid_fric_gains_.L.resize(L_vec.size(),L_vec.size()); 
            thrid_fric_gains_.L.setZero();

            thrid_fric_gains_.Lp.resize(Lp_vec.size(),Lp_vec.size()); 
            thrid_fric_gains_.Lp.setZero();

            thrid_fric_gains_.Li.resize(Li_vec.size(),Li_vec.size()); 
            thrid_fric_gains_.Li.setZero();

            thrid_fric_gains_.motor_inertia_matrix.resize(motor_inertia_vec.size(),motor_inertia_vec.size()); 
            thrid_fric_gains_.motor_inertia_matrix.setZero();
     

            thrid_fric_gains_.L.diagonal() = Eigen::Map<Eigen::VectorXd>(L_vec.data(),L_vec.size());
            thrid_fric_gains_.Lp.diagonal() = Eigen::Map<Eigen::VectorXd>(Lp_vec.data(),Lp_vec.size());
            thrid_fric_gains_.Li.diagonal() = Eigen::Map<Eigen::VectorXd>(Li_vec.data(),Li_vec.size());
            thrid_fric_gains_.motor_inertia_matrix.diagonal() = Eigen::Map<Eigen::VectorXd>(motor_inertia_vec.data(),motor_inertia_vec.size());

            const bool verbose = gain_node["verbose"].as<bool>();
            if(verbose)
            {
                print_matrix(thrid_fric_gains_.L,"thrid_fric_gains_.L");
                print_matrix(thrid_fric_gains_.Lp,"thrid_fric_gains_.Lp");
                print_matrix(thrid_fric_gains_.Li,"thrid_fric_gains_.Li");
                print_matrix(thrid_fric_gains_.motor_inertia_matrix,"thrid_fric_gains_.motor_inertia_matrix");
            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: thrid_fric_gains_ are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}

void ControllerGain::read_INERTIA_RESHAPING_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& gain_node = node["inertia_reshaping_gain"];

        if (!gain_node.IsDefined() || gain_node.IsNull())
        {
            std::cout<<"inertia_reshaping_gains_ is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> motor_inertia_vec = gain_node["motor_inertia_matrix"].as<std::vector<double>>();
            std::vector<double> desired_motor_inertia_vec = gain_node["desired_motor_inertia"].as<std::vector<double>>();

            inertia_reshaping_gains_.motor_inertia_matrix.resize(motor_inertia_vec.size(),motor_inertia_vec.size()); 
            inertia_reshaping_gains_.motor_inertia_matrix.setZero();

            inertia_reshaping_gains_.desired_motor_inertia_matrix.resize(desired_motor_inertia_vec.size(),desired_motor_inertia_vec.size()); 
            inertia_reshaping_gains_.desired_motor_inertia_matrix.setZero();
         

            inertia_reshaping_gains_.motor_inertia_matrix.diagonal() = Eigen::Map<Eigen::VectorXd>(motor_inertia_vec.data(),motor_inertia_vec.size());          
            inertia_reshaping_gains_.desired_motor_inertia_matrix.diagonal() = Eigen::Map<Eigen::VectorXd>(desired_motor_inertia_vec.data(),desired_motor_inertia_vec.size());          

            const bool verbose = gain_node["verbose"].as<bool>();
            if(verbose)
            {
                print_matrix(inertia_reshaping_gains_.motor_inertia_matrix,"inertia_reshaping_gains_.motor_inertia_matrix");
                print_matrix(inertia_reshaping_gains_.desired_motor_inertia_matrix,"inertia_reshaping_gains_.desired_motor_inertia_matrix");
            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: inertia_reshaping_gains_ are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}

void ControllerGain::read_L1_FRIC_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& gain_node = node["l1_fric_gain"];

        if (!gain_node.IsDefined() || gain_node.IsNull())
        {
            std::cout<<"l1_fric_gains_ is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> As_vec = gain_node["As"].as<std::vector<double>>();
            std::vector<double> Gamma_vec = gain_node["Gamma"].as<std::vector<double>>();
            std::vector<double> motor_inertia_vec = gain_node["motor_inertia_matrix"].as<std::vector<double>>();
            std::vector<double> W_vec = gain_node["W"].as<std::vector<double>>();

            l1_fric_gains_.As.resize(As_vec.size(),As_vec.size()); 
            l1_fric_gains_.As.setZero();

            l1_fric_gains_.Gamma.resize(Gamma_vec.size(),Gamma_vec.size()); 
            l1_fric_gains_.Gamma.setZero();

            l1_fric_gains_.motor_inertia_matrix.resize(motor_inertia_vec.size(),motor_inertia_vec.size()); 
            l1_fric_gains_.motor_inertia_matrix.setZero();

            l1_fric_gains_.W.resize(W_vec.size(),W_vec.size()); 
            l1_fric_gains_.W.setZero();

            l1_fric_gains_.As.diagonal() = Eigen::Map<Eigen::VectorXd>(As_vec.data(),As_vec.size());
            l1_fric_gains_.Gamma.diagonal() = Eigen::Map<Eigen::VectorXd>(Gamma_vec.data(),Gamma_vec.size());
            l1_fric_gains_.motor_inertia_matrix.diagonal() = Eigen::Map<Eigen::VectorXd>(motor_inertia_vec.data(),motor_inertia_vec.size());
            l1_fric_gains_.W.diagonal() = Eigen::Map<Eigen::VectorXd>(W_vec.data(),W_vec.size());
            
            const bool verbose = gain_node["verbose"].as<bool>();
            if(verbose)
            {
                print_matrix(l1_fric_gains_.As,"l1_fric_gains_.As");
                print_matrix(l1_fric_gains_.Gamma,"l1_fric_gains_.Gamma");
                print_matrix(l1_fric_gains_.motor_inertia_matrix,"l1_fric_gains_.motor_inertia_matrix");
                print_matrix(l1_fric_gains_.W,"l1_fric_gains_.W");
            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: l1_fric_gains_ are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}

void ControllerGain::read_IMPLICIT_L1_FRIC_Gains(const YAML::Node& node)
{    
    try
    {
        const YAML::Node& gain_node = node["implicit_l1_fric_gain"];

        if (!gain_node.IsDefined() || gain_node.IsNull())
        {
            std::cout<<"implicit_l1_fric_gains_ is not defined or null" << std::endl;
        }
        else
        {
            std::vector<double> Gamma_vec = gain_node["Gamma"].as<std::vector<double>>();
            std::vector<double> Gamma_p_vec = gain_node["Gamma_p"].as<std::vector<double>>();
            std::vector<double> motor_inertia_vec = gain_node["motor_inertia_matrix"].as<std::vector<double>>();
            std::vector<double> W_vec = gain_node["W"].as<std::vector<double>>();

            implicit_l1_fric_gains_.Gamma.resize(Gamma_vec.size(),Gamma_vec.size()); 
            implicit_l1_fric_gains_.Gamma.setZero();

            implicit_l1_fric_gains_.Gamma_p.resize(Gamma_p_vec.size(),Gamma_p_vec.size()); 
            implicit_l1_fric_gains_.Gamma_p.setZero();

            implicit_l1_fric_gains_.motor_inertia_matrix.resize(motor_inertia_vec.size(),motor_inertia_vec.size()); 
            implicit_l1_fric_gains_.motor_inertia_matrix.setZero();

            implicit_l1_fric_gains_.W.resize(W_vec.size(),W_vec.size()); 
            implicit_l1_fric_gains_.W.setZero();

            implicit_l1_fric_gains_.Gamma.diagonal() = Eigen::Map<Eigen::VectorXd>(Gamma_vec.data(),Gamma_vec.size());
            implicit_l1_fric_gains_.Gamma_p.diagonal() = Eigen::Map<Eigen::VectorXd>(Gamma_p_vec.data(),Gamma_p_vec.size());
            implicit_l1_fric_gains_.motor_inertia_matrix.diagonal() = Eigen::Map<Eigen::VectorXd>(motor_inertia_vec.data(),motor_inertia_vec.size());
            implicit_l1_fric_gains_.W.diagonal() = Eigen::Map<Eigen::VectorXd>(W_vec.data(),W_vec.size());
            
            const bool verbose = gain_node["verbose"].as<bool>();
            if(verbose)
            {
                print_matrix(implicit_l1_fric_gains_.Gamma,"implicit_l1_fric_gains_.Gamma");
                print_matrix(implicit_l1_fric_gains_.Gamma_p,"implicit_l1_fric_gains_.Gamma_p");
                print_matrix(implicit_l1_fric_gains_.motor_inertia_matrix,"implicit_l1_fric_gains_.motor_inertia_matrix");
                print_matrix(implicit_l1_fric_gains_.W,"implicit_l1_fric_gains_.W");
            }            
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"ERROR: implicit_l1_fric_gains_ are poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';  
    }
}

void ControllerGain::read_COULOMB_OBSERVER_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& gain_node = node["coulomb_observer_gain"];

        if (!gain_node.IsDefined() || gain_node.IsNull())
        {
            std::cout << "coulomb_observer_gain is not defined or null" << std::endl;
        }
        else
        {
            const std::vector<double> K_vec = gain_node["K"].as<std::vector<double>>();
            const std::vector<double> L_vec = gain_node["L"].as<std::vector<double>>();
            const std::vector<double> motor_inertia_vec =
                gain_node["motor_inertia_matrix"].as<std::vector<double>>();

            coulomb_observer_gains_.K = Eigen::MatrixXd::Zero(K_vec.size(), K_vec.size());
            coulomb_observer_gains_.L = Eigen::MatrixXd::Zero(L_vec.size(), L_vec.size());
            coulomb_observer_gains_.motor_inertia_matrix =
                Eigen::MatrixXd::Zero(motor_inertia_vec.size(), motor_inertia_vec.size());

            coulomb_observer_gains_.K.diagonal() =
                Eigen::Map<const Eigen::VectorXd>(K_vec.data(), K_vec.size());
            coulomb_observer_gains_.L.diagonal() =
                Eigen::Map<const Eigen::VectorXd>(L_vec.data(), L_vec.size());
            coulomb_observer_gains_.motor_inertia_matrix.diagonal() =
                Eigen::Map<const Eigen::VectorXd>(motor_inertia_vec.data(), motor_inertia_vec.size());

            const bool verbose = gain_node["verbose"].as<bool>();
            if (verbose)
            {
                print_matrix(coulomb_observer_gains_.K, "coulomb_observer_gains_.K");
                print_matrix(coulomb_observer_gains_.L, "coulomb_observer_gains_.L");
                print_matrix(
                    coulomb_observer_gains_.motor_inertia_matrix,
                    "coulomb_observer_gains_.motor_inertia_matrix");
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cout << "ERROR: coulomb_observer_gain is poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';
    }
}

void ControllerGain::read_ASYM_FRICTION_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& gain_node = node["asym_friction_gain"];

        if (!gain_node.IsDefined() || gain_node.IsNull())
        {
            std::cout << "asym_friction_gain is not defined or null" << std::endl;
            return;
        }

        const auto read_vector = [&gain_node](const char* key)
        {
            const std::vector<double> values = gain_node[key].as<std::vector<double>>();
            if (values.empty())
            {
                return Eigen::VectorXd();
            }
            return Eigen::VectorXd(Eigen::Map<const Eigen::VectorXd>(values.data(), values.size()));
        };

        asym_friction_gains_.f_v1_p = read_vector("f_v1_p");
        asym_friction_gains_.f_v1_n = read_vector("f_v1_n");
        asym_friction_gains_.f_v2_p = read_vector("f_v2_p");
        asym_friction_gains_.f_v2_n = read_vector("f_v2_n");
        asym_friction_gains_.f_v3_p = read_vector("f_v3_p");
        asym_friction_gains_.f_v3_n = read_vector("f_v3_n");
        asym_friction_gains_.f_s_p = read_vector("f_s_p");
        asym_friction_gains_.f_s_n = read_vector("f_s_n");
        asym_friction_gains_.f_c_p = read_vector("f_c_p");
        asym_friction_gains_.f_c_n = read_vector("f_c_n");
        asym_friction_gains_.qdot_s_p = read_vector("qdot_s_p");
        asym_friction_gains_.qdot_s_n = read_vector("qdot_s_n");
        asym_friction_gains_.qdot_c_p = read_vector("qdot_c_p");
        asym_friction_gains_.qdot_c_n = read_vector("qdot_c_n");
        asym_friction_gains_.delta_s_p = read_vector("delta_s_p");
        asym_friction_gains_.delta_s_n = read_vector("delta_s_n");
        asym_friction_gains_.delta_c_p = read_vector("delta_c_p");
        asym_friction_gains_.delta_c_n = read_vector("delta_c_n");
        asym_friction_gains_.f_l1_p = read_vector("f_l1_p");
        asym_friction_gains_.f_l1_n = read_vector("f_l1_n");
        asym_friction_gains_.f_l2_p = read_vector("f_l2_p");
        asym_friction_gains_.f_l2_n = read_vector("f_l2_n");
        asym_friction_gains_.f_l3_p = read_vector("f_l3_p");
        asym_friction_gains_.f_l3_n = read_vector("f_l3_n");
        asym_friction_gains_.f_l4_p = read_vector("f_l4_p");
        asym_friction_gains_.f_l4_n = read_vector("f_l4_n");
        asym_friction_gains_.f_T1 = read_vector("f_T1");
        asym_friction_gains_.f_T2 = read_vector("f_T2");
        asym_friction_gains_.f_T3 = read_vector("f_T3");
        asym_friction_gains_.f_T4 = read_vector("f_T4");
        asym_friction_gains_.mu = read_vector("mu");

        if (gain_node["verbose"].as<bool>())
        {
            print_matrix(asym_friction_gains_.f_v1_p, "asym_friction_gains_.f_v1_p");
            print_matrix(asym_friction_gains_.f_v1_n, "asym_friction_gains_.f_v1_n");
            print_matrix(asym_friction_gains_.f_c_p, "asym_friction_gains_.f_c_p");
            print_matrix(asym_friction_gains_.f_c_n, "asym_friction_gains_.f_c_n");
            print_matrix(asym_friction_gains_.mu, "asym_friction_gains_.mu");
        }
    }
    catch(const std::exception& e)
    {
        std::cout << "ERROR: asym_friction_gain is poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';
    }
}

void ControllerGain::read_STATIC_FRICTION_Gains(const YAML::Node& node)
{
    try
    {
        const YAML::Node& gain_node = node["static_friction_gain"];

        if (!gain_node.IsDefined() || gain_node.IsNull())
        {
            std::cout << "static_friction_gain is not defined or null" << std::endl;
            return;
        }

        const auto read_diagonal = [&gain_node](const char* key)
        {
            const std::vector<double> values = gain_node[key].as<std::vector<double>>();
            Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(values.size(), values.size());
            if (!values.empty())
            {
                matrix.diagonal() =
                    Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
            }
            return matrix;
        };

        static_friction_gains_.Fc = read_diagonal("Fc");
        static_friction_gains_.Fv1 = read_diagonal("Fv1");
        static_friction_gains_.Fv2 = read_diagonal("Fv2");
        static_friction_gains_.friction_compensation_weights =
            read_diagonal("friction_compensation_weights");

        if (gain_node["verbose"].as<bool>())
        {
            print_matrix(static_friction_gains_.Fc, "static_friction_gains_.Fc");
            print_matrix(static_friction_gains_.Fv1, "static_friction_gains_.Fv1");
            print_matrix(static_friction_gains_.Fv2, "static_friction_gains_.Fv2");
            print_matrix(
                static_friction_gains_.friction_compensation_weights,
                "static_friction_gains_.friction_compensation_weights");
        }
    }
    catch(const std::exception& e)
    {
        std::cout << "ERROR: static_friction_gain is poorly defined in the yaml file" << std::endl;
        std::cerr << e.what() << '\n';
    }
}
