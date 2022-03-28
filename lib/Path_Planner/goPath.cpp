#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "gpmp.h"
#include <math.h>

// Override Constructor ------------------------------------------------
root::MeanTraj::MeanTraj(int DOF, int num_of_steps, int ocv) {

    // Prior Variables --------------------------
    DsOF = DOF; // Degrees of Freedom

    init_parameter.resize(4,DOF); // input required to initiate a DOFs path

    steps = num_of_steps; // trajectory steps

    B.resize(steps+2,steps+1); // MATRIX

    Q.resize(steps+2,steps+2); // MATRIX

    Ka_inv.resize(steps+1,steps+1); // ACCELERATION INVERSE KERNAL

    Kv_inv.resize(steps+1,steps+1); // VELOCITY INVERSE KERNAL

    Kv.resize(steps+1,steps+1); // VELOCITY KERNAL

    Kp_inv.resize(steps+1,steps+1); // POSITION INVERSE KERNAL

    Kp.resize(steps+1,steps+1); // POSITION KERNAL
    // -------------------------------------------


    // Optimization Variabels --------------------
    ocv_size = ocv; // DOFs contrained by workspace objects

    mag_ocv.resize(1,steps+1); // Velocity vector magnitude for each DOF

    J.resize(steps+1,ocv_size); // Jacobian

    unit_prime.resize(ocv_size,steps+1); 

    unit_dubprime.resize(ocv_size,steps+1);

    del_c.resize(ocv_size,1);

    final_pos.resize(steps+1,ocv_size); // FINAL POSITION TRAJECTORY

    mean_pos.resize(steps+1,ocv_size); // NON OPTIMIZED POSITION TRAJECTORY

    final_vel.resize(steps+1,ocv_size); // FINAL VELOCITY TRAJECTORY

    mean_vel.resize(steps+1,ocv_size); // NON OPTIMIZED VELOCITY TRAJECTORY

    I = Eigen::Matrix<float,3,3>::Identity(); // identity matrix

    IJ = I;

    g_pos.resize(ocv,steps+1);

    g_vel.resize(ocv,steps+1);

    obs.resize(ocv,1); // OBSTICAL MATRIX

    check_pos.resize(steps+1,ocv_size);
    // -------------------------------------------
}
// --------------------------------------------------------------------


// Default Destructor -------------------------------------------------
root::MeanTraj::~MeanTraj(){ 
}
// --------------------------------------------------------------------



// Prior -------------------------------------------------------------------------------------------------------------- **

// Define each DOF w/ B.C.s -------------------------------------------
void root::MeanTraj::add_DOF(float a, float v0, float p0, float pf, int ForThisDOF, bool ocv) {

    // Input parameters / B.C.s ---------------------
    init_parameter(ACCEL,ForThisDOF) = a;
    init_parameter(V0,ForThisDOF) = v0;
    init_parameter(P0,ForThisDOF) = p0; 
    init_parameter(Pf,ForThisDOF) = pf;
    // ----------------------------------------------


    // Determine the max time -----------------------
    if (a == 0) {
        final_time.push_back((pf-p0)/v0);
    }
    else {
        float tplus = (-v0 + sqrt(pow(v0,2)-4*(a/2)*(p0-pf)))/a;
        float tminus = (-v0 - sqrt(pow(v0,2)-4*(a/2)*(p0-pf)))/a;
        if (tplus > tminus) {
            final_time.push_back(abs(tplus));
        }
        else {
            final_time.push_back(abs(tminus));
        }
    }
    if (final_time[ForThisDOF] >= max_time) {
        max_time = final_time[ForThisDOF];
    }
    // ----------------------------------------------


    // If defined, add Object Concering Variable ----
    if (ocv) {
        add_ocv(ForThisDOF);
    }
    // ----------------------------------------------


    // If all DOF added, setup obtimization ---------
    if (ForThisDOF+1 == DsOF) {
        if (max_time == 0) {
            std::cout << "No action specified\n";
            std::cout << "------------------------------------\n";
            std::cout << "Need at least one DOF to have an end pose different that its current pose\n\n";
        }

        define_step(); // Compute step size of trajectory
        
        update_accel(); // Update acceleration values for each DOF based on max time
        
        define_states(); // Define mean position and velocity trajectory

        define_B(); // Matrix for solving KERNALS
        
        define_Q(); // Matrix dor solving KERNALS
        
        define_Ka_inv(); // DEFINE ACCELERATION KERNAL
        
        define_kernals(); // DEFINE POSITION AND VELOCITY KERNAL

        declare_optimization(); // Define variables needed for optimization
    }

    if (ForThisDOF+1 > DsOF) {
        std::cout << "TOO MANY DOF SPECIFIED" << std::endl;
    }
    //-----------------------------------------------
}
// --------------------------------------------------------------------

// Define trajectory time step ----------------------------------------
void root::MeanTraj::define_step() {
    step = max_time/steps;
}
// --------------------------------------------------------------------


// Update each DOF acceleration based on found DOF max time -----------
void root::MeanTraj::update_accel() {
    for (int i=0; i<DsOF-1;i++) {
        init_parameter(ACCEL,i) = 2/pow(max_time,2)*((init_parameter(Pf,i)-init_parameter(P0,i))-init_parameter(V0,i)*max_time);
    }
}
// --------------------------------------------------------------------


// Define non optimized position and velocity trajectory --------------
void root::MeanTraj::define_states() {
    float time_step=0;
    int i=0;
    while (time_step < max_time+.01) {
        for (int j=0; j<ocv_size; j++) {
            mean_pos(i,j) = init_parameter(ACCEL,j)*pow(time_step,2)/2 + init_parameter(V0,j)*time_step + init_parameter(P0,j);
            mean_vel(i,j) = init_parameter(ACCEL,j)*time_step + init_parameter(V0,j);
        }
        time_step+=step;
        i+=1;
    }
}
// --------------------------------------------------------------------


// Define needed matrix -----------------------------------------------
void root::MeanTraj::define_B() {
    for (int i = 0; i<B.cols();i++) {
        B(i,i) = 1;
        B(i+1,i) = -1;
    }
}
// --------------------------------------------------------------------


// Define needed matrix -----------------------------------------------
void root::MeanTraj::define_Q() {
    for (int i=0; i<Q.cols(); i++) {
        Q(i,i) = sensitivity[2]*step;
    }
    //std::cout << step << std::endl;
}
// --------------------------------------------------------------------


// DEFINE ACCELERATION INVERSE KERNAL
void root::MeanTraj::define_Ka_inv() {
    Ka_inv = B.transpose()*Q.inverse()*B;
}
// --------------------------------------------------------------------


// DEFINE POSITION AND VELOCITY KERNALS
void root::MeanTraj::define_kernals() {
    Kv_inv = Ka_inv*(step*step/2);
    Kv = Ka_inv.inverse()*(step*step/2);
    Kp_inv = Kv_inv*(step*step/2);
    Kp = Kv*(step*step/2);
}
// --------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------- **


// Optimization -------------------------------------------------------------------------------------------------------- **
// Set algorithm sensitivity parameters ------------------------------
void root::MeanTraj::set_sensitivity(float whole, float ability, float power) {
    sensitivity.push_back(whole);
    sensitivity.push_back(ability);
    sensitivity.push_back(power); // for prior
}
// --------------------------------------------------------------------


// Add Object Concerning Variable -------------------------------------
void root::MeanTraj::add_ocv(int index) {
    ocv_idx.push_back(index);
}
// --------------------------------------------------------------------


// Declare optimization variables -------------------------------------
void root::MeanTraj::declare_optimization() { // Need to get rid of this step... It is unecessary as mean_traj is just acting as a place holder. In other words when I am defining the states I should be doing this as well
    for (int i=0;i<=steps;i++) {
        mag_ocv(0,i) = sqrt(pow(mean_vel(i,0),2)+pow(mean_vel(i,1),2)+pow(mean_vel(i,2),2));
        for (int j=0;j<ocv_size;j++) {
            J(i,j) = mean_vel(i,ocv_idx[j]);
            if (mag_ocv(i) == 0) {
                unit_prime(j,i) = 0;
            }
            else {
                unit_prime(j,i) = mean_vel(i,ocv_idx[j])/mag_ocv(i); // (ocv,s1)
            }
            unit_dubprime(j,i) = init_parameter(ACCEL,ocv_idx[j]); // (ocv,s1)
            final_pos(i,j) = mean_pos(i,j); // (ocv,s1)
            final_vel(i,j) = mean_vel(i,j); // (ocv,s1)
        }    
    }
    check_pos = final_pos;
}
// --------------------------------------------------------------------


// Iterative update on optimization -----------------------------------
void root::MeanTraj::update_optimizaion() {
    for (int i=0;i<=steps;i++) {
        mag_ocv(i) = sqrt(pow(final_vel(i,0),2)+pow(final_vel(i,1),2)+pow(final_vel(i,2),2)); // (1,s1)
        J.row(i) = final_vel.row(i);
        if (mag_ocv(i) == 0) {
            unit_prime(0,i) = 0;
            unit_prime(1,i) = 0;
            unit_prime(2,i) = 0;
        }
        else {
            unit_prime(0,i) = final_vel(i,0)/mag_ocv(i);
            unit_prime(1,i) = final_vel(i,1)/mag_ocv(i);
            unit_prime(2,i) = final_vel(i,2)/mag_ocv(i);
        }
    }
    check_pos = final_pos;

    update_g(1,.5);
}
// --------------------------------------------------------------------


// Optimize Trajectory ------------------------------------------------
void root::MeanTraj::optimize(std::vector<float> obj_x,std::vector<float> obj_y,std::vector<float> obj_z) {
    obs.resize(obj_x.size(),ocv_size);
    if (obj_x.size() != obj_y.size() || obj_x.size() != obj_z.size()) {
        exit(1);
    }
    for (int z=0; z<obj_x.size(); z++) {
        obs(z,0) = obj_x[z];
        obs(z,1) = obj_y[z];
        obs(z,2) = obj_z[z];
    }
    update_g(1,.5);


    for (int i=0;i<500;i++) { // number of iteration of optimizaion.  This needs a new criterion to define when it finishes
        final_pos = final_pos - (sensitivity[0])*Kp*(sensitivity[1]*Kp_inv*(final_pos-mean_pos) + g_pos.transpose());
        final_vel = final_vel - (sensitivity[0])*Kv*(sensitivity[1]*Kv_inv*(final_vel-mean_vel) + g_vel.transpose());
        
        /*
        if (i == 10 || i == 20 || i == 350 || i == 375 || i == 390 || i == 400 || i == 450) { //i == 10 || i == 20 || i == 350 || i == 375 || i == 390 || i == 400 || i == 450
            std::cout << final_pos.transpose() << std::endl;
        }
        */
        if (i == 1 || i == 3 || i == 5 || i == 7 || i == 9 || i == 11 || i == 13) { //i == 10 || i == 20 || i == 350 || i == 375 || i == 390 || i == 400 || i == 450
            std::cout << final_pos.transpose() << std::endl;
        }
        
        if (final_pos.isApprox(check_pos,.00000001)) {
            //std::cout << i << std::endl;
            break;
        }

        update_optimizaion();
    }
}
// --------------------------------------------------------------------


// Update specific optimization parameter g ---------------------------
void root::MeanTraj::update_g(float radius, float scale_error) {

    for (int i=0;i<=steps;i++) {
        update_J(i); 
        for (int k=0;k<obs.rows();k++) {
            for (int j=0;j<ocv_size;j++) {
                del_c(j) = obs(k,j) - final_pos(i,j); // vector computed distance between object and robot state(i).  Both vectors are taken from the robots initial position
                if (del_c(j) < radius && del_c(j) > 0) {
                    //std::cout << "positive  ";
                    //std:: cout << i << std::endl;
                    del_c(j) = scale_error*(radius-del_c(j));
                }
                else if (del_c(j) > -radius && del_c(j) < 0) {
                    //std::cout << "negative  ";
                    //std:: cout << i << std::endl;
                    del_c(j) = -scale_error*(radius+del_c(j));
                }
                else {
                    del_c(j) = 0;
                }
            }
            c = sqrt(pow(del_c(0),2) + pow(del_c(1),2) + pow(del_c(2),2));  /// <----- ** hard coded indexes, ultimitly you wouldnt know how many indexes**************
            if (del_c(0)*del_c(1)*del_c(2) == 0) { 
                del_c(0) = 0;
                del_c(1) = 0;
                del_c(2) = 0;
                c = 0;
            }
            if (del_c(0)*del_c(1)*del_c(2) != 0) {
                //std::cout << "TOLERENCE HIT   ", std::cout << i << std::endl;
                break;
            }
        }
        if (mag_ocv(i) == 0) {
            mag_ocv(i) = 0.0001;
        }
        g_pos.col(i) = IJ.transpose()*mag_ocv(i)*(I - unit_prime.col(i)*unit_prime.col(i).transpose())*del_c - c*pow(mag_ocv(i),-2)*(I - unit_prime.col(i)*unit_prime.col(i).transpose())*unit_dubprime.col(i);
        g_vel.col(i) = IJ.transpose()*c*unit_prime.col(i);

    }
    /*
    std::cout << "POS\n";
    std::cout << g_pos << std::endl;
    //std:: cout << "   \n\n";
    std::cout << "VEl\n";
    std::cout << g_vel << std::endl;
    std::cout << "\n\n";
    */
}
// --------------------------------------------------------------------


// Update Jacobian ----------------------------------------------------
void root::MeanTraj::update_J(int step) {
    for (int j=0;j<ocv_size;j++) {
        IJ(j,j) = J(step,j); // (ocv,ocv)
    }
}
// --------------------------------------------------------------------


// Print out trajectory and obstical locations to consol -------------------------------------
void root::MeanTraj::traj(int go) {

    if (go == 1) {
        std::cout << final_pos.transpose() << std::endl;
        obs_plot.resize(obs.rows(),final_pos.transpose().cols());
        for (int i=0;i<obs.rows();i++) {
            obs_plot.row(i) = final_pos.transpose().row(1)*0;
            for (int j=0;j<3;j++) {
                obs_plot(i,j) = obs(i,j);
        } 
        }
        std::cout << obs_plot << std::endl;
    }
}
// --------------------------------------------------------------------


// Plot 2D image of kernal --------------------------------------------
void root::MeanTraj::surf(int play) {
    if (play == 1) {
        std::cout << Kp << std::endl;
    }
    else if (play == 2) {
        std::cout << Kp_inv << std::endl;
    }
    else if (play == 3) {
        std::cout << Kv << std::endl;
    }
    else if (play == 4) {
        std::cout << Kv_inv << std::endl;
    }
    else if (play == 5) {
        std::cout << Ka_inv << std::endl;
    }
}
// --------------------------------------------------------------------


// Print things to console --------------------------------------------
void root::MeanTraj::get_something() {

    //std::cout << Q(21,21) << std::endl;
    //std::cout << Q.rows() << "," << Q.cols() << std::endl;      
    //std::cout << "  " << std::endl;

    //std::cout << "  " << std::endl;


    //std::cout << final_pos.rows() << "," << final_pos.cols() << std::endl;
    //std::cout << "  " << std::endl;

    //std::cout << del_c << std::endl;

    //std::cout << mean_pos << std::endl;
    //std::cout << mean_pos.rows() << "," << mean_pos.cols() << std::endl;
    //std::cout << "  " << std::endl;

    //std::cout << g_pos << std::endl;
    //std::cout << g_pos.rows() << "," << g_pos.cols() << std::endl;
    //std::cout << "  " << std::endl;

    //std::cout << mean_state << std::endl;
    //std::cout << mean_state.rows() << "," << mean_state.cols() << std::endl;
    //std::cout << "  " << std::endl;

    //std::cout << sensitivity[0] << std::endl;
    //std::cout << sensitivity[1] << std::endl;
    //std::cout << sensitivity[2] << std::endl;

    //std::cout << Q.rows() << "  " << Q.cols() << std::endl;
}
// --------------------------------------------------------------------


