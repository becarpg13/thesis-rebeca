// ===============================================================================
//
// VIPER Single Wheel Test - Smoothed Particle Hydrodynamics (SPH) implementation
// Author: Rebeca Pinho Guimarães
//
// ===============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All right reserved.
//
// Adapted from demo_ROBOT_ViperWheel_CRM by Radu Serban and Huzaifa Unjhawala
// ===============================================================================



#include <algorithm>
#include <limits>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"
#include "viper_wheel.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;


// -----------------------------------------------------------------------------


double render_fps = 100;
double output_fps = 30;
bool gnuplot_output = true;

bool render = true;
bool output = true;
bool verbose = true;

bool output_paraview = true; // VTK data for Paraview visualization
bool blender_output = true; // output data in files for Blender visualization
// Note: The Blender option only generates data for the rigid bodies, whereas the Paraview option only generates the particles' and the wheel's data
// You'll need both to generate a complete visualization of the simulation in Blender

bool step_convergence = false;
bool space_convergence = false;

double wheel_slip = 0.1;
double wheel_vel = 0.2; 
double step_size = 3.5e-4; 
double sim_time_max = 6; 
double inner_radius = 0.21; 
double spacing_ratio = 0.75;  
double crm_initial_spacing = 0.01;


// -----------------------------------------------------------------------------


int main(int argc, char* argv[]) {

    // ===============================================================
    // INPUT PARAMETERS FROM COMMAND LINE

    // For convergence mode, the program should receive 5 arguments 
    // ./program <slip> <vel> <convergence_value> <convergence_mode>

    // Convergence mode: "time" or "space"
    // ===============================================================

    if (argc == 5) {
        wheel_slip = std::stod(argv[1]);
        wheel_vel = std::stod(argv[2]);
        double convergence_value = std::stod(argv[3]);
        std::string mode = argv[4];

        output_paraview = false;
        blender_output = false;

        if (mode == "time") {
            step_size = convergence_value;
            step_convergence = true;
            cout << ">> MODE: time step convergence (dT = " << step_size << " s)" << endl;
        } 
        else if (mode == "space") {
            crm_initial_spacing = convergence_value;
            space_convergence = true;
            cout << ">> MODE: initial spacing convergence (d0 = " << crm_initial_spacing << " m)" << endl;
        } 
        else {
            cerr << "Error: invalid convergence mode '" << mode << "'. Use 'time' or 'space'." << endl;
            return 1;
        }
    }
    else if (argc == 3) {
        // If only two arguments are provided, the program assumes they are the slip and longitudinal velocity of the wheel
        // ./program <slip> <vel>
        wheel_slip = std::stod(argv[1]);
        wheel_vel = std::stod(argv[2]);
    } else if (argc == 2) {
        // If only one argument is provided, the program assumes it is the slip of the wheel
        wheel_slip = std::stod(argv[1]);
    } else if (argc == 1) {
        // If no arguments are provided, the program assumes the default parameters defined at the beginning of the code
        cout << "USING DEFAULT PARAMETERS" << endl;
    }

    int slip_percent = static_cast<int>(wheel_slip * 100);
    cout << "Starting simulation with slip ratio: " << slip_percent << "%" << endl;
    cout << "And longitudinal velocity of the wheel: " << wheel_vel << " m/s" << endl;

    // ---------------------------------
    // Create wheel and tire subsystems
    // ---------------------------------

    auto wheel = chrono_types::make_shared<DummyViperWheel>();
    auto tire = chrono_types::make_shared<ViperTire>();


    // -------------------------------------------------
    // Create system and set solver and integrator types
    // -------------------------------------------------

    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR;
    ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    SetChronoSolver(sys, solver_type, integrator_type);

    // -----------------------------------
    // Create and configure tire test rig
    // -----------------------------------

    ChTireTestRig rig(wheel, tire, &sys);

    rig.SetGravitationalAcceleration(1.625);  // Lunar gravity
    //rig.SetGravitationalAcceleration(9.81);  // Earth gravity
    rig.SetNormalLoad(17.5*9.81); // ATENTION: provide desired NORMAL LOAD (N)

    rig.SetTireStepsize(step_size);
    rig.SetTireVisualizationType(VisualizationType::COLLISION);

    // Set SPH terrain
    ChTireTestRig::TerrainPatchSize size;
    size.length = 2.2;
    size.width = 1;
    size.depth = 0.20;

    ChTireTestRig::TerrainParamsCRM params;
    params.sph_params.initial_spacing = crm_initial_spacing;
    params.mat_props.density = 1627.0;
    params.mat_props.Young_modulus = 1e6;
    params.mat_props.cohesion_coeff = 0.0;
    params.mat_props.mu_I0 = 0.03;
    params.mat_props.mu_fric_2 = 0.77;
    params.mat_props.mu_fric_s = 0.77;
    params.mat_props.average_diam = 0.004;

    rig.SetTerrainCRM(size, params);

    // Callback for generation of BCE marker particles
    auto bce_callback = chrono_types::make_shared<ViperTireBCE>(tire, params.sph_params.initial_spacing * spacing_ratio, inner_radius);
    rig.RegisterWheelBCECreationCallback(bce_callback);

    // ----------------------
    // Define test scenario
    // ----------------------

    rig.SetConstantLongitudinalSlip(wheel_slip, wheel_vel);

    // Delay time for beginning of input (s).
    double input_time_delay = 0.1;
    rig.SetTimeDelay(input_time_delay);

    rig.SetWheelInitialClearance(0.02);  // Adjust the initial distance between wheel and soil (m).

    rig.Initialize(ChTireTestRig::Mode::TEST);

    cout << "Rig normal load: " << rig.GetNormalLoad() << endl;
    cout << "Rig total mass:  " << rig.GetMass() << endl;



    // -------------------------------
    // Create the output directories
    // -------------------------------

    const std::string out_dir_root = GetChronoOutputPath() + "ViperWheel_SPH_TCC2/";
    std::string out_dir = GetChronoOutputPath() + "ViperWheel_SPH_TCC2/Slip_" + std::to_string(slip_percent) + "/";
    std::string out_dir_slip = "";

    if (step_convergence == true) {
        out_dir_slip = GetChronoOutputPath() + "ViperWheel_SPH_TCC2/Slip_" + std::to_string(slip_percent) + "_Convergence_Time/";
        out_dir = out_dir_slip + "Step_" + std::to_string(static_cast<int>(step_size * 1e6)) + "us";
    }
    else if (space_convergence == true) {
        out_dir_slip = GetChronoOutputPath() + "ViperWheel_SPH_TCC2/Slip_" + std::to_string(slip_percent) + "_Convergence_Space/";
        out_dir = out_dir_slip + "Spacing_" + std::to_string(static_cast<int>(crm_initial_spacing * 1e3)) + "mm";
    }

    std::cout << "Output directory: " << out_dir << std::endl;

    if (!filesystem::create_directory(filesystem::path(out_dir_root))) {
        cerr << "Error creating directory " << out_dir_root << endl;
        return 1;
    }
    if (step_convergence == true) {
        if (!filesystem::create_directory(filesystem::path(out_dir_slip))) {
            cerr << "Error creating directory " << out_dir_slip << endl;
            return 1;
        }
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
        return 1;
    }

    // Create the output files
    std::ofstream myFile;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
    }

    // ==================================================
    // SAVE FILE METADATA AND SIMULATION CONFIGURATIONS
    // ==================================================

    std::ofstream configFile(out_dir + "/sim_config.txt");
    if (configFile.is_open()) {
        configFile << "==================================================\n";
        configFile << "                 SIMULATION METADATA               \n";
        configFile << "==================================================\n\n";
        
        configFile << "Wheel Slip Ratio: " << wheel_slip << " (" << slip_percent << "%)\n";
        configFile << "Wheel Longitudinal Velocity (m/s): " << wheel_vel << "\n";
        configFile << "Wheel Angular Velocity (rad/s): " << rig.GetWheelAngularVelocity() << "\n\n";

        configFile << "[WHEEL PROPERTIES]\n";
        configFile << "Wheel Type:                " << (tire->GetGrouserHeightConfig() > 0.0 ? "Grousered" : "Smooth Cylindrical") << "\n";
        configFile << "Wheel Radius (m):          " << tire->GetRadiusConfig() << "\n";
        configFile << "Wheel Width (m):           " << tire->GetWidthConfig() << "\n";
        configFile << "Grouser Height (m):        " << tire->GetGrouserHeightConfig() << "\n";
        configFile << "Tire Mass (kg):            " << tire->GetTireMassConfig() << "\n";
        configFile << "Dummy Wheel Mass (kg):     " << wheel->GetWheelMass() << "\n\n";
        
        configFile << "[TEST RIG SETTINGS]\n";
        configFile << "Target Normal Load (N):    " << rig.GetNormalLoad() << "\n";
        configFile << "Total Rig Mass (kg):       " << rig.GetMass() << "\n";
        configFile << "Gravitational Acceleration (m/s^2): " << rig.GetGravitationalAcceleration() << "\n";
        configFile << "Initial Delay Time (s):    " << input_time_delay << "\n";
        configFile << "Max Simulation Time (s):   " << sim_time_max << "\n\n";
        
        configFile << "[CRM / SPH SOIL PARAMETERS]\n";
        configFile << "Particle Spacing (m):      " << params.sph_params.initial_spacing << "\n";
        configFile << "Average Soil Diameter (m): " << params.mat_props.average_diam << "\n";
        configFile << "Soil Density (kg/m3):      " << params.mat_props.density << "\n";
        configFile << "Young's Modulus (Pa):      " << params.mat_props.Young_modulus << "\n";
        configFile << "Friction Coefficient mu_s (dimensionless): " << params.mat_props.mu_fric_s << "\n";
        configFile << "Friction Coefficient mu_2 (dimensionless): " << params.mat_props.mu_fric_2 << "\n";
        configFile << "Soil Cohesion:             " << params.mat_props.cohesion_coeff << "\n\n";

        configFile << "[TERRAIN DIMENSIONS]\n";
        configFile << "Patch Length (m):          " << size.length << "\n";
        configFile << "Patch Width (m):           " << size.width << "\n";
        configFile << "Patch Depth (m):           " << size.depth << "\n\n";

        configFile << "[SIMULATION SETTINGS]\n";
        configFile << "Inner Radius for BCE (m):              " << inner_radius << "\n";
        configFile << "BCE/CRM Spacing Ratio:                 " << spacing_ratio << "\n";
        configFile << "BCE Type:                              " << (tire->GetUseObjBCE() ? "Generated from OBJ vertices" : "Manually generated by code") << "\n";
        configFile << "Wheel Active Domain (m):               " << rig.GetWheelActiveDomain() << "\n";
        configFile << "Minimum Computational Domain (m):      " << rig.GetTerrainDomain_Min() << "\n";
        configFile << "Maximum Computational Domain (m):      " << rig.GetTerrainDomain_Max() << "\n\n";

        configFile << "[SOLVER AND INTEGRATION]\n";
        configFile << "Time Step - dT (s):        " << step_size << "\n";
        configFile << "Solver Type:               " << (solver_type == ChSolver::Type::SPARSE_QR ? "SPARSE_QR" : solver_type == ChSolver::Type::BARZILAIBORWEIN ? "BARZILAIBORWEIN" : "OTHER") << "\n";
        configFile << "==================================================\n";
        
        configFile.close();
        std::cout << ">> File 'sim_config.txt' successfully written to: " << out_dir << std::endl;
    }

    // -------------------------------
    // Create real time visualization
    // -------------------------------

    std::shared_ptr<ChVisualSystem> vis;
    auto sysFSI = std::dynamic_pointer_cast<CRMTerrain>(rig.GetTerrain())->GetFsiSystemSPH();
    auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());

    if (render) {
        auto color_callback = chrono_types::make_shared<ParticleHeightColorCallback>(rig.GetTerrainDomain_Min().z()-tire->GetRadiusConfig(), rig.GetTerrainDomain_Max().z()-tire->GetRadiusConfig());

        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(true);
        visFSI->SetSPHColorCallback(color_callback, ChColormap::Type::BLUE);
        visFSI->SetColorBoundaryMarkers({0.85, 0.85, 0.85});
        visFSI->SetColorRigidBodyMarkers({0.3, 0.6, 0.0});

        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sys);
        visVSG->SetWindowTitle("Viper wheel on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);

        visVSG->AddCamera(ChVector3d(1.0, 2.5, -0.6), ChVector3d(0, 0.25, -0.6));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }

    #ifdef CHRONO_POSTPROCESS
    // -----------------------------
    // Create Blender data exporter
    // -----------------------------

    postprocess::ChBlender blender_exporter(&sys);

    if (blender_output) {
        std::string blender_dir = out_dir + "/blender";
        if (!filesystem::create_directory(filesystem::path(blender_dir))) {
            cerr << "Error creating directory " << blender_dir << endl;
            return 1;
        }

        blender_exporter.SetBlenderUp_is_ChronoZ();
        blender_exporter.SetBasePath(blender_dir);
        blender_exporter.AddAll();
        blender_exporter.SetCamera(ChVector3d(1.0, 2.5, -0.6), ChVector3d(0, 0.25, -0.6), 50);
        blender_exporter.ExportScript();
    }
    #endif


    ChFsiFluidSystemSPH& sysSPH = sysFSI->GetFluidSystemSPH();

    // -----------------
    // SIMULATION LOOP
    // -----------------

    // Timers and counters
    ChTimer timer;         // timer for measuring total run time
    double time = 0;       // simulated time
    double sim_time = 0;   // simulation time
    int render_frame = 0;  // render frame counter
    int out_frame = 0;     // output frame counter


    // Interpolation functions for data collection
    ChFunctionInterp long_slip_fct;
    ChFunctionInterp dbp_fct;
    ChFunctionInterp torque_fct;
    ChFunctionInterp sinkage_fct;

    auto actuator = rig.GetMotorCarrier();
    auto motor = rig.GetMotorWheel();

    double min_dbp = std::numeric_limits<double>::max();
    double max_dbp = std::numeric_limits<double>::lowest();
    double min_torque = std::numeric_limits<double>::max();
    double max_torque = std::numeric_limits<double>::lowest();
    double min_sinkage = std::numeric_limits<double>::max();
    double max_sinkage = std::numeric_limits<double>::lowest();



    timer.start();
    while (time < sim_time_max) {
        time = sys.GetChTime();

        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;

        }


        rig.Advance(step_size);
        sim_time += sys.GetTimerStep();

        auto long_slip = rig.GetLongitudinalSlip();
        auto dbp = rig.GetDBP();
        min_dbp = std::min(min_dbp, dbp);
        max_dbp = std::max(max_dbp, dbp);

        auto force = -actuator->GetMotorForce();
        auto torque = motor->GetMotorTorque();
        min_torque = std::min(min_torque, torque);
        max_torque = std::max(max_torque, torque);

        auto sinkage = rig.GetSinkage();
        min_sinkage = std::min(min_sinkage, sinkage);
        max_sinkage = std::max(max_sinkage, sinkage);

        if (verbose) {

        cout << "time: " << time << endl;
        cout << "  drawbar pull:           " << dbp << endl;
        cout << "  wheel torque:           " << torque << endl;
        cout << " sinkage:                 " << sinkage << endl;
        cout << " " << endl;
        }

        if (output) {
            myFile << time << "\t" << dbp << "\t" << torque << "\t" << sinkage << "\n";
        }

        if (gnuplot_output && time > input_time_delay) {
            long_slip_fct.AddPoint(time, long_slip);
            dbp_fct.AddPoint(time, dbp);
            torque_fct.AddPoint(time, torque);
            sinkage_fct.AddPoint(time, sinkage);
        }

        if (output_paraview && time >= out_frame / output_fps) {
            cout << "\n-------- SALVANDO VTK --------\n";
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);
            static int counter = 0;
            std::string filename = out_dir + "/vtk/wheel." + std::to_string(counter++) + ".vtk";
            tire->WriteVTK(filename);

            #ifdef CHRONO_POSTPROCESS
            if (blender_output)
                blender_exporter.ExportData();
            #endif

            out_frame++;
        }

        cout << "\rRTF: " << sys.GetRTF() << endl;
    }
    timer.stop();

    double step_time = timer();
    cout << "\rSimulated time: " << time << endl;
    cout << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << endl;
    cout << "Run time (total):      " << step_time << "  |  RTF: " << step_time / time << endl;

    std::ofstream configFileAppend(out_dir + "/sim_config.txt", std::ios::app); 
    
    if (configFileAppend.is_open()) {
        configFileAppend << "\n";
        configFileAppend << "[COMPUTATIONAL PERFORMANCE RESULTS]\n";
        configFileAppend << "Simulated Time (s):      " << time << "\n";
        configFileAppend << "Execution Time (s):   " << step_time << "\n";
        configFileAppend << "Real-Time Factor (RTF):  " << (step_time / time) << "\n";
        configFileAppend << "==================================================\n";
        
        configFileAppend.close();
        std::cout << ">> Performance data added to 'sim_config.txt' successfully!" << std::endl;
    }


#ifdef CHRONO_POSTPROCESS
    // ----------------------------------------------
    // Quick plots of results using Gnuplot
    // ----------------------------------------------

    if (gnuplot_output && sys.GetChTime() > input_time_delay) {
        postprocess::ChGnuPlot gplot_long_slip(out_dir + "/tmp1.gpl");
        gplot_long_slip.SetGrid();
        gplot_long_slip.SetLabelX("time (s)");
        gplot_long_slip.SetLabelY("Long. slip");
        gplot_long_slip.SetRangeY(-2, +2);
        gplot_long_slip.Plot(long_slip_fct, "", " with lines lt -1 lc rgb'#00AAEE' ");

        postprocess::ChGnuPlot gplot_dbp(out_dir + "/tmp4.gpl");
        gplot_dbp.SetGrid();
        gplot_dbp.SetLabelX("time (s)");
        gplot_dbp.SetLabelY("traction force (N)");
        double margin_dbp = 0.1 * std::max(1.0, std::max(std::abs(min_dbp), std::abs(max_dbp)));
        gplot_dbp.SetRangeY(min_dbp - margin_dbp, max_dbp + margin_dbp);
        gplot_dbp.Plot(dbp_fct, "", " with lines lt -1 lc rgb'#c0081d' ");

        postprocess::ChGnuPlot gplot_torque(out_dir + "/tmp5.gpl");
        gplot_torque.SetGrid();
        gplot_torque.SetLabelX("time (s)");
        gplot_torque.SetLabelY("wheel torque (N.m)");
        double margin_torque = 0.1 * std::max(1.0, std::max(std::abs(min_torque), std::abs(max_torque)));
        gplot_torque.SetRangeY(min_torque - margin_torque, max_torque + margin_torque);
        gplot_torque.Plot(torque_fct, "", " with lines lt -1 lc rgb'#088a58' ");

        postprocess::ChGnuPlot gplot_sinkage(out_dir + "/tmp6.gpl");
        gplot_sinkage.SetGrid();
        gplot_sinkage.SetLabelX("time (s)");
        gplot_sinkage.SetLabelY("sinkage (m)");
        double margin_sinkage = 0.1 * std::max(1.0, std::max(std::abs(min_sinkage), std::abs(max_sinkage)));
        gplot_sinkage.SetRangeY(min_sinkage - margin_sinkage, max_sinkage + margin_sinkage);
        gplot_sinkage.Plot(sinkage_fct, "", " with lines lt -1 lc rgb'#8810af' ");
    }
#endif

    if (output) {
        myFile.close();
    }

    return 0;
}
