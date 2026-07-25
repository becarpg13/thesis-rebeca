// =============================================================================
//
// TESTE DE RODA ÚNICA SPH 
// Autora: Rebeca Pinho Guimarães
//
// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All right reserved.
//
// Adaptado de demo_ROBOT_ViperWheel_CRM por Radu Serban e Huzaifa Unjhawala
// =============================================================================



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

bool output_paraview = true; // opção para salvar os dados de saída em arquivos VTK para visualização no Paraview
bool blender_output = true; // opção para salvar os dados de saída em arquivos para visualização no Blender
// Obs: A opção do Blender gera apenas os arquivos da estrutura e a opção do Paraview gera apenas os arquivos das partículas SPH e roda. 
// Recomenda-se sempre manter a opção do Blender ativa por seu baixo custo computacional.

bool step_convergence = false;
bool space_convergence = false;

double wheel_slip = 0.1;  // razão de escorregamento desejada
double wheel_vel = 0.2;   // velocidade linear desejada
double step_size = 3.5e-4;  // passo de tempo de integração (s)
double sim_time_max = 6; // tempo de simulação (s)
double inner_radius = 0.21;  // raio interno para geração de BCE (m)
double spacing_ratio = 0.75;  // razão entre o espaçamento das partículas BCE e o espaçamento das partículas do solo (CRM)
double crm_initial_spacing = 0.01; // espaçamento inicial das partículas do solo (CRM) (m)


// -----------------------------------------------------------------------------


int main(int argc, char* argv[]) {
    // PARÂMETROS DE ENTRADA DO USUÁRIO
    // Para usar o modo de convergência, o programa deve receber 5 argumentos: ./programa <slip> <vel> <valor_convergencia> <modo_convergencia>
    // Modo de convergência: "time" para convergência de passo de tempo, "space" para convergência de espaçamento inicial do CRM.
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
            cout << ">> MODO: Convergencia de Passo de Tempo (dT = " << step_size << " s)" << endl;
        } 
        else if (mode == "space") {
            crm_initial_spacing = convergence_value;
            space_convergence = true;
            cout << ">> MODO: Convergencia de Espacamento Inicial CRM (spacing = " << crm_initial_spacing << " m)" << endl;
        } 
        else {
            cerr << "Erro: Modo de convergencia '" << mode << "' invalido. Use 'time' ou 'space'." << endl;
            return 1;
        }
    }
    else if (argc == 3) {
        // Se foram digitados apenas dois argumentos, o programa assume que são o escorregamento e a velocidade linear da roda
        // ./programa <slip> <vel>
        wheel_slip = std::stod(argv[1]);
        wheel_vel = std::stod(argv[2]);
    } else if (argc == 2) {
        // Se foi digitado apenas um argumento, o programa assume que é o escorregamento da roda
        wheel_slip = std::stod(argv[1]);
    } else if (argc == 1) {
        // Se não foram digitados argumentos, o programa assume os parâmetros padrão definidos no início do código
        cout << "USANDO PARAMETROS PADRAO" << endl;
    }

    int slip_percent = static_cast<int>(wheel_slip * 100);
    cout << "Comecando simulacao com razao de escorregamento: " << slip_percent << "%" << << endl;
    cout << "E velocidade linear da roda: " << wheel_vel << " m/s" << endl;

    // -----------------------------------------
    // Criação dos subsistemas da roda e do pneu
    // -----------------------------------------

    auto wheel = chrono_types::make_shared<DummyViperWheel>();
    auto tire = chrono_types::make_shared<ViperTire>();


    // --------------------------
    // Criação do sistema físico 
    // --------------------------

    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // --------------------------------------------
    // Configuração do solucionador e do integrador
    // --------------------------------------------

    ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR;
    ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    SetChronoSolver(sys, solver_type, integrator_type);

    // ------------------------------------------
    // Criação e configuração da bancada de teste
    // ------------------------------------------

    ChTireTestRig rig(wheel, tire, &sys);

    rig.SetGravitationalAcceleration(1.625);  // Gravidade lunar
    //rig.SetGravitationalAcceleration(9.81);  // Gravidade terrestre
    rig.SetNormalLoad(17.5*9.81); // ATENÇÃO: recebe a CARGA NORMAL desejada (N)

    rig.SetTireStepsize(step_size);
    rig.SetTireVisualizationType(VisualizationType::COLLISION);

    // Definição do terreno SPH
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
    params.mat_props.average_diam = 0.004;  // diâmetro médio das partículas do solo (m)

    rig.SetTerrainCRM(size, params);

    // Criação do callback para geração dos marcadores BCE
    auto bce_callback = chrono_types::make_shared<ViperTireBCE>(tire, params.sph_params.initial_spacing * spacing_ratio, inner_radius);
    rig.RegisterWheelBCECreationCallback(bce_callback);

    // -----------------------------
    // Definição do cenário de teste
    // -----------------------------

    rig.SetConstantLongitudinalSlip(wheel_slip, wheel_vel);

    // Tempo de delay para o início do funcionamento da roda (s).
    double input_time_delay = 0.1;
    rig.SetTimeDelay(input_time_delay);

    rig.SetWheelInitialClearance(0.02);  // Ajuste para alterar a distância inicial entre a roda e o solo (m).

    // Inicialização da bancada de teste
    rig.Initialize(ChTireTestRig::Mode::TEST);

    cout << "Rig normal load: " << rig.GetNormalLoad() << endl;
    cout << "Rig total mass:  " << rig.GetMass() << endl;



    // --------------------------------
    // Criação dos diretórios de saída
    // --------------------------------

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

    // Criação do arquivo de resultados
    std::ofstream myFile;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
    }

    // ========================================================
    // GERAR ARQUIVO DE METADADOS E CONFIGURAÇÕES DA SIMULAÇÃO
    // ========================================================
    std::ofstream configFile(out_dir + "/sim_config.txt");
    if (configFile.is_open()) {
        configFile << "==================================================\n";
        configFile << "       METADADOS DA SIMULAÇÃO - PROJECT CHRONO     \n";
        configFile << "==================================================\n\n";
        
        configFile << "Razão de Escorregamento da Roda: " << wheel_slip << " (" << slip_percent << "%)\n";
        configFile << "Velocidade Longitudinal da Roda (m/s): " << wheel_vel << "\n";
        configFile << "Velocidade Angular da Roda (rad/s): " << rig.GetWheelAngularVelocity() << "\n\n";

        configFile << "[PROPRIEDADES DA RODA]\n";
        configFile << "Tipo de Roda:              " << (tire->GetGrouserHeightConfig() > 0.0 ? "Com Garras (Grousers)" : "Cilindrica Lisa") << "\n";
        configFile << "Raio da Roda (m):          " << tire->GetRadiusConfig() << "\n";
        configFile << "Largura da Roda (m):       " << tire->GetWidthConfig() << "\n";
        configFile << "Altura da Garra (m):       " << tire->GetGrouserHeightConfig() << "\n";
        configFile << "Massa do Pneu (kg):        " << tire->GetTireMassConfig() << "\n";
        configFile << "Massa da Roda Dummy (kg):  " << wheel->GetWheelMass() << "\n\n";
        
        configFile << "[CONFIGURAÇÕES DO TEST RIG]\n";
        configFile << "Carga Normal Alvo (N):     " << rig.GetNormalLoad() << "\n";
        configFile << "Massa Total do Rig (kg):     " << rig.GetMass() << "\n";
        configFile << "Gravidade (m/s^2):          " << rig.GetGravitationalAcceleration() << "\n";
        configFile << "Tempo Inicial de Delay (s): " << input_time_delay << "\n";
        configFile << "Tempo Max de Simulação (s): " << sim_time_max << "\n\n";
        
        configFile << "[PARÂMETROS DO SOLO CRM / SPH]\n";
        configFile << "Espaçamento de Partícula (m): " << params.sph_params.initial_spacing << "\n";
        configFile << "Diâmetro Médio do Solo (m):  " << params.mat_props.average_diam << "\n";
        configFile << "Densidade do Solo (kg/m3):    " << params.mat_props.density << "\n";
        configFile << "Módulo de Young (Pa):         " << params.mat_props.Young_modulus << "\n";
        configFile << "Coeficiente de Fricção mu_s (adimensional): " << params.mat_props.mu_fric_s << "\n";
        configFile << "Coeficiente de Fricção mu_2 (adimensional): " << params.mat_props.mu_fric_2 << "\n";
        configFile << "Coesão do Solo (cohesion):    " << params.mat_props.cohesion_coeff << "\n\n";

        configFile << "[MEDIDAS DO TERRENO]\n";
        configFile << "Comprimento do Patch (m):     " << size.length << "\n";
        configFile << "Largura do Patch (m):        " << size.width << "\n";
        configFile << "Profundidade do Patch (m):    " << size.depth << "\n\n";

        configFile << "[CONFIGURAÇÕES DE SIMULAÇÃO]\n";
        configFile << "Raio Interno para BCE (m):     " << inner_radius << "\n";
        configFile << "Razão de Espaçamento BCE/CRM: " << spacing_ratio << "\n";
        configFile << "Tipo de BCE:                     " << (tire->GetUseObjBCE() ? "Gerado pelos vértices do OBJ" : "Gerado manualmente por código") << "\n";
        configFile << "Domínio Ativo da Roda (m):             " << rig.GetWheelActiveDomain() << "\n";
        configFile << "Domínio Computacional Mínimo (m):             " << rig.GetTerrainDomain_Min() << "\n";
        configFile << "Domínio Computacional Máximo (m):             " << rig.GetTerrainDomain_Max() << "\n\n";

        configFile << "[RESOLVEDOR E INTEGRAÇÃO]\n";
        configFile << "Passo de Tempo - dT (s):   " << step_size << "\n";
        configFile << "Tipo de Solucionador:      " << (solver_type == ChSolver::Type::SPARSE_QR ? "SPARSE_QR" : solver_type == ChSolver::Type::BARZILAIBORWEIN ? "BARZILAIBORWEIN" : "OUTRO") << "\n";
        configFile << "==================================================\n";
        
        configFile.close();
        std::cout << ">> Arquivo 'sim_config.txt' gravado com sucesso em: " << out_dir << std::endl;
    }

    // -------------------------------------------------------------------
    // Criação da vizualização em tempo real e do exportador para Blender
    // -------------------------------------------------------------------

    std::shared_ptr<ChVisualSystem> vis;
    auto sysFSI = std::dynamic_pointer_cast<CRMTerrain>(rig.GetTerrain())->GetFsiSystemSPH();
    auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());

    if (render) {
        auto color_callback = chrono_types::make_shared<ParticleHeightColorCallback>(rig.GetTerrainDomain_Min().z()-tire->GetRadiusConfig(), rig.GetTerrainDomain_Max().z()-tire->GetRadiusConfig());  // color particles based on height (range 0 to 1 m)
        // FSI plugin
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(true);
        visFSI->SetSPHColorCallback(color_callback, ChColormap::Type::BLUE);
        visFSI->SetColorBoundaryMarkers({0.85, 0.85, 0.85});
        visFSI->SetColorRigidBodyMarkers({0.3, 0.6, 0.0});

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sys);
        visVSG->SetWindowTitle("Viper wheel on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        // Static camera: place the view high enough and behind the wheel to see the tire-terrain gap.
        visVSG->AddCamera(ChVector3d(1.0, 2.5, -0.6), ChVector3d(0, 0.25, -0.6));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }

    #ifdef CHRONO_POSTPROCESS
    // --------------------------------------
    // Criar exportador de dados para Blender
    // --------------------------------------

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
    // LOOP DE SIMULAÇÃO
    // -----------------

    // Timers e contadores
    ChTimer timer;         // timer para medir o tempo total de execução
    double time = 0;       // tempo simulado
    double sim_time = 0;   // tempo de simulação
    int render_frame = 0;  // contador de frames para renderização
    int out_frame = 0;     // contador de frames para saída de dados

    // Funções de interpolação para coleta de dados
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
        configFileAppend << "\n"; // Pula uma linha para organizar
        configFileAppend << "[RESULTADOS DE DESEMPENHO COMPUTACIONAL]\n";
        configFileAppend << "Tempo Simulado (s):      " << time << "\n";
        configFileAppend << "Tempo de Execucao (s):   " << step_time << "\n";
        configFileAppend << "Real-Time Factor (RTF):  " << (step_time / time) << "\n";
        configFileAppend << "==================================================\n";
        
        configFileAppend.close();
        std::cout << ">> Dados de desempenho apensados em 'sim_config.txt' com sucesso!" << std::endl;
    }


#ifdef CHRONO_POSTPROCESS
    // ----------------------------------------------
    // Gráficos rápidos dos resultados usando Gnuplot
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
        gplot_dbp.SetLabelX("tempo (s)");
        gplot_dbp.SetLabelY("forca de tracao (N)");
        double margin_dbp = 0.1 * std::max(1.0, std::max(std::abs(min_dbp), std::abs(max_dbp)));
        gplot_dbp.SetRangeY(min_dbp - margin_dbp, max_dbp + margin_dbp);
        gplot_dbp.Plot(dbp_fct, "", " with lines lt -1 lc rgb'#c0081d' ");

        postprocess::ChGnuPlot gplot_torque(out_dir + "/tmp5.gpl");
        gplot_torque.SetGrid();
        gplot_torque.SetLabelX("tempo (s)");
        gplot_torque.SetLabelY("torque da roda (N.m)");
        double margin_torque = 0.1 * std::max(1.0, std::max(std::abs(min_torque), std::abs(max_torque)));
        gplot_torque.SetRangeY(min_torque - margin_torque, max_torque + margin_torque);
        gplot_torque.Plot(torque_fct, "", " with lines lt -1 lc rgb'#088a58' ");

        postprocess::ChGnuPlot gplot_sinkage(out_dir + "/tmp6.gpl");
        gplot_sinkage.SetGrid();
        gplot_sinkage.SetLabelX("tempo (s)");
        gplot_sinkage.SetLabelY("afundamento (m)");
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
