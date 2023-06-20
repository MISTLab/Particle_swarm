#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/buzz_controller_kheperaiv.h>
#include "ros_controller/src/controller/controller.h"
#include "ros_controller/src/controller/controller_footbot.h"
#include "ros_controller/src/controller/controller_spiri.h"
// #include <buzz/argos/buzz_controller_spiri.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/spiri/simulator/spiri_entity.h>
#include <fstream>
#include <string>
#include <cerrno>

using namespace argos;

class Planningloop : public CLoopFunctions {

public:

   Planningloop();
   virtual ~Planningloop();

   virtual void Init(TConfigurationNode& t_tree);
   // virtual void PreStep();
   virtual void PostStep();
   virtual void Reset();
   virtual void Destroy();
   virtual bool IsExperimentFinished();

private:

   
   void OpenFile(std::ofstream& c_stream,
                 const std::string& str_prefix);

   void OpenMapFile(std::ofstream& c_stream,
                 const std::string& str_name);

   void CloseFile(std::ofstream& c_stream);


   void Hir_PostStep();

private:

   std::string m_strOutFile;
   std::string m_map_file_name;
   std::string RUN_DIR_Data_file;
   std::string forest_m_map_file_name;
   double map_height;
   double map_length;
   bool m_bDone;
   std::ofstream m_cOutFile;
   std::ofstream m_posFile;
   std::ofstream m_cForestMapOutFile;
   int faulty_number;
   int healing_time;
   int healing_started_num[5];
   int num_unresponsive;
   // std::ofstream m_cQueueP2PFile;
   std::vector<CBuzzControllerKheperaIV*> m_vecControllers;
   std::vector<buzzvm_t> buzz_vms;
   std::vector<CSpiriEntity*> m_vecSpiriControllers;
   std::vector<CKheperaIVEntity*> m_guides;
   std::vector<CFootBotEntity*> m_workers;
   std::vector<CSpiriEntity*> m_spirivec;
   std::vector<bool> m_vecDone;
   std::vector<bool> m_vecGetDone;
   int number_of_links;
   Real m_fault_percent, m_fault_set;
   std::vector<int> m_faulty_robots;
   int m_targets_reached[4];
   CVector2 Start_state;
   CVector2 Goal_state;
   UInt32 unRobots;
   std::vector<int> robots_ids_faulty;
   int PlanningDone;
   int PlanTime;
   int exp_end_counter;
   int algo;
   int number_of_targets;
   int num_of_targets_reached;
   Real Exploration_Quota;
   std::vector<CVector2> Trees_pos;
   

};

