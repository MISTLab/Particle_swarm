#include "Planning.h"
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <sstream>
#include <list>
/****************************************/
/****************************************/

     
static const Real        FB_RADIUS        = 0.0704f;
static const Real        FB_AREA          = ARGOS_PI * Square(0.0704f);
static const std::string FB_CONTROLLER    = "khivbz";
static const std::string SPIRI_CONTROLLER    = "bcs";
static const UInt32      MAX_PLACE_TRIALS = 400;
static const UInt32      MAX_ROBOT_TRIALS = 400;

static const Real        WALL_HEIGHT      = 0.3;
static const Real        MAP_RESOLUTION =0.108;
static const Real        RAB_RANGE = 4;
static const Real un_data_size = 200;
static const char free_space = '.';
static const char free_space_l = 'G';
static const char outofbound_space = '@';
static const char outofbound_space_l = 'O';
static const char Tree_space = 'T';
static const char Swap_space = 'S';
static const char Water_space = 'W';
static const char START_space = 'A';
static const char TARGET_space = 'X';
static const char WALL_space = 'Q';
static const Real TREE_RADIUS = 0.2f;
static const Real OUTOFBOUND_BOX_SIZE = 0.5f;
static const Real BOX_POS_CONST = 0.5f;
static const Real BOX_SIZE = 1.0f;
static const Real ROBOT_DISTRIBUTION_CLEARENCE = 3.0;
static const Real FOREST_Density = 0.1;
static UInt16 NUM_OF_FOLLOWERS=16;
static const UInt16 GUIDE_START_ID=9000;
static int TARGET_START_ID=1000;



/****************************************/
/****************************************/

Planningloop::Planningloop() :
   m_bDone(false),
   Trees_pos(0) {
}

/****************************************/
/****************************************/

Planningloop::~Planningloop() {
}

/****************************************/
/****************************************/

void Planningloop::Init(TConfigurationNode& t_tree) {
  LOG.Flush();
   LOGERR.Flush();
   try {
      /* Parse the configuration file */
      GetNodeAttribute(t_tree, "outfile", m_strOutFile);
      m_strOutFile +=".csv";

      UInt16 Num_guide=2;
      GetNodeAttribute(t_tree, "Num_guide", Num_guide);
      
      GetNodeAttribute(t_tree, "Num_worker", NUM_OF_FOLLOWERS);

     
      float density = 1;
      GetNodeAttribute(t_tree, "density", density);
      
      PlaceWorker(NUM_OF_FOLLOWERS, density);

      PlaceGuide(Num_guide, NUM_OF_FOLLOWERS, density);
      
      /* Get reference to buzz controllers, robots and stack them up in a vec*/
      for(int i=0; i<NUM_OF_FOLLOWERS; i++){
         std::stringstream os;
         os << "kh" << i;
         CEntity& c_entity = (GetSpace().GetEntity(os.str()));
         CFootBotEntity* my_entity_add = (CFootBotEntity*)&c_entity;
         CFootBotEntity* cfootbot = any_cast<CFootBotEntity*>(my_entity_add);
         m_workers.push_back(cfootbot);
         CBuzzControllerKheperaIV& cController = dynamic_cast<CBuzzControllerKheperaIV&>(cfootbot->GetControllableEntity().GetController());
         buzzvm_t tBuzzVM = cController.GetBuzzVM();
         buzz_vms.push_back(tBuzzVM);
         
      
      }
      for(int i=0; i<Num_guide; ++i){
         std::stringstream os;
         os << "kh" << GUIDE_START_ID+i;
         CEntity& c_entity = (GetSpace().GetEntity(os.str()));
         CKheperaIVEntity* my_entity_add = (CKheperaIVEntity*)&c_entity;
         CKheperaIVEntity* cKhepera = any_cast<CKheperaIVEntity*>(my_entity_add);
         m_guides.push_back(cKhepera);
         CBuzzControllerKheperaIV& cController = dynamic_cast<CBuzzControllerKheperaIV&>(cKhepera->GetControllableEntity().GetController());
         buzzvm_t tBuzzVM = cController.GetBuzzVM();
         buzz_vms.push_back(tBuzzVM);
         
      }
     
      m_vecDone.resize(buzz_vms.size());
      /* Initialize the rest */
      Reset();
      exp_end_counter =0;
   
   }  
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }
}

/****************************************/
/****************************************/
void write_obj_to_file(buzzobj_t o, std::ofstream& m_posFile){
   switch (o->o.type)
    {
      case BUZZTYPE_NIL:
        m_posFile << "," << "nil";
        break;
      case BUZZTYPE_INT:
        m_posFile << "," << o->i.value;
        break;
      case BUZZTYPE_FLOAT:
        m_posFile << "," << o->f.value;
        break;
      case BUZZTYPE_STRING:
        m_posFile << "," << o->s.value.str;
        break;
      default:
        break;
    }
}


void Planningloop::PostStep(){
 
   Hir_PostStep();
 
}

void Planningloop::Hir_PostStep() {
   /* Go through the robots */
  buzzvm_t vm;
  m_posFile << std::endl;
  m_posFile << GetSpace().GetSimulationClock();
  num_of_targets_reached = 0;
  for (size_t i = 0; i < buzz_vms.size(); ++i) {
      vm = buzz_vms[i];
      // if(i < NUM_OF_FOLLOWERS){
      //    CFootBotEntity& cEFootBot = *any_cast<CFootBotEntity *>(m_workers[i]);
         m_posFile << "," << vm->robot;
      //          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX()
      //          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY()
      //          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();
      // }
      // else{
      //    /* At first you get the footbot object */
      //    CKheperaIVEntity& cEKhepera = *any_cast<CKheperaIVEntity *>(m_guides[i-NUM_OF_FOLLOWERS]);
      //    m_posFile << "," << vm->robot
      //          << "," << cEKhepera.GetEmbodiedEntity().GetOriginAnchor().Position.GetX()
      //          << "," << cEKhepera.GetEmbodiedEntity().GetOriginAnchor().Position.GetY()
      //          << "," << cEKhepera.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();
      // }
      // BVMSTATE
      {
         buzzvm_pushs(vm, buzzvm_string_register(vm, "value_num_of_conflicts", 1));
         buzzvm_gload(vm);
         buzzobj_t o = buzzvm_stack_at(vm, 1);
         write_obj_to_file(o,m_posFile);
         buzzvm_pop(vm);
      }
      {
         buzzvm_pushs(vm, buzzvm_string_register(vm, "robot_num_of_conflicts", 1));
         buzzvm_gload(vm);
         buzzobj_t o = buzzvm_stack_at(vm, 1);
         write_obj_to_file(o,m_posFile);
         buzzvm_pop(vm);
      }
      // EXPLORE_STATE - exploration state
      {
         buzzvm_pushs(vm, buzzvm_string_register(vm, "exp_done", 1));
         buzzvm_gload(vm);
         buzzobj_t o = buzzvm_stack_at(vm, 1);
         write_obj_to_file(o,m_posFile);
         buzzvm_pop(vm);
         if(vm->robot <  GUIDE_START_ID && o->i.value == 1){
            num_of_targets_reached++;
         }
      }
     
   }
   // Experiment end condition. 
   if(num_of_targets_reached >= NUM_OF_FOLLOWERS || GetSpace().GetSimulationClock() > 120000){
      m_bDone = true;
      // if(exp_end_counter > 100){
      //    THROW_ARGOSEXCEPTION("Experiment done ... "); // Experiment does't end all the time, this is a hack for now.
      // }
      // else if(exp_end_counter == 60){
      //    if(num_of_targets_reached >= number_of_targets){
      //       std::stringstream cpcommmand;
      //       cpcommmand <<"cp "<<RUN_DIR_Data_file<<" "<<"/scratch/viveks/Hir_hetro/data/";
      //       printf(" Copying data .. with cmd %s\n",cpcommmand.str().c_str());
      //       system(cpcommmand.str().c_str());
      //    }
      //    else{
      //       printf(" SIMULATION TIME LIMIT Reached exp did not complete\n");
      //    }
      // }
      
      // exp_end_counter = exp_end_counter + 1;
   }
   // m_bDone = true;
}

/****************************************/
/****************************************/

void Planningloop::Reset() {
   /* Clear all flags in the 'done' vectors */
   m_bDone = false;

   OpenFile(m_posFile, "H");
   
}

/****************************************/
/****************************************/

void Planningloop::Destroy() {
   CloseFile(m_posFile);
}

/****************************************/
/****************************************/

bool Planningloop::IsExperimentFinished() {
   return m_bDone;
}

void Planningloop::PlaceWorker(int num, float density){
  UInt32 unTrials;
  CKheperaIVEntity* pcFB;
  std::ostringstream cFBId;
  CVector3 cFBPos;
  CQuaternion cFBRot;
  float arena_side_length = sqrt(num/density);
  CRange<Real> c_area_range( -arena_side_length/2, 
                                arena_side_length/2);
  /* Create a RNG (it is automatically disposed of by ARGoS) */
  CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
  /* For each robot */
  for (size_t i = 0; i < num; ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "kh" << i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CKheperaIVEntity(
        cFBId.str(),
        FB_CONTROLLER,
        CVector3(),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    
    /* Try to place it in the arena */
    unTrials = 0;
    bool bDone;
    do {
      /* Choose a random position */
      ++unTrials;
      cFBPos.Set(pcRNG->Uniform(c_area_range),
                 pcRNG->Uniform(c_area_range),
                 0.0f);
      cFBRot.FromAngleAxis(pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                           CVector3::Z);
      bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
    } while (!bDone && unTrials <= MAX_PLACE_TRIALS);
    if (!bDone) {
      THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
    }
  }

}

void Planningloop::PlaceGuide(int num_guide, int num_worker, float density){
   UInt32 unTrials;
   CCylinderEntity* pccyl;
   std::ostringstream ccylinId;
   CVector3 ccylinPos;
   CQuaternion ccylinRot;
   CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
   float arena_side_length = sqrt(num_worker/density);
   float placement_angles[8] ={0, 3.14159, 1.5708, -1.5708, 0.785398, -0.785398, 4.71239, -4.71239};

   if(num_guide > 8){
      THROW_ARGOSEXCEPTION("Can't place more than 8 guides asked to place more robots" );
   }
   /*Place the targets */
   for(int i=0; i< num_guide; ++i){
      CVector2 robot_pose((arena_side_length/2),CRadians( placement_angles[i]) );
      float target_pose[2]={0,0};
      CKheperaIVEntity* pckh;
      pckh = new CKheperaIVEntity(
         "kh"+std::to_string(GUIDE_START_ID+i),
         FB_CONTROLLER,
         CVector3(target_pose[0],target_pose[1],0),
         CQuaternion(),
         RAB_RANGE,
         un_data_size);
      AddEntity(*pckh);
      CRange<Real> cTarRange_x( robot_pose.GetX() - 1 , 
                                robot_pose.GetX() + 1.0);
      CRange<Real> cTarRange_y( robot_pose.GetY() - 1 , 
                                robot_pose.GetY() + 1.0);

      /* Try to place it in the arena */
      unTrials = 0;
      bool bDone;
      do {
         /* Choose a random position */
         ++unTrials;
         ccylinRot.FromAngleAxis(pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                              CVector3::Z);
         Real pos_x = pcRNG->Uniform(cTarRange_x);
         Real pos_y = pcRNG->Uniform(cTarRange_y);
         bDone = false;
         ccylinPos.Set(pos_x,
                  pos_y,
                  0.0f);
         bDone = MoveEntity(pckh->GetEmbodiedEntity(), ccylinPos, ccylinRot);
      } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
      if(!bDone) {
         THROW_ARGOSEXCEPTION("Can't place " << ccylinId.str());
      }
   }
   
}



void Planningloop::CloseFile(std::ofstream& c_stream) {
   if(c_stream.is_open()) c_stream.close();
}


/****************************************/
/****************************************/

void Planningloop::OpenFile(std::ofstream& c_stream,
                      const std::string& str_prefix) {
   /* Make filename */
   std::string strFName = str_prefix + m_strOutFile;
   /* Close file and reopen it */
   CloseFile(c_stream);
   c_stream.open(strFName.c_str(),
                 std::ofstream::out | std::ofstream::trunc);
   if(c_stream.fail())
      THROW_ARGOSEXCEPTION("Error opening \"" << strFName << "\": " << strerror(errno));
}


/****************************************/
/****************************************/


REGISTER_LOOP_FUNCTIONS(Planningloop, "Convergence");
