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
static const Real        INDOOR_MAP_RESOLUTION = 0.05;
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
static const UInt16 GUIDE_START_ID=1200;
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
      GetNodeAttribute(t_tree, "density", number_of_guides_required);
      

      
      /* Get reference to buzz controllers, robots and stack them up in a vec*/
      for(int i=0; i<NUM_OF_FOLLOWERS; i++){
         std::stringstream os;
         os << "bot" << i;
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
      if(i < NUM_OF_FOLLOWERS){
         CFootBotEntity& cEFootBot = *any_cast<CFootBotEntity *>(m_workers[i]);
         m_posFile << "," << vm->robot
               << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX()
               << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY()
               << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();
      }
      else{
         /* At first you get the footbot object */
         CKheperaIVEntity& cEKhepera = *any_cast<CKheperaIVEntity *>(m_guides[i-NUM_OF_FOLLOWERS]);
         m_posFile << "," << vm->robot
               << "," << cEKhepera.GetEmbodiedEntity().GetOriginAnchor().Position.GetX()
               << "," << cEKhepera.GetEmbodiedEntity().GetOriginAnchor().Position.GetY()
               << "," << cEKhepera.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();
      }
      // BVMSTATE
      {
         buzzvm_pushs(vm, buzzvm_string_register(vm, "BVMSTATE", 1));
         buzzvm_gload(vm);
         buzzobj_t o = buzzvm_stack_at(vm, 1);
         write_obj_to_file(o,m_posFile);
         buzzvm_pop(vm);
      }
      // EXPLORE_STATE - exploration state
      {
         buzzvm_pushs(vm, buzzvm_string_register(vm, "EXPLORE_STATE", 1));
         buzzvm_gload(vm);
         buzzobj_t o = buzzvm_stack_at(vm, 1);
         write_obj_to_file(o,m_posFile);
         buzzvm_pop(vm);
      }
     
   }
   // Experiment end condition. 
   if(num_of_targets_reached >= number_of_targets || GetSpace().GetSimulationClock() > 120000){
      // m_bDone = true;
      if(exp_end_counter > 100){
         THROW_ARGOSEXCEPTION("Experiment done ... "); // Experiment does't end all the time, this is a hack for now.
      }
      else if(exp_end_counter == 60){
         if(num_of_targets_reached >= number_of_targets){
            std::stringstream cpcommmand;
            cpcommmand <<"cp "<<RUN_DIR_Data_file<<" "<<"/scratch/viveks/Hir_hetro/data/";
            printf(" Copying data .. with cmd %s\n",cpcommmand.str().c_str());
            system(cpcommmand.str().c_str());
         }
         else{
            printf(" SIMULATION TIME LIMIT Reached exp did not complete\n");
         }
      }
      
      exp_end_counter = exp_end_counter + 1;
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


   UInt32 unTrials;
   CCylinderEntity* pccyl;
   std::ostringstream ccylinId;
   CVector3 ccylinPos;
   CQuaternion ccylinRot;
   Real number_of_trees = (map_size*map_size/TREE_RADIUS)*FOREST_Density;
  /* Create a RNG (it is automatically disposed of by ARGoS) */
  CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
  CRange<Real> cAreaRange(-map_size/2, map_size/2);
   /* For each tree */
   for(size_t i = 0; i < number_of_trees; ++i) {
      ccylinId.str("cyl");
      ccylinId << i;
      pccyl = new CCylinderEntity(ccylinId.str(),
                     CVector3(),
                     CQuaternion(),
                     false,
                     TREE_RADIUS,
                     WALL_HEIGHT);
      AddEntity(*pccyl);
      /* Try to place it in the arena */
      unTrials = 0;
      bool bDone;
      do {
         /* Choose a random position */
         ++unTrials;
         ccylinRot.FromAngleAxis(pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                              CVector3::Z);
         Real pos_x = pcRNG->Uniform(cAreaRange);
         Real pos_y = pcRNG->Uniform(cAreaRange);
         bDone = false;
         if( ! (pos_x > -ROBOT_DISTRIBUTION_CLEARENCE && pos_x < ROBOT_DISTRIBUTION_CLEARENCE &&
                  pos_y > -ROBOT_DISTRIBUTION_CLEARENCE && pos_y < ROBOT_DISTRIBUTION_CLEARENCE) ){
            ccylinPos.Set(pos_x,
                     pos_y,
                     0.0f);
            bDone = MoveEntity(pccyl->GetEmbodiedEntity(), ccylinPos, ccylinRot);
         }
         if(bDone){
            CVector2 c_tree(pos_x,pos_y);
            Trees_pos.push_back(c_tree);
         }
      } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
      if(!bDone) {
         THROW_ARGOSEXCEPTION("Can't place " << ccylinId.str());
      }
   }
}
/****************************************/
/****************************************/


REGISTER_LOOP_FUNCTIONS(Planningloop, "Planning");
