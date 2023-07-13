#include "kh_exp_lf.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>

#include <sstream>
#include <list>
#include <string>  

/****************************************/
/****************************************/

static const Real        FB_RADIUS        = 0.085036758f;
static const Real        FB_AREA          = ARGOS_PI * Square(0.085036758f);
static const std::string FB_CONTROLLER    = "khivbs";
static const UInt32      MAX_PLACE_TRIALS = 20;
static const UInt32      MAX_ROBOT_TRIALS = 20;
static const Real        RAB_RANGE        = 4.0f;
static const Real        SF_RANGE         = 3.0f / Sqrt(2);
static const Real        HALF_SF_RANGE    = SF_RANGE * 0.5f;
static const Real        WALL_THICKNESS   = 0.1;
static const Real        WALL_HEIGHT      = 2.0;

/****************************************/
/****************************************/

CExpLF::CExpLF() :
  m_bDone(false)  {
}

/****************************************/
/****************************************/

CExpLF::~CExpLF() {
}

/****************************************/
/****************************************/

void CExpLF::Init(TConfigurationNode& t_tree) {
  LOG.Flush();
  LOGERR.Flush();
  try {
    /* Parse the configuration file */
    GetNodeAttribute(t_tree, "outfile", m_strOutFile);
    GetNodeAttribute(t_tree, "posfile", m_strPosFile);
    bool shepherdMode;
    GetNodeAttribute(t_tree, "shepherdMode", shepherdMode);
    UInt32 unDataSize;
    GetNodeAttribute(t_tree, "data_size", unDataSize);
    Real fDensity;
    GetNodeAttribute(t_tree, "density", fDensity);
    std::string str_movement;
    UInt32 sMovement=0;
    GetNodeAttribute(t_tree, "movement", str_movement);
    if(str_movement == "straight"){
      sMovement = 0;
    }
    else if(str_movement == "diagonal"){
      sMovement = 1;
    }

    std::string str_shape;

    UInt32 shape=2;
    GetNodeAttribute(t_tree, "shape", str_shape);
    if(str_shape == "clover"){
      shape = 0;
    }
    else if(str_shape == "dumbbell"){
      shape = 1;
    }
    else if(str_shape == "none"){
      shape = 2;
    }
    else if(str_shape == "torus"){
      shape = 3;
    }
    /* Run in special dog-sheep setup, or just make a normal group of robots */
    if (shepherdMode) {
      UInt32 dogStartIdx;
      GetNodeAttribute(t_tree, "sheep", unSheep);
      GetNodeAttribute(t_tree, "dogStartIdx", dogStartIdx);
      UInt32 sheepStartIdx = 0;
      PlaceClusterCustom(unSheep, sheepStartIdx, 0, 0, unDataSize, fDensity);
      bool isDogCluster = false;
      GetNodeAttribute(t_tree, "isDogCluster", isDogCluster);
      if (isDogCluster) {
        GetNodeAttribute(t_tree, "dogs", unDogs);
        // unDogs =4;
        // if(unSheep == 100) unDogs = 8;
        // if(unSheep == 300) unDogs = 16;
        // if(unSheep == 1000) unDogs = 32;

        Real fHalfSide = Sqrt((FB_AREA * unSheep) / fDensity) / 2.0f;
        PlaceClusterCustom(unDogs, dogStartIdx, fHalfSide+4, fHalfSide+4, unDataSize, fDensity);
      } else {
        std::vector<CVector3*> dogLocations;
        dogLocations.push_back(new CVector3(3, 0.0, 0.0));
        dogLocations.push_back(new CVector3(0.0, 3.0, 0.0));
        dogLocations.push_back(new CVector3(-3.0, 0.0, 0.0));
        dogLocations.push_back(new CVector3(0.0, -3.0, 0.0));
        PlaceAbsolute(dogStartIdx, unDataSize, dogLocations);
      }
    } else {
      UInt32 unRobots;
      GetNodeAttribute(t_tree, "robots", unRobots);

      bool bWalls;
      GetNodeAttribute(t_tree, "walls", bWalls);
      if (bWalls) {
        PlaceWalls(unRobots, unDataSize, fDensity);
      }
      else {
        std::string strTopology;
        GetNodeAttribute(t_tree, "topology", strTopology);
        if(strTopology == "line")           PlaceLine     (unRobots, unDataSize);
        else if(strTopology == "cluster")   PlaceCluster  (unRobots, unDataSize, fDensity);
        else if(strTopology == "scalefree") PlaceScaleFree(unRobots, unDataSize);
        else {
          THROW_ARGOSEXCEPTION("Unknown topology \"" << strTopology << "\"");
        }
      }
    }

    /*Once the robots are placed configure the right experimental parameters on all robots*/    
    for (size_t i = 0; i < m_vecControllers.size(); ++i) {
      /* At first you get the footbot object */
      CKheperaIVEntity& cEFootBot = *any_cast<CKheperaIVEntity *>(m_fbEntities[i]);
      buzzvm_t vm = m_vecControllers[i]->GetBuzzVM();


      // buzzvm_function_call(vm, fun_string.c_str(), 0);
      buzzvm_pushs(vm, buzzvm_string_register(vm, "NUM_OF_SHEEP", 1));
      buzzvm_pushi(vm, unSheep);
      buzzvm_gstore(vm);

      buzzvm_pushs(vm, buzzvm_string_register(vm, "NUM_OF_DOGS_TO_USE", 1));
      buzzvm_pushi(vm, unDogs);
      buzzvm_gstore(vm);

      // Possible values for reference
      // std::enum shapes { clover dumbbell none torus };
      buzzvm_pushs(vm, buzzvm_string_register(vm, "SET_SHAPE", 1));
      buzzvm_pushi(vm, shape);
      buzzvm_gstore(vm);

      // std::enum movements { straight diagonal rotation };
      buzzvm_pushs(vm, buzzvm_string_register(vm, "MOVEMENT_TYPE", 1));
      buzzvm_pushi(vm, sMovement);
      buzzvm_gstore(vm);

      buzzvm_function_call(vm, "load_targets_from_loop_fun", 0);

      /*Clear the nil value returned*/
      buzzvm_pop(vm);
    }

    /* Initialize the rest */
    Reset();
  }
  catch (CARGoSException& ex) {
    THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
  }
}

/****************************************/
/****************************************/


void CExpLF::PostStep() {
  /* Go through the robots */
  buzzvm_t vm;
  m_posFile << std::endl;
  m_cOutFile << std::endl;
  m_posFile << GetSpace().GetSimulationClock();
  m_cOutFile << GetSpace().GetSimulationClock();
  int num_of_dogs_reached =0;
  for (size_t i = 0; i < m_vecControllers.size(); ++i) {
    /* At first you get the footbot object */
    CKheperaIVEntity& cEFootBot = *any_cast<CKheperaIVEntity *>(m_fbEntities[i]);
    m_posFile << "," << i
          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX()
          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY()
          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();
    // /* Get reference to the VM */
    vm = m_vecControllers[i]->GetBuzzVM();
    if(vm->robot > unSheep){
      /* Make sure no error occurred in the script */
      if (vm->state != BUZZVM_STATE_ERROR ) {
        float center_of_mass[2];
        /* Output the record in the file */
        m_cOutFile << ","<< vm->robot ;
        buzzvm_pushs(vm, buzzvm_string_register(vm, "Center_of_mass", 1));
        buzzvm_gload(vm);
        buzzvm_dup(vm);
        buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 1));
        buzzvm_tget(vm);
        center_of_mass[0] = buzzvm_stack_at(vm, 1)->f.value/100;
        buzzvm_pop(vm);
        buzzvm_dup(vm);
        buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 1));
        buzzvm_tget(vm);
        center_of_mass[1] = buzzvm_stack_at(vm, 1)->f.value/100;
        buzzvm_pop(vm);
        buzzvm_dup(vm);
        CQuaternion& c_quat = cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
        CRadians cYaw, cPitch, cRoll;
        c_quat.ToEulerAngles(cYaw, cPitch, cRoll);
        /*Convert it to global coordinates */
        rotate_Vec2(center_of_mass, cYaw.GetValue());
        center_of_mass[0] =cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX()+center_of_mass[0];
        center_of_mass[1] =cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY()+center_of_mass[1];
        m_cOutFile << ","<<cYaw.GetValue()
                   << ","<< center_of_mass[0]
                   << ","<< center_of_mass[1];
        buzzvm_pushs(vm, buzzvm_string_register(vm, "px", 1));
        buzzvm_tget(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->f.value;
        buzzvm_pop(vm);
        buzzvm_dup(vm);
        buzzvm_pushs(vm, buzzvm_string_register(vm, "py", 1));
        buzzvm_tget(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->f.value;
        buzzvm_pop(vm);
        buzzvm_dup(vm);
        buzzvm_pushs(vm, buzzvm_string_register(vm, "cx", 1));
        buzzvm_tget(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->f.value;
        buzzvm_pop(vm);
        buzzvm_dup(vm);
        buzzvm_pushs(vm, buzzvm_string_register(vm, "cy", 1));
        buzzvm_tget(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->f.value;
        buzzvm_pop(vm);
        buzzvm_dup(vm);
        buzzvm_pushs(vm, buzzvm_string_register(vm, "fx", 1));
        buzzvm_tget(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->f.value;
        buzzvm_pop(vm);
        buzzvm_dup(vm);
        buzzvm_pushs(vm, buzzvm_string_register(vm, "fy", 1));
        buzzvm_tget(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->f.value;
        buzzvm_pop(vm);
        buzzvm_dup(vm);
        buzzvm_pushs(vm, buzzvm_string_register(vm, "setpang", 1));
        buzzvm_tget(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->f.value;
        buzzvm_pop(vm);
        buzzvm_pushs(vm, buzzvm_string_register(vm, "curang", 1));
        buzzvm_tget(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->f.value;
        buzzvm_pop(vm);

        buzzvm_pushs(vm, buzzvm_string_register(vm, "Mode", 1));
        buzzvm_gload(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->i.value;
        buzzvm_pop(vm);

        buzzvm_pushs(vm, buzzvm_string_register(vm, "shaping_time", 1));
        buzzvm_gload(vm);
        m_cOutFile << ","<< buzzvm_stack_at(vm, 1)->i.value;
        buzzvm_pop(vm);

        buzzvm_pushs(vm, buzzvm_string_register(vm, "TAR_REACHED", 1));
        buzzvm_gload(vm);

        if(buzzvm_stack_at(vm, 1)->i.value == 1){
          num_of_dogs_reached++;
        }
        buzzvm_pop(vm); 
      }
    }
  }
  if(num_of_dogs_reached >= unDogs) m_bDone = true;
}

/****************************************/
/****************************************/

void CExpLF::Reset() {
  /* Close file and reopen it */
  CloseFile(m_cOutFile);
  m_cOutFile.open(m_strOutFile.c_str(),
                  std::ofstream::out | std::ofstream::trunc);
  if (m_cOutFile.fail())
    THROW_ARGOSEXCEPTION("Error opening \"" << m_strOutFile << "\": "); // << strerror(errno));

  /* Close positions file and reopen it */
  CloseFile(m_posFile);
  m_posFile.open(m_strPosFile.c_str(),
                 std::ofstream::out | std::ofstream::trunc);
  if (m_posFile.fail())
    THROW_ARGOSEXCEPTION("Error opening \"" << m_strPosFile << "\": "); // << strerror(errno));
  /* Prep the output positions file: */
  m_posFile << "time,";
  for (size_t i = 0; i < m_fbEntities.size(); ++i) {
    m_posFile << "robotId:" << i << ","
              << "x" << i << ","
              << "y" << i << ","
              << "z" << i << (i+1 < m_fbEntities.size() ? "," : "");
  }
  // m_posFile << std::endl;
}

/****************************************/
/****************************************/

void CExpLF::Destroy() {
  CloseFile(m_cOutFile);
  CloseFile(m_posFile);
}


bool CExpLF::IsExperimentFinished() {
   return m_bDone;
}
/****************************************/
/****************************************/

void CExpLF::PlaceLine(UInt32 un_robots,
                       UInt32 un_data_size) {
  CKheperaIVEntity* pcFB;
  std::ostringstream cFBId;
  /* For each robot */
  for (size_t i = 0; i < un_robots; ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "fb" << i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CKheperaIVEntity(
        cFBId.str(),
        FB_CONTROLLER,
        CVector3(i, i, 0),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    /* Add its controller to the list */
    m_vecControllers.push_back(
        &dynamic_cast<CBuzzControllerKheperaIV&>(
            pcFB->GetControllableEntity().GetController()));
  }
}

/****************************************/
/****************************************/

void CExpLF::PlaceCluster(UInt32 un_robots,
                          UInt32 un_data_size,
                          Real f_density) {
  /* Calculate side of the region in which the robots are scattered */
  Real fHalfSide = Sqrt((FB_AREA * un_robots) / f_density) / 2.0f;
  CRange<Real> cAreaRange(-fHalfSide, fHalfSide);
  /* Place robots */
  PlaceUniformly(un_robots, un_data_size, cAreaRange);
}

/* Place in a square region specified by min and max */
void CExpLF::PlaceClusterCustom(UInt32 un_robots,
                                UInt32 startIdx,
                                UInt32 min,
                                UInt32 max,
                                UInt32 un_data_size,
                                Real f_density) {
  /* Calculate side of the region in which the robots are scattered */
  Real fHalfSide = Sqrt((FB_AREA * un_robots) / f_density) / 2.0f;
  CRange<Real> cAreaRange(min - fHalfSide, max + fHalfSide);
  /* Place robots */
  PlaceUniformlyCustom(un_robots, startIdx, un_data_size, cAreaRange);
}

/****************************************/
/****************************************/

struct SFData {

  struct SEntry {
    UInt32 Conns;
    CVector3& Pos;
    SEntry(UInt32 un_conns,
           CVector3& c_pos) : Conns(un_conns),
                              Pos(c_pos) {}
  };

  SFData() :
      TotConns(0),
      RNG(CRandom::CreateRNG("argos")) {}

  ~SFData() {
    while (!Data.empty()) {
      delete Data.front();
      Data.pop_front();
    }
  }

  void Insert(CKheperaIVEntity& c_entity) {
    /* Two connections to be added: entity <-> pivot */
    TotConns += 2;
    Data.push_back(
        new SEntry(1,
                   c_entity.GetEmbodiedEntity().GetOriginAnchor().Position));
  }

  SEntry* Pick() {
    if (Data.size() > 1) {
      /* More than 1 element stored, look for the pivot */
      UInt32 x = RNG->Uniform(CRange<UInt32>(0, TotConns));
      UInt32 unSum = 0;
      std::list<SEntry*>::iterator it = Data.begin();
      while (it != Data.end() && unSum <= x) {
        unSum += (*it)->Conns;
        ++it;
      }
      if (it != Data.end()) {
        --it;
        return *it;
      }
      else {
        return Data.back();
      }
    }
    else if (Data.size() == 1) {
      /* One element stored, just return that one */
      return Data.front();
    }
    else THROW_ARGOSEXCEPTION("SFData::Pick(): empty structure");
  }

private:
  std::list<SEntry*> Data;
  UInt32 TotConns;
  CRandom::CRNG* RNG;
};

static Real GenerateCoordinate(CRandom::CRNG* pc_rng) {
  Real v = pc_rng->Uniform(CRange<Real>(-HALF_SF_RANGE, HALF_SF_RANGE));
  if (v > 0.0) v += HALF_SF_RANGE;
  else v -= HALF_SF_RANGE;
  return v;
}

void CExpLF::PlaceScaleFree(UInt32 un_robots,
                            UInt32 un_data_size) {
  /* Data structures for the insertion of new robots */
  UInt32 unRobotTrials, unPlaceTrials, unPivot;
  CKheperaIVEntity* pcFB;
  std::ostringstream cFBId;
  CVector3 cFBPos;
  CQuaternion cFBRot;
  SFData sData;
  SFData::SEntry* psPivot;
  bool bDone;
  /* Create a RNG (it is automatically disposed of by ARGoS) */
  CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");

  /* Add first robot in the origin */
  /* Create the robot in the origin and add it to ARGoS space */
  pcFB = new CKheperaIVEntity(
      "fb0",
      FB_CONTROLLER,
      CVector3(),
      CQuaternion(),
      RAB_RANGE,
      un_data_size);
  AddEntity(*pcFB);
  sData.Insert(*pcFB);
  /* Add its controller to the list */
  m_vecControllers.push_back(
      &dynamic_cast<CBuzzControllerKheperaIV&>(
          pcFB->GetControllableEntity().GetController()));

  /* Add other robots */
  for (UInt32 i = 1; i < un_robots; ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "fb" << i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CKheperaIVEntity(
        cFBId.str(),
        FB_CONTROLLER,
        CVector3(),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    /* Add its controller to the list */
    m_vecControllers.push_back(
        &dynamic_cast<CBuzzControllerKheperaIV&>(
            pcFB->GetControllableEntity().GetController()));
    /* Retry choosing a pivot until you get a position or have an error */
    unRobotTrials = 0;
    do {
      /* Choose a pivot */
      ++unRobotTrials;
      psPivot = sData.Pick();
      cFBRot.FromAngleAxis(pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                           CVector3::Z);
      /* Try placing a robot close to this pivot */
      unPlaceTrials = 0;
      do {
        ++unPlaceTrials;
        /* Pick a position within the range of the pivot */
        cFBPos.Set(GenerateCoordinate(pcRNG),
                   GenerateCoordinate(pcRNG),
                   0.0f);
        cFBPos += psPivot->Pos;
        /* Try placing the robot */
        bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
      } while (!bDone && unPlaceTrials <= MAX_PLACE_TRIALS);
    } while (!bDone && unRobotTrials <= MAX_ROBOT_TRIALS);
    /* Was the robot placed successfully? */
    if (!bDone) {
      THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
    }
    /* Yes, insert it in the data structure */
    ++psPivot->Conns;
    sData.Insert(*pcFB);
  }
}

/****************************************/
/****************************************/

void CExpLF::PlaceWalls(UInt32 un_robots,
                        UInt32 un_data_size,
                        Real f_density) {
  /* Calculate arena side */
  Real fArenaSide =
      RAB_RANGE *
      Sqrt((25.0 * ARGOS_PI * un_robots) /
           ((100.0 - 4.0 * ARGOS_PI) * f_density));
  Real fArenaSide2 = fArenaSide / 2.0;
  Real fArenaSide5 = fArenaSide / 5.0;
  Real fArenaSide10 = fArenaSide / 10.0;
  /* Place the north wall */
  AddEntity(
      *new CBoxEntity("wall_north",
                      CVector3(fArenaSide2, 0, 0),
                      CQuaternion(),
                      false,
                      CVector3(WALL_THICKNESS, fArenaSide, WALL_HEIGHT)));
  /* Place the south wall */
  AddEntity(
      *new CBoxEntity("wall_south",
                      CVector3(-fArenaSide2, 0, 0),
                      CQuaternion(),
                      false,
                      CVector3(WALL_THICKNESS, fArenaSide, WALL_HEIGHT)));
  /* Place the west wall */
  AddEntity(
      *new CBoxEntity("wall_west",
                      CVector3(0, fArenaSide2, 0),
                      CQuaternion(),
                      false,
                      CVector3(fArenaSide, WALL_THICKNESS, WALL_HEIGHT)));
  /* Place the east wall */
  AddEntity(
      *new CBoxEntity("wall_east",
                      CVector3(0, -fArenaSide2, 0),
                      CQuaternion(),
                      false,
                      CVector3(fArenaSide, WALL_THICKNESS, WALL_HEIGHT)));
  /* Place the NW column */
  AddEntity(
      *new CCylinderEntity("col_nw",
                           CVector3(fArenaSide5, fArenaSide5, 0),
                           CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
  /* Place the NE column */
  AddEntity(
      *new CCylinderEntity("col_ne",
                           CVector3(fArenaSide5, -fArenaSide5, 0),
                           CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
  /* Place the SW column */
  AddEntity(
      *new CCylinderEntity("col_sw",
                           CVector3(-fArenaSide5, fArenaSide5, 0),
                           CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
  /* Place the SE column */
  AddEntity(
      *new CCylinderEntity("col_se",
                           CVector3(-fArenaSide5, -fArenaSide5, 0),
                           CQuaternion(),
                           false,
                           fArenaSide10,
                           WALL_HEIGHT));
  /* Calculate side of the region in which the robots are scattered */
  CRange<Real> cAreaRange(-fArenaSide2, fArenaSide2);
  /* Place robots */
  PlaceUniformly(un_robots, un_data_size, cAreaRange);
}

/****************************************/
/****************************************/

void CExpLF::PlaceUniformly(UInt32 un_robots,
                            UInt32 un_data_size,
                            CRange<Real> c_area_range) {
  UInt32 unTrials;
  CKheperaIVEntity* pcFB;
  std::ostringstream cFBId;
  CVector3 cFBPos;
  CQuaternion cFBRot;
  /* Create a RNG (it is automatically disposed of by ARGoS) */
  CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
  /* For each robot */
  for (size_t i = 0; i < un_robots; ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "fb" << i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CKheperaIVEntity(
        cFBId.str(),
        FB_CONTROLLER,
        CVector3(),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    /* Add its controller to the list */
    m_vecControllers.push_back(
        &dynamic_cast<CBuzzControllerKheperaIV&>(
            pcFB->GetControllableEntity().GetController()));
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

void CExpLF::PlaceUniformlyCustom(UInt32 un_robots,
                                  UInt32 startIdx,
                                  UInt32 un_data_size,
                                  CRange<Real> c_area_range) {
  UInt32 unTrials;
  CKheperaIVEntity* pcFB;
  std::ostringstream cFBId;
  CVector3 cFBPos;
  CQuaternion cFBRot;
  /* Create a RNG (it is automatically disposed of by ARGoS) */
  CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
  /* For each robot */
  for (size_t i = startIdx; i < startIdx + un_robots; ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "fb" << i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CKheperaIVEntity(
        cFBId.str(),
        FB_CONTROLLER,
        CVector3(0.0, 0.0, 0.0),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    /* Add entity */
    m_fbEntities.push_back(pcFB);
    /* Add its controller to the list */
    m_vecControllers.push_back(
        &dynamic_cast<CBuzzControllerKheperaIV&>(
            pcFB->GetControllableEntity().GetController()));
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
    } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
    if(!bDone) {
       THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
    }
  }
}

void CExpLF::PlaceAbsolute(UInt32 startIdx,
                            UInt32 un_data_size,
                            std::vector<CVector3*> locations) {
  CKheperaIVEntity* pcFB;
  std::ostringstream cFBId;
  /* Create a RNG (it is automatically disposed of by ARGoS) */
  CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
  /* For each robot */
  for (size_t i = 0; i < locations.size(); ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "fb" << startIdx + i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CKheperaIVEntity(
        cFBId.str(),
        FB_CONTROLLER,
        *(locations[i]),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    /* Add entity */
    m_fbEntities.push_back(pcFB);
    /* Add its controller to the list */
    m_vecControllers.push_back(
        &dynamic_cast<CBuzzControllerKheperaIV&>(
            pcFB->GetControllableEntity().GetController()));
  }
}

/****************************************/
/****************************************/

void CExpLF::CloseFile(std::ofstream& c_stream) {
  if (c_stream.is_open()) c_stream.close();
}

/****************************************/
/****************************************/

void CExpLF::rotate_Vec2(float* c_pos, Real ang){
  float calc_pos[2];

  calc_pos[0] = c_pos[0] * cos(ang) - c_pos[1] * sin(ang);
  calc_pos[1] = c_pos[0] * sin(ang) + c_pos[1] * cos(ang);
  c_pos[0] = calc_pos[0];
  c_pos[1] = calc_pos[1];
}

REGISTER_LOOP_FUNCTIONS(CExpLF, "exp_lf");
