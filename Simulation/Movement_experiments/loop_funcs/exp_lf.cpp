#include "exp_lf.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>

#include <sstream>
#include <list>

/****************************************/
/****************************************/

static const Real        FB_RADIUS        = 0.085036758f;
static const Real        FB_AREA          = ARGOS_PI * Square(0.085036758f);
static const std::string FB_CONTROLLER    = "fbc";
static const UInt32      MAX_PLACE_TRIALS = 20;
static const UInt32      MAX_ROBOT_TRIALS = 20;
static const Real        RAB_RANGE        = 6.0f;
static const Real        SF_RANGE         = RAB_RANGE / Sqrt(2);
static const Real        HALF_SF_RANGE    = SF_RANGE * 0.5f;
static const Real        WALL_THICKNESS   = 0.1;
static const Real        WALL_HEIGHT      = 2.0;

/****************************************/
/****************************************/

CExpLF::CExpLF() {
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

    /* Run in special dog-sheep setup, or just make a normal group of robots */
    if (shepherdMode) {
      UInt32 unSheep;
      UInt32 dogStartIdx;
      GetNodeAttribute(t_tree, "sheep", unSheep);
      GetNodeAttribute(t_tree, "dogStartIdx", dogStartIdx);
      UInt32 sheepStartIdx = 0;
      PlaceClusterCustom(unSheep, sheepStartIdx, 0, 0, unDataSize, fDensity);
      bool isDogCluster = false;
      GetNodeAttribute(t_tree, "isDogCluster", isDogCluster);
      if (isDogCluster) {
        UInt32 unDogs;
        GetNodeAttribute(t_tree, "dogs", unDogs);
        PlaceClusterCustom(unDogs, dogStartIdx, 3, 3, unDataSize, fDensity);
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
  buzzvm_t tBuzzVM;
  buzzswarm_members_t tMembers;
  UInt16 unTestSwarmId, unSwarmId = 1024;
  m_posFile << std::endl;
  m_posFile << GetSpace().GetSimulationClock();
  for (size_t i = 0; i < m_vecControllers.size(); ++i) {
    /* At first you get the footbot object */
    CFootBotEntity& cEFootBot = *any_cast<CFootBotEntity *>(m_fbEntities[i]);
    m_posFile << "," << i
          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX()
          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY()
          << "," << cEFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();
    // /* Get reference to the VM */
    // tBuzzVM = m_vecControllers[i]->GetBuzzVM();
    // /* Make sure no error occurred in the script */
    // if (tBuzzVM->state != BUZZVM_STATE_ERROR &&
    //     !buzzdict_isempty(tBuzzVM->swarms)) {
    //   /* Get the current swarm to which the robot belongs */
    //   unTestSwarmId = 1;
    //   if (*buzzdict_get(tBuzzVM->swarms, &unTestSwarmId, uint8_t)) {
    //     unSwarmId = unTestSwarmId;
    //   }
    //   else {
    //     unTestSwarmId = 2;
    //     if (*buzzdict_get(tBuzzVM->swarms, &unTestSwarmId, uint8_t))
    //       unSwarmId = unTestSwarmId;
    //   }
    //   /* Make sure this robot is part of a swarm */
    //   if (unSwarmId != 1024) {
    //     /* Get the swarm member table */
    //     tMembers = tBuzzVM->swarmmembers;
    //     /* Count the number of correct and incorrect members */
    //     SSwarmMemberCount sSMC(unSwarmId);
    //     buzzdict_foreach(tBuzzVM->swarmmembers,
    //                      SwarmMemberCount,
    //                      &sSMC);
    //     /* Output the record in the file */
    //     m_cOutFile << GetSpace().GetSimulationClock() << "\t"
    //                << tBuzzVM->robot << "\t"
    //                << sSMC.Correct << "\t"
    //                << sSMC.Incorrect
    //                << std::endl;
    //   }
    // }
  }
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

/****************************************/
/****************************************/

void CExpLF::PlaceLine(UInt32 un_robots,
                       UInt32 un_data_size) {
  CFootBotEntity* pcFB;
  std::ostringstream cFBId;
  /* For each robot */
  for (size_t i = 0; i < un_robots; ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "fb" << i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CFootBotEntity(
        cFBId.str(),
        FB_CONTROLLER,
        CVector3(i, i, 0),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    /* Add its controller to the list */
    m_vecControllers.push_back(
        &dynamic_cast<CBuzzControllerFootBot&>(
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

  void Insert(CFootBotEntity& c_entity) {
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
  CFootBotEntity* pcFB;
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
  pcFB = new CFootBotEntity(
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
      &dynamic_cast<CBuzzControllerFootBot&>(
          pcFB->GetControllableEntity().GetController()));

  /* Add other robots */
  for (UInt32 i = 1; i < un_robots; ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "fb" << i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CFootBotEntity(
        cFBId.str(),
        FB_CONTROLLER,
        CVector3(),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    /* Add its controller to the list */
    m_vecControllers.push_back(
        &dynamic_cast<CBuzzControllerFootBot&>(
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
  CFootBotEntity* pcFB;
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
    pcFB = new CFootBotEntity(
        cFBId.str(),
        FB_CONTROLLER,
        CVector3(),
        CQuaternion(),
        RAB_RANGE,
        un_data_size);
    AddEntity(*pcFB);
    /* Add its controller to the list */
    m_vecControllers.push_back(
        &dynamic_cast<CBuzzControllerFootBot&>(
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
  CFootBotEntity* pcFB;
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
    pcFB = new CFootBotEntity(
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
        &dynamic_cast<CBuzzControllerFootBot&>(
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
  CFootBotEntity* pcFB;
  std::ostringstream cFBId;
  /* Create a RNG (it is automatically disposed of by ARGoS) */
  CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
  /* For each robot */
  for (size_t i = 0; i < locations.size(); ++i) {
    /* Make the id */
    cFBId.str("");
    cFBId << "fb" << startIdx + i;
    /* Create the robot in the origin and add it to ARGoS space */
    pcFB = new CFootBotEntity(
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
        &dynamic_cast<CBuzzControllerFootBot&>(
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

REGISTER_LOOP_FUNCTIONS(CExpLF, "exp_lf");
