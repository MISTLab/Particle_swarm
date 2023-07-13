#include <argos3/core/simulator/loop_functions.h>
#include <buzz/argos/buzz_controller_footbot.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <fstream>
#include <string>

using namespace argos;

class CExpLF : public CLoopFunctions {

public:

   CExpLF();
   virtual ~CExpLF();

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void Reset();
   virtual void Destroy();

private:

   void PlaceLine(UInt32 un_robots,
                  UInt32 un_data_size);
   void PlaceCluster(UInt32 un_robots,
                     UInt32 un_data_size,
                     Real f_density);
   void PlaceClusterCustom(UInt32 un_robots,
                           UInt32 startIdx,
                           UInt32 min,
                           UInt32 max,
                           UInt32 un_data_size,
                           Real f_density);
   void PlaceScaleFree(UInt32 un_robots,
                       UInt32 un_data_size);
   void PlaceWalls(UInt32 un_robots,
                   UInt32 un_data_size,
                   Real f_distance);
   void PlaceUniformly(UInt32 un_robots,
                       UInt32 un_data_size,
                       CRange<Real> c_area_range);
   void PlaceUniformlyCustom(UInt32 un_robots,
                           UInt32 startIdx,
                           UInt32 un_data_size,
                           CRange<Real> c_area_range);
   void PlaceAbsolute(UInt32 startIdx,
                        UInt32 un_data_size,
                        std::vector<CVector3*> locations);

   void CloseFile(std::ofstream& c_stream);

private:

   enum ETopology {
      TOPOLOGY_LINE,
      TOPOLOGY_CLUSTER,
      TOPOLOGY_SCALEFREE
   };

   std::string m_strOutFile;
   std::ofstream m_cOutFile;
   std::string m_strPosFile;
   std::ofstream m_posFile;
   std::vector<CBuzzControllerFootBot*> m_vecControllers;
   std::vector<CFootBotEntity*> m_fbEntities;

};

