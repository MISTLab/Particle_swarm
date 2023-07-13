#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/buzz_controller_kheperaiv.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>

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
   virtual bool IsExperimentFinished();

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

   void rotate_Vec2(float* c_pos, Real ang);

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
   std::vector<CBuzzControllerKheperaIV*> m_vecControllers;
   std::vector<CKheperaIVEntity*> m_fbEntities;
   UInt32 unSheep;
   UInt32 unDogs;
   bool m_bDone;

};

