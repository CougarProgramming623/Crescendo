#pragma once 

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
//#include "networktables/NetworkTableValue.h" //idk


class COB {
 public:
  inline nt::NetworkTableInstance& GetTable() { return m_Table; }

 private:
  nt::NetworkTableInstance m_Table = nt::NetworkTableInstance::GetDefault();
};