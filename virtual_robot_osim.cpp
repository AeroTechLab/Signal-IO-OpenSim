////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2019 Leonardo Consoni <consoni_2519@hotmail.com>       //
//                                                                            //
//  This file is part of RobotSystem-Lite.                                    //
//                                                                            //
//  RobotSystem-Lite is free software: you can redistribute it and/or modify  //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobotSystem-Lite is distributed in the hope that it will be useful,       //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobotSystem-Lite. If not, see <http://www.gnu.org/licenses/>.  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include "signal_io/signal_io.h"

#include <OpenSim/OpenSim.h>
// #include <OpenSim/Simulation/Model/Model.h>
// #include <OpenSim/Common/osimCommon.h>
// #include <simbody/SimTKcommon.h>
// #include <simbody/SimTKsimbody.h>

#include <cstdlib>
#include <thread>
#include <map>

using SimTK::Vec3;
using SimTK::Inertia;
using SimTK::Pi;

typedef struct _OSimProcess
{
  OpenSim::Model* model;
  OpenSim::Body* body;
  OpenSim::SliderJoint* groundJoint;
  std::thread thread;
}
OSimProcess;

std::map<OSimProcess, const char*> processMap;

OSimProcess* processesList = NULL;

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE );

int InitDevice( const char* taskConfig )
{  
  OSimModel osimModel = (OSimModel) malloc( sizeof(OSimModelData) );
  
  return 0;
}

void EndDevice( int taskID )
{
  return;
}

size_t GetMaxInputSamplesNumber( int taskID )
{
  return 1;
}

size_t Read( int taskID, unsigned int channel, double* ref_value )
{
  *ref_value = ( rand() % 1001 ) / 1000.0 - 0.5;
  
  return 1;
}

bool HasError( int taskID )
{
  return false;
}

void Reset( int taskID )
{
  return;
}

bool CheckInputChannel( int taskID, unsigned int channel )
{
  return true;
}

bool Write( int taskID, unsigned int channel, double value )
{
  return true;
}

bool AcquireOutputChannel( int taskID, unsigned int channel )
{
  return true;
}

void ReleaseOutputChannel( int taskID, unsigned int channel )
{
  return;
}
