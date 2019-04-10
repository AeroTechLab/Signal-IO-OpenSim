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

#include <iostream>
#include <cstdlib>
#include <thread>
#include <chrono>
//#include <iterator>
//#include <map>
#include <algorithm>
#include <vector>

const SimTK::Vec3 BLOCK_COLORS[] = { SimTK::Blue, SimTK::Red, SimTK::Green, SimTK::Yellow }; 

class OSimProcess;

// static std::map<const char*, OSimProcess*> processMap;
static std::vector<OSimProcess*> processList;

class OSimProcess
{
public:
  OpenSim::Model* model; 
  SimTK::State state;
  OpenSim::CoordinateActuator* inputActuator;
  OpenSim::CoordinateActuator* feedbackActuator;
  
  OSimProcess( const char* modelName, SimTK::Vec3 modelColor )
  {
    std::cout << "creating osim process" << std::endl;
    model = new OpenSim::Model();
//     model->setUseVisualizer( true );
  
    model->setName( modelName );
    model->setGravity( SimTK::Vec3( 0, 0, 0 ) );

    body = new OpenSim::Body( "body", 1.0, SimTK::Vec3( 0, 0, 0 ), SimTK::Inertia( 0, 0, 0 ) );
    model->addBody( body );

    const OpenSim::Ground& ground = model->getGround();
    OpenSim::SliderJoint* groundJoint = new OpenSim::SliderJoint( "body2ground", ground, *body );
    model->addJoint( groundJoint );
  
    OpenSim::Brick* blockMesh = new OpenSim::Brick( SimTK::Vec3( 0.5, 0.5, 0.5 ) );
    blockMesh->setColor( modelColor );
    body->attachGeometry( blockMesh );
  
    OpenSim::Coordinate& coordinate = groundJoint->updCoordinate();
    inputActuator = new OpenSim::CoordinateActuator( "input" );
    inputActuator->setCoordinate( &coordinate );
    model->addForce( inputActuator );
    feedbackActuator = new OpenSim::CoordinateActuator( "feedback" );
    feedbackActuator->setCoordinate( &coordinate );
    model->addForce( feedbackActuator );
  
    //model->finalizeFromProperties()
    //model->printBasicInfo()
  
    state = model->initSystem();

    inputActuator->overrideActuation( state, true );
    feedbackActuator->overrideActuation( state, true );
  
    coordinate.setValue( state, 0.0 );

    std::cout << "starting update thread" << std::endl;
    thread = std::thread( &OSimProcess::Process, this );
  }
  
  ~OSimProcess()
  {
    running = false;
    thread.join();
    
//     model->setUseVisualizer( false );
    delete model;
  }
  
private:
  OpenSim::Body* body;
  
  std::thread thread;
  volatile bool running;
  
  static void Process( OSimProcess* process )
  {
    const long TIME_STEP = 20;
    
    OpenSim::Manager manager( *(process->model) );
    process->state.setTime( 0 );
    manager.initialize( process->state );
    
    std::chrono::system_clock::time_point initialTime = std::chrono::system_clock::now();
    
    process->running = true;
    while( process->running )
    {
      std::chrono::system_clock::time_point simTime = std::chrono::system_clock::now();
      
      process->state = manager.integrate( ( simTime - initialTime ).count() / 1000.0 );
      
      std::this_thread::sleep_until( simTime + std::chrono::milliseconds( TIME_STEP ) );
      
      std::cout << "running update loop" << std::endl;
    }
  }
};

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE );

int InitDevice( const char* modelName )
{  
//   if( OSimProcess::processMap.find( modelName ) == OSimProcess::processMap.end() ) 
//     OSimProcess::processMap[ modelName ] = new OSimProcess( modelName, BLOCK_COLORS[ processMap.size() ] );
//   
//   return int( std::distance( OSimProcess::processMap.begin(), OSimProcess::processMap.find( modelName ) ) );
  
  for( int deviceIndex = 0; deviceIndex < processList.size(); deviceIndex++ )
    if( processList[ deviceIndex ]->model->getName() == modelName ) return deviceIndex;
    
  processList.push_back( new OSimProcess( modelName, BLOCK_COLORS[ processList.size() ] ) );
  std::cout << "device index: " << (int) processList.size() - 1 << std::endl;
  return (int) processList.size() - 1;
}

void EndDevice( int processID )
{
  return;
}

size_t GetMaxInputSamplesNumber( int processID )
{
  return 1;
}

size_t Read( int processID, unsigned int channel, double* ref_value )
{
  if( processID == SIGNAL_IO_DEVICE_INVALID_ID ) return 0;
  
//   OSimProcess* process = std::next( OSimProcess::processMap.begin(), size_t(processID) )->second;
  OSimProcess* process = processList[ processID ];
  
  if( channel == 0 ) *ref_value = process->inputActuator->getCoordinate()->getValue( process->state );
  else if( channel == 1 ) *ref_value = process->inputActuator->getCoordinate()->getSpeedValue( process->state );
  else if( channel == 2 ) *ref_value = process->inputActuator->getCoordinate()->getAccelerationValue( process->state );
  else if( channel == 3 ) *ref_value = process->inputActuator->getOverrideActuation( process->state );
  else *ref_value = 0.0;
  
  return 1;
}

bool HasError( int processID )
{
  return false;
}

void Reset( int processID )
{
  return;
}

bool CheckInputChannel( int processID, unsigned int channel )
{
  if( processID == SIGNAL_IO_DEVICE_INVALID_ID ) return false;
  
  if( channel > 3 ) return false;
  
  return true;
}

bool Write( int processID, unsigned int channel, double value )
{
  if( processID == SIGNAL_IO_DEVICE_INVALID_ID ) return false;
  
  if( channel != 0 ) return false;
  
//   OSimProcess* process = std::next( OSimProcess::processMap.begin(), size_t(processID) )->second;
  OSimProcess* process = processList[ processID ];
  
  process->feedbackActuator->setOverrideActuation( process->state, value );
  
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
