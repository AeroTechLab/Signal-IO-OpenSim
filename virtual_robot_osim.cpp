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
#include <simbody/internal/Visualizer_InputListener.h>

#include <iostream>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <vector>

class OSimProcess;

static std::vector<OSimProcess*> processList;

class OSimProcess
{
public:  
  OSimProcess( const char* modelName, int index )
  {
    const SimTK::Vec3 BLOCK_COLORS[] = { SimTK::Blue, SimTK::Red, SimTK::Green, SimTK::Yellow };
    
    std::cout << "creating osim process" << std::endl;
  
    model.setUseVisualizer( true );
    
    model.setName( modelName );
    model.setGravity( SimTK::Vec3( 0, 0, 0 ) );

    body = new OpenSim::Body( "body", 1.0, SimTK::Vec3( 0, 0, 0 ), SimTK::Inertia( 1, 1, 1 ) );
    model.addBody( body );

    const OpenSim::Ground& ground = model.getGround();
    OpenSim::PinJoint* groundJoint = new OpenSim::PinJoint( "body2ground", ground, *body );
    //OpenSim::SliderJoint* groundJoint = new OpenSim::SliderJoint( "body2ground", ground, *body );
    model.addJoint( groundJoint );
  
    OpenSim::Brick* blockMesh = new OpenSim::Brick( SimTK::Vec3( 0.5, 0.5, 0.5 ) );
    blockMesh->setColor( BLOCK_COLORS[ index ] );
    body->attachGeometry( blockMesh );
  
    OpenSim::Coordinate& coordinate = groundJoint->updCoordinate( OpenSim::PinJoint::Coord::RotationZ );
    double coordinateRange[ 2 ] = { -SimTK::Pi, SimTK::Pi };
    coordinate.setRange( coordinateRange );
    inputActuator = new OpenSim::CoordinateActuator( "input" );
    inputActuator->setCoordinate( &coordinate );
    model.addForce( inputActuator );
    feedbackActuator = new OpenSim::CoordinateActuator( "feedback" );
    feedbackActuator->setCoordinate( &coordinate );
    model.addForce( feedbackActuator );
    
    state = model.initSystem();

    inputSliderID = index * 2 + 1;
    model.updVisualizer().updSimbodyVisualizer().addSlider( "Input Force", inputSliderID, -1.0, 1.0, 0.0 );
    feedbackSliderID = index * 2 + 2;
    model.updVisualizer().updSimbodyVisualizer().addSlider( "Feedback Force", inputSliderID, -2.0, 2.0, 0.0 );
    
    inputActuator->overrideActuation( state, true );
    feedbackActuator->overrideActuation( state, true );
  
    coordinate.setValue( state, 0.0 );
    
    state.setTime( 0.0 );
    
    measuresList.resize( VARS_NUMBER );
    
//     initialTime = std::chrono::steady_clock::now();
//     simulationTime = std::chrono::steady_clock::now();
    
    std::cout << "starting update thread" << std::endl;
    updateThread = std::thread( &OSimProcess::Update, this );
  }
  
  ~OSimProcess()
  {
    isUpdating = false;
    updateThread.join();
    
    model.setUseVisualizer( false );
  }
  
//   void Update()
//   {
//     const double MIN_TIME_STEP = 0.02;
//     
//     std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
//     
//     if( std::chrono::duration_cast<std::chrono::duration<double>>( currentTime - simulationTime ).count() > MIN_TIME_STEP )
//     {
//       while( model.updVisualizer().updInputSilo().isAnyUserInput() ) 
//       {
//         SimTK::Real forceValue;
//         while( model.updVisualizer().updInputSilo().takeSliderMove( inputSliderID, forceValue ) ) 
//           inputActuator->setOverrideActuation( state, forceValue );
//         while( model.updVisualizer().updInputSilo().takeSliderMove( feedbackSliderID, forceValue ) )
//           feedbackActuator->setOverrideActuation( state, forceValue );
//       }
//       
//       simulationTime = currentTime;
//       std::cout << "simulation time: " << state.getTime() << ", position: " << inputActuator->getCoordinate()->getValue( state ) << std::endl;
//       OpenSim::Manager manager( model );
//       manager.initialize( state );
//       state = manager.integrate( std::chrono::duration_cast<std::chrono::duration<double>>( simulationTime - initialTime ).count() );
//       
//       measuresList[ POSITION ] = inputActuator->getCoordinate()->getValue( state );
//       measuresList[ VELOCITY ] = inputActuator->getCoordinate()->getSpeedValue( state );
//       measuresList[ ACCELERATION ] = inputActuator->getCoordinate()->getAccelerationValue( state );
//       measuresList[ FORCE ] = inputActuator->getOverrideActuation( state );
//     }
//   }
  
  inline std::string GetName() { return model.getName(); }
  inline double GetMeasurement( size_t index ) { return index < VARS_NUMBER ? measuresList[ index ] : 0.0; }
  inline void SetForce( double value ) { model.updVisualizer().updSimbodyVisualizer().setSliderValue( feedbackSliderID, value ); }
  
private:
  enum OutputVariable { POSITION, VELOCITY, ACCELERATION, FORCE, VARS_NUMBER };
  
  OpenSim::Model model; 
  OpenSim::Body* body;
  OpenSim::CoordinateActuator* inputActuator;
  OpenSim::CoordinateActuator* feedbackActuator;
  SimTK::State state;
  
  std::thread updateThread;
  volatile bool isUpdating;
  std::chrono::steady_clock::time_point initialTime, simulationTime;
  
  int inputSliderID, feedbackSliderID;
  
  std::vector<double> measuresList;
  
  static void Update( OSimProcess* process )
  {
    const long TIME_STEP_MS = 20;
    
    process->state.setTime( 0 );
    
    std::chrono::steady_clock::time_point initialTime = std::chrono::steady_clock::now();
    
    process->isUpdating = true;
    while( process->isUpdating )
    {
      std::chrono::steady_clock::time_point simulationTime = std::chrono::steady_clock::now();
      
      while( process->model.updVisualizer().updInputSilo().isAnyUserInput() ) 
      {
        SimTK::Real forceValue;
        while( process->model.updVisualizer().updInputSilo().takeSliderMove( process->inputSliderID, forceValue ) ) 
          process->inputActuator->setOverrideActuation( process->state, forceValue );
        while( process->model.updVisualizer().updInputSilo().takeSliderMove( process->feedbackSliderID, forceValue ) )
          process->feedbackActuator->setOverrideActuation( process->state, forceValue );
      }
      
      //process->model.updVisualizer().updSimbodyVisualizer().setSliderValue( process->feedbackSliderID, process->feedbackActuator->getOverrideActuation( process->state ) );
      
      OpenSim::Manager manager( process->model );
      manager.initialize( process->state );
      process->state = manager.integrate( std::chrono::duration_cast<std::chrono::duration<double>>( simulationTime - initialTime ).count() );
      
      process->measuresList[ POSITION ] = process->inputActuator->getCoordinate()->getValue( process->state );
      process->measuresList[ VELOCITY ] = process->inputActuator->getCoordinate()->getSpeedValue( process->state );
      process->measuresList[ ACCELERATION ] = process->inputActuator->getCoordinate()->getAccelerationValue( process->state );
      process->measuresList[ FORCE ] = process->inputActuator->getOverrideActuation( process->state );
      
      std::this_thread::sleep_until( simulationTime + std::chrono::milliseconds( TIME_STEP_MS ) );
    }
  }
};

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE );

int InitDevice( const char* modelName )
{  
  for( int deviceIndex = 0; deviceIndex < processList.size(); deviceIndex++ )
    if( processList[ deviceIndex ]->GetName() == modelName ) return deviceIndex;
    
  processList.push_back( new OSimProcess( modelName, processList.size() ) );

  return (int) processList.size() - 1;
}

void EndDevice( int processID )
{
  for( OSimProcess* process : processList )
    delete process;
  
  return;
}

size_t GetMaxInputSamplesNumber( int processID )
{
  return 1;
}

size_t Read( int processID, unsigned int channel, double* ref_value )
{
  *ref_value = 0.0;
  
  if( processID < 0 ) return 0;
  
  OSimProcess* process = processList[ (size_t) processID ];
  
//   process->Update();
  *ref_value = process->GetMeasurement( channel );
  
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
  if( processID < 0 ) return false;
  
  if( channel > 3 ) return false;
  
  return true;
}

bool Write( int processID, unsigned int channel, double value )
{
  if( processID < 0 ) return false;
  
  if( channel != 0 ) return false;
  
  OSimProcess* process = processList[ (size_t) processID ];
  
  //process->SetForce( value );
  //process->Update();
  
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
