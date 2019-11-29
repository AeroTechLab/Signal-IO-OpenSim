/////////////////////////////////////////////////////////////////////////////////////
//                                                                                 //
//  Copyright (c) 2016-2019 Leonardo Consoni <leonardojc@protonmail.com>           //
//                                                                                 //
//  This file is part of Robot-Control-OpenSim.                                    //
//                                                                                 //
//  Robot-Control-OpenSim is free software: you can redistribute it and/or modify  //
//  it under the terms of the GNU Lesser General Public License as published       //
//  by the Free Software Foundation, either version 3 of the License, or           //
//  (at your option) any later version.                                            //
//                                                                                 //
//  Robot-Control-OpenSim is distributed in the hope that it will be useful,       //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of                 //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                   //
//  GNU Lesser General Public License for more details.                            //
//                                                                                 //
//  You should have received a copy of the GNU Lesser General Public License       //
//  along with Robot-Control-OpenSim. If not, see <http://www.gnu.org/licenses/>.  //
//                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////


#include <OpenSim/OpenSim.h>
#include <simbody/internal/Visualizer_InputListener.h>

#include <iostream>
#include <cstdlib>
#include <chrono>

class SliderEventHandler : public SimTK::PeriodicEventHandler
{
public:
  SliderEventHandler( SimTK::Visualizer::InputSilo& inputSilo, int& sliderID, OpenSim::CoordinateActuator* actuator, SimTK::Real updateInterval )
  : PeriodicEventHandler( updateInterval ), sliderID( sliderID ), inputSilo( inputSilo )
  {
    this->actuator = actuator;
  }
  
  void handleEvent( SimTK::State& state, SimTK::Real accuracy, bool & shouldTerminate ) const
  {
    while( inputSilo.isAnyUserInput() ) 
    {
      std::cout << "received input" << std::endl;
      SimTK::Real userInputValue;
      while( inputSilo.takeSliderMove( sliderID, userInputValue ) ) 
        actuator->setOverrideActuation( state, userInputValue );
    }
  }
  
private:
  int& sliderID;
  SimTK::Visualizer::InputSilo& inputSilo;
  OpenSim::CoordinateActuator* actuator;
};

int main()
{
  OpenSim::Model model;
  
  model.setUseVisualizer( true );
    
  model.setName( "teste" );
  model.setGravity( SimTK::Vec3( 0, 0, 0 ) );

  OpenSim::Body* body = new OpenSim::Body( "body", 1.0, SimTK::Vec3( 0, 0, 0 ), SimTK::Inertia( 1, 1, 1, 0, 0, 0 ) );
  model.addBody( body );

  const OpenSim::Ground& ground = model.getGround();
  OpenSim::PinJoint* groundJoint = new OpenSim::PinJoint( "body2ground", ground, *body );
  //OpenSim::SliderJoint* groundJoint = new OpenSim::SliderJoint( "body2ground", ground, *body );
  model.addJoint( groundJoint );
  
  OpenSim::Brick* blockMesh = new OpenSim::Brick( SimTK::Vec3( 0.5, 0.5, 0.5 ) );
  blockMesh->setColor( SimTK::Blue );
  body->attachGeometry( blockMesh );
  
  OpenSim::Coordinate& coordinate = groundJoint->updCoordinate();
  double coordinateRange[ 2 ] = { -SimTK::Pi, SimTK::Pi };
  coordinate.setRange( coordinateRange );
  OpenSim::CoordinateActuator* inputActuator = new OpenSim::CoordinateActuator( coordinate.getName() );
  //inputActuator->setCoordinate( &coordinate );
  model.addForce( inputActuator );
//     feedbackActuator = new OpenSim::CoordinateActuator( "feedback" );
//     feedbackActuator->setCoordinate( &coordinate );
//     model->addForce( feedbackActuator );
  
  //model.setUseVisualizer( true );
  //model.buildSystem();
  
  //SimTK::Visualizer::InputSilo* inputSilo = new SimTK::Visualizer::InputSilo();
  //model.updVisualizer().updSimbodyVisualizer().addInputListener( inputSilo );
  //SimTK::Visualizer::InputSilo& inputSilo = model.updVisualizer().updInputSilo();
  
  //model.updMultibodySystem().addEventHandler( new SliderEventHandler( inputSilo, sliderID, inputActuator, 0.1 ) );
  
  //model.setUseVisualizer( false );
  SimTK::State& state = model.initSystem();

  int sliderID = 1;
  
  model.updVisualizer().updSimbodyVisualizer().addSlider( "Input Force", sliderID, -1.0, 1.0, 0.0 );
  
  inputActuator->overrideActuation( state, true );
//     feedbackActuator->overrideActuation( state, true );
  
  coordinate.setValue( state, 0.0 );
    
  state.setTime( 0.0 );
    
  std::chrono::steady_clock::time_point initialTime = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point simulationTime = std::chrono::steady_clock::now();
  
  while( true )
  {
    simulationTime = std::chrono::steady_clock::now();
    
    while( model.updVisualizer().updInputSilo().isAnyUserInput() ) 
    {
      std::cout << "received input" << std::endl;
      SimTK::Real userInputValue;
      while( model.updVisualizer().updInputSilo().takeSliderMove( sliderID, userInputValue ) ) 
        inputActuator->setOverrideActuation( state, userInputValue );
    }
    
    OpenSim::Manager manager( model );
    manager.initialize( state );
    state = manager.integrate( std::chrono::duration_cast<std::chrono::duration<double>>( simulationTime - initialTime ).count() );
      
    std::this_thread::sleep_until( simulationTime + std::chrono::milliseconds( 10 ) );
     
    //std::cout << "simulation time: " << state.getTime() << ", position: " << inputActuator->getCoordinate()->getValue( state ) << std::endl;
  }
  
  return 0;
}
