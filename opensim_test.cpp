#include <OpenSim/OpenSim.h>
#include <simbody/internal/Visualizer_InputListener.h>

#include <iostream>
#include <cstdlib>
#include <chrono>

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
    
  model.finalizeFromProperties();
    
  SimTK::State& state = model.initSystem();

  //model->updVisualizer().updSimbodyVisualizer().addInputListener( new SliderListener( inputActuator, state ) );
  //model->updVisualizer().updSimbodyVisualizer().addSlider( "Input Force", 1, -1.0, 1.0, 0.0 );
    
  inputActuator->overrideActuation( state, true );
//     feedbackActuator->overrideActuation( state, true );
  
  coordinate.setValue( state, 0.0 );
    
  state.setTime( 0.0 );
    
  std::chrono::steady_clock::time_point initialTime = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point simulationTime = std::chrono::steady_clock::now();
  
  while( true )
  {
    simulationTime = std::chrono::steady_clock::now();
    
    inputActuator->setOverrideActuation( state, fmod( std::chrono::duration_cast<std::chrono::duration<double>>( simulationTime - initialTime ).count(), 20.0 ) - 10.0 );
    
    OpenSim::Manager manager( model );
    manager.initialize( state );
    state = manager.integrate( std::chrono::duration_cast<std::chrono::duration<double>>( simulationTime - initialTime ).count() );
      
    std::this_thread::sleep_until( simulationTime + std::chrono::milliseconds( 10 ) );
     
    std::cout << "simulation time: " << state.getTime() << ", position: " << inputActuator->getCoordinate()->getValue( state ) << std::endl;
  }
  
  return 0;
}
