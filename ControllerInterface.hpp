#ifndef __CONTROLLER_INTERFACE_INCLUDED__
#define __CONTROLLER_INTERFACE_INCLUDED__

#include <type_traits>

namespace Controller{

/// \brief  Controller Interface defines behaviour of all closed-loop controllers implemented in embedded projects
///         Ex: STR, PID,....
/// \note   Currently, controlled signals in author understanding are measured in 1 unit only
///         e.g: Voltage (V), Current (I), height(meter), etc...
/// \fn     getOutput: Every controller calculation must adhere to this function. This function is the only interface which 
///         user could use to get control output
/// \fn     setInput: Every controller input fetching must adhere to this function. This function is the only interface which 
///         user could use to input their signal which is needed to be controlled

template<typename inputsignalType = float, typename outputSignalType = float>
class ControllerInterface
{
  ///Typecheck 
  static_assert(std::is_arithmetic<inputsignalType>::value == true);
  static_assert(std::is_arithmetic<outputSignalType>::value == true);

public: 
  ControllerInterface():outputSignal(0.0f), inputSignal(0.0f){} 
  //virtual ~ControllerInterface();

  virtual float getOutput() = 0;
  virtual void getSystemResponse() = 0; 
  virtual void setInput(inputsignalType _inputSignal)
  {
    inputSignal = _inputSignal;
  }

protected: 
  outputSignalType outputSignal;
  inputsignalType  inputSignal;
  inputsignalType systemResponse;
};
}

#endif
