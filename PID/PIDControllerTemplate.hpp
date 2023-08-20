#ifndef __PID_CONTROLLER_INCLUDED__
#define __PID_CONTROLLER_INCLUDED__

#include "ControllerInterface.hpp"

namespace P_I_DController{

/// \brief    This is template class used to define PID controller class
/// \details  As there are various variants of PID controller: P controller, I controller, D controller, PD, PI, etc...
///           Template is defined to be adhered by all variants of PID controller
/// \tparam   signalType required for signal integration into controller class
/// \tparam   ...params: Define the number of controlling hyperparams
template<typename inputSignalType, typename outputSignalType, typename... params>
class PIDController: public Controller::ControllerInterface<inputSignalType, outputSignalType>
{
public: 
  ///SignalType is checked in Interface
  using Base = Controller::ControllerInterface<inputSignalType, outputSignalType>
  PIDController():Base(){};

protected: 
  float errorGap{0.0f};
  float calculatedControlOutput{0.0f};
/// As this is template class, it would do nothing
/// To prevent user to use this class, pure virtual function in interface would not be defined
};

/// \brief    Specialization of base template class: 3 HyperParam controller
/// \details  There's only 1 controller for this specialization version: PID controller
/// \details  PID control algorithm: 
///           (pE*Error_value) + (ppE*pre_Error_value) + (pppE*pre_pre_Error_value)
template<typename inputSignalType, typename outputSignalType, typename HyperParameterType>
class PIDController<SignalType, HyperParameterType, HyperParameterType, HyperParameterType>: public Controller::ControllerInterface<inputSignalType, outputSignalType>
{
public: 
  using Base = Controller::ControllerInterface<inputSignalType, outputSignalType>;
  PIDController(HyperParameterType _Kp, HyperParameterType _Ki, HyperParameterType _Kd, float _sampleTime): 
                Base(), Kp(_Kp), Ki(_Ki), Kd(_Kd), sampleTime(_sampleTime){}

  static_assert(std::is_arithmetic<HyperParameterType>::value == true);

  void calculateErrorGap() 
  {
    errorGap = Base::inputSignal - Base::systemResponse;
  }

  float getOutput() override
  {
    calculateErrorGap();
    float PGapCompensate{(errorGap - preErrorGap) * Kp}; 
    float IGapCompensate{(errorGap + preErrorGap) * Ki * sampleTime /2};
    float DGapCompensate{Kd* (errorGap - (2*preErrorGap) + prepreErrorGap)} ;
    calculatedControlOutput +=  PGapCompensate + IGapCompensate + DGapCompensate ;

    prepreErrorGap = preErrorGap;
    preErrorGap = errorGap;

    //Do nothing
    return calculatedControlOutput;
  }

  void setKp(HyperParameterType _Kp){Kp = _Kp}
  void setKi(HyperParameterType _Ki){Ki = _Ki}
  void setKd(HyperParameterType _Kd){Kd = _Kd}

  void enableController(){isControllerDisabled = false;}
  void disableController(){isControllerDisabled = true;}

private: 
  bool isControllerDisabled{false};
  float errorGap{0.0f};
  float preErrorGap{0.0f};
  float prepreErrorGap{0.0f};
  float calculatedControlOutput{0.0f};
  float sampleTime{0.0f};
  HyperParameterType Kp;
  HyperParameterType Kd;
  HyperParameterType Ki;
};

/// \brief    Specialization of base template class: 1 HyperParam controller 
/// \details  This Specialization version would implement 1 HyperParam controller Kp/Ki/Kd
template <typename inputSignalType, typename outputSignalType, typename hyperParamType>
class PIDController<inputSignalType, outputSignalType, hyperParamType>: public Controller::ControllerInterface<inputSignalType, outputSignalType>
{

  static_assert(std::is_floating_point<hyperParamType>::value == true);

public: 
  ///SignalType is checked in Interface
  using Base = Controller::ControllerInterface<inputSignalType, outputSignalType>;
  PIDController(hyperParamType _hyperParam):Base(), hyperParam (_hyperParam){}


  void setcontrolParam(hyperParamType _hyperParam)
  {
    hyperParam = _hyperParam;
  }
  
  /// As this is still template for 3 specific controllers: Kp, Ki, Kd 
  /// The algorithm for 3 specific controllers are distinctive
  /// Implementation of output calculation are left blank and tobe implemented in concrete classes

protected: 
  hyperParamType hyperParam;
  float errorGap{0.0f};
  float calculatedControlOutput{0.0f};
};

}
#endif
