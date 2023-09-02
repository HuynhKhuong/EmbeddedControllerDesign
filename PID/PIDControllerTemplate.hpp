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
  using Base = Controller::ControllerInterface<inputSignalType, outputSignalType>;
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
class PIDController<inputSignalType, outputSignalType, HyperParameterType, HyperParameterType, HyperParameterType>: public Controller::ControllerInterface<inputSignalType, outputSignalType>
{
public: 
  using Base = Controller::ControllerInterface<inputSignalType, outputSignalType>;
  PIDController(HyperParameterType _Kp, HyperParameterType _Ki, HyperParameterType _Kd, float _sampleTime
                float _controlOutputMaxValue, float _controlOutputMinValue): 
                Base(), Kp(_Kp), Ki(_Ki), Kd(_Kd), sampleTime(_sampleTime), controlOutputMaxValue(_controlOutputMaxValue),
                                                                            controlOutputMinValue(_controlOutputMinValue){}

  static_assert(std::is_arithmetic<HyperParameterType>::value == true);

  void calculateErrorGap() 
  {
    errorGap = Base::inputSignal - Base::systemResponse;
  }

  float getOutput() override
  {
    if(isControllerDisabled)
    {
      //calculatedControllOuptut is not computed
      return 0;
    }
    else
    {
      calculateErrorGap();
      float PGapCompensate{static_cast<float>((errorGap - preErrorGap) * Kp)}; 
      float IGapCompensate{static_cast<float>((errorGap + preErrorGap) * Ki * sampleTime /2)};
      float DGapCompensate{static_cast<float>(Kd* (errorGap - (2*preErrorGap) + prepreErrorGap))} ;
      calculatedControlOutput +=  PGapCompensate + IGapCompensate + DGapCompensate ;

      //limit check
      if(calculatedControlOutput > controlOutputMaxValue) calculatedControlOutput = controlOutputMaxValue; 
      if(calculatedControlOutput < controlOutputMinValue) calculatedControlOutput = controlOutputMinValue; 

      Base::outputSignal = calculatedControlOutput;
      prepreErrorGap = preErrorGap;
      preErrorGap = errorGap;

      return calculatedControlOutput;
    }
  }

  void reset()
  {
    errorGap = 0.0f;
    preErrorGap = 0.0f;
    prepreErrorGap = 0.0f;
    calculatedControlOutput = 0.0f;
    disableController();
  }

  void setKp(HyperParameterType _Kp){Kp = _Kp;}
  void setKi(HyperParameterType _Ki){Ki = _Ki;}
  void setKd(HyperParameterType _Kd){Kd = _Kd;}

  void enableController(){isControllerDisabled = false;}
  void disableController(){isControllerDisabled = true;}

private: 
  const float controlOutputMaxValue;
  const float controlOutputMinValue;

  bool isControllerDisabled{false};

  inputSignalType errorGap{0.0f};
  inputSignalType preErrorGap{0.0f};
  inputSignalType prepreErrorGap{0.0f};

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
  
  virtual void reset() = 0;

  /// As this is still template for 3 specific controllers: Kp, Ki, Kd 
  /// The algorithm for 3 specific controllers are distinctive
  /// Implementation of output calculation are left blank and tobe implemented in concrete classes

protected: 
  hyperParamType hyperParam;
  float calculatedControlOutput{0.0f};
  const float controlOutputMaxValue;
  const float controlOutputMinValue;
};

}
#endif
