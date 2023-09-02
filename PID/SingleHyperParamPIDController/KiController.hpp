#ifndef __KICONTROLLER_INCLUDED__
#define __KICONTROLLER_INCLUDED__

#include "PIDControllerTemplate.hpp"

namespace P_I_DController
{
/// \brief    Implementation of base template class: 1 HyperParam controller 
/// \details  This implementation version would implement 1 HyperParam controller: Ki
///           In this controller, the output signal would have integral relationship with input signal: 
///                             Output = (errorGap + preErrorGap)*Ki*sampleTime/2
template <typename inputSignalType, typename outputSignalType, typename HyperParamType = float>
class KiController: public PIDController<inputSignalType, outputSignalType, HyperParamType >
{
public: 
  ///signalType is checked in base class
  using Base = PIDController<inputSignalType, outputSignalType, HyperParamType>;
  KiController(HyperParamType _Ki, float _sampleTime): Base(_Ki), sampleTime(_sampleTime){};

  /// \note This function MUST be called after getSystemResponse 
  ///       getSystemResponse would be designed in higher level to match with different embedded system
  void calculateErrorGap()
  {
    Base::errorGap = static_cast<float>(Base::inputSignal) - static_cast<float>(Base::systemResponse);
    return;
  }

  float getOutput() override 
  {
    calculateErrorGap();

    if(isControllerDisabled)
    {
      Base::calculatedControlOutput = 0.0f;
    }
    else 
    {
      Base::calculatedControlOutput = Base::hyperParam * sampleTime * static_cast<float>(errorGap + preErrorGap)/2;
      preErrorGap = errorGap;
    }
    return Base::calculatedControlOutput;
  } 
  void reset() override
  {
    disableController();
    preErrorGap = 0.0f;
    errorGap = 0.0f;
    Base::calculatedControlOutput = 0.0f;
  }

  void enableController(){isControllerDisabled = false;}
  void disableController(){isControllerDisabled = true;}

private: 
  bool isControllerDisabled{false};
  bool isTriggeredFirstTime{true};

  float sampleTime{0.0f}; 
  inputSignalType preErrorGap{0U};
  inputSignalType  errorGap{0U};
};
}

#endif 
