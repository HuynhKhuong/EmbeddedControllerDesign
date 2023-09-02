#ifndef __KPCONTROLLER_INCLUDED__
#define __KPCONTROLLER_INCLUDED__

#include "PIDControllerTemplate.hpp"

namespace P_I_DController{
/// \brief    Implementation of base template class: 1 HyperParam controller 
/// \details  This implementation version would implement 1 HyperParam controller: Kp
///           In this controller, the output signal would have linear relationship with input signal: 
///                             Output = Input * Kp
/// \todo     move getSystemResponse implementation down to lower level as it is hardware specific
template <typename inputSignalType, typename outputSignalType, typename HyperParamType = float>
class KpController: public PIDController<inputSignalType, outputSignalType, HyperParamType>
{
public: 
  ///signalType is checked in base class

  using Base = PIDController<inputSignalType, outputSignalType, HyperParamType>;
  
  KpController(HyperParamType _Kp): Base(_Kp){};

  float getOutput() override 
  {
    if(isControllerDisabled)
    {
      Base::calculatedControlOutput = 0.0f;
    }
    else 
    {
      errorGap = static_cast<float>(Base::inputSignal) - static_cast<float>(Base::systemResponse);
      Base::calculatedControlOutput = Base::hyperParam * errorGap;
    }

    return Base::calculatedControlOutput;
  } 
 
  void reset() override
  {
    disableController();
  }

  void enableController(){isControllerDisabled = false;}
  void disableController(){isControllerDisabled = true;}

private: 
  inputSignalType errorGap{0.0f};
  bool isControllerDisabled{false};
};
} //End of namespace P_I_DController

#endif
