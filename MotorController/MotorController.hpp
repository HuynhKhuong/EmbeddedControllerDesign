#ifndef __MOTORCONTROLLER_INCLUDED__
#define __MOTORCONTROLLER_INCLUDED__

#include <cstdint>
#include "MotorSpec.hpp"
#include "MotorSystemInterface.hpp"

namespace MotorController{
/// \brief    Motor Controller designed as the top mix-in layer 
///           It would accept any type controller as \class BaseController in lower mix-in layer
/// 
/// \details  Motor Controller works on signal type as uint16_t:
///           input signal: encoder value 
///           output signal: PWM resolution
///           Motor Controller would control 2 target types of motor
///           - Velocity 
///           - Position
/// \note     As Motor Controller works on signal type uint16_t, input & output signal type of the controller (lower mix-in layer) 
///           must be uint16_t type

template <class BaseController> 
class DCMotorController: public BaseController
{
public: 
  using Base = BaseController;

  template<typename... ParamsPack>
  DCMotorController(uint16_t _PWMMaxValue, uint16_t _PWMMinValue, DCMotor::DCMotorSpecification& _motorModel, 
                          MotorSystemInterface::DCMotorInterface& _SystemInterface, ParamsPack... params) : 
                          Base(params...), PWMMaxValue(_PWMMaxValue), PWMMinValue(_PWMMinValue), 
                          motorModel(&(_motorModel)), SystemInterface(&(_SystemInterface)){}

  /// \note     There are 2 variants of Motor Controller: Velocity and Position
  ///           getSystemResponse and getOutput differs between 2 variants. Therefore, getSystemResponse would be left blank and 
  ///           its functional logic would be delegated into 2 variants.
  void reverseMotorDirection()
  {
    SystemInterface->reverseMotorDirection();
  }

  void triggerPWMPin()
  {
    SystemInterface->outputPWMControlSignal(Base::outputSignal);
  }

  /// \brief  Reset all information of the Motor Interface and the Motor Controller 
  void reset()
  {
    Base::reset();
    SystemInterface->reset();
  }

protected: 
  const uint16_t PWMMaxValue; 
  const uint16_t PWMMinValue;

  DCMotor::DCMotorSpecification* const motorModel; ///  and motor target 
  MotorSystemInterface::DCMotorInterface* const SystemInterface; ///MotorInterface of the Embedded System

  /// \brief Private function used for output calculation
  /// \details  Private function converts from physical output value to PWM value
  /// \note     input param must be posivite
  uint16_t convertPhysicalToPWMValue(float physicalValue)
  {
    uint16_t result{0U};
    if(physicalValue >= 0.0f)
    {
      result = static_cast<uint16_t>((physicalValue*PWMMaxValue)/motorModel->Vrms);
    }
    return result;
  }

};

template <class BaseController>
class MotorVelocityController: public DCMotorController<BaseController>
{
public: 
  using Base = DCMotorController<BaseController>;

  template<typename... ParamPack>
  MotorVelocityController(ParamPack... params): Base(params...){} 

  void getSystemResponse() override
  {
    static_cast<void>(Base::SystemInterface->getEncoderPulseCount());

    if(Base::sampleTime > 0)
    {
      Base::systemResponse = static_cast<float>(Base::SystemInterface->getDeltaEncoderPulseCount())/Base::sampleTime;
    }
  }

  /// \brief output control signal to control a DC Motor velocity 
  ///        According to author's design, DC Motor velocity value would only be positive, hence its output control value would only
  ///        be positive. 
  ///        Its negative velocity shows that the DC Motor has changed the motor direction. 
  ///        The motor direction is handled by the System Interface
  float getOutput() override
  {
    float calculatedOutput{0.0f};
    uint16_t calculatedPWMValue{0U};

    getSystemResponse();

    calculatedOutput = Base::getOutput();


    /// Boundaries limit
    if(calculatedOutput  < 0.0f)
    {
      calculatedOutput =  0.0f;
    }
    if(calculatedOutput > Base::motorModel->Vrms) 
    {
      calculatedOutput = Base::motorModel->Vrms;
    }

    /// Convert output signal to PWM value based on converted rate of Motor's Vrms and PWM value
    calculatedPWMValue = Base::convertPhysicalToPWMValue(calculatedOutput);

    /// Boundaries limit 
    calculatedPWMValue  = (calculatedPWMValue > Base::PWMMaxValue)?(Base::PWMMaxValue):(calculatedPWMValue);
    calculatedPWMValue  = (calculatedPWMValue < Base::PWMMinValue)?(Base::PWMMinValue):(calculatedPWMValue);

    Base::outputSignal= calculatedPWMValue;

    return calculatedPWMValue;
  }
};

/// \todo 2 variants of Motor Controller Variants: 
/*
  /// \brief    Function getSystemResponse get the response Encoder value of the motor 
  /// \details  Function getSystemResponse get the response Encoder value and feed to the controller in order to form a closed-loop
  ///           controller
  /// \note     getSystemResponse is hardware specific
  void getSystemResponse() override 
  {
    /// Depend on API provided by each HAL library
    Base::systemResponse = 0U;
    return;
  }

  float getOutput() override
  {
    float calculatedOuptut{0.0f};
    uint16_t calculatedPWMValue{0.0f};
    getSystemResponse();
    calculatedOuptut = Base::getOutput();

    /// Convert negative value -> positive value as triggering PWM value on Hardware only accepts positive value
    /// Computed negative value could be showed via changing Motor direction
    if(calculatedOutput  < 0.0f)
    {
      calculatedOutput = -calculatedOutput; 
      reverseMotorDirection();
    }
    /// Boundaries limit
    if(calculatedOutput > DCMotor.Vrms) 
    {
      calculatedOutput = DCMotor.Vrms;
    }

    /// Convert output signal to PWM value based on converted rate of Motor's Vrms and PWM value
    calculatedPWMValue = convertPhysicalToPWMValue(calculatedOutput);

    /// Boundaries limit 
    calculatedPWMValue  = (calculatedPWMValue > static_cast<float>(PWMMaxValue))?(static_cast<float>(PWMMaxValue)):(calculatedPWMValue);
    calculatedPWMValue  = (calculatedPWMValue < static_cast<float>(PWMMinValue))?(static_cast<float>(PWMMinValue)):(calculatedPWMValue);

    Base::outputSignal = calculatedPWMValue;

    return calculatedPWMValue;

  }
*/
}

#endif 