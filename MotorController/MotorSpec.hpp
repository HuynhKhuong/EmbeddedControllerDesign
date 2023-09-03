#ifndef __MOTORSPEC_INCLUDED__
#define __MOTORSPEC_INCLUDED__

/// \todo  Handle motor direction reverses when do velocity control
/// \todo  Design the EmbeddedSystemInterface class 

/// \brief This file contains all specifications about a DC motor. 
///        specifications in a DC motor are included in a struct which would be used 
///        for processing data (encoder)  (specs provided by the manufacturer)
#include <cstdint>

namespace DCMotor
{
struct DCMotorSpecification
{
  DCMotorSpecification(const uint16_t _dualChannelPulsesPerRound, const uint8_t _singleChannelPulsesPerRound,
                        const uint8_t _Vrms, const uint16_t _maxRPM):
                        dualChannelPulsesPerRound(_dualChannelPulsesPerRound),singleChannelPulsesPerRound(_singleChannelPulsesPerRound),
                        Vrms(_Vrms), maxRPM(_maxRPM){}

  const uint16_t dualChannelPulsesPerRound;
  const uint8_t  singleChannelPulsesPerRound;
  const uint8_t  Vrms;
  const uint16_t  maxRPM;
};

}

#endif
