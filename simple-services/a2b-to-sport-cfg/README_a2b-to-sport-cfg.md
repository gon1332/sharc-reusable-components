# simple-services/a2b-to-sport-cfg

## Overview

The a2b-to-sport-cfg service converts A2B I2SGCFG/I2SCFG registers into a compatible simple SPORT driver configuration.  One can read the registers out of the A2B transceiver and use this module to guarantee a compatible SPORT configuration on the DSP.

## Required components

- simple-drivers/peripherals/sport

## Recommended components

- simple-services/syslog

## Integrate the source

- Copy the 'src' directory into an appropriate place in the host project
- Copy the 'inc' directory into a project include directory.  The header files in the 'inc' directory contain the configurable options for the a2b-to-sport-cfg service.

## Configure

The a2b-to-sport-cfg service has some compile-time configuration options.  See 'inc/a2b_to_sport_cfg.h'.

## Run

- Call `a2b_to_sport_cfg()` with the appropriate parameters.  The function will overwrite relevant portions of the SPORT config structure and leave unaffected sections untouched.
