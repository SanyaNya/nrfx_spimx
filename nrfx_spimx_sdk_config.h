// <e> NRFX_SPIMX_ENABLED - nrfx_spimx - SPIM peripheral driver
//==========================================================
#ifndef NRFX_SPIMX_ENABLED
#define NRFX_SPIMX_ENABLED 1
#endif
// <q> NRFX_SPIMX0_ENABLED  - Enable SPIMX0 instance
 

#ifndef NRFX_SPIMX0_ENABLED
#define NRFX_SPIMX0_ENABLED 0
#endif

// <q> NRFX_SPIMX1_ENABLED  - Enable SPIMX1 instance
 

#ifndef NRFX_SPIMX1_ENABLED
#define NRFX_SPIMX1_ENABLED 0
#endif

// <q> NRFX_SPIMX2_ENABLED  - Enable SPIMX2 instance
 

#ifndef NRFX_SPIMX2_ENABLED
#define NRFX_SPIMX2_ENABLED 0
#endif

// <q> NRFX_SPIMX3_ENABLED  - Enable SPIMX3 instance
 

#ifndef NRFX_SPIMX3_ENABLED
#define NRFX_SPIMX3_ENABLED 0
#endif

// <q> NRFX_SPIMX_EXTENDED_ENABLED  - Enable extended SPIMX features
 

#ifndef NRFX_SPIMX_EXTENDED_ENABLED
#define NRFX_SPIMX_EXTENDED_ENABLED 0
#endif

// <q> NRFX_SPIMX_UNSAFE_3WIRE_RECONFIGURE_ONLINE  - do not disable spim during 3-wire reconfiguration to input/output


#ifndef NRFX_SPIMX_UNSAFE_3WIRE_RECONFIGURE_ONLINE
#define NRFX_SPIMX_UNSAFE_3WIRE_RECONFIGURE_ONLINE 0
#endif

// <o> NRFX_SPIMX_MISO_PULL_CFG  - MISO pin pull configuration.
 
// <0=> NRF_GPIO_PIN_NOPULL 
// <1=> NRF_GPIO_PIN_PULLDOWN 
// <3=> NRF_GPIO_PIN_PULLUP 

#ifndef NRFX_SPIMX_MISO_PULL_CFG
#define NRFX_SPIMX_MISO_PULL_CFG NRF_GPIO_PIN_PULLDOWN
#endif

// <o> NRFX_SPIMX_PIN_DRIVE_CFG - SCK,MOSI,MISO,SS pin drive configuration.

// <0=> NRF_GPIO_PIN_S0S1  - Standard '0', standard '1'.
// <1=> NRF_GPIO_PIN_H0S1  - High-drive '0', standard '1'.
// <2=> NRF_GPIO_PIN_S0H1  - Standard '0', high-drive '1'.
// <3=> NRF_GPIO_PIN_H0H1  - High drive '0', high-drive '1'.
// <4=> NRF_GPIO_PIN_D0S1  - Disconnect '0' standard '1'.
// <5=> NRF_GPIO_PIN_D0H1  - Disconnect '0', high-drive '1'.
// <6=> NRF_GPIO_PIN_S0D1  - Standard '0', disconnect '1'.
// <7=> NRF_GPIO_PIN_H0D1  - High-drive '0', disconnect '1'.

#ifndef NRFX_SPIMX_PIN_DRIVE_CFG
#define NRFX_SPIMX_PIN_DRIVE_CFG NRF_GPIO_PIN_H0H1
#endif

// <o> NRFX_SPIMX_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_SPIMX_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_SPIMX_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_SPIMX_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_SPIMX_CONFIG_LOG_ENABLED
#define NRFX_SPIMX_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_SPIMX_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_SPIMX_CONFIG_LOG_LEVEL
#define NRFX_SPIMX_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_SPIMX_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_SPIMX_CONFIG_INFO_COLOR
#define NRFX_SPIMX_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_SPIMX_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_SPIMX_CONFIG_DEBUG_COLOR
#define NRFX_SPIMX_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>
