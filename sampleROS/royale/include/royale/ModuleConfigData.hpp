/****************************************************************************\
* Copyright (C) 2016 Infineon Technologies
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
\****************************************************************************/

#pragma once

#include <config/ModuleConfig.hpp>

namespace royale {
namespace config {
/**
 * The module configurations are not exported directly from the library.
 * The full list is available via royale::config::getUsbProbeDataRoyale()
 */
namespace moduleconfig {
/**
 * Configuration for the Selene camera modules with ICM
 */
extern const royale::config::ModuleConfig SeleneIcm;

/**
 * Default configuration for the Selene camera modules that only
 * offers a calibration use case
 */
extern const royale::config::ModuleConfig SeleneDefault;

/**
 * Default configuration for an unknown pmd module (M2453 versions)
 */
extern const royale::config::ModuleConfig PmdModule238xDefault;

/**
 * Default configuration for an unknown pmd module (M2455 versions)
 */
extern const royale::config::ModuleConfig PmdModule277xDefault;

/**
 * Default configuration for an unknown pmd module (M2457 versions)
 */
extern const royale::config::ModuleConfig PmdModule2877Default;

/**
 * Default configuration for an unknown pmd module (M2442 versions)
 */
extern const royale::config::ModuleConfig PmdModule2877aDefault;

/**
 * Default configuration for an unknown pmd module (M2459 versions)
 */
extern const royale::config::ModuleConfig PmdModule2875Default;

/**
 * Configuration for the OZT167C module
 */
extern const royale::config::ModuleConfig Ozt167C;

/**
 * Configuration for the Floreo module
 */
extern const royale::config::ModuleConfig Floreo;

/**
 * Configuration for the Flexx2 module
 */
extern const royale::config::ModuleConfig Flexx2;

/**
 * Configuration for the OZT378 module
 */
extern const royale::config::ModuleConfig Ozt378;

/**
 * Configuration for the OZT321B module
 */
extern const royale::config::ModuleConfig Ozt321B;

/**
 * Configuration for the FloreoXP module
 */
extern const royale::config::ModuleConfig FloreoXp;

/**
 * Configuration for the Floki module
 */
extern const royale::config::ModuleConfig Floki;

/**
 * Configuration for the picoAuto module
 */
extern const royale::config::ModuleConfig picoAuto;

/**
 * Configuration for the OZT613 module
 */
extern const royale::config::ModuleConfig Ozt613;

/**
 * Configuration for the Flexx2VGA module
 */
extern const royale::config::ModuleConfig Flexx2VGA;

/**
 * Configuration for the picoAuto A14 module
 */
extern const royale::config::ModuleConfig picoAutoA14;

/**
 * Configuration for the Flexx2Wide module
 */
extern const royale::config::ModuleConfig Flexx2Wide;

/**
 * Configuration for the OZT378D+ module
 */
extern const royale::config::ModuleConfig Ozt378DPlus;
} // namespace moduleconfig
} // namespace config
} // namespace royale
