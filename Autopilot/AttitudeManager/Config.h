/* 
 * @file Config.h
 * @author Serj Babayan
 * @created February 22, 2017, 8:59 PM
 * Central source of configuration for the entire autopilot. Configure your settings here.
 * Only settings that are changed often should be stored in this file. Otherwise keep
 * the configuration on a per-module basis
 */

#ifndef CONFIG_H
#define	CONFIG_H

/**
 * Whether to enable the logger. Disabling it will cause all the functions defined
 * in the module to do nothing
 */
#define ENABLE_LOGGER 1

#endif

