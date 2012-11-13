/**
 * Copyright 2012 Sony Corporation
 */
/*
 * Version Definition of ath6k Driver
 * 
 * Version Format:
 *   {major version}.{minor version}.{QCA release number}.{Sony version}
 *
 * Macro Definition
 *   __VER_MAJOR_ : {major version}
 *   __VER_MINOR_ : {minor version}
 *   __VER_QCA_RELEASE_ : {QCA release number}
 *   __VER_SONY_ : {Sony version}
 *
 * Description:
 *   {major version}.{minor version} is the basic version of ath6k driver
 *   which is given by Qualcomm Atheros. (e.g. "3.2" for ICS Update4)
 * 
 *   {QCA release number} is the number of the driver release counted from
 *   the first release of the same basic version.
 *   As for the basic version "3.2", "1" for Alpha1, "2" for Alpha2, etc.
 *
 *   {Sony version} is defined to distinguish modifications by Sony.
 *   The format is "MMDDI", where "MMDD" is the date when the modification
 *   is made and "I" is the index of the modification in a day.
 *   "I" is "0" for the 1st modification in a day and "1" for the 2nd.
 *
 *   "MM" will be larger than "12" if the month number of the previous
 *   modification is larger than that of the new modification.
 *   (e.g. previous: Dec.31,2011, new: Feb.1,2012, then "MMDD" is "1401")
 *   However, this measure is NOT necessary if the upper version numbers
 *   (that is, {major version}.{minor version}.{QCA release number}) have
 *   been changed.     
 */

#ifdef SONY
#define __VER_MAJOR_ "3"
#define __VER_MINOR_ "2"
#define __VER_QCA_RELEASE_ "16" /* 16 is for Android Olca32 CS Patch 9 Release 3 */
#define __VER_SONY_ "09270"
#endif
