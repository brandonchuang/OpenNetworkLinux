/**************************************************************************//**
 *
 * @file
 * @brief x86_64_accton_as9817_64_nb Porting Macros.
 *
 * @addtogroup x86_64_accton_as9817_64_nb-porting
 * @{
 *
 *****************************************************************************/
#ifndef __x86_64_accton_as9817_64_nb_PORTING_H__
#define __x86_64_accton_as9817_64_nb_PORTING_H__


/* <auto.start.portingmacro(ALL).define> */
#if X86_64_ACCTON_AS9817_64_NB_CONFIG_PORTING_INCLUDE_STDLIB_HEADERS == 1
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <memory.h>
#endif

#ifndef X86_64_ACCTON_AS9817_64_NB_MALLOC
    #if defined(GLOBAL_MALLOC)
        #define X86_64_ACCTON_AS9817_64_NB_MALLOC GLOBAL_MALLOC
    #elif X86_64_ACCTON_AS9817_64_NB_CONFIG_PORTING_STDLIB == 1
        #define X86_64_ACCTON_AS9817_64_NB_MALLOC malloc
    #else
        #error The macro X86_64_ACCTON_AS9817_64_NB_MALLOC is required but cannot be defined.
    #endif
#endif

#ifndef X86_64_ACCTON_AS9817_64_NB_FREE
    #if defined(GLOBAL_FREE)
        #define X86_64_ACCTON_AS9817_64_NB_FREE GLOBAL_FREE
    #elif X86_64_ACCTON_AS9817_64_NB_CONFIG_PORTING_STDLIB == 1
        #define X86_64_ACCTON_AS9817_64_NB_FREE free
    #else
        #error The macro X86_64_ACCTON_AS9817_64_NB_FREE is required but cannot be defined.
    #endif
#endif

#ifndef X86_64_ACCTON_AS9817_64_NB_MEMSET
    #if defined(GLOBAL_MEMSET)
        #define X86_64_ACCTON_AS9817_64_NB_MEMSET GLOBAL_MEMSET
    #elif X86_64_ACCTON_AS9817_64_NB_CONFIG_PORTING_STDLIB == 1
        #define X86_64_ACCTON_AS9817_64_NB_MEMSET memset
    #else
        #error The macro X86_64_ACCTON_AS9817_64_NB_MEMSET is required but cannot be defined.
    #endif
#endif

#ifndef X86_64_ACCTON_AS9817_64_NB_MEMCPY
    #if defined(GLOBAL_MEMCPY)
        #define X86_64_ACCTON_AS9817_64_NB_MEMCPY GLOBAL_MEMCPY
    #elif X86_64_ACCTON_AS9817_64_NB_CONFIG_PORTING_STDLIB == 1
        #define X86_64_ACCTON_AS9817_64_NB_MEMCPY memcpy
    #else
        #error The macro X86_64_ACCTON_AS9817_64_NB_MEMCPY is required but cannot be defined.
    #endif
#endif

#ifndef X86_64_ACCTON_AS9817_64_NB_STRNCPY
    #if defined(GLOBAL_STRNCPY)
        #define X86_64_ACCTON_AS9817_64_NB_STRNCPY GLOBAL_STRNCPY
    #elif X86_64_ACCTON_AS9817_64_NB_CONFIG_PORTING_STDLIB == 1
        #define X86_64_ACCTON_AS9817_64_NB_STRNCPY strncpy
    #else
        #error The macro X86_64_ACCTON_AS9817_64_NB_STRNCPY is required but cannot be defined.
    #endif
#endif

#ifndef X86_64_ACCTON_AS9817_64_NB_VSNPRINTF
    #if defined(GLOBAL_VSNPRINTF)
        #define X86_64_ACCTON_AS9817_64_NB_VSNPRINTF GLOBAL_VSNPRINTF
    #elif X86_64_ACCTON_AS9817_64_NB_CONFIG_PORTING_STDLIB == 1
        #define X86_64_ACCTON_AS9817_64_NB_VSNPRINTF vsnprintf
    #else
        #error The macro X86_64_ACCTON_AS9817_64_NB_VSNPRINTF is required but cannot be defined.
    #endif
#endif

#ifndef X86_64_ACCTON_AS9817_64_NB_SNPRINTF
    #if defined(GLOBAL_SNPRINTF)
        #define X86_64_ACCTON_AS9817_64_NB_SNPRINTF GLOBAL_SNPRINTF
    #elif X86_64_ACCTON_AS9817_64_NB_CONFIG_PORTING_STDLIB == 1
        #define X86_64_ACCTON_AS9817_64_NB_SNPRINTF snprintf
    #else
        #error The macro X86_64_ACCTON_AS9817_64_NB_SNPRINTF is required but cannot be defined.
    #endif
#endif

#ifndef X86_64_ACCTON_AS9817_64_NB_STRLEN
    #if defined(GLOBAL_STRLEN)
        #define X86_64_ACCTON_AS9817_64_NB_STRLEN GLOBAL_STRLEN
    #elif X86_64_ACCTON_AS9817_64_NB_CONFIG_PORTING_STDLIB == 1
        #define X86_64_ACCTON_AS9817_64_NB_STRLEN strlen
    #else
        #error The macro X86_64_ACCTON_AS9817_64_NB_STRLEN is required but cannot be defined.
    #endif
#endif

/* <auto.end.portingmacro(ALL).define> */


#endif /* __x86_64_accton_as9817_64_nb_PORTING_H__ */
/* @} */
