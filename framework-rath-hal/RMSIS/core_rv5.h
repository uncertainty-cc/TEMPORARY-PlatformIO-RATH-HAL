/**
 * @file core_rv5.h
 * @brief RV5 Core Peripheral Access Layer Header File
 * @version 0.1
 * @date 2021-01-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __CORE_RV5_H
#define __CORE_RV5_H

/* IO definitions (access restrictions to peripheral registers) */
/**
 * @defgroup RV5 Global Defines
 *
 * <strong>IO Type Qualifiers</strong> are used
 * @li to specify the access to peripheral variables.
 * @li for automatic generation of peripheral register debug information.
*/
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */


#define SET_BITS(REG, BIT)                    ((REG) |= (BIT))
#define CLEAR_BITS(REG, BIT)                  ((REG) &= ~(BIT))
#define READ_BITS(REG, BIT)                   ((REG) & (BIT))
#define WRITE_BITS(REG, CLEARMASK, SETMASK)   ((REG) = (((REG) & (~(CLEARMASK))) | (SETMASK)))

typedef enum {
  RESET = 0UL,
  SET   = !RESET,
} FlagStatus;

#endif  // __CORE_RV5_H
