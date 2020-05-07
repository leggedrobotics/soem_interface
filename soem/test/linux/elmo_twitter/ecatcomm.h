#ifndef _ECATCOMM_H
#define _ECATCOMM_H

// Standard headers
#include <stdint.h>

// Module headers
#include "ethercat.h"

typedef enum ECAT_DSP402_COMMAND_TYPE
{
    SWITCH_ON           = 0x00,
    SHUTDOWN            ,
    DISABLE_VOLTAGE     ,
    ENABLE_VOLTAGE      ,
    QUICK_STOP          ,
    DISABLE_OPERATION   ,
    ENABLE_OPERATION    ,
    FAULT_RESET         ,
    HALT                ,
    HALT_RESET          ,
    CLEAR_CONTROLWORD   ,

} dsp402_command_e;

// Note: Bit field ordering depends on the Endianness which is implementation dependent.
typedef struct DSP402_STATUSWORD_BITS
{
    uint16_t ready_to_switch_on:1;
    uint16_t switched_on:1;
    uint16_t operation_enabled:1;
    uint16_t fault:1;
    uint16_t voltage_enabled:1;
    uint16_t quick_stop:1;
    uint16_t switch_on_disabled:1;
    uint16_t warning:1;
    uint16_t manufacturer_specific_0:1;
    uint16_t remote:1;
    uint16_t operation_mode_specific_0:1;
    uint16_t internal_limit_active:1;
    uint16_t operation_mode_specific_1:1;
    uint16_t operation_mode_specific_2:1;
    uint16_t manufacturer_specific_1:2;

} dsp402_statusword_bits_t;

typedef union DSP402_STATUSWORD_TYPE
{
    uint16_t all;
    dsp402_statusword_bits_t bits;

} dsp402_statusword_t;

typedef struct DSP402_CONTROLWORD_BITS
{
    uint16_t switch_on:1;
    uint16_t enable_voltage:1;
    uint16_t quick_stop:1;
    uint16_t enable_operation:1;
    uint16_t operation_mode_specific_0:3;
    uint16_t fault_reset:1;
    uint16_t halt:1;
    uint16_t operation_mode_specific_1:1;
    uint16_t rsrvd:1;
    uint16_t manufacturer_specific:5;

} dsp402_controlword_bits_t;

typedef union DSP402_CONTROLWORD_TYPE
{
    uint16_t all;
    dsp402_controlword_bits_t bits;

} dsp402_controlword_t;

typedef struct ELMO_INDATA_TYPE
{
    dsp402_statusword_t statusword;

    int position;
    int velocity;

    int digitalin;

    int busvoltage;
    int motorcurrent;

} elmo_twitter_indata_t;

typedef struct ELMO_OUTDATA_TYPE
{
    dsp402_controlword_t controlword;

    int torque;

} elmo_twitter_outdata_t;


// Core operations
void ecatcomm_init(char *ifname);
void ecatcomm_run(void);
void ecatcomm_exit(void);

// Slave operations
void ecatcomm_slave_info(void);
void ecatcomm_slave_config(void);
void ecatcomm_slave_state_check(void);
void ecatcomm_slave_check_errors(void);
void ecatcomm_slave_check_sdo(int index, int subindex, boolean CA);

void ecatcomm_slave_set_rxpdo(elmo_twitter_outdata_t *pdata, int command, double torque);
void ecatcomm_slave_get_txpdo(elmo_twitter_indata_t *pdata);

void ecatcomm_slave_print_statusword(dsp402_statusword_t statusword);
void ecatcomm_slave_print_controlword(dsp402_controlword_t controlword);

void ecatcomm_slave_dump_status_info(void);

#endif /* _ECATCOMM_H */
