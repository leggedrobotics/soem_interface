// Standard headers
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

// Module headers
#include "ethercat.h"

// EtherCAT Master Data
/** Main slave data array.
 *  Each slave found on the network gets its own record.
 *  ec_slave[0] is reserved for the master. Structure gets filled
 *  in by the configuration function ec_config().
 */
ec_slavet               ecat_slave[EC_MAXSLAVE];
/** number of slaves found on the network */
int                     ecat_slavecount;
/** slave group structure */
ec_groupt               ecat_group[EC_MAXGROUP];

/** cache for EEPROM read functions */
static uint8            ecat_esibuf[EC_MAXEEPBUF];
/** bitmap for filled cache buffer bytes */
static uint32           ecat_esimap[EC_MAXEEPBITMAP];
/** current slave for EEPROM cache buffer */
static ec_eringt        ecat_elist;
static ec_idxstackT     ecat_idxstack;

/** SyncManager Communication Type struct to store data of one slave */
static ec_SMcommtypet   ecat_SMcommtype[EC_MAX_MAPT];
/** PDO assign struct to store data of one slave */
static ec_PDOassignt    ecat_PDOassign[EC_MAX_MAPT];
/** PDO description struct to store data of one slave */
static ec_PDOdesct      ecat_PDOdesc[EC_MAX_MAPT];

/** buffer for EEPROM SM data */
static ec_eepromSMt     ecat_SM;
/** buffer for EEPROM FMMU data */
static ec_eepromFMMUt   ecat_FMMU;
/** Global variable TRUE if error available in error stack */
boolean                 ecat_error = FALSE;

int64                   ecat_DCtime;

ecx_portt               ecat_port;
ecx_redportt            ecat_redport;

ecx_contextt  ecat_context = {
    &ecat_port,          // .port          =
    &ecat_slave[0],       // .slavelist     =
    &ecat_slavecount,     // .slavecount    =
    EC_MAXSLAVE,        // .maxslave      =
    &ecat_group[0],       // .grouplist     =
    EC_MAXGROUP,        // .maxgroup      =
    &ecat_esibuf[0],      // .esibuf        =
    &ecat_esimap[0],      // .esimap        =
    0,                  // .esislave      =
    &ecat_elist,          // .elist         =
    &ecat_idxstack,       // .idxstack      =
    &ecat_error,         // .ecaterror     =
    0,                  // .DCtO          =
    0,                  // .DCl           =
    &ecat_DCtime,         // .DCtime        =
    &ecat_SMcommtype[0],  // .SMcommtype    =
    &ecat_PDOassign[0],   // .PDOassign     =
    &ecat_PDOdesc[0],     // .PDOdesc       =
    &ecat_SM,             // .eepSM         =
    &ecat_FMMU,           // .eepFMMU       =
    NULL                // .FOEhook()
};
