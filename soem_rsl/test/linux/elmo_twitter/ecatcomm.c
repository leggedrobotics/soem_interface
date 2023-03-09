// Standard headers
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

// Module headers
#include "ecatcomm.h"

// Application definitions
#define EC_TIMEOUTMON 500

// Appliction data
char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int oloop, iloop, chk;
boolean needlf = FALSE;
boolean inOP = FALSE;

// External data
extern ecx_contextt ecat_context;

// Local slave info
ec_ODlistt odinfo;
ec_OElistt odentryinfo;

//
// Application functions
//

void ecatcomm_slave_check_sdo(int index, int subindex, boolean CA)
{
    int sdodata[4]={0,0}, sdodatasize=16;
    wkc = ecx_SDOread(&ecat_context, 1, index, subindex, CA, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
    printf(" OD entry {Index 0x%x, Subindex 0x%x} is  0x%x 0x%x 0x%x 0x%x\n", index, subindex, sdodata[3], sdodata[2], sdodata[1], sdodata[0]);
}

void ecatcomm_slave_set_rxpdo(elmo_twitter_outdata_t *pdata, int command, double torque)
{
    static dsp402_controlword_t controlword = {0};
    int16_t torque_data=0;
    char databuffer[4];

    // Set controlword data
    switch(command)
    {
        case SWITCH_ON:
            controlword.bits.switch_on = 1;
            break;

        case SHUTDOWN:
            controlword.bits.switch_on = 0;
            break;

        case DISABLE_VOLTAGE:
            controlword.bits.enable_voltage = 0;
            break;

        case ENABLE_VOLTAGE:
            controlword.bits.enable_voltage = 1;
            break;

        case QUICK_STOP:
            controlword.bits.quick_stop = 1;
            break;

        case DISABLE_OPERATION:
            controlword.bits.enable_operation = 0;
            break;

        case ENABLE_OPERATION:
            controlword.bits.enable_operation = 1;
            break;

        case FAULT_RESET:
            controlword.bits.fault_reset = 1;
            break;

        case HALT:
            controlword.bits.halt = 1;
            break;

        case HALT_RESET:
            controlword.bits.halt = 0;
            break;

        case CLEAR_CONTROLWORD:
            controlword.all = 0;
            return;
    }

    // Covert torque data to INT16 from double
    double rated_current = 20000.0;
    double rated_torque = (20000.0*0.27)*0.001;
    torque_data = (int16_t)(torque/rated_torque*1000.0);

    // Copy to internal struct
    pdata->controlword.all = controlword.all;
    pdata->torque = torque_data;

    // Write to output buffer
    databuffer[0] = ((controlword.all >> 0) & 0xff);
    databuffer[1] = ((controlword.all >> 8) & 0xff);
    databuffer[2] = ((torque_data >> 0) & 0xff);
    databuffer[3] = ((torque_data >> 8) & 0xff);

    // Set data into buffer
    memcpy(ecat_context.slavelist[1].outputs, &databuffer[0], 4);
};

void ecatcomm_slave_get_txpdo(elmo_twitter_indata_t *pdata)
{
    uint16 statusword=0;
    char databuffer[21];

    // Get data
    memcpy(&databuffer[0], ecat_context.slavelist[1].inputs, 21);

    // Store data
    pdata->statusword.all = ((databuffer[13] << 8 ) & 0xff00) | (databuffer[12] & 0xff);
    pdata->position = ((databuffer[3] << 24) & 0xff000000) | ((databuffer[2] << 16) & 0x00ff0000) | ((databuffer[1] << 8) & 0x0000ff00) | ((databuffer[0] << 0) & 0x000000ff);
    pdata->digitalin = ((databuffer[7] << 24) & 0xff000000) | ((databuffer[6] << 16) & 0x00ff0000) | ((databuffer[5] << 8) & 0x0000ff00) | ((databuffer[4] << 0) & 0x000000ff);
    pdata->velocity = ((databuffer[11] << 24) & 0xff000000) | ((databuffer[10] << 16) & 0x00ff0000) | ((databuffer[9] << 8) & 0x0000ff00) | ((databuffer[8] << 0) & 0x000000ff);
    pdata->busvoltage = ((databuffer[18] << 24) & 0xff000000) | ((databuffer[17] << 16) & 0x00ff0000) | ((databuffer[16] << 8) & 0x0000ff00) | ((databuffer[15] << 0) & 0x000000ff);
    pdata->motorcurrent = ((databuffer[20] << 8) & 0xff00) | ((databuffer[19] << 0) & 0x00ff);
};

void ecatcomm_slave_print_statusword(dsp402_statusword_t statusword)
{
    // Printout
    printf("\n\n");
    printf("statusword.ready_to_switch_on         = %d\n", statusword.bits.ready_to_switch_on);
    printf("statusword.switched_on                = %d\n", statusword.bits.switched_on);
    printf("statusword.operation_enabled          = %d\n", statusword.bits.operation_enabled);
    printf("statusword.fault                      = %d\n", statusword.bits.fault);
    printf("statusword.volt_enabled               = %d\n", statusword.bits.voltage_enabled);
    printf("statusword.quick_stop                 = %d\n", statusword.bits.quick_stop);
    printf("statusword.switch_on_disabled         = %d\n", statusword.bits.switch_on_disabled);
    printf("statusword.warning                    = %d\n", statusword.bits.warning);
    printf("statusword.manufacturer_specific      = %d\n", statusword.bits.manufacturer_specific_0);
    printf("statusword.remote                     = %d\n", statusword.bits.remote);
    printf("statusword.operation_mode_specific_0  = %d\n", statusword.bits.operation_mode_specific_0);
    printf("statusword.internal_limit_active      = %d\n", statusword.bits.internal_limit_active);
    printf("statusword.operation_mode_specific_1  = %d\n", statusword.bits.operation_mode_specific_1);
    printf("statusword.operation_mode_specific_2  = %d\n", statusword.bits.operation_mode_specific_2);
    printf("statusword.manufacturer_specific      = %d\n", statusword.bits.manufacturer_specific_1);
}

void ecatcomm_slave_print_controlword(dsp402_controlword_t controlword)
{
    // Printout
    printf("\n\n");
    printf("controlword.switch_on                 = %d\n", controlword.bits.switch_on);
    printf("controlword.enable_voltage            = %d\n", controlword.bits.enable_voltage);
    printf("controlword.quick_stop                = %d\n", controlword.bits.quick_stop);
    printf("controlword.enable_operation          = %d\n", controlword.bits.enable_operation);
    printf("controlword.operation_mode_specific_0 = %d\n", controlword.bits.operation_mode_specific_0);
    printf("controlword.fault_reset               = %d\n", controlword.bits.fault_reset);
    printf("controlword.halt                      = %d\n", controlword.bits.halt);
    printf("controlword.operation_mode_specific_1 = %d\n", controlword.bits.operation_mode_specific_1);
    printf("controlword.manufacturer_specific     = %d\n", controlword.bits.manufacturer_specific);
}

void ecatcomm_init(char *ifname)
{
    int i, j;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ecx_init(&ecat_context, ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);

        int slavenum = ecx_config_init(&ecat_context, FALSE);
        printf("slavenum is %d.\n", slavenum);

        /* find and auto-config slaves */
        if ( slavenum > 0 )
        {
            printf("%d slaves found and configured.\n", *ecat_context.slavecount);

            ecx_config_map_group(&ecat_context, &IOmap, 0);
            ecx_configdc(&ecat_context);

            printf("Slaves mapped, state to SAFE_OP.\n");

            /* wait for all slaves to reach SAFE_OP state */
            ecx_statecheck(&ecat_context, 0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            oloop = ecat_context.slavelist[0].Obytes;
            if ((oloop == 0) && (ecat_context.slavelist[0].Obits > 0)) oloop = 1;
            //if (oloop > 8) oloop = 8;
            iloop = ecat_context.slavelist[0].Ibytes;
            if ((iloop == 0) && (ecat_context.slavelist[0].Ibits > 0)) iloop = 1;
            //if (iloop > 8) iloop = 8;

            printf("segments : %d : %d %d %d %d\n",ecat_context.grouplist[0].nsegments, ecat_context.grouplist[0].IOsegment[0], ecat_context.grouplist[0].IOsegment[1], ecat_context.grouplist[0].IOsegment[2], ecat_context.grouplist[0].IOsegment[3]);

            // Disable symmetrical transfers
            // ecat_context.grouplist[0].blockLRW = 1;

            // printf("Request operational state for all slaves\n");
            // expectedWKC = (ecat_context.grouplist[0].outputsWKC * 2) + ecat_context.grouplist[0].inputsWKC;
            // printf("Calculated workcounter %d\n", expectedWKC);
            // ecat_context.slavelist[0].state = EC_STATE_OPERATIONAL;
            //
            /* send one valid process data to make outputs in slaves happy*/
            // ecx_send_processdata(&ecat_context);
            // ecx_receive_processdata(&ecat_context, EC_TIMEOUTRET);
            //
            // /* request OP state for all slaves */
            // ecx_writestate(&ecat_context, 0);
            // chk = 40;
            //
            // /* wait for all slaves to reach OP state */
            // do {
            //     ecx_send_processdata(&ecat_context);
            //     ecx_receive_processdata(&ecat_context, EC_TIMEOUTRET);
            //     ecx_statecheck(&ecat_context, 0, EC_STATE_OPERATIONAL, 50000);
            // } while (chk-- && (ecat_context.slavelist[0].state != EC_STATE_OPERATIONAL));
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
};

void ecatcomm_slave_info(void)
{
    int ret = 0;
    int i = 0, k = 0, j = 0, n = 0;
    int wkc = 0;
    int32_t actual_position = 0;
    int posdatasize = sizeof(int32_t);
    int Osize = 0, Isize = 0;

    #define Nsdo  16
    #define Ndata 64
    int sdodata[Nsdo], sdodatasize;
    char *databuf;
    databuf = (char*)&sdodata[0];

    // Check PDO mapping size
    ret = ecx_readPDOmap(&ecat_context, 1, &Osize, &Isize);
    printf("\n\nec_readPDOmap returned %d\n", ret);
    printf("Osize in bits = %d\n", Osize);
    printf("Isize in bits = %d\n", Isize);

    // Get complete OD dump
    ret = ecx_readODlist(&ecat_context, 1, &odinfo);
    printf("\nc_readODlist returned %d\n", ret);
    printf("Slave = %d, Entries = %d\n", odinfo.Slave, odinfo.Entries);
    for (k = 0; k < odinfo.Entries; k++)
    {
        wkc = ecx_readODdescription(&ecat_context, k, &odinfo);
        wkc = ecx_readOE(&ecat_context, k, &odinfo, &odentryinfo);
        printf("\nIndex = 0x%x\n", odinfo.Index[k]);
        printf("    MaxSub     = %d\n", odinfo.MaxSub[k]+1);
        printf("    ObjectCode = %d\n", odinfo.ObjectCode[k]);
        printf("    DataType   = %d\n", odinfo.DataType[k]);
        printf("    Description: %s\n", &odinfo.Name[k][0]);
        printf("    OE Entries = %d\n", odentryinfo.Entries);
        for (j = 0; j < odentryinfo.Entries; j++)
        {
            for (n=0; n<Nsdo; n++) sdodata[n] = 0;
            sdodatasize = Nsdo*sizeof(int);
            wkc = ecx_SDOread(&ecat_context, odinfo.Slave, odinfo.Index[k], j, 0, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
            printf("    OE = %d\n", j);
            printf("        ValueInfo  = %d\n", odentryinfo.ValueInfo[j]);
            printf("        DataType   = %d\n", odentryinfo.DataType[j]);
            printf("        BitLength  = %d\n", odentryinfo.BitLength[j]);
            printf("        ObjAccess  = %d\n", odentryinfo.ObjAccess[j]);
            printf("        Name       = %s\n", &odentryinfo.Name[j][0]);
            printf("        Value      =");
            for (n=0; n<sdodatasize; n++) printf(" 0x%x", (0xFF & databuf[n]));
            printf("\n");
        }
    }
}

void ecatcomm_slave_config(void)
{
    int i=0;
    uint8 bufferu8 = 0;
    uint16 bufferu16 = 0;
    uint32 bufferu32 = 0;
    uint64 bufferu64 = 0;
    int8 buffers8 = 0;
    int16 buffers16 = 0;
    int32 buffers32 = 0;
    int64 buffers64 = 0;

    dsp402_controlword_t controlword = {0};
    elmo_twitter_indata_t indata;
    elmo_twitter_outdata_t outdata;

    // // Goto ready
    // bufferu16 = 0x86;
    // wkc = ecx_SDOwrite(&ecat_context, 1, 0x6040, 0, FALSE, 2, &bufferu16, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x6040, 0, FALSE);

    // Set state
    ecat_context.slavelist[0].state = EC_STATE_PRE_OP;
    ecx_writestate(&ecat_context, 0);
    ecx_statecheck(&ecat_context, 0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE * 4);

    printf("\nSetting device configuratons...\n");

    // RxPDO assignments in SM2
    bufferu8 = 1;
    wkc = ecx_SDOwrite(&ecat_context, 1, 0x1c12, 0, FALSE, 1, &bufferu8, EC_TIMEOUTRXM);
    ecatcomm_slave_check_sdo(0x1c12, 0, FALSE);

    bufferu16 = 0x1602;
    wkc = ecx_SDOwrite(&ecat_context, 1, 0x1c12, 1, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
    ecatcomm_slave_check_sdo(0x1c12, 1, TRUE);

    // RxPDO assignments in SM3
    bufferu8 = 3;
    wkc = ecx_SDOwrite(&ecat_context, 1, 0x1c13, 0, FALSE, 1, &bufferu8, EC_TIMEOUTRXM);
    ecatcomm_slave_check_sdo(0x1c13, 0, FALSE);

    bufferu64 = 0x1a1f1a181a03;
    wkc = ecx_SDOwrite(&ecat_context, 1, 0x1c13, 1, TRUE, 8, &bufferu64, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x1c13, 1, TRUE);
    ecatcomm_slave_check_sdo(0x1c13, 1, FALSE);
    ecatcomm_slave_check_sdo(0x1c13, 2, FALSE);
    ecatcomm_slave_check_sdo(0x1c13, 3, FALSE);

    // bufferu16 = 0x1a03; // position + velocity feedback
    // wkc = ecx_SDOwrite(&ecat_context, 1, 0x1c13, 1, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x1c13, 1, TRUE);
    //
    // bufferu16 = 0x1a18; // DC bus voltage
    // wkc = ecx_SDOwrite(&ecat_context, 1, 0x1c13, 2, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x1c13, 2, TRUE);
    //
    // bufferu16 = 0x1a1f; // motor current
    // wkc = ecx_SDOwrite(&ecat_context, 1, 0x1c13, 3, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x1c13, 3, TRUE);

    // DC Sync0
    double time_step = 1e-3;
    ecx_dcsync0(&ecat_context, 1, TRUE, (int64)(time_step*1e9), (int64)(time_step*0.5*1e9));

    // Set state
    ecat_context.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ecat_context, 0);
    ecx_statecheck(&ecat_context, 0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

    // Access back-door CA[] command for commutation configuration
    // buffers32 = 0x5f;
    // wkc = ecx_SDOwrite(&ecat_context, 1, 0x3034, 7, FALSE, 4, &buffers32, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x3034, 7, FALSE);

    // Halp Option Code
    buffers16 = 2;
    wkc = ecx_SDOwrite(&ecat_context, 1, 0x605D, 0, FALSE, 2, &buffers16, EC_TIMEOUTRXM);
    ecatcomm_slave_check_sdo(0x605D, 0, FALSE);

    // Set slave operation mode
    bufferu8 = 10;
    wkc = ecx_SDOwrite(&ecat_context, 1, 0x6060, 0, FALSE, 1, &bufferu8, EC_TIMEOUTRXM);
    ecatcomm_slave_check_sdo(0x6060, 0, FALSE);
    ecatcomm_slave_check_sdo(0x6061, 0, FALSE);

    // Set state to Ethercat Operatoinal
    ecat_context.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ecat_context, 0);
    ecx_statecheck(&ecat_context, 0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE * 4);

    // send one valid process data to make outputs in slaves happy
    ecx_send_processdata(&ecat_context);
    wkc = ecx_receive_processdata(&ecat_context, EC_TIMEOUTRET);

    // wait for all slaves to reach OP state
    int chk = 40;
    do {
        ecx_send_processdata(&ecat_context);
        wkc = ecx_receive_processdata(&ecat_context, EC_TIMEOUTRET);
        ecx_statecheck(&ecat_context, 0, EC_STATE_OPERATIONAL,  50000);
    } while (chk-- && (ecat_context.slavelist[0].state != EC_STATE_OPERATIONAL));

    // Get error data
    ecatcomm_slave_dump_status_info();
}

void ecatcomm_slave_state_check(void)
{
    int slavestate=0, wkc=0;
    int sdodata=0, sdodatasize=sizeof(int);

    slavestate = ecx_readstate(&ecat_context);
    printf("\nSlave EtherCAT StateMachine state is 0x%x\n", slavestate);

    sdodata = 0;
    sdodatasize = sizeof(int);
    wkc = ecx_SDOread(&ecat_context, 1, 0x6041, 0, 0, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
    printf("Slave DSP402 StateMachine state is 0x%x\n", (uint16)sdodata);
}

void ecatcomm_slave_check_errors(void)
{
    int i=0, Nt=0, Nh=0, Nerr=0;
    ec_errort error;
    char *errstr;

    printf("\nERROR REPORT:\n");

    // Built-in error reporting
    if (ecx_iserror(&ecat_context))
    {
        do {
            errstr = ecx_elist2string(&ecat_context);
            printf("    Error %d: %s", ++i, errstr);
        } while(ecx_iserror(&ecat_context));
    }
}

void ecatcomm_run(void)
{
    int i, j;
    int test_step=0;
    int step_counter=0;
    int running=0;
    elmo_twitter_indata_t indata;
    elmo_twitter_outdata_t outdata;
    int print_counter=0;

    if (ecat_context.slavelist[0].state == EC_STATE_OPERATIONAL )
    {
        printf("Operational state reached for all slaves.\n");
        inOP = TRUE;

        // Start preparation
        test_step = 0;
        step_counter = 0;
        running = 1;
        i=0;

        // Handle PDO streams
        outdata.controlword.all = 0;
        ecatcomm_slave_set_rxpdo(&outdata, CLEAR_CONTROLWORD, 0.0);
        ecatcomm_slave_set_rxpdo(&outdata, SHUTDOWN, 0.0);
        ecx_send_processdata(&ecat_context);
        wkc = ecx_receive_processdata(&ecat_context, EC_TIMEOUTRET);
        ecatcomm_slave_get_txpdo(&indata);
        osal_usleep(1000);

        /* cyclic loop */
        while(running)
        {
            // Step
            step_counter++;
            i++;

            // Test procedure
            switch (test_step)
            {
                case 0: // Reset errors
                    if (step_counter >= 2000)
                    {
                        step_counter = 0;
                        test_step++;
                    }
                    else if (step_counter == 1)
                    {
                        ecatcomm_slave_set_rxpdo(&outdata, CLEAR_CONTROLWORD, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, FAULT_RESET, 0.0);
                        ecatcomm_slave_print_controlword(outdata.controlword);
                        ecatcomm_slave_print_statusword(indata.statusword);
                        printf("Clearing errors...\n\n");
                    }
                    break;

                case 1: // Startup
                    if (step_counter >= 5000)
                    {
                        step_counter = 0;
                        test_step++;
                    }
                    else if (step_counter == 1)
                    {
                        ecatcomm_slave_set_rxpdo(&outdata, CLEAR_CONTROLWORD, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_VOLTAGE, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, QUICK_STOP, 0.0);
                        ecatcomm_slave_print_controlword(outdata.controlword);
                        ecatcomm_slave_print_statusword(indata.statusword);
                        printf("Startup up drive...\n\n");
                    }
                    break;

                case 2: // Switch on
                    if (step_counter >= 5000)
                    {
                        step_counter = 0;
                        test_step++;
                    }
                    else if (step_counter == 1)
                    {
                        ecatcomm_slave_set_rxpdo(&outdata, CLEAR_CONTROLWORD, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_VOLTAGE, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, QUICK_STOP, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, SWITCH_ON, 0.0);
                        ecatcomm_slave_print_controlword(outdata.controlword);
                        ecatcomm_slave_print_statusword(indata.statusword);
                        printf("Switching on drive...\n\n");
                    }
                    break;
                case 3: // enable operation
                    if(step_counter >= 5000)
                    {
                        step_counter = 0;
                        test_step++;
                    }
                    else if (step_counter == 1)
                    {
                        ecatcomm_slave_set_rxpdo(&outdata, CLEAR_CONTROLWORD, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, SWITCH_ON, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_VOLTAGE, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, QUICK_STOP, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_OPERATION, 0.0);
                        ecatcomm_slave_print_controlword(outdata.controlword);
                        ecatcomm_slave_print_statusword(indata.statusword);
                        printf("Enabling operation...\n\n");
                    }
                break;
                case 4: // wait delay
                    if(step_counter >= 7000)
                    {
                        step_counter = 0;
                        test_step++;
                    }
                    else if (step_counter == 1)
                    {
                        ecatcomm_slave_set_rxpdo(&outdata, CLEAR_CONTROLWORD, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, SWITCH_ON, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_VOLTAGE, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, QUICK_STOP, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_OPERATION, 1.0);
                        ecatcomm_slave_print_controlword(outdata.controlword);
                        ecatcomm_slave_print_statusword(indata.statusword);
                        printf("Running test...\n\n");
                    }
                    break;
                case 5: // Run test output
                    if(step_counter >= 7000)
                    {
                        step_counter = 0;
                        test_step++;
                    }
                    else if (step_counter == 1)
                    {
                        ecatcomm_slave_set_rxpdo(&outdata, CLEAR_CONTROLWORD, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, SWITCH_ON, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_VOLTAGE, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, QUICK_STOP, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_OPERATION, -1.0);
                        ecatcomm_slave_print_controlword(outdata.controlword);
                        ecatcomm_slave_print_statusword(indata.statusword);
                        printf("Running test...\n\n");
                    }
                    break;
                case 6: // Stop before end
                    if(step_counter >= 5000)
                    {
                        step_counter = 0;
                        test_step++;
                    }
                    else if (step_counter == 1)
                    {
                        ecatcomm_slave_set_rxpdo(&outdata, CLEAR_CONTROLWORD, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, SWITCH_ON, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_VOLTAGE, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, QUICK_STOP, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_OPERATION, 0.0);
                        ecatcomm_slave_print_controlword(outdata.controlword);
                        ecatcomm_slave_print_statusword(indata.statusword);
                        printf("Stopping test...\n\n");
                    }
                    break;
                case 7: // Stop before end
                    if(step_counter >= 5000)
                    {
                        step_counter = 0;
                        test_step++;
                        running = 0;
                    }
                    else if (step_counter == 1)
                    {
                        ecatcomm_slave_set_rxpdo(&outdata, CLEAR_CONTROLWORD, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, SWITCH_ON, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, ENABLE_VOLTAGE, 0.0);
                        ecatcomm_slave_set_rxpdo(&outdata, QUICK_STOP, 0.0);
                        ecatcomm_slave_print_controlword(outdata.controlword);
                        ecatcomm_slave_print_statusword(indata.statusword);
                        printf("Exiting test...\n\n");
                    }
                    break;
            }

            // Handle PDO streams
            ecx_send_processdata(&ecat_context);
            wkc = ecx_receive_processdata(&ecat_context, EC_TIMEOUTRET);
            ecatcomm_slave_get_txpdo(&indata);

            if((wkc >= expectedWKC) && (++print_counter>=5))
            {
                printf("Processdata cycle %4d, WKC %d", i, wkc);

                printf(", Outputs:");
                for(j = 0 ; j < oloop; j++)
                {
                    printf(" %2.2x", *(ecat_context.slavelist[0].outputs + j));
                }
                // printf(", Inputs:");
                // for(j = 0 ; j < iloop; j++)
                // {
                //     printf(" %2.2x", *(ecat_context.slavelist[0].inputs + j));
                // }

                // printf(" T:%"PRId64"",ecat_context.DCtime[0]);
                printf(", Command Data: 0x%4x, %4d", outdata.controlword.all, outdata.torque);
                printf(", Feedback Data: 0x%4x, %8d, %8d, %8d, %8d", indata.statusword.all, indata.position, indata.velocity, indata.busvoltage, indata.motorcurrent);
                printf("\r");
                print_counter = 0;
            }

            osal_usleep(1000);
        }

        printf("\n\n");
        inOP = FALSE;
    }
    else
    {
        printf("Not all slaves reached operational state.\n");
        ecx_readstate(&ecat_context);
        for(i = 1; i <= *ecat_context.slavecount ; i++)
        {
            if(ecat_context.slavelist[i].state != EC_STATE_OPERATIONAL)
            {
                printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                i, ecat_context.slavelist[i].state, ecat_context.slavelist[i].ALstatuscode, ec_ALstatuscode2string(ecat_context.slavelist[i].ALstatuscode));
            }
        }
    }
};

void ecatcomm_exit(void)
{
    /* request INIT state for all slaves */
    printf("\nRequest init state for all slaves\n");
    ecat_context.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ecat_context, 0);

    /* stop SOEM, close socket */
    printf("End simple test, close socket\n");
    ecx_close(&ecat_context);
};

void ecatcomm_slave_dump_status_info(void)
{
    printf("\nDevice Errors: \n");
    ecatcomm_slave_check_sdo(0x1001, 0, FALSE);

    ecatcomm_slave_check_sdo(0x1002, 0, FALSE);

    ecatcomm_slave_check_sdo(0x1003, 0, TRUE);

    ecatcomm_slave_check_sdo(0x2081, 0, FALSE);
    ecatcomm_slave_check_sdo(0x2081, 1, FALSE);
    ecatcomm_slave_check_sdo(0x2081, 2, FALSE);
    ecatcomm_slave_check_sdo(0x2081, 3, FALSE);
    ecatcomm_slave_check_sdo(0x2081, 4, FALSE);
    ecatcomm_slave_check_sdo(0x2081, 5, FALSE);
    ecatcomm_slave_check_sdo(0x2081, 6, FALSE);

    ecatcomm_slave_check_sdo(0x2085, 0, FALSE);
}
