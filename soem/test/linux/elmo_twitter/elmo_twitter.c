// Standard headers
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

// #define USE_PROMPT

// Module headers
#include "ecatcomm.h"

int main(int argc, char *argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        // Setup communication
        ecatcomm_init(argv[1]);

        #ifdef USE_PROMPT
        printf("\nPress Any Key to Continue\n");
        getchar();
        #endif

        // Check state of slave - it should timeout due to disruption of PDO datagrams.
        ecatcomm_slave_state_check();

        #ifdef USE_PROMPT
        printf("\nPress Any Key to Continue\n");
        getchar();
        #endif

        //Get Device information - Before configuration
        // ecatcomm_slave_info();

        #ifdef USE_PROMPT
        printf("\nPress Any Key to Continue\n");
        getchar();
        #endif

        // Set Device configuration
        ecatcomm_slave_config();

        #ifdef USE_PROMPT
        printf("\nPress Any Key to Continue\n");
        getchar();
        #endif

        // Check state of slave - it should timeout due to disruption of PDO datagrams.
        ecatcomm_slave_state_check();

        #ifdef USE_PROMPT
        printf("\nPress Any Key to Continue\n");
        getchar();
        #endif

        // Get Device information - After configuration
        // ecatcomm_slave_info();

        #ifdef USE_PROMPT
        printf("\nPress Any Key to Continue\n");
        getchar();
        #endif

        // Run communication
        ecatcomm_run();

        #ifdef USE_PROMPT
        printf("\nPress Any Key to Continue\n");
        getchar();
        #endif

        // Check state of slave - it should timeout due to disruption of PDO datagrams.
        ecatcomm_slave_state_check();

        #ifdef USE_PROMPT
        printf("\nPress Any Key to Continue\n");
        getchar();
        #endif

        ecatcomm_slave_check_errors();

        #ifdef USE_PROMPT
        printf("\nPress Any Key to Continue\n");
        getchar();
        #endif

        // Terminate session
        ecatcomm_exit();
    }
    else
    {
        printf("Usage: elmo_twitter ifname1\nifname = eth0 for example\n");
    }

    printf("End program\n");
    return (0);
};
