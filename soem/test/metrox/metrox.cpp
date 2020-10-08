
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

#include <iostream>

int main(void)
{
    char* ifname = "enp0s31f6";
    
    //init etercat
    ec_init(ifname);
    
    if(ec_config_init(FALSE) > 0)
    {
        std::cout << "Slaves found: " << ec_slavecount << std::endl;
        ec_configdc();
        
        for(int i= 0; i < ec_slavecount; i++)
        {
            std::cout << i <<" address: " << ec_slave[i].configadr << ": " << ec_slave[i].name<< std::endl;
            int osize;
            int isize;
 
            if(ec_readIDNmap(i,&osize, &isize) > 0)
                std::cout << "idn mapping success" << std::endl;
                
            
        }
        
        ecx_contextt& context = ecx_context;
        
        int psize =2;
        uint16_t data = 0;
        std::cout << ec_SoEread(1,0, 0x40, 32, &psize, &data,EC_TIMEOUTRXM ) << std::endl;
        std::cout << psize << ": " << data << std::endl;
        data = 2;
        std::cout << ec_SoEwrite(1,0,0x40,32,2,&data, EC_TIMEOUTRXM);
                std::cout << ec_SoEread(1,0, 0x40, 32, &psize, &data,EC_TIMEOUTRXM ) << std::endl;
        std::cout << psize << ": " << data << std::endl;
        
        
        uint32_t vel = 2;
        psize = 4;
        std::cout << ec_SoEread(1,0, 0x40, 40, &psize, &data,EC_TIMEOUTRXM ) << std::endl;
        std::cout << psize << ": " << data << std::endl;

        
    }
    ec_close();
}
