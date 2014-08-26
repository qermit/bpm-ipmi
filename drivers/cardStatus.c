/*
 * cardStatus.c
 *
 *  Created on: 14-09-2013
 *      Author: Bartek
 */

#include "../include/cardType.h"
#include "../include/gpio.h"
#include "../include/cardStatus.h"
#include "../stdPeriphLibs/lpc_types.h"
#include "../IC/ic_commonHeader.h"
#include "../include/CardDiagnostic.h"
#include "../include/comExpress.h"
#include "../include/fmc_hpc_tester.h"

#ifndef AMC_CPU_COM_Express
  volatile uint8_t clk_initialised = 0;
  volatile uint8_t FPGA_programming = 0;
#else
  volatile uint8_t callbackInitFlag = 0;
  volatile uint8_t COM_Express_initialized = 0;
  volatile uint8_t commExpressLoggedIn = 0;
#endif



void checkCardStatus( void )
{


#ifdef AMC_FMC_Carierr
        /*checking if FPGA programming cycle is done
         *and reset FPGA if so
         */
        if( ! checkDONEpin( ) )
        {
                FPGA_programming = 1;
        }

        if( ( (FPGA_programming == 1) & checkDONEpin() ) )
        {
                reset_FPGA(  );
                FPGA_programming = 0;
        }
        //////////////////////////////////////////////

        /*Checking if DC-DC converter voltage is set properly
         *and configure ADN4604 clocks connections if so and
         *if they aren't configure yet.
         */
        if( checkPGOODpin() & checkMasterRESETpin() )
        {
#ifdef AFC_TEST_MODE
            test_i2c_chips( );
#endif
            setRTM_ConnectorON();

            if(!clk_initialised){

                     clk_initialised = initClockMgnt();
#ifndef AFC_TEST_MODE
                     initDiagnosticMonitor();
                     diagnosticMonitorCallback (ENABLE);
#endif
                     //test_i2c_chips( );
                }
        }
        else
        {
                clk_initialised = 0;
                diagnosticMonitorCallback (DISABLE);

        }
        //////////////////////////////////////////////////
#endif


#ifdef AMC_PCIe_Adapter
        if( !callbackInitFlag )
        {
                diagnosticMonitorCallback( ENABLE );
                callbackInitFlag = 1;
        }
#endif

#ifdef AMC_CPU_COM_Express
        if( (!COM_Express_initialized) & commExpressLoggedIn == 0xff )
        {
                initComExpress();
                COM_Express_initialized = 0xff;
                //commExpressLoggedIn = 0x00;
        }
#endif

}
