/**
 * Copyright (c) 2020 SparkFun Electronics
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "qwiic_ak975x.h"

AK975X myDevice;


int main() {

     // setup stdio. Output device is controlled in CMakeLists.txt file
    stdio_init_all();

	printf("Starting AK975X Test\n\n");

    // Init the system 

    if(!myDevice.begin()){
    	printf("Error starting the qwiic AK975X device. Aboarting\n");
  		return 0;
    }

    while (1) {

        if(myDevice.available()){
            printf("1:DWN[%d]\t2:LFT[%d]\t3:UP[%d]\t4:RGH[%d]\ttempF[%f]\n",
                myDevice.getIR1(), myDevice.getIR2(), myDevice.getIR3(), myDevice.getIR4(), 
                myDevice.getTemperatureF());

            myDevice.refresh();

        }

        sleep_ms(1000);

    }

    return 0;
}
