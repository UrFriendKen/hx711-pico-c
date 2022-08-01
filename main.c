// MIT License
// 
// Copyright (c) 2022 Daniel Robertson
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdio.h"
#include "include/scale.h"
#include "hx711_noblock.pio.h"
#include "i2c_slave/include/i2c_fifo.h"
#include "i2c_slave/include/i2c_slave.h"
#include <pico/stdlib.h>
#include <string.h>


static const uint I2C_SLAVE_ADDRESS = 0x17;      // TO MAKE SETTABLE
static const uint I2C_BAUDRATE = 1000; // 1 kHz

static const uint I2C_SLAVE_SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN; // 5

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

// 2 Sensors
scale_t sc0; // Scale 1
scale_t sc1; // Scale 2
scale_options_t opt = SCALE_DEFAULT_OPTIONS;

bool read_flag;
bool *read_flag_ptr = &read_flag;

// The slave implements a 256 byte memory. To write a series of bytes, the master first
// writes the memory address, followed by the data. The address is automatically incremented
// for each byte transferred, looping back to 0 upon reaching the end. Reading is done
// sequentially from the current memory address.
static struct
{
    uint8_t mem[256];
    uint8_t mem_address;
    bool mem_address_written;
} context;

void read_hx711_to_mem(scale_t *sc, scale_options_t *opt) {
    mass_t mass;
    char buff[MASS_TO_STRING_BUFF_SIZE];

    memset(buff, 0, MASS_TO_STRING_BUFF_SIZE);

    //8. obtain a mass from the scale
    if(scale_weight(sc, &mass, opt)) {
        //11. display the newly obtained mass...
        mass_to_string(&mass, buff);
        printf("Scale 0: %s", buff);
    }
    else {
        printf("Failed to read weight\n");
    }
}

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    gpio_put(LED_PIN, 1);
    switch (event) {
    case I2C_SLAVE_RECEIVE: ;// master has written some data
        uint8_t addr = i2c_read_byte(i2c);
        if (addr == 0x06) {
            *read_flag_ptr = true;
        }
        if (!context.mem_address_written) {
            // writes always start with the memory address
            context.mem_address = addr;
            printf("Mem addr: %X\n", context.mem_address);
            context.mem_address_written = true;
        } else if (context.mem_address < 0x80) {
            // save into memory
            context.mem[context.mem_address] = i2c_read_byte(i2c);
            printf("WRITE\t%X: %X\n", context.mem_address, context.mem[context.mem_address]);
            context.mem_address++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        printf("READ\t%X: %X\n", context.mem_address, context.mem[context.mem_address]);
        i2c_write_byte(i2c, context.mem[context.mem_address]);
        context.mem_address++;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        printf("FINISH\n");
            printf("%d\n", read_flag);
        context.mem_address_written = false;
        break;
    default:
        break;
    }
    gpio_put(LED_PIN, 0);
}

static void setup_slave() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode, passing in I2C ISR handler
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

int main(void) {
    context.mem_address_written = false;
    stdio_init_all();

    // SET THESE TO THE GPIO PINS CONNECTED TO THE
    // HX711's CLOCK AND DATA PINS
    // PINOUT REFERENCE: https://learn.adafruit.com/assets/99339
    const uint clkPin0 = 14; // GP14, PAD19
    const uint datPin0 = 15; // GP15, PAD20

    // CALIBRATE YOUR SCALE TO OBTAIN THESE VALUES
    // https://github.com/endail/hx711-pico-c#how-to-calibrate
    const mass_unit_t unit = mass_g;
    const int32_t refUnit0 = -453;
    const int32_t offset0 = -8024964;

    hx711_t hx0;


    /*  SENSOR 1 */

    //1. init the hx711 struct
    hx711_init(
        &hx0,
        clkPin0,
        datPin0,
        pio0,
        &hx711_noblock_program,
        &hx711_noblock_program_init);

    //2. power up the hx711
    hx711_set_power(&hx0, hx711_pwr_up);

    //3. [OPTIONAL] set gain and save it to the HX711
    //chip by powering down then back up
    hx711_set_gain(&hx0, hx711_gain_128);
    hx711_set_power(&hx0, hx711_pwr_down);
    hx711_wait_power_down();
    hx711_set_power(&hx0, hx711_pwr_up);

    //4. pause to allow the readings to settle based on the
    //sample rate of the chip
    hx711_wait_settle(hx711_rate_10);

    //at this point, the hx711 can reliably produce values
    //with hx711_get_value or hx711_get_value_timeout

    //5. init the scale
    scale_init(&sc0, &hx0, unit, refUnit0, offset0);

    //6. spend 10 seconds obtaining as many samples as
    //possible to zero (aka. tare) the scale
    opt.strat = strategy_type_time;
    opt.timeout = 10000000;

    if(scale_zero(&sc0, &opt)) {
        printf("Scale zeroed successfully\n");
    }
    else {
        printf("Scale failed to zero\n");
    }





    // SET THESE TO THE GPIO PINS CONNECTED TO THE
    // HX711's CLOCK AND DATA PINS
    // PINOUT REFERENCE: https://learn.adafruit.com/assets/99339
    const uint clkPin1 = 12; // GP14, PAD19
    const uint datPin1 = 13; // GP15, PAD20

    // CALIBRATE YOUR SCALE TO OBTAIN THESE VALUES
    // https://github.com/endail/hx711-pico-c#how-to-calibrate
    const int32_t refUnit1 = -453;
    const int32_t offset1 = -8024964;

    hx711_t hx1;

    /*  SENSOR 2 */

    //1. init the hx711 struct
    hx711_init(
        &hx1,
        clkPin1,
        datPin1,
        pio1,
        &hx711_noblock_program,
        &hx711_noblock_program_init);

    //2. power up the hx711
    hx711_set_power(&hx1, hx711_pwr_up);

    //3. [OPTIONAL] set gain and save it to the HX711
    //chip by powering down then back up
    hx711_set_gain(&hx1, hx711_gain_128);
    hx711_set_power(&hx1, hx711_pwr_down);
    hx711_wait_power_down();
    hx711_set_power(&hx1, hx711_pwr_up);

    //4. pause to allow the readings to settle based on the
    //sample rate of the chip
    hx711_wait_settle(hx711_rate_10);

    //at this point, the hx711 can reliably produce values
    //with hx711_get_value or hx711_get_value_timeout

    //5. init the scale
    scale_init(&sc1, &hx1, unit, refUnit1, offset1);

    //6. spend 10 seconds obtaining as many samples as
    //possible to zero (aka. tare) the scale
    opt.strat = strategy_type_time;
    opt.timeout = 10000000;

    if(scale_zero(&sc1, &opt)) {
        printf("Scale zeroed successfully\n");
    }
    else {
        printf("Scale failed to zero\n");
    }

    *read_flag_ptr = false;

    setup_slave();

    printf("READY\n");


    //7. change to spending 250 milliseconds obtaining
    //as many samples as possible
    opt.timeout = 250000;

    //NOTE
    // Read flag not entering if
    for(;;) {
        if (read_flag) {
            printf("here");
            read_hx711_to_mem(&sc0, &opt);
            printf("\t");
            read_hx711_to_mem(&sc1, &opt);
            printf("\n");
            *read_flag_ptr = false;
        }
    }
    return EXIT_SUCCESS;
}
