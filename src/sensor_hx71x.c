// Support for bit-banging commands to HX711 and HX717 ADC chips
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include <stdbool.h>
#include <stdint.h>

struct hx71x_adc {
    struct timer timer;
    uint8_t gain_channel;   // the gain+channel selection (1-4)
    uint8_t flags;
    uint32_t rest_ticks;
    uint32_t last_error;
    uint32_t old_data; // save old data
    uint32_t query_ticks;
    struct gpio_in dout; // pin used to receive data from the hx71x
    struct gpio_out sclk; // pin used to generate clock for the hx71x
    struct sensor_bulk sb;
};

enum {
    HX_PENDING = 1<<0, HX_OVERFLOW = 1<<1,
};

#define BYTES_PER_SAMPLE 4
#define SAMPLE_ERROR_DESYNC 1L << 31
#define SAMPLE_ERROR_READ_TOO_LONG 1L << 30

static struct task_wake wake_hx71x;


/****************************************************************
 * Low-level bit-banging
 ****************************************************************/
#define PULSE_TIME_NS_NUM 200
#define MIN_PULSE_TIME nsecs_to_ticks(PULSE_TIME_NS_NUM)

static uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

// Pause for a minimum of 200ns
static void
hx71x_delay(void)
{
    if (CONFIG_MACH_AVR)
        // Optimize avr, as calculating time takes longer than needed delay
        return;
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

// Read 'num_bits' from the sensor
static uint32_t
hx71x_raw_read(struct gpio_in dout, struct gpio_out sclk, int num_bits)
{
    uint32_t bits_read = 0;
    while (num_bits--) {
        irq_disable();
        gpio_out_toggle_noirq(sclk);
        irq_enable();

        hx71x_delay();

        irq_disable();
        gpio_out_toggle_noirq(sclk);
        uint_fast8_t bit = gpio_in_read(dout);
        bits_read = (bits_read << 1) | bit;
        irq_enable();

        hx71x_delay();
    }
    return bits_read;
}


/****************************************************************
 * HX711 and HX717 Sensor Support
 ****************************************************************/

// Check if data is ready
static uint_fast8_t
hx71x_is_data_ready(struct hx71x_adc *hx71x)
{
    return !gpio_in_read(hx71x->dout);
}

// Event handler that wakes wake_hx71x() periodically
static uint_fast8_t
hx71x_event(struct timer *timer)
{
    struct hx71x_adc *hx71x = container_of(timer, struct hx71x_adc, timer);
    uint32_t rest_ticks = hx71x->rest_ticks;
    uint8_t flags = hx71x->flags;
    sched_wake_task(&wake_hx71x);
    hx71x->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void
add_sample(struct hx71x_adc *hx71x, uint8_t oid, uint32_t counts,
                uint8_t force_flush) {
    // Add measurement to buffer
    hx71x->sb.data[hx71x->sb.data_count] = counts;
    hx71x->sb.data[hx71x->sb.data_count + 1] = counts >> 8;
    hx71x->sb.data[hx71x->sb.data_count + 2] = counts >> 16;
    hx71x->sb.data[hx71x->sb.data_count + 3] = counts >> 24;
    hx71x->sb.data_count += BYTES_PER_SAMPLE;

    if (hx71x->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(hx71x->sb.data)
        || force_flush)
        sensor_bulk_report(&hx71x->sb, oid);
}

// hx71x ADC query
static void
hx71x_read_adc(struct hx71x_adc *hx71x, uint8_t oid)
{
    // Read from sensor
    uint_fast8_t gain_channel = hx71x->gain_channel;
    uint32_t adc = hx71x->old_data;
    uint32_t counts = adc;
    uint8_t flags = hx71x->flags;
    if (hx71x_is_data_ready(hx71x)) {
        // New sample pending
        hx71x->flags = HX_PENDING;
        uint32_t time1 = timer_read_time();
        adc = hx71x_raw_read(hx71x->dout, hx71x->sclk, 24 + gain_channel);
        uint32_t time2 = timer_read_time();
        hx71x->query_ticks = time2 - time1;

        irqstatus_t flag = irq_save();
        
        // Clear pending flag (and note if an overflow occurred)
        hx71x->flags = 0;
        hx71x->last_error = 0;
        // Extract report from raw data
        counts = (adc >> gain_channel) ^ 0x800000;

        // Check for errors
        uint_fast8_t extras_mask = (1 << gain_channel) - 1;
        if ((adc & extras_mask) != extras_mask) {
            // Transfer did not complete correctly
            hx71x->last_error = SAMPLE_ERROR_DESYNC;
        }
        
        irq_restore(flag);
        
    }
    else if (flags & HX_PENDING) {
        hx71x->sb.possible_overflows++;
        hx71x->flags |= HX_OVERFLOW;
        // Transfer took too long
        hx71x->last_error = SAMPLE_ERROR_READ_TOO_LONG;
    }

    // forever send errors until reset
    if (hx71x->last_error != 0) {
        counts = hx71x->last_error;
    }

    // Add measurement to buffer
    add_sample(hx71x, oid, counts, false);
    hx71x->old_data = counts;
}

// Create a hx71x sensor
void
command_config_hx71x(uint32_t *args)
{
    struct hx71x_adc *hx71x = oid_alloc(args[0]
                , command_config_hx71x, sizeof(*hx71x));
    hx71x->timer.func = hx71x_event;
    uint8_t gain_channel = args[1];
    if (gain_channel < 1 || gain_channel > 4) {
        shutdown("HX71x gain/channel out of range 1-4");
    }
    hx71x->gain_channel = gain_channel;
    hx71x->dout = gpio_in_setup(args[2], 1);
    hx71x->sclk = gpio_out_setup(args[3], 0);
    gpio_out_write(hx71x->sclk, 1); // put chip in power down state
}
DECL_COMMAND(command_config_hx71x, "config_hx71x oid=%c gain_channel=%c"
             " dout_pin=%u sclk_pin=%u");

// start/stop capturing ADC data
void
command_query_hx71x(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    sched_del_timer(&hx71x->timer);
    hx71x->flags = 0;
    hx71x->last_error = 0;
    hx71x->rest_ticks = args[1];
    if (!hx71x->rest_ticks) {
        // End measurements
        gpio_out_write(hx71x->sclk, 1); // put chip in power down state
        return;
    }
    // Start new measurements
    gpio_out_write(hx71x->sclk, 0); // wake chip from power down
    sensor_bulk_reset(&hx71x->sb);
    irq_disable();
    hx71x->timer.waketime = timer_read_time() + hx71x->rest_ticks;
    sched_add_timer(&hx71x->timer);
    irq_enable();
}
DECL_COMMAND(command_query_hx71x, "query_hx71x oid=%c rest_ticks=%u");

void
command_query_hx71x_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = hx71x_is_data_ready(hx71x);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&hx71x->sb, oid, start_t, hx71x->query_ticks,
                        pending_bytes);
}
DECL_COMMAND(command_query_hx71x_status, "query_hx71x_status oid=%c");

// Background task that performs measurements
void
hx71x_capture_task(void)
{
    if (!sched_check_wake(&wake_hx71x))
        return;
    uint8_t oid;
    struct hx71x_adc *hx71x;
    foreach_oid(oid, hx71x, command_config_hx71x) {
        hx71x_read_adc(hx71x, oid);
    }
}
DECL_TASK(hx71x_capture_task);
