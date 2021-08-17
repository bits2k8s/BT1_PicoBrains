#include <stdio.h>
#include "math.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#define RELAY_GPIO_PIN 2

#define CAPTURE_CHANNEL 0
#define CAPTURE_CHANNEL1 1
#define CAPTURE_CHANNEL2 2
#define TOTAL_CHANNELS 3
#define CAPTURE_DEPTH 500 * TOTAL_CHANNELS
#define CHANNEL_DEPTH CAPTURE_DEPTH / TOTAL_CHANNELS
#define CONVERT_FACTOR 3.3f / (float)(1<<12)
uint16_t capture_buf[CAPTURE_DEPTH];
uint32_t relay_state;

void run_adc_with_dma_blocking(dma_channel_config dma_config, uint dma_chan) {
    dma_channel_configure(dma_chan, &dma_config,
        capture_buf,
        &adc_hw->fifo,
        CAPTURE_DEPTH,
        true
    );
    adc_select_input(CAPTURE_CHANNEL); // start round robin at 0
    adc_run(true);

    dma_channel_wait_for_finish_blocking(dma_chan);
    adc_run(false);
    adc_fifo_drain();
}

float get_mean(uint chan)
{
    uint32_t mean = 0;
    for (int i = chan; i < CAPTURE_DEPTH; i+=TOTAL_CHANNELS) {
        // correct adc with 0x04 offset
        if (capture_buf[i] > 4)
            capture_buf[i] -=4;
        else
            capture_buf[i] = 0;
        mean += capture_buf[i];
    }
    return (float)mean / (float)CHANNEL_DEPTH;
}

float get_sigma(uint chan, float fMean) {
    float fSigma = 0.0f;
    for (int i = chan; i < CAPTURE_DEPTH; i+=TOTAL_CHANNELS) {
        fSigma += ((float)capture_buf[i] - fMean) * ((float)capture_buf[i] - fMean);
    }
    fSigma = fSigma / (float)CHANNEL_DEPTH;
    
    fSigma = sqrtf(fSigma);
    return fSigma;
}

void write_captured_data() {
    float fSigma = 0.0f;
    float fMean = 0.0f;
    printf("%x ", relay_state & 0xf);
    for(int i = 0; i < TOTAL_CHANNELS; i++) {
        fMean = get_mean(i);
        //printf("Chan %d  mean %.4f ", i, fMean);
        fSigma = get_sigma(i, fMean);
        printf("%09.4f %09.4f ", fMean, fSigma);
    }
    printf("\n");
}

int main() {
    stdio_init_all();
    relay_state = 0x0;

    for (int gpio = RELAY_GPIO_PIN; gpio < RELAY_GPIO_PIN + 7; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
    }

    adc_gpio_init(26 + CAPTURE_CHANNEL);
    adc_gpio_init(26 + CAPTURE_CHANNEL1);
    adc_gpio_init(26 + CAPTURE_CHANNEL2);
    adc_init();
    adc_set_round_robin(7);
    adc_fifo_setup(
        true,
        true,
        1,
        false,
        false
    );
    adc_set_clkdiv(0);
    uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);

    while(1) {
        int c;
        c = getchar_timeout_us(0);
        if (c >= '0' && c <= '9')
            relay_state = c - '0';
        else if (c >= 'A' && c <= 'F')
            relay_state = 10 + c - 'A';

        run_adc_with_dma_blocking(dma_config, dma_chan);
        write_captured_data();
    }
}