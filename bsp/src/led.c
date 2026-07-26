//
// Created by fish on 2025/9/26.
//

#include "bsp/led.h"

#include "math.h"
#include "tim.h"

#define WS2812_HIGH 168
#define WS2812_LOW  84

enum {
    WS2812_DATA_WORDS = 24,
    WS2812_RESET_WORDS = 48,
    WS2812_DMA_WORDS = WS2812_DATA_WORDS + WS2812_RESET_WORDS,
    DCACHE_LINE_WORDS = 32 / sizeof(uint16_t),
    WS2812_BUFFER_WORDS =
        (WS2812_DMA_WORDS + DCACHE_LINE_WORDS - 1) / DCACHE_LINE_WORDS * DCACHE_LINE_WORDS,
};

__ALIGNED(32) static uint16_t buf[WS2812_BUFFER_WORDS];

static void clean_dma_buffer() {
    SCB_CleanDCache_by_Addr((uint32_t *) buf, (int32_t) sizeof(buf));
}

bsp_status_t bsp_led_init() {
    for (uint8_t i = 0; i < WS2812_BUFFER_WORDS; i++) buf[i] = 0;
    clean_dma_buffer();
    return bsp_status_from_hal(HAL_TIMEx_PWMN_Start_DMA(
        &htim8, TIM_CHANNEL_1, (uint32_t *) buf, WS2812_DMA_WORDS
    ));
}

void bsp_led_set(uint8_t r, uint8_t g, uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
        buf[7 - i]  = ((g >> i) & 1) ? WS2812_HIGH : WS2812_LOW;
        buf[15 - i] = ((r >> i) & 1) ? WS2812_HIGH : WS2812_LOW;
        buf[23 - i] = ((b >> i) & 1) ? WS2812_HIGH : WS2812_LOW;
    }
    clean_dma_buffer();
}

void bsp_led_set_hsv(float h, float s, float v) {
    if (!isfinite(h) || !isfinite(s) || !isfinite(v)) return;
    h = fmodf(fmodf(h, 1.0f) + 1.0f, 1.0f);
    s = s < 0 ? 0 : s > 1 ? 1 : s;
    v = v < 0 ? 0 : v > 1 ? 1 : v;
    h = fmodf(h, 1.0f) * 6.0f;

    float f = h - floorf(h), p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));

    float r,g,b;
    switch((int) (floorf(h)) % 6){
        case 0: r=v; g=t; b=p; break;
        case 1: r=q; g=v; b=p; break;
        case 2: r=p; g=v; b=t; break;
        case 3: r=p; g=q; b=v; break;
        case 4: r=t; g=p; b=v; break;
        default:r=v; g=p; b=q; break;
    }

    bsp_led_set((uint8_t) (r*255+0.5f), (uint8_t) (g*255+0.5f), (uint8_t) (b*255+0.5f));
}
