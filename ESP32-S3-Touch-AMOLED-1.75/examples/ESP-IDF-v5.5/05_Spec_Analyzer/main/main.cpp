#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "esp_dsp.h"
#include "bsp_board_extra.h"

#define TAG "synth"

#define N_SAMPLES 1024
#define SAMPLE_RATE 16000
#define CHANNELS 2
#define STRIPE_COUNT 64

#define CANVAS_WIDTH BSP_LCD_H_RES   /* 466 — fill the round AMOLED */
#define CANVAS_HEIGHT BSP_LCD_V_RES  /* 466 */

/* ---- audio + FFT buffers ---- */
__attribute__((aligned(16))) static int16_t raw_data[N_SAMPLES * CHANNELS];
__attribute__((aligned(16))) static int16_t out_data[N_SAMPLES * CHANNELS];
__attribute__((aligned(16))) static float audio_buffer[N_SAMPLES];
__attribute__((aligned(16))) static float wind[N_SAMPLES];
__attribute__((aligned(16))) static float fft_buffer[N_SAMPLES * 2];
__attribute__((aligned(16))) static float spectrum[N_SAMPLES / 2];
static float display_spectrum[STRIPE_COUNT];
static float peak[STRIPE_COUNT];

/* ============================ polyphonic Karplus-Strong ============================ */
#define KS_MAX     1024             /* per-voice delay line */
#define NUM_VOICES 6

typedef struct {
    float buf[KS_MAX];
    int   w;
    float lp;
    float freq;
    float delay;
    int   pluck_n;
    float energy;   /* leaky |output| — used for voice stealing */
} voice_t;

static voice_t   V[NUM_VOICES];
static uint32_t  rng = 0x1234567u;

static inline float frand(void)     /* fast white noise in [-1, 1) */
{
    rng = rng * 1664525u + 1013904223u;
    return (float)(int32_t)rng * (1.0f / 2147483648.0f);
}

/* lock-free pluck queue (touch thread -> audio thread) */
#define PEND_MAX 16
static volatile float    pend_freq[PEND_MAX];
static volatile uint32_t pend_w = 0, pend_r = 0;

static inline void pluck_push(float f)
{
    uint32_t n = (pend_w + 1) & (PEND_MAX - 1);
    if (n != pend_r) { pend_freq[pend_w] = f; pend_w = n; }
}

/* shared params */
static volatile float g_cutoff    = 0.50f;  /* damping LPF coef (bottom-half X) */
static volatile float g_feedback  = 0.97f;  /* loop gain / sustain (bottom-half Y) */
static volatile int   g_finger_top = 0;     /* finger currently in the pitch zone */
static volatile int   g_touch_x   = -1;
static volatile int   g_touch_y   = -1;

#define EXCITE 0.40f                /* how hard the mic drives the held voice */
#define DRY    0.12f                /* a little dry mic so input is always audible */

/* ---- scale / note quantisation (top half) ---- */
#define ROOT_MIDI   48              /* C3 */
#define OCTAVES     3
static const int8_t SCALE[]  = { 0, 2, 4, 7, 9 };   /* major pentatonic */
#define SCALE_LEN   (int)(sizeof(SCALE) / sizeof(SCALE[0]))
#define NUM_NOTES   (SCALE_LEN * OCTAVES)            /* 15 zones across the top */
static volatile int g_active_note = -1;             /* highlighted zone while playing, else -1 */

static inline float note_freq(int idx)
{
    int oct = idx / SCALE_LEN;
    int deg = idx % SCALE_LEN;
    int midi = ROOT_MIDI + oct * 12 + SCALE[deg];
    return 440.0f * powf(2.0f, (float)(midi - 69) / 12.0f);
}

static int steal_voice(void)        /* pick the most-decayed voice */
{
    int best = 0;
    float lo = V[0].energy;
    for (int i = 1; i < NUM_VOICES; i++) {
        if (V[i].energy < lo) { lo = V[i].energy; best = i; }
    }
    return best;
}

/* ============================ audio engine ============================ */
static void audio_engine_task(void *arg)
{
    if (dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE) != ESP_OK) {
        ESP_LOGE(TAG, "FFT init failed");
        vTaskDelete(NULL);
    }
    dsps_wind_hann_f32(wind, N_SAMPLES);

    if (bsp_extra_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "Audio codec init failed");
        vTaskDelete(NULL);
    }
    int vset = 0;
    bsp_extra_codec_volume_set(90, &vset);

    memset(V, 0, sizeof(V));
    for (int i = 0; i < NUM_VOICES; i++) { V[i].freq = 220.0f; V[i].delay = SAMPLE_RATE / 220.0f; }
    int held = -1;
    ESP_LOGI(TAG, "Polyphonic Karplus-Strong ready (%d voices)", NUM_VOICES);

    size_t br = 0, bw = 0;
    while (1) {
        if (bsp_extra_i2s_read(raw_data, sizeof(raw_data), &br, portMAX_DELAY) != ESP_OK) {
            continue;
        }

        /* allocate any queued plucks to voices */
        while (pend_r != pend_w) {
            float f = pend_freq[pend_r];
            pend_r = (pend_r + 1) & (PEND_MAX - 1);
            int v = steal_voice();
            float d = (float)SAMPLE_RATE / f;
            if (d < 2.0f) d = 2.0f; else if (d > (KS_MAX - 2)) d = KS_MAX - 2;
            V[v].freq = f;
            V[v].delay = d;
            V[v].pluck_n = (int)d;
            V[v].energy = 1.0f;     /* mark busy so it isn't immediately stolen */
            held = v;
        }
        bool finger_top = g_finger_top;
        float g   = g_feedback;
        float cut = g_cutoff;

        for (int i = 0; i < N_SAMPLES; i++) {
            float l = raw_data[i * CHANNELS]     / 32768.0f;
            float r = raw_data[i * CHANNELS + 1] / 32768.0f;
            float mic = 0.5f * (l + r);

            float sum = 0.0f;
            for (int v = 0; v < NUM_VOICES; v++) {
                voice_t *vc = &V[v];

                float rpos = (float)vc->w - vc->delay;
                while (rpos < 0.0f) rpos += KS_MAX;
                int i0 = (int)rpos;
                float frac = rpos - (float)i0;
                int i1 = i0 + 1; if (i1 >= KS_MAX) i1 -= KS_MAX;
                float d = vc->buf[i0] * (1.0f - frac) + vc->buf[i1] * frac;

                vc->lp += cut * (d - vc->lp);

                float exc = 0.0f;
                if (finger_top && v == held) exc += EXCITE * mic;
                if (vc->pluck_n > 0) { exc += 0.9f * frand(); vc->pluck_n--; }

                float nv = exc + g * vc->lp;
                if (nv > 1.2f) nv = 1.2f; else if (nv < -1.2f) nv = -1.2f;
                vc->buf[vc->w] = nv;
                vc->w++; if (vc->w >= KS_MAX) vc->w = 0;

                float o = vc->lp;
                vc->energy += 0.001f * (fabsf(o) - vc->energy);
                sum += o;
            }

            float y = sum * 0.6f + DRY * mic;
            if (y > 1.0f) y = 1.0f; else if (y < -1.0f) y = -1.0f;
            int16_t s = (int16_t)(y * 32767.0f);
            out_data[i * CHANNELS]     = s;
            out_data[i * CHANNELS + 1] = s;
            audio_buffer[i] = y;
        }
        bsp_extra_i2s_write(out_data, sizeof(out_data), &bw, portMAX_DELAY);

        /* FFT of the output signal */
        dsps_mul_f32(audio_buffer, wind, audio_buffer, N_SAMPLES, 1, 1, 1);
        for (int i = 0; i < N_SAMPLES; i++) {
            fft_buffer[2 * i]     = audio_buffer[i];
            fft_buffer[2 * i + 1] = 0;
        }
        dsps_fft2r_fc32(fft_buffer, N_SAMPLES);
        dsps_bit_rev_fc32(fft_buffer, N_SAMPLES);
        for (int i = 0; i < N_SAMPLES / 2; i++) {
            float re = fft_buffer[2 * i];
            float im = fft_buffer[2 * i + 1];
            float mag = sqrtf(re * re + im * im);
            spectrum[i] = 20 * log10f(mag / (N_SAMPLES / 2) + 1e-9f);
        }
        for (int i = 0; i < STRIPE_COUNT; i++) {
            int fft_idx = i * (N_SAMPLES / 2) / STRIPE_COUNT;
            display_spectrum[i] = fmaxf(-90.0f, fminf(0.0f, spectrum[fft_idx]));
        }
    }
}

/* ============================ display + touch ============================ */
static void timer_cb(lv_timer_t *timer)
{
    lv_obj_t *canvas = (lv_obj_t *)lv_timer_get_user_data(timer);
    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);

    const int center_y = CANVAS_HEIGHT / 2;

    /* ---- poll the touchscreen and map to synth params ---- */
    static bool prev_pressed = false;
    static bool prev_top = false;
    lv_indev_t *indev = bsp_display_get_input_dev();
    bool pressed = false;
    bool in_top = false;
    if (indev && lv_indev_get_state(indev) == LV_INDEV_STATE_PRESSED) {
        lv_point_t p;
        lv_indev_get_point(indev, &p);
        int x = p.x, y = p.y;
        if (x < 0) x = 0; else if (x >= CANVAS_WIDTH)  x = CANVAS_WIDTH - 1;
        if (y < 0) y = 0; else if (y >= CANVAS_HEIGHT) y = CANVAS_HEIGHT - 1;
        g_touch_x = x;
        g_touch_y = y;
        pressed = true;
        float tx = (float)x / CANVAS_WIDTH;   /* 0..1 */

        if (y < center_y) {
            /* TOP: scale-quantised pitch (pentatonic, 3 octaves) + pluck per note */
            in_top = true;
            int idx = (int)(tx * NUM_NOTES);
            if (idx < 0) idx = 0; else if (idx >= NUM_NOTES) idx = NUM_NOTES - 1;
            if (!prev_pressed || !prev_top || idx != g_active_note) {
                pluck_push(note_freq(idx));   /* fresh touch or new note zone -> pluck a voice */
            }
            g_active_note = idx;
        } else {
            /* BOTTOM: filter. X = brightness, Y = sustain */
            float by = (float)(y - center_y) / (float)(CANVAS_HEIGHT - center_y); /* 0..1 */
            g_cutoff   = 0.05f + tx * 0.90f;
            g_feedback = 0.90f + by * 0.099f;   /* 0.90 .. 0.999 */
            g_active_note = -1;
        }
    } else {
        g_touch_x = -1;
        g_touch_y = -1;
        g_active_note = -1;
    }
    g_finger_top = in_top ? 1 : 0;
    prev_pressed = pressed;
    prev_top = in_top;

    /* ---- highlight the active note zone (drawn under the bars) ---- */
    if (g_active_note >= 0) {
        int x0 = g_active_note * CANVAS_WIDTH / NUM_NOTES;
        int x1 = (g_active_note + 1) * CANVAS_WIDTH / NUM_NOTES;
        lv_draw_rect_dsc_t zb;
        lv_draw_rect_dsc_init(&zb);
        zb.bg_color = lv_color_white();
        zb.bg_opa = LV_OPA_20;
        lv_area_t za = { x0, 0, x1 - 1, center_y - 1 };
        lv_draw_rect(&layer, &zb, &za);
    }

    /* ---- spectrum bars (output signal) ---- */
    const int stripe_width = CANVAS_WIDTH / STRIPE_COUNT;
    const int bar_gap_px = 2;

    for (int i = 0; i < STRIPE_COUNT; i++) {
        float db = display_spectrum[i];
        float db_min = -90.0f, db_max = 0.0f;
        float norm = (db - db_min) / (db_max - db_min);
        norm = fmaxf(0.0f, fminf(1.0f, norm));
        norm = sqrtf(norm);

        int bar_height = (int)(norm * (CANVAS_HEIGHT / 2));

        if (peak[i] < bar_height) {
            peak[i] = bar_height;
        } else {
            peak[i] -= 2;
            if (peak[i] < 0) peak[i] = 0;
        }

        float hue_step = 270.0f / STRIPE_COUNT;
        uint16_t hue = (uint16_t)(i * hue_step);
        lv_color_t color = lv_color_hsv_to_rgb(hue, 100, 100);

        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = color;
        rect_dsc.bg_opa = LV_OPA_COVER;

        int x_start = i * stripe_width + bar_gap_px / 2;
        int x_end   = (i + 1) * stripe_width - bar_gap_px / 2 - 1;

        lv_area_t bar_area = { x_start, center_y - bar_height, x_end, center_y + bar_height };
        lv_draw_rect(&layer, &rect_dsc, &bar_area);

        int peak_y_top = center_y - (int)peak[i] - 2;
        int peak_y_bot = center_y + (int)peak[i];
        lv_area_t particle_area_top = { x_start, peak_y_top, x_end, peak_y_top + 2 };
        lv_draw_rect(&layer, &rect_dsc, &particle_area_top);
        lv_area_t particle_area_bot = { x_start, peak_y_bot, x_end, peak_y_bot + 2 };
        lv_draw_rect(&layer, &rect_dsc, &particle_area_bot);
    }

    /* ---- note-zone boundaries on the top half (octave lines brighter) ---- */
    for (int k = 0; k <= NUM_NOTES; k++) {
        int x = k * CANVAS_WIDTH / NUM_NOTES;
        if (x >= CANVAS_WIDTH) x = CANVAS_WIDTH - 1;
        bool octave = (k % SCALE_LEN) == 0;
        lv_draw_rect_dsc_t tl;
        lv_draw_rect_dsc_init(&tl);
        tl.bg_color = lv_color_hex(octave ? 0x909090 : 0x383838);
        tl.bg_opa = octave ? LV_OPA_70 : LV_OPA_40;
        lv_area_t la = { x, 0, x + (octave ? 1 : 0), center_y - 1 };
        lv_draw_rect(&layer, &tl, &la);
    }

    /* ---- center divider (top = pitch, bottom = filter) ---- */
    lv_draw_rect_dsc_t line_dsc;
    lv_draw_rect_dsc_init(&line_dsc);
    line_dsc.bg_color = lv_color_hex(0x404040);
    line_dsc.bg_opa = LV_OPA_50;
    lv_area_t divider = { 0, center_y - 1, CANVAS_WIDTH - 1, center_y };
    lv_draw_rect(&layer, &line_dsc, &divider);

    /* ---- finger marker ---- */
    if (g_touch_x >= 0) {
        lv_draw_rect_dsc_t m;
        lv_draw_rect_dsc_init(&m);
        m.bg_color = lv_color_white();
        m.bg_opa = LV_OPA_COVER;
        m.radius = LV_RADIUS_CIRCLE;
        const int rr = 10;
        lv_area_t ma = { g_touch_x - rr, g_touch_y - rr, g_touch_x + rr, g_touch_y + rr };
        lv_draw_rect(&layer, &m, &ma);
    }

    lv_canvas_finish_layer(canvas, &layer);
}

static void build_ui(void)
{
    /* Black out the whole screen so there's no white border around the canvas */
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* Full-screen 466x466 RGB565 canvas (~434 KB) — allocated in PSRAM by lv_draw_buf_create */
    lv_draw_buf_t *draw_buf = lv_draw_buf_create(CANVAS_WIDTH, CANVAS_HEIGHT, LV_COLOR_FORMAT_RGB565, 0);
    if (!draw_buf) {
        ESP_LOGE(TAG, "Failed to allocate canvas draw buffer");
        return;
    }

    lv_obj_t *canvas = lv_canvas_create(scr);
    lv_canvas_set_draw_buf(canvas, draw_buf);
    lv_obj_set_size(canvas, CANVAS_WIDTH, CANVAS_HEIGHT);
    lv_obj_center(canvas);

    lv_timer_create(timer_cb, 33, canvas);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting espSynth — polyphonic Karplus-Strong resonator");

    lv_display_t *disp = bsp_display_start();
    if (disp) {
        bsp_display_backlight_on();
    }

    bsp_display_lock(-1);
    build_ui();
    bsp_display_unlock();

    xTaskCreate(audio_engine_task, "audio_engine", 8 * 1024, NULL, 5, NULL);
}
