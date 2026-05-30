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

#define CANVAS_WIDTH  BSP_LCD_H_RES   /* 466 */
#define CANVAS_HEIGHT BSP_LCD_V_RES   /* 466 */

#define NAV_H        44               /* reserved bottom strip: swipe-up handle */
#define SWIPE_THRESH 45               /* px of vertical drag to open/close menu */

/* ---- audio I/O buffers ---- */
__attribute__((aligned(16))) static int16_t raw_data[N_SAMPLES * CHANNELS];
__attribute__((aligned(16))) static int16_t out_data[N_SAMPLES * CHANNELS];

/* ---- FFT spectrum (Keys+Spectrum mode) ---- */
#define STRIPE_COUNT 64
__attribute__((aligned(16))) static float audio_buffer[N_SAMPLES];
__attribute__((aligned(16))) static float wind[N_SAMPLES];
__attribute__((aligned(16))) static float fft_buffer[N_SAMPLES * 2];
__attribute__((aligned(16))) static float spectrum[N_SAMPLES / 2];
static float display_spectrum[STRIPE_COUNT];
static float peak[STRIPE_COUNT];

/* ---- scale strip (Keys+Spectrum mode) ---- */
static const int8_t SCALE[] = { 0, 2, 4, 7, 9 };   /* major pentatonic */
#define SCALE_LEN  (int)(sizeof(SCALE) / sizeof(SCALE[0]))
#define XY_OCTAVES 3
#define XY_ROOT    48                               /* C3 */
#define XY_NOTES   (SCALE_LEN * XY_OCTAVES)         /* 15 zones */
static volatile int g_active_note = -1;
static inline float xy_note_freq(int idx)
{
    int oct = idx / SCALE_LEN, deg = idx % SCALE_LEN;
    int midi = XY_ROOT + oct * 12 + SCALE[deg];
    return 440.0f * powf(2.0f, (float)(midi - 69) / 12.0f);
}

/* ============================ polyphonic Karplus-Strong ============================ */
#define KS_MAX     1024
#define NUM_VOICES 6

typedef struct {
    float buf[KS_MAX];
    int   w;
    float lp;
    float freq;
    float delay;
    int   pluck_n;
    float energy;
    float phase;    /* synth osc phase 0..1 */
    float modphase; /* FM/AM/RM modulator phase */
    float env;      /* synth AR envelope 0..1 */
} voice_t;

static voice_t  V[NUM_VOICES];
static uint32_t rng = 0x1234567u;

/* ---- saw wavetable synth (raw osc + AR envelope) ---- */
#define SAW_SIZE 1024
static float saw_table[SAW_SIZE];
static void build_saw(void) { for (int i = 0; i < SAW_SIZE; i++) saw_table[i] = -1.0f + 2.0f * (float)i / (float)SAW_SIZE; }
static inline float saw_lerp(float ph)
{
    float pos = ph * SAW_SIZE;
    int i0 = (int)pos & (SAW_SIZE - 1);
    int i1 = (i0 + 1) & (SAW_SIZE - 1);
    float fr = pos - (float)(int)pos;
    return saw_table[i0] + fr * (saw_table[i1] - saw_table[i0]);
}
static volatile float g_attack    = 8.0f;    /* ms  */
static volatile float g_release   = 280.0f;  /* ms  */
static volatile float g_syn_level = 0.55f;   /* 0..1 */
static volatile float g_syn_cut   = 0.65f;   /* base LPF cutoff (norm) -> "Cutoff" */
static volatile float g_reso      = 0.15f;   /* LPF resonance 0..1     -> "Resonance" */
static volatile float g_cut_live  = 0.60f;   /* Synth-mode Y cutoff (live) */

/* ---- wavetable warp / waveshaping (ported from SerumLikeOsc::readWarped) ---- */
enum { W_NONE, W_SYNC, W_BEND_P, W_BEND_M, W_BEND_PM, W_PWM,
       W_ASYM_P, W_ASYM_M, W_ASYM_PM, W_FLIP, W_MIRROR, W_QUANT, W_FM, W_AM, W_RM, N_WARP };
static const char *WARP_NAMES[N_WARP] = {
    "None","Sync","Bend+","Bend-","Bend+-","PWM","Asym+","Asym-","Asym+-",
    "Flip","Mirror","Quant","FM","AM","RM"
};
static volatile float g_warpmode = 0.0f;   /* stepped 0..N_WARP-1 */
static volatile float g_warp_base = 0.0f;  /* menu base warp 0..1 */
static volatile float g_warp_live = 0.0f;  /* live morph from Synth-mode Y */

/* ---- IMU (QMI8658) motion modulation ---- */
static volatile float g_imu_x = 0.0f;      /* tilt L/R  -1..1 */
static volatile float g_imu_y = 0.0f;      /* tilt F/B  -1..1 */
static volatile float g_bend  = 1.0f;      /* pitch-bend ratio from tilt-X */
static volatile float g_imu_amt = 0.0f;    /* motion-mod master depth 0..1 */
static volatile float g_imu_pitch = 1.0f;  /* tilt-X -> pitch amount */
static volatile float g_imu_filt  = 1.0f;  /* tilt-Y -> filter/brightness amount */

static inline float w_clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
static inline float w_lerp(float a, float b, float t) { return a + (b - a) * t; }
static inline float w_bias(float b, float x) {
    b = b < 0.0001f ? 0.0001f : (b > 0.9999f ? 0.9999f : b);
    x = w_clamp01(x);
    return x / ((((1.0f / b) - 2.0f) * (1.0f - x)) + 1.0f);
}
static inline float w_piece(float x, float mid) {
    x = w_clamp01(x); mid = mid < 0.0001f ? 0.0001f : (mid > 0.9999f ? 0.9999f : mid);
    return (x < mid) ? 0.5f * x / mid : 0.5f + 0.5f * ((x - mid) / (1.0f - mid));
}
static float osc_warp(float ph, float warp, float ext, int mode)
{
    warp = w_clamp01(warp);
    switch (mode) {
        case W_SYNC:    return saw_lerp(ph * w_lerp(1.0f, 16.0f, warp * warp * warp));
        case W_BEND_P:  { float sd = ph > 0.5f ? 0.5f : -0.5f; return saw_lerp(0.5f + sd * w_bias(0.5f + 0.4f * warp, fabsf(ph - 0.5f) * 2.0f)); }
        case W_BEND_M:  { float sd = ph > 0.5f ? 0.5f : -0.5f; return saw_lerp(0.5f + sd * w_bias(0.5f - 0.4f * warp, fabsf(ph - 0.5f) * 2.0f)); }
        case W_BEND_PM: { float sd = ph > 0.5f ? 0.5f : -0.5f; return saw_lerp(0.5f + sd * w_bias(w_lerp(0.9f, 0.1f, warp), fabsf(ph - 0.5f) * 2.0f)); }
        case W_PWM:     { float wd = 1.0f - 0.9999f * warp; float p = ph / wd; if (p > 1) p = 1; return saw_lerp(p); }
        case W_ASYM_P:  return saw_lerp(w_piece(ph, 0.5f - warp * 0.375f));
        case W_ASYM_M:  return saw_lerp(w_piece(ph, 0.5f + warp * 0.375f));
        case W_ASYM_PM: return saw_lerp(w_piece(ph, 0.5f + w_lerp(-1.0f, 1.0f, warp) * 0.375f));
        case W_FLIP:    { float s = saw_lerp(ph); bool f = ((ph - warp * 2.0f) < 0.0f) && ((ph - warp * 2.0f) > -1.0f); return f ? -s : s; }
        case W_MIRROR:  { float p2 = 1.0f - 2.0f * fabsf(ph - 0.5f); return saw_lerp(w_piece(p2, 0.5f + w_lerp(-1.0f, 1.0f, warp) * 0.375f)); }
        case W_QUANT:   { float steps = w_lerp(2.0f, 64.0f, 1.0f - warp); float q = floorf(ph * steps) / steps; return saw_lerp(q); }
        case W_FM:      return saw_lerp(ph + warp * ext);
        case W_AM:      return saw_lerp(ph) * w_lerp(1.0f, fabsf(ext), warp);
        case W_RM:      return saw_lerp(ph) * w_lerp(1.0f, ext, warp);
        default:        return saw_lerp(ph);
    }
}

static inline float frand(void)
{
    rng = rng * 1664525u + 1013904223u;
    return (float)(int32_t)rng * (1.0f / 2147483648.0f);
}

/* lock-free chord queue (touch thread -> audio thread): up to 3 notes per gesture */
#define PEND_MAX 16
typedef struct { uint8_t n; float f[3]; } chord_t;
static volatile chord_t  pend[PEND_MAX];
static volatile uint32_t pend_w = 0, pend_r = 0;

static inline void chord_push(int n, const float *f)
{
    uint32_t nx = (pend_w + 1) & (PEND_MAX - 1);
    if (nx == pend_r) return;
    pend[pend_w].n = (uint8_t)n;
    for (int i = 0; i < n; i++) pend[pend_w].f[i] = f[i];
    pend_w = nx;
}

/* shared params (editable in the menu) */
static volatile float g_cutoff     = 0.55f;  /* damping LPF coef -> "Brightness" */
static volatile float g_feedback   = 0.985f; /* loop gain        -> "Sustain"    */
static volatile float g_excite     = 0.45f;  /* mic drive        -> "Mic Drive"  */
static volatile float g_pluck      = 0.35f;  /* pluck attack amt -> "Pluck" (0 = mic only) */
static volatile float g_breath     = 0.0f;   /* continuous shaped-noise breath -> "Breath" (blown sustain) */
static volatile float g_volume     = 90.0f;  /* codec out vol    -> "Volume"     */
static volatile int   g_finger_down = 0;     /* finger playing -> mic excites held voices */
static volatile int   g_touch_x    = -1;
static volatile int   g_touch_y    = -1;

#define DRY    0.10f

/* ---- output FX: delay + flanger (page 2) ---- */
static volatile float g_dly_time = 250.0f;  /* ms */
static volatile float g_dly_fb   = 0.35f;   /* 0..0.9 */
static volatile float g_dly_mix  = 0.0f;    /* 0..1 (0 = off) */
static volatile float g_flg_amt  = 0.0f;    /* 0..1 flanger depth+mix (0 = off) */

#define DLY_MAX 16000               /* 1.0 s @ 16 kHz */
#define FLG_MAX 512                 /* ~32 ms */
static float dly_buf[DLY_MAX];
static int   dly_w = 0;
static float flg_buf[FLG_MAX];
static int   flg_w = 0;
static float flg_phase = 0.0f;

/* ---- play modes ---- */
enum { MODE_TONNETZ, MODE_CHORD, MODE_XY, MODE_SYNTH, MODE_OCARINA, N_MODES };
static const char *MODE_NAMES[] = { "Tonnetz", "Chords", "Keys+Spectrum", "Synth", "Ocarina" };
static volatile int g_mode = MODE_TONNETZ;
/* Synth + Ocarina use the synth (saw+env) engine; others use Karplus */
static inline bool synth_engine(void) { return g_mode == MODE_SYNTH || g_mode == MODE_OCARINA; }

static int steal_voice(void)
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
    if (bsp_extra_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "Audio codec init failed");
        vTaskDelete(NULL);
    }
    int vset = 0;
    bsp_extra_codec_volume_set(90, &vset);

    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    dsps_wind_hann_f32(wind, N_SAMPLES);

    memset(V, 0, sizeof(V));
    for (int i = 0; i < NUM_VOICES; i++) { V[i].freq = 220.0f; V[i].delay = SAMPLE_RATE / 220.0f; }
    int  held[3] = { -1, -1, -1 };
    int  n_held = 0;
    ESP_LOGI(TAG, "Polyphonic Karplus-Strong + Tonnetz ready (%d voices)", NUM_VOICES);

    size_t br = 0, bw = 0;
    while (1) {
        if (bsp_extra_i2s_read(raw_data, sizeof(raw_data), &br, portMAX_DELAY) != ESP_OK) {
            continue;
        }

        /* drain queued chords -> allocate voices, remember them as the held set */
        while (pend_r != pend_w) {
            chord_t c = { pend[pend_r].n, { pend[pend_r].f[0], pend[pend_r].f[1], pend[pend_r].f[2] } };
            pend_r = (pend_r + 1) & (PEND_MAX - 1);
            n_held = c.n;
            for (int j = 0; j < c.n; j++) {
                int v = steal_voice();
                float d = (float)SAMPLE_RATE / c.f[j];
                if (d < 2.0f) d = 2.0f; else if (d > (KS_MAX - 2)) d = KS_MAX - 2;
                V[v].freq = c.f[j];
                V[v].delay = d;
                V[v].pluck_n = (int)d;
                V[v].energy = 1.0f;
                held[j] = v;
            }
        }
        float imy  = g_imu_y * g_imu_amt;                            /* tilt F/B -> timbre */
        float bend = 1.0f + (g_bend - 1.0f) * g_imu_amt * g_imu_pitch; /* tilt L/R -> pitch */
        static float svf_ic1 = 0.0f, svf_ic2 = 0.0f;                 /* SVF state */
        float g    = g_feedback;
        float cut  = w_clamp01(g_cutoff + imy * 0.30f);
        float drive = g_excite;
        float pluck_amp = g_pluck;
        float breath = g_breath;
        /* mic transient detector -> impulse excitation (persists across blocks) */
        static float env_f = 0.0f, env_s = 0.0f, imp_amp = 0.0f;
        static int   gate_refr = 0, imp_n = 0;
        static float noise_lp = 0.0f;   /* shaped (low-passed) excitation noise */

        /* synth engine: AR envelope + per-voice gate (note held && finger down) */
        bool  synth  = synth_engine();
        bool  fdown  = g_finger_down;
        float atk    = 1.0f - expf(-1.0f / (fmaxf(0.5f, g_attack)  * 0.001f * SAMPLE_RATE));
        float rel    = 1.0f - expf(-1.0f / (fmaxf(1.0f, g_release) * 0.001f * SAMPLE_RATE));
        float slevel = g_syn_level;
        /* synth resonant low-pass (TPT SVF) coefficients — cutoff from menu/Synth-Y + motion */
        float base_cut = (g_mode == MODE_SYNTH && fdown) ? g_cut_live : g_syn_cut;
        float cnorm = base_cut + imy * g_imu_filt * 0.5f;
        if (cnorm < 0) cnorm = 0; else if (cnorm > 1) cnorm = 1;
        float fc = 80.0f * powf(2.0f, cnorm * 6.4f);
        if (fc > SAMPLE_RATE * 0.45f) fc = SAMPLE_RATE * 0.45f;
        float svf_g = tanf(3.14159265f * fc / SAMPLE_RATE);
        float svf_k = 1.0f / (0.5f + g_reso * 9.5f);
        float svf_a1 = 1.0f / (1.0f + svf_g * (svf_g + svf_k));
        float svf_a2 = svf_g * svf_a1;
        float svf_a3 = svf_g * svf_a2;
        bool  vgate[NUM_VOICES];
        for (int v = 0; v < NUM_VOICES; v++) {
            bool h = false;
            for (int j = 0; j < n_held; j++) if (held[j] == v) { h = true; break; }
            vgate[v] = h && fdown;
        }
        float dly_t_ms = g_dly_time;
        float dly_fb   = g_dly_fb;
        float dly_mix  = g_dly_mix;
        float flg_amt  = g_flg_amt;
        const float flg_inc = 0.35f / SAMPLE_RATE;   /* ~0.35 Hz flanger LFO */

        static int last_vol = -1;
        int vol = (int)g_volume;
        if (vol != last_vol) { bsp_extra_codec_volume_set(vol, &vset); last_vol = vol; }

        for (int i = 0; i < N_SAMPLES; i++) {
            float l = raw_data[i * CHANNELS]     / 32768.0f;
            float r = raw_data[i * CHANNELS + 1] / 32768.0f;
            float mic = 0.5f * (l + r);

            /* onset detection: fast peak vs slow baseline -> fire a short impulse */
            float amic = fabsf(mic);
            env_f = (amic > env_f) ? amic : env_f * 0.55f;
            env_s += 0.0006f * (amic - env_s);
            if (gate_refr > 0) gate_refr--;
            if (gate_refr == 0 && amic > 0.04f && env_f > env_s * 3.5f + 0.03f) {
                imp_n = 80; imp_amp = drive * 1.3f; gate_refr = 1200;   /* ~75 ms refractory */
            }

            /* shaped excitation noise: brightness controls how airy the pluck/breath is */
            float ncoef = 0.05f + cut * 0.55f;
            noise_lp += ncoef * (frand() - noise_lp);

            float sum = 0.0f;
            for (int v = 0; v < NUM_VOICES; v++) {
                voice_t *vc = &V[v];
                float o;

                if (synth) {
                    /* ---- saw oscillator + AR envelope (resonant LPF on the sum) ---- */
                    vc->phase += vc->freq * bend / SAMPLE_RATE;
                    if (vc->phase >= 1.0f) vc->phase -= 1.0f;
                    float s = saw_lerp(vc->phase);
                    float tgt = vgate[v] ? 1.0f : 0.0f;
                    float c   = vgate[v] ? atk : rel;
                    vc->env += c * (tgt - vc->env);
                    o = s * vc->env * slevel;
                } else {
                    /* ---- Karplus-Strong resonator ---- */
                    float rpos = (float)vc->w - vc->delay / bend;
                    while (rpos < 0.0f) rpos += KS_MAX;
                    int i0 = (int)rpos;
                    float frac = rpos - (float)i0;
                    int i1 = i0 + 1; if (i1 >= KS_MAX) i1 -= KS_MAX;
                    float d = vc->buf[i0] * (1.0f - frac) + vc->buf[i1] * frac;

                    vc->lp += cut * (d - vc->lp);

                    float exc = 0.0f;
                    if (vc->pluck_n > 0) { exc += pluck_amp * noise_lp; vc->pluck_n--; }   /* shaped pluck */
                    if (breath > 0.001f && vgate[v]) exc += breath * noise_lp;             /* blown sustain while held */
                    if (imp_n > 0) {   /* mic-transient impulse excites the held note(s) */
                        for (int j = 0; j < n_held; j++) if (held[j] == v) { exc += imp_amp * frand(); break; }
                    }

                    float nv = exc + g * vc->lp;
                    if (nv > 1.2f) nv = 1.2f; else if (nv < -1.2f) nv = -1.2f;
                    vc->buf[vc->w] = nv;
                    vc->w++; if (vc->w >= KS_MAX) vc->w = 0;
                    o = vc->lp;
                }

                vc->energy += 0.001f * (fabsf(o) - vc->energy);
                sum += o;
            }

            if (imp_n > 0) imp_n--;

            if (synth) {   /* resonant low-pass on the synth mix */
                float v3 = sum - svf_ic2;
                float v1 = svf_a1 * svf_ic1 + svf_a2 * v3;
                float v2 = svf_ic2 + svf_a2 * svf_ic1 + svf_a3 * v3;
                svf_ic1 = 2.0f * v1 - svf_ic1;
                svf_ic2 = 2.0f * v2 - svf_ic2;
                sum = v2;
            }

            float y = sum * 0.6f + DRY * mic;

            /* --- delay FX --- */
            int dsamp = (int)(dly_t_ms * (SAMPLE_RATE / 1000.0f));
            if (dsamp < 1) dsamp = 1; else if (dsamp > DLY_MAX - 1) dsamp = DLY_MAX - 1;
            int dr = dly_w - dsamp; if (dr < 0) dr += DLY_MAX;
            float dly_out = dly_buf[dr];
            dly_buf[dly_w] = y + dly_fb * dly_out;
            dly_w++; if (dly_w >= DLY_MAX) dly_w = 0;
            y += dly_mix * dly_out;

            /* --- flanger FX (modulated short delay) --- */
            if (flg_amt > 0.001f) {
                flg_phase += flg_inc; if (flg_phase >= 1.0f) flg_phase -= 1.0f;
                float lfo = 0.5f * (1.0f - cosf(6.2831853f * flg_phase)); /* 0..1 */
                float fd = 2.0f + lfo * 110.0f;                          /* ~0.1..7 ms */
                float fr = (float)flg_w - fd; while (fr < 0.0f) fr += FLG_MAX;
                int f0 = (int)fr; float ff = fr - (float)f0; int f1 = f0 + 1; if (f1 >= FLG_MAX) f1 -= FLG_MAX;
                float fdel = flg_buf[f0] * (1.0f - ff) + flg_buf[f1] * ff;
                flg_buf[flg_w] = y + 0.5f * flg_amt * fdel;
                flg_w++; if (flg_w >= FLG_MAX) flg_w = 0;
                y += flg_amt * fdel;
            }

            if (y > 1.0f) y = 1.0f; else if (y < -1.0f) y = -1.0f;
            int16_t s = (int16_t)(y * 32767.0f);
            out_data[i * CHANNELS]     = s;
            out_data[i * CHANNELS + 1] = s;
            audio_buffer[i] = y;
        }
        bsp_extra_i2s_write(out_data, sizeof(out_data), &bw, portMAX_DELAY);

        /* spectrum of the output (Keys+Spectrum and Synth modes show it) */
        if (g_mode == MODE_XY || g_mode == MODE_SYNTH) {
            dsps_mul_f32(audio_buffer, wind, audio_buffer, N_SAMPLES, 1, 1, 1);
            for (int i = 0; i < N_SAMPLES; i++) { fft_buffer[2 * i] = audio_buffer[i]; fft_buffer[2 * i + 1] = 0; }
            dsps_fft2r_fc32(fft_buffer, N_SAMPLES);
            dsps_bit_rev_fc32(fft_buffer, N_SAMPLES);
            for (int i = 0; i < N_SAMPLES / 2; i++) {
                float re = fft_buffer[2 * i], im = fft_buffer[2 * i + 1];
                float mag = sqrtf(re * re + im * im);
                spectrum[i] = 20 * log10f(mag / (N_SAMPLES / 2) + 1e-9f);
            }
            for (int i = 0; i < STRIPE_COUNT; i++) {
                int fi = i * (N_SAMPLES / 2) / STRIPE_COUNT;
                display_spectrum[i] = fmaxf(-90.0f, fminf(0.0f, spectrum[fi]));
            }
        }
    }
}

/* ============================ Tonnetz hex grid ============================ */
#define HEX_R       52.0f           /* hex circumradius (centre -> vertex) */
#define CHORD_R     (HEX_R * 0.72f)  /* generous catch radius around a vertex for chords */
#define ROOT_MIDI   60              /* C4 at grid centre */
#define MAX_HEX     64
#define MAX_VTX     256

typedef struct { float x, y; int note; float freq; } hex_t;
static hex_t HEX[MAX_HEX];
static int   n_hex = 0;

/* chord vertices = points where 3 mutually-adjacent hexes meet */
typedef struct { float x, y; int cnt; int h[3]; } vtx_t;
static vtx_t VTX[MAX_VTX];
static int   n_vtx = 0;

static inline float midi_freq(int note)
{
    return 440.0f * powf(2.0f, (float)(note - 69) / 12.0f);
}

static void build_hex_grid(void)
{
    const float cx = CANVAS_WIDTH * 0.5f;
    const float cy = CANVAS_HEIGHT * 0.5f;
    const float sqrt3 = 1.7320508f;
    n_hex = 0;
    for (int r = -3; r <= 3; r++) {
        for (int q = -3; q <= 3; q++) {
            float x = cx + HEX_R * sqrt3 * ((float)q + (float)r * 0.5f);
            float y = cy + HEX_R * 1.5f * (float)r;
            if (x < HEX_R * 0.6f || x > CANVAS_WIDTH - HEX_R * 0.6f) continue;
            if (y < HEX_R * 0.6f || y > CANVAS_HEIGHT - NAV_H - HEX_R * 0.4f) continue;
            if (n_hex >= MAX_HEX) break;
            int note = ROOT_MIDI + 7 * q + 4 * r;        /* Tonnetz: +5th along q, +maj3rd along r */
            HEX[n_hex].x = x;
            HEX[n_hex].y = y;
            HEX[n_hex].note = note;
            HEX[n_hex].freq = midi_freq(note);
            n_hex++;
        }
    }
    ESP_LOGI(TAG, "Tonnetz grid: %d hexes", n_hex);
}

static inline void hex_vertex(float cx, float cy, int k, float *vx, float *vy)
{
    float a = (3.14159265f / 180.0f) * (60.0f * (float)k - 30.0f);
    *vx = cx + HEX_R * cosf(a);
    *vy = cy + HEX_R * sinf(a);
}

static void draw_hex_outline(lv_layer_t *layer, float cx, float cy, lv_color_t col, lv_opa_t opa, int width)
{
    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = col;
    ld.opa = opa;
    ld.width = width;
    float vx[6], vy[6];
    for (int k = 0; k < 6; k++) hex_vertex(cx, cy, k, &vx[k], &vy[k]);
    for (int k = 0; k < 6; k++) {
        int n = (k + 1) % 6;
        ld.p1.x = vx[k]; ld.p1.y = vy[k];
        ld.p2.x = vx[n]; ld.p2.y = vy[n];
        lv_draw_line(layer, &ld);
    }
}

static void fill_hex(lv_layer_t *layer, float cx, float cy, lv_color_t col, lv_opa_t opa)
{
    float vx[6], vy[6];
    for (int k = 0; k < 6; k++) hex_vertex(cx, cy, k, &vx[k], &vy[k]);
    lv_draw_triangle_dsc_t td;
    lv_draw_triangle_dsc_init(&td);
    td.color = col;
    td.opa = opa;
    for (int k = 0; k < 6; k++) {
        int n = (k + 1) % 6;
        td.p[0].x = cx;    td.p[0].y = cy;
        td.p[1].x = vx[k]; td.p[1].y = vy[k];
        td.p[2].x = vx[n]; td.p[2].y = vy[n];
        lv_draw_triangle(layer, &td);
    }
}

/* draw a hexagon of arbitrary radius (for chord-zone markers) */
static void draw_hexR(lv_layer_t *layer, float cx, float cy, float R,
                      bool fill, lv_color_t fcol, lv_opa_t fopa,
                      lv_color_t bcol, int bw)
{
    float vx[6], vy[6];
    for (int k = 0; k < 6; k++) {
        float a = (3.14159265f / 180.0f) * (60.0f * (float)k - 30.0f);
        vx[k] = cx + R * cosf(a);
        vy[k] = cy + R * sinf(a);
    }
    if (fill) {
        lv_draw_triangle_dsc_t td;
        lv_draw_triangle_dsc_init(&td);
        td.color = fcol; td.opa = fopa;
        for (int k = 0; k < 6; k++) {
            int n = (k + 1) % 6;
            td.p[0].x = cx;    td.p[0].y = cy;
            td.p[1].x = vx[k]; td.p[1].y = vy[k];
            td.p[2].x = vx[n]; td.p[2].y = vy[n];
            lv_draw_triangle(layer, &td);
        }
    }
    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = bcol; ld.opa = LV_OPA_COVER; ld.width = bw;
    for (int k = 0; k < 6; k++) {
        int n = (k + 1) % 6;
        ld.p1.x = vx[k]; ld.p1.y = vy[k];
        ld.p2.x = vx[n]; ld.p2.y = vy[n];
        lv_draw_line(layer, &ld);
    }
}

static void build_vertices(void)
{
    n_vtx = 0;
    for (int i = 0; i < n_hex; i++) {
        for (int k = 0; k < 6; k++) {
            float vx, vy;
            hex_vertex(HEX[i].x, HEX[i].y, k, &vx, &vy);
            int found = -1;
            for (int j = 0; j < n_vtx; j++) {
                float dx = VTX[j].x - vx, dy = VTX[j].y - vy;
                if (dx * dx + dy * dy < 81.0f) { found = j; break; }   /* within ~9 px */
            }
            if (found >= 0) {
                if (VTX[found].cnt < 3) VTX[found].h[VTX[found].cnt] = i;
                VTX[found].cnt++;
            } else if (n_vtx < MAX_VTX) {
                VTX[n_vtx].x = vx; VTX[n_vtx].y = vy; VTX[n_vtx].cnt = 1; VTX[n_vtx].h[0] = i; n_vtx++;
            }
        }
    }
    int m = 0;
    for (int j = 0; j < n_vtx; j++) if (VTX[j].cnt >= 3) VTX[m++] = VTX[j];   /* interior vertices only */
    n_vtx = m;
    ESP_LOGI(TAG, "chord vertices: %d", n_vtx);
}

/* ============================ Ocarina (3-octave, 5 OoT notes) ============================ */
#define OCA_COLS 5
#define OCA_ROWS 3
#define OCA_N    (OCA_COLS * OCA_ROWS)   /* 15 pads */
#define OCA_R    32
#define OCA_ROOT 50                      /* D3 */
static const int8_t OCA_OFFS[OCA_COLS] = { 0, 3, 7, 9, 12 };  /* D F A B D' (OoT buttons) */
static const char *NOTE_NAMES[12] = { "C","C#","D","D#","E","F","F#","G","G#","A","A#","B" };
static float oca_x[OCA_N], oca_y[OCA_N], oca_freq[OCA_N];
static int   oca_midi[OCA_N];

static void build_ocarina(void)
{
    for (int r = 0; r < OCA_ROWS; r++) {
        for (int c = 0; c < OCA_COLS; c++) {
            int i = r * OCA_COLS + c;
            int oct = (OCA_ROWS - 1) - r;     /* top row = highest octave */
            oca_midi[i] = OCA_ROOT + oct * 12 + OCA_OFFS[c];
            oca_freq[i] = midi_freq(oca_midi[i]);
            oca_x[i] = 73.0f + c * 80.0f;
            oca_y[i] = 112.0f + r * 124.0f;
        }
    }
}

/* OoT jingles (creative names) as mid-octave button sequences (col 0..4 -> pads 5..9) */
static const uint8_t SONG_LULLABY[] = { 3, 4, 2, 3, 4, 2 };
static const uint8_t SONG_FOREST[]  = { 1, 2, 4, 1, 2, 4 };
static const uint8_t SONG_HORSE[]   = { 4, 3, 2, 4, 3, 2 };
static const uint8_t SONG_DAWN[]    = { 2, 1, 4, 2, 1, 4 };
static const uint8_t SONG_TIME[]    = { 2, 0, 1, 2, 0, 1 };
static const uint8_t SONG_STORM[]   = { 0, 1, 4, 0, 1, 4 };
typedef struct { const char *name; const uint8_t *seq; int len; } song_t;
static const song_t SONGS[] = {
    { "Royal Lullaby", SONG_LULLABY, 6 },
    { "Forest Air",    SONG_FOREST,  6 },
    { "Horse Call",    SONG_HORSE,   6 },
    { "Dawn Hymn",     SONG_DAWN,    6 },
    { "Time Spiral",   SONG_TIME,    6 },
    { "Tempest",       SONG_STORM,   6 },
};
#define N_SONGS (int)(sizeof(SONGS) / sizeof(SONGS[0]))
static volatile int g_song_sel = 0;

/* ============================ params menu ============================ */
typedef struct { const char *name; volatile float *val; float lo, hi; bool as_pct; const char **names; int nnames; } param_t;
static param_t PAGE_RES[] = {
    { "Brightness", &g_cutoff,   0.05f, 0.95f,  true  },
    { "Sustain",    &g_feedback, 0.90f, 0.999f, true  },
    { "Mic Drive",  &g_excite,   0.00f, 1.00f,  true  },
    { "Pluck",      &g_pluck,    0.00f, 1.00f,  true  },
    { "Breath",     &g_breath,   0.00f, 1.00f,  true  },
    { "Volume",     &g_volume,   0.00f, 100.0f, false },
};
static param_t PAGE_SYN[] = {
    { "Attack",    &g_attack,  0.5f,  500.0f,  false },
    { "Release",   &g_release, 5.0f,  1000.0f, false },
    { "Cutoff",    &g_syn_cut, 0.00f, 1.00f,   true  },
    { "Resonance", &g_reso,    0.00f, 1.00f,   true  },
    { "Volume",    &g_volume,  0.00f, 100.0f,  false },
};
#define N_SYN (int)(sizeof(PAGE_SYN) / sizeof(PAGE_SYN[0]))
static param_t PAGE_MOTION[] = {
    { "Depth",  &g_imu_amt,   0.00f, 1.00f, true },
    { "Pitch",  &g_imu_pitch, 0.00f, 1.00f, true },
    { "Filter", &g_imu_filt,  0.00f, 1.00f, true },
};
static param_t PAGE_FX[] = {
    { "Delay Time", &g_dly_time, 20.0f, 1000.0f, false },
    { "Delay Fbk",  &g_dly_fb,   0.00f, 0.90f,   true  },
    { "Delay Mix",  &g_dly_mix,  0.00f, 1.00f,   true  },
    { "Flanger",    &g_flg_amt,  0.00f, 1.00f,   true  },
};
typedef struct { const char *title; param_t *p; int n; } page_t;
static page_t PAGES[] = {
    { "MODE",      NULL,        0 },   /* special: mode selector */
    { "RESONATOR", PAGE_RES,    6 },
    { "FX",        PAGE_FX,     4 },
    { "MOTION",    PAGE_MOTION, 3 },
};
#define N_PAGES (int)(sizeof(PAGES) / sizeof(PAGES[0]))
static bool g_menu_open = false;
static int  g_menu_page = 0;

#define MENU_TOP    60
#define MENU_ROW_H  66
#define MENU_MARGIN 56

static void menu_row_bounds(int i, int *y0, int *y1)   /* full touch row */
{
    *y0 = MENU_TOP + i * MENU_ROW_H;
    *y1 = *y0 + MENU_ROW_H - 12;
}

/* page 1 is the "voice" page: shows Synth or Resonator params depending on engine */
static param_t *voice_params(int page, int *n, const char **title)
{
    if (page == 1) {
        if (synth_engine()) { *n = N_SYN; *title = "SYNTH"; return PAGE_SYN; }
        *n = (int)(sizeof(PAGE_RES) / sizeof(PAGE_RES[0])); *title = "RESONATOR"; return PAGE_RES;
    }
    *n = PAGES[page].n; *title = PAGES[page].title; return PAGES[page].p;
}

static void draw_menu(lv_layer_t *layer, int page, int editing_row)
{
    page_t *pg = &PAGES[page];
    int pn; const char *ptitle;
    param_t *plist = voice_params(page, &pn, &ptitle);

    lv_draw_rect_dsc_t bg;
    lv_draw_rect_dsc_init(&bg);
    bg.bg_color = lv_color_hex(0x0a0a12);
    bg.bg_opa = LV_OPA_COVER;
    lv_area_t full = { 0, 0, CANVAS_WIDTH - 1, CANVAS_HEIGHT - 1 };
    lv_draw_rect(layer, &bg, &full);

    /* title (swipe the title area left/right for pages, down to close) */
    lv_draw_label_dsc_t ld;
    lv_draw_label_dsc_init(&ld);
    ld.font = &lv_font_montserrat_24;
    ld.color = lv_color_white();
    ld.align = LV_TEXT_ALIGN_CENTER;
    /* static: lv_draw_label keeps the text pointer until the layer is flushed */
    static char tbuf[40];
    snprintf(tbuf, sizeof(tbuf), "%s   %s   %s", page > 0 ? "<" : " ", ptitle, page < N_PAGES - 1 ? ">" : " ");
    ld.text = tbuf;
    lv_area_t title = { 0, 18, CANVAS_WIDTH - 1, 50 };
    lv_draw_label(layer, &ld, &title);

    /* page dots */
    for (int p = 0; p < N_PAGES; p++) {
        lv_draw_rect_dsc_t dot;
        lv_draw_rect_dsc_init(&dot);
        dot.bg_color = (p == page) ? lv_color_white() : lv_color_hex(0x404850);
        dot.bg_opa = LV_OPA_COVER;
        dot.radius = LV_RADIUS_CIRCLE;
        int cx = CANVAS_WIDTH / 2 - (N_PAGES * 14) / 2 + p * 14;
        lv_area_t da = { cx, 56, cx + 7, 63 };
        lv_draw_rect(layer, &dot, &da);
    }

    if (pg->p == NULL) {
        /* ---- MODE selector ---- */
        for (int m = 0; m < N_MODES; m++) {
            int y0, y1; menu_row_bounds(m, &y0, &y1);
            bool sel = (m == g_mode);
            lv_draw_rect_dsc_t tr;
            lv_draw_rect_dsc_init(&tr);
            tr.bg_color = sel ? lv_color_hex(0x224058) : lv_color_hex(0x181c24);
            tr.bg_opa = LV_OPA_COVER;
            tr.radius = 10;
            tr.border_color = sel ? lv_color_white() : lv_color_hex(0x303840);
            tr.border_width = sel ? 3 : 1;
            lv_area_t row = { MENU_MARGIN, y0, CANVAS_WIDTH - MENU_MARGIN, y0 + 54 };
            lv_draw_rect(layer, &tr, &row);

            static char mbuf[6][20];
            int mi = (m < 6) ? m : 5;
            snprintf(mbuf[mi], sizeof(mbuf[mi]), "%s%s", sel ? "> " : "", MODE_NAMES[m]);
            ld.font = &lv_font_montserrat_24;
            ld.color = sel ? lv_color_white() : lv_color_hex(0xc0c8d0);
            ld.align = LV_TEXT_ALIGN_CENTER;
            ld.text = mbuf[mi];
            lv_area_t la = { MENU_MARGIN, y0 + 12, CANVAS_WIDTH - MENU_MARGIN, y0 + 46 };
            lv_draw_label(layer, &ld, &la);
        }
        ld.font = &lv_font_montserrat_20;
        ld.color = lv_color_hex(0x808890);
        ld.align = LV_TEXT_ALIGN_CENTER;
        ld.text = "title: < > pages   v close";
        lv_area_t hint2 = { 0, CANVAS_HEIGHT - 38, CANVAS_WIDTH - 1, CANVAS_HEIGHT - 14 };
        lv_draw_label(layer, &ld, &hint2);
        return;
    }

    for (int i = 0; i < pn; i++) {
        int y0, y1;
        menu_row_bounds(i, &y0, &y1);
        float v = *plist[i].val;
        float t = (v - plist[i].lo) / (plist[i].hi - plist[i].lo);
        if (t < 0) t = 0; else if (t > 1) t = 1;
        bool sel = (i == editing_row);

        /* name (own line, white on dark = always legible) + value on the right.
         * Per-row STATIC buffers: lv_draw_label only stores the pointer and renders
         * at finish_layer, so reusing one local buffer would show the last row's text. */
        static char nmbuf[8][24];
        static char vbuf[8][16];
        int bi = (i < 8) ? i : 7;
        snprintf(nmbuf[bi], sizeof(nmbuf[bi]), "%s", plist[i].name);
        if (plist[i].names) {
            int mi2 = (int)(v + 0.5f);
            if (mi2 < 0) mi2 = 0; else if (mi2 >= plist[i].nnames) mi2 = plist[i].nnames - 1;
            snprintf(vbuf[bi], sizeof(vbuf[bi]), "%s", plist[i].names[mi2]);
        } else if (plist[i].as_pct) snprintf(vbuf[bi], sizeof(vbuf[bi]), "%d%%", (int)(t * 100.0f + 0.5f));
        else                        snprintf(vbuf[bi], sizeof(vbuf[bi]), "%d", (int)v);
        ld.font = &lv_font_montserrat_20;
        ld.color = sel ? lv_color_white() : lv_color_hex(0xc0c8d0);
        ld.align = LV_TEXT_ALIGN_LEFT;
        ld.text = nmbuf[bi];
        lv_area_t na = { MENU_MARGIN, y0, CANVAS_WIDTH - MENU_MARGIN, y0 + 24 };
        lv_draw_label(layer, &ld, &na);
        ld.align = LV_TEXT_ALIGN_RIGHT;
        ld.text = vbuf[bi];
        lv_draw_label(layer, &ld, &na);

        /* bar */
        int by0 = y0 + 30, by1 = y0 + 56;
        lv_draw_rect_dsc_t tr;
        lv_draw_rect_dsc_init(&tr);
        tr.bg_color = lv_color_hex(0x181c24);
        tr.bg_opa = LV_OPA_COVER;
        tr.radius = 6;
        tr.border_color = sel ? lv_color_white() : lv_color_hex(0x303840);
        tr.border_width = sel ? 3 : 1;
        lv_area_t track = { MENU_MARGIN, by0, CANVAS_WIDTH - MENU_MARGIN, by1 };
        lv_draw_rect(layer, &tr, &track);

        int fill_w = (int)((CANVAS_WIDTH - 2 * MENU_MARGIN) * t);
        if (fill_w > 2) {
            lv_draw_rect_dsc_t fl;
            lv_draw_rect_dsc_init(&fl);
            fl.bg_color = lv_color_hsv_to_rgb(190, 75, sel ? 100 : 75);
            fl.bg_opa = LV_OPA_COVER;
            fl.radius = 6;
            lv_area_t fa = { MENU_MARGIN, by0, MENU_MARGIN + fill_w, by1 };
            lv_draw_rect(layer, &fl, &fa);
        }
    }

    ld.font = &lv_font_montserrat_20;
    ld.color = lv_color_hex(0x808890);
    ld.align = LV_TEXT_ALIGN_CENTER;
    ld.text = "title: < > pages   v close";
    lv_area_t hint = { 0, CANVAS_HEIGHT - 38, CANVAS_WIDTH - 1, CANVAS_HEIGHT - 14 };
    lv_draw_label(layer, &ld, &hint);
}

/* ============================ display + touch ============================ */
static void timer_cb(lv_timer_t *timer)
{
    lv_obj_t *canvas = (lv_obj_t *)lv_timer_get_user_data(timer);
    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);

    static int  prev_sig = -1;
    static bool prev_pressed = false;
    static int  start_x = 0, start_y = 0;
    static bool gesture_consumed = false;
    static int  editing_row = -1;
    int  active[3];
    int  n_active = 0;
    float chord_cx = 0, chord_cy = 0;

    /* ---- read touch ---- */
    lv_indev_t *indev = bsp_display_get_input_dev();
    bool pressed = (indev && lv_indev_get_state(indev) == LV_INDEV_STATE_PRESSED);
    int tx = -1, ty = -1;
    if (pressed) {
        lv_point_t p;
        lv_indev_get_point(indev, &p);
        tx = p.x; ty = p.y;
        if (tx < 0) tx = 0; else if (tx >= CANVAS_WIDTH)  tx = CANVAS_WIDTH - 1;
        if (ty < 0) ty = 0; else if (ty >= CANVAS_HEIGHT) ty = CANVAS_HEIGHT - 1;
    }

    /* ---- gestures: open = swipe up from bottom; in menu = swipe title strip (< > pages, v close) ---- */
    if (pressed && !prev_pressed) { start_x = tx; start_y = ty; gesture_consumed = false; }
    if (pressed && !gesture_consumed) {
        int dx = tx - start_x, dy = ty - start_y;
        int adx = dx < 0 ? -dx : dx;
        int ady = dy < 0 ? -dy : dy;
        if (!g_menu_open) {
            if (start_y >= CANVAS_HEIGHT - NAV_H && dy <= -SWIPE_THRESH) {
                g_menu_open = true; gesture_consumed = true; g_finger_down = 0; prev_sig = -1;
            }
        } else if (start_y < MENU_TOP) {              /* gestures only from the title strip */
            if (ady >= SWIPE_THRESH && ady > adx && dy > 0) {
                g_menu_open = false; gesture_consumed = true; editing_row = -1;
            } else if (adx >= SWIPE_THRESH && adx > ady) {
                if (dx < 0 && g_menu_page < N_PAGES - 1) g_menu_page++;
                else if (dx > 0 && g_menu_page > 0)      g_menu_page--;
                gesture_consumed = true; editing_row = -1;
            }
        }
    }

    /* ---- MENU mode ---- */
    if (g_menu_open) {
        g_finger_down = 0;
        g_touch_x = -1;
        page_t *pg = &PAGES[g_menu_page];
        if (pressed && !gesture_consumed && ty >= MENU_TOP) {
            if (pg->p == NULL) {
                for (int m = 0; m < N_MODES; m++) {
                    int y0, y1; menu_row_bounds(m, &y0, &y1);
                    if (ty >= y0 - 6 && ty <= y0 + 54 + 6) { g_mode = m; break; }
                }
            } else {
                int epn; const char *etitle;
                param_t *eplist = voice_params(g_menu_page, &epn, &etitle);
                for (int i = 0; i < epn; i++) {
                    int y0, y1; menu_row_bounds(i, &y0, &y1);
                    if (ty >= y0 - 6 && ty <= y1 + 6) {
                        float t = (float)(tx - MENU_MARGIN) / (float)(CANVAS_WIDTH - 2 * MENU_MARGIN);
                        if (t < 0) t = 0; else if (t > 1) t = 1;
                        float nv = eplist[i].lo + t * (eplist[i].hi - eplist[i].lo);
                        if (eplist[i].names) nv = floorf(nv + 0.5f);   /* stepped selector */
                        *eplist[i].val = nv;
                        editing_row = i;
                        break;
                    }
                }
            }
        }
        draw_menu(&layer, g_menu_page, editing_row);
        prev_pressed = pressed;
        lv_canvas_finish_layer(canvas, &layer);
        return;
    }

    /* ---- ocarina jingle player (auto-plays a song, lights the pads) ---- */
    static int song_play = 0, song_pos = 0, song_ms = 0;
    if (g_mode != MODE_OCARINA) song_play = 0;
    if (song_play) {
        song_ms -= 33;
        if (song_ms <= 0) {
            const song_t *sg = &SONGS[g_song_sel];
            if (song_pos >= sg->len) { song_play = 0; g_finger_down = 0; g_active_note = -1; prev_sig = -1; }
            else {
                int pad = OCA_COLS + sg->seq[song_pos];   /* mid-octave row */
                float f = oca_freq[pad];
                chord_push(1, &f);
                g_finger_down = 1; g_active_note = pad;
                song_pos++; song_ms = 360;
            }
        }
    }

    /* ---- PLAY mode ---- */
    bool play_touch = pressed && !gesture_consumed && ty < CANVAS_HEIGHT - NAV_H;
    if (play_touch && g_mode == MODE_OCARINA) {
        g_touch_x = tx; g_touch_y = ty;
        if (ty < 72) {
            if (!prev_pressed) {   /* song bar — edge-triggered */
                if (tx < 110)                     g_song_sel = (g_song_sel + N_SONGS - 1) % N_SONGS;
                else if (tx > CANVAS_WIDTH - 110)  g_song_sel = (g_song_sel + 1) % N_SONGS;
                else { song_play = !song_play; song_pos = 0; song_ms = 0;
                       if (!song_play) { g_finger_down = 0; g_active_note = -1; } }
            }
        } else {
            int pad = -1;
            for (int i = 0; i < OCA_N; i++) {
                float dx = (float)tx - oca_x[i], dy = (float)ty - oca_y[i];
                if (dx * dx + dy * dy < (float)((OCA_R + 6) * (OCA_R + 6))) { pad = i; break; }
            }
            if (pad >= 0) {
                song_play = 0;   /* manual touch stops the jingle */
                if (pad != g_active_note) { float f = oca_freq[pad]; chord_push(1, &f); g_active_note = pad; }
                g_finger_down = 1;
            } else if (!song_play) {
                g_active_note = -1; g_finger_down = 0; prev_sig = -1;
            }
        }
    } else if (play_touch && g_mode == MODE_SYNTH) {
        /* Synth 2D pad: X = scale note, Y = warp morph (real-time waveshaping) */
        g_touch_x = tx; g_touch_y = ty;
        float xn = (float)tx / CANVAS_WIDTH;             if (xn < 0) xn = 0; else if (xn > 1) xn = 1;
        float yn = (float)ty / (float)(CANVAS_HEIGHT - NAV_H); if (yn < 0) yn = 0; else if (yn > 1) yn = 1;
        int idx = (int)(xn * XY_NOTES); if (idx < 0) idx = 0; else if (idx >= XY_NOTES) idx = XY_NOTES - 1;
        if (idx != g_active_note) { float f = xy_note_freq(idx); chord_push(1, &f); g_active_note = idx; }
        g_cut_live = 1.0f - yn;   /* top = bright (open filter), bottom = dark */
        g_finger_down = 1;
    } else if (play_touch && g_mode == MODE_XY) {
        /* Keys+Spectrum: top half = scale note (pentatonic, 3 oct), bottom half = filter */
        g_touch_x = tx; g_touch_y = ty;
        const int cy = CANVAS_HEIGHT / 2;
        float txn = (float)tx / CANVAS_WIDTH;
        if (txn < 0) txn = 0; else if (txn > 1) txn = 1;
        if (ty < cy) {
            int idx = (int)(txn * XY_NOTES);
            if (idx < 0) idx = 0; else if (idx >= XY_NOTES) idx = XY_NOTES - 1;
            if (idx != g_active_note) { float f = xy_note_freq(idx); chord_push(1, &f); g_active_note = idx; }
        } else {
            float by = (float)(ty - cy) / (float)(CANVAS_HEIGHT - NAV_H - cy);
            if (by < 0) by = 0; else if (by > 1) by = 1;
            g_cutoff   = 0.05f + txn * 0.90f;
            g_feedback = 0.90f + by * 0.099f;
            g_active_note = -1;
        }
        g_finger_down = 1;
    } else if (play_touch && (n_hex > 0 || n_vtx > 0)) {
        g_touch_x = tx;
        g_touch_y = ty;

        /* nearest hex */
        int hi = -1; float hd = 1e18f;
        for (int i = 0; i < n_hex; i++) {
            float dx = (float)tx - HEX[i].x, dy = (float)ty - HEX[i].y;
            float dd = dx * dx + dy * dy;
            if (dd < hd) { hd = dd; hi = i; }
        }
        /* nearest chord vertex */
        int vi = -1; float vd = 1e18f;
        for (int i = 0; i < n_vtx; i++) {
            float dx = (float)tx - VTX[i].x, dy = (float)ty - VTX[i].y;
            float dd = dx * dx + dy * dy;
            if (dd < vd) { vd = dd; vi = i; }
        }

        bool do_chord = (g_mode == MODE_CHORD) && (vi >= 0);   /* Tonnetz = single notes only */

        if (do_chord && vi >= 0) {
            active[0] = VTX[vi].h[0]; active[1] = VTX[vi].h[1]; active[2] = VTX[vi].h[2];
            n_active = 3;
            chord_cx = VTX[vi].x; chord_cy = VTX[vi].y;
        } else if (hi >= 0) {
            active[0] = hi; n_active = 1;
        }

        int sig = 0;
        for (int j = 0; j < n_active; j++) sig = sig * 131 + (active[j] + 1);
        sig = sig * 4 + n_active;
        if (sig != prev_sig && n_active > 0) {
            float f[3];
            for (int j = 0; j < n_active; j++) f[j] = HEX[active[j]].freq;
            chord_push(n_active, f);
            prev_sig = sig;
        }
        g_finger_down = 1;
    } else {
        g_touch_x = -1;
        g_touch_y = -1;
        if (!song_play) {   /* don't kill an auto-playing jingle */
            g_finger_down = 0;
            prev_sig = -1;
            g_active_note = -1;
        }
    }
    prev_pressed = pressed;

    if (g_mode == MODE_OCARINA) {
        /* ---- song bar: <  Name  >  (tap title to auto-play) ---- */
        static char sbuf[48];
        snprintf(sbuf, sizeof(sbuf), "<   %s   >", SONGS[g_song_sel].name);
        lv_draw_label_dsc_t sb; lv_draw_label_dsc_init(&sb);
        sb.font = &lv_font_montserrat_24; sb.color = lv_color_white(); sb.align = LV_TEXT_ALIGN_CENTER;
        sb.text = sbuf;
        lv_area_t sba = { 0, 20, CANVAS_WIDTH - 1, 50 };
        lv_draw_label(&layer, &sb, &sba);
        sb.font = &lv_font_montserrat_20; sb.color = lv_color_hex(0x808890);
        sb.text = "tap title to play";
        lv_area_t sh = { 0, 52, CANVAS_WIDTH - 1, 72 };
        lv_draw_label(&layer, &sb, &sh);

        /* ---- 15 pads (3 octaves x 5 notes), active pad glows ---- */
        for (int i = 0; i < OCA_N; i++) {
            bool act = (g_active_note == i);
            uint16_t hue = (uint16_t)((oca_midi[i] % 12) * 30);
            int rr = act ? OCA_R + 5 : OCA_R;
            if (act) {
                lv_draw_rect_dsc_t gl; lv_draw_rect_dsc_init(&gl);
                gl.bg_opa = LV_OPA_TRANSP; gl.radius = LV_RADIUS_CIRCLE;
                gl.border_color = lv_color_hsv_to_rgb(hue, 90, 100); gl.border_opa = LV_OPA_COVER; gl.border_width = 4;
                int gr = rr + 10;
                lv_area_t ga = { (int)oca_x[i] - gr, (int)oca_y[i] - gr, (int)oca_x[i] + gr, (int)oca_y[i] + gr };
                lv_draw_rect(&layer, &gl, &ga);
            }
            lv_draw_rect_dsc_t pc; lv_draw_rect_dsc_init(&pc);
            pc.bg_color = lv_color_hsv_to_rgb(hue, act ? 90 : 55, act ? 100 : 50);
            pc.bg_opa = LV_OPA_COVER; pc.radius = LV_RADIUS_CIRCLE;
            pc.border_color = lv_color_white(); pc.border_opa = LV_OPA_COVER; pc.border_width = act ? 4 : 2;
            lv_area_t pa = { (int)oca_x[i] - rr, (int)oca_y[i] - rr, (int)oca_x[i] + rr, (int)oca_y[i] + rr };
            lv_draw_rect(&layer, &pc, &pa);

            lv_draw_label_dsc_t ld; lv_draw_label_dsc_init(&ld);
            ld.font = &lv_font_montserrat_20; ld.color = lv_color_white(); ld.align = LV_TEXT_ALIGN_CENTER;
            ld.text = NOTE_NAMES[oca_midi[i] % 12];
            lv_area_t la = { (int)oca_x[i] - rr, (int)oca_y[i] - 13, (int)oca_x[i] + rr, (int)oca_y[i] + 13 };
            lv_draw_label(&layer, &ld, &la);
        }
    } else if (g_mode == MODE_XY || g_mode == MODE_SYNTH) {
        /* ---- KEYS+SPECTRUM / SYNTH: output spectrum + pentatonic note strip ---- */
        const int cy = CANVAS_HEIGHT / 2;
        const int stripe_w = CANVAS_WIDTH / STRIPE_COUNT;

        if (g_active_note >= 0) {
            int x0 = g_active_note * CANVAS_WIDTH / XY_NOTES;
            int x1 = (g_active_note + 1) * CANVAS_WIDTH / XY_NOTES;
            lv_draw_rect_dsc_t zb; lv_draw_rect_dsc_init(&zb);
            zb.bg_color = lv_color_white(); zb.bg_opa = LV_OPA_20;
            lv_area_t za = { x0, 0, x1 - 1, cy - 1 };
            lv_draw_rect(&layer, &zb, &za);
        }
        for (int i = 0; i < STRIPE_COUNT; i++) {
            float norm = (display_spectrum[i] + 90.0f) / 90.0f;
            norm = fmaxf(0.0f, fminf(1.0f, norm)); norm = sqrtf(norm);
            int bh = (int)(norm * (CANVAS_HEIGHT / 2));
            if (peak[i] < bh) peak[i] = bh; else { peak[i] -= 2; if (peak[i] < 0) peak[i] = 0; }
            uint16_t hue = (uint16_t)(i * (270.0f / STRIPE_COUNT));
            lv_draw_rect_dsc_t rd; lv_draw_rect_dsc_init(&rd);
            rd.bg_color = lv_color_hsv_to_rgb(hue, 100, 100); rd.bg_opa = LV_OPA_COVER;
            int xs = i * stripe_w + 1, xe = (i + 1) * stripe_w - 2;
            lv_area_t ba = { xs, cy - bh, xe, cy + bh }; lv_draw_rect(&layer, &rd, &ba);
            lv_area_t pt = { xs, cy - (int)peak[i] - 2, xe, cy - (int)peak[i] }; lv_draw_rect(&layer, &rd, &pt);
            lv_area_t pb = { xs, cy + (int)peak[i], xe, cy + (int)peak[i] + 2 }; lv_draw_rect(&layer, &rd, &pb);
        }
        for (int k = 0; k <= XY_NOTES; k++) {
            int x = k * CANVAS_WIDTH / XY_NOTES; if (x >= CANVAS_WIDTH) x = CANVAS_WIDTH - 1;
            bool oct = (k % SCALE_LEN) == 0;
            lv_draw_rect_dsc_t tl; lv_draw_rect_dsc_init(&tl);
            tl.bg_color = lv_color_hex(oct ? 0x909090 : 0x383838); tl.bg_opa = oct ? LV_OPA_70 : LV_OPA_40;
            lv_area_t la = { x, 0, x + (oct ? 1 : 0), cy - 1 }; lv_draw_rect(&layer, &tl, &la);
        }
        lv_draw_rect_dsc_t ln; lv_draw_rect_dsc_init(&ln);
        ln.bg_color = lv_color_hex(0x404040); ln.bg_opa = LV_OPA_50;
        lv_area_t dv = { 0, cy - 1, CANVAS_WIDTH - 1, cy }; lv_draw_rect(&layer, &ln, &dv);
    } else {
        /* ---- TONNETZ / CHORD: the note grid, with played notes lit ---- */
        for (int i = 0; i < n_hex; i++) {
            draw_hex_outline(&layer, HEX[i].x, HEX[i].y, lv_color_hex(0x303840), LV_OPA_COVER, 2);
            uint16_t hue = (uint16_t)((HEX[i].note % 12) * 30);
            lv_draw_rect_dsc_t dot;
            lv_draw_rect_dsc_init(&dot);
            dot.bg_color = lv_color_hsv_to_rgb(hue, 70, 80);
            dot.bg_opa = LV_OPA_COVER;
            dot.radius = LV_RADIUS_CIRCLE;
            const int rr = 7;
            lv_area_t da = { (int)HEX[i].x - rr, (int)HEX[i].y - rr, (int)HEX[i].x + rr, (int)HEX[i].y + rr };
            lv_draw_rect(&layer, &dot, &da);
        }
        if (g_mode == MODE_CHORD) {
            /* faint hexagon overlays mark where the chords live */
            for (int i = 0; i < n_vtx; i++) {
                draw_hexR(&layer, VTX[i].x, VTX[i].y, CHORD_R * 0.9f,
                          false, lv_color_black(), LV_OPA_TRANSP, lv_color_hex(0x4a5560), 2);
            }
        }
        for (int j = 0; j < n_active; j++) {
            hex_t *h = &HEX[active[j]];
            uint16_t hue = (uint16_t)((h->note % 12) * 30);
            fill_hex(&layer, h->x, h->y, lv_color_hsv_to_rgb(hue, 90, 100), LV_OPA_60);
            draw_hex_outline(&layer, h->x, h->y, lv_color_white(), LV_OPA_COVER, 3);
        }
    }

    /* ---- bottom handle bar (swipe up here for the menu) ---- */
    {
        lv_draw_rect_dsc_t hb;
        lv_draw_rect_dsc_init(&hb);
        hb.bg_color = lv_color_hex(0x808890);
        hb.bg_opa = LV_OPA_60;
        hb.radius = LV_RADIUS_CIRCLE;
        const int bw = 120, bh = 6;
        int bx = (CANVAS_WIDTH - bw) / 2;
        int by = CANVAS_HEIGHT - NAV_H / 2 - bh / 2;
        lv_area_t ba = { bx, by, bx + bw, by + bh };
        lv_draw_rect(&layer, &hb, &ba);
    }

    /* ---- finger marker ---- */
    if (g_touch_x >= 0) {
        lv_draw_rect_dsc_t m;
        lv_draw_rect_dsc_init(&m);
        m.bg_color = lv_color_white();
        m.bg_opa = LV_OPA_70;
        m.radius = LV_RADIUS_CIRCLE;
        const int rr = 8;
        lv_area_t ma = { g_touch_x - rr, g_touch_y - rr, g_touch_x + rr, g_touch_y + rr };
        lv_draw_rect(&layer, &m, &ma);
    }

    lv_canvas_finish_layer(canvas, &layer);
}

static void build_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_draw_buf_t *draw_buf = lv_draw_buf_create(CANVAS_WIDTH, CANVAS_HEIGHT, LV_COLOR_FORMAT_RGB565, 0);
    if (!draw_buf) {
        ESP_LOGE(TAG, "Failed to allocate canvas draw buffer");
        return;
    }

    lv_obj_t *canvas = lv_canvas_create(scr);
    lv_canvas_set_draw_buf(canvas, draw_buf);
    lv_obj_set_size(canvas, CANVAS_WIDTH, CANVAS_HEIGHT);
    lv_obj_center(canvas);

    build_hex_grid();
    build_vertices();
    build_ocarina();
    build_saw();
    lv_timer_create(timer_cb, 33, canvas);
}

/* ============================ IMU: QMI8658 over BSP I2C ============================ */
static i2c_master_dev_handle_t imu_dev = NULL;

static esp_err_t imu_w(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(imu_dev, b, 2, 100);
}
static esp_err_t imu_r(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(imu_dev, &reg, 1, buf, n, 100);
}

static bool imu_init(void)
{
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) return false;
    const uint8_t addrs[2] = { 0x6B, 0x6A };
    for (int a = 0; a < 2; a++) {
        if (imu_dev) { i2c_master_bus_rm_device(imu_dev); imu_dev = NULL; }
        i2c_device_config_t cfg = {};
        cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        cfg.device_address = addrs[a];
        cfg.scl_speed_hz = 400000;
        if (i2c_master_bus_add_device(bus, &cfg, &imu_dev) != ESP_OK) continue;
        uint8_t who = 0;
        if (imu_r(0x00, &who, 1) == ESP_OK && who == 0x05) {
            imu_w(0x60, 0xB0); vTaskDelay(pdMS_TO_TICKS(15));   /* reset */
            imu_w(0x02, 0x40);   /* CTRL1: addr auto-increment, little-endian */
            imu_w(0x03, 0x15);   /* CTRL2: accel +-4g, 250 Hz */
            imu_w(0x04, 0x55);   /* CTRL3: gyro +-512 dps, 250 Hz */
            imu_w(0x08, 0x03);   /* CTRL7: enable accel + gyro */
            ESP_LOGI(TAG, "QMI8658 IMU found at 0x%02X", addrs[a]);
            return true;
        }
    }
    ESP_LOGE(TAG, "QMI8658 IMU not found");
    return false;
}

static void imu_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(400));   /* let BSP bring up I2C */
    if (!imu_init()) { vTaskDelete(NULL); return; }

    float ax0 = 0, ay0 = 0; bool based = false;
    float fx = 0, fy = 0;
    while (1) {
        uint8_t b[6];
        if (imu_r(0x35, b, 6) == ESP_OK) {
            int16_t rax = (int16_t)(b[0] | (b[1] << 8));
            int16_t ray = (int16_t)(b[2] | (b[3] << 8));
            float gx = rax / 8192.0f;   /* +-4g -> 8192 LSB/g */
            float gy = ray / 8192.0f;
            if (!based) { ax0 = gx; ay0 = gy; based = true; }   /* resting orientation = neutral */
            float dx = gx - ax0, dy = gy - ay0;
            if (fabsf(dx) < 0.04f) dx = 0;   /* deadzone */
            if (fabsf(dy) < 0.04f) dy = 0;
            fx += 0.25f * (dx - fx);
            fy += 0.25f * (dy - fy);
            float ix = fx * 1.6f; if (ix > 1) ix = 1; else if (ix < -1) ix = -1;
            float iy = fy * 1.6f; if (iy > 1) iy = 1; else if (iy < -1) iy = -1;
            g_imu_x = ix;
            g_imu_y = iy;
            g_bend = powf(2.0f, ix * 2.0f / 12.0f);   /* +-2 semitones */
        }
        vTaskDelay(pdMS_TO_TICKS(10));   /* ~100 Hz */
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting espSynth — Tonnetz hex synth");

    lv_display_t *disp = bsp_display_start();
    if (disp) {
        bsp_display_backlight_on();
    }

    bsp_display_lock(-1);
    build_ui();
    bsp_display_unlock();

    xTaskCreate(audio_engine_task, "audio_engine", 8 * 1024, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu", 4 * 1024, NULL, 4, NULL);
}
