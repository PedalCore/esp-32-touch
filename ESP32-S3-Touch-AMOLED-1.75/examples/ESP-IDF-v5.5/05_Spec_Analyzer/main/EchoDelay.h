/* ------------------------------------------------------------
name: "EchoDelay"
Code generated with Faust 2.82.0 (https://faust.grame.fr)
Compilation options: -lang cpp -ct 1 -es 1 -mcd 16 -mdd 1024 -mdy 33 -single -ftz 0

NOTE (espSynth): the only edits to the Faust output are
  (1) the minimal stub base classes below (dsp/Meta/UI) so this compiles
      without the full Faust architecture, and
  (2) two public setters (setFeedback / setDurationMs) so we can drive the
      params from touch. The DSP/compute() is verbatim.
  The fRec0[131072] buffer (~512 KB) means this object must be allocated in
  PSRAM (placement-new) — see main.cpp.
------------------------------------------------------------ */

#ifndef  __mydsp_H__
#define  __mydsp_H__

#ifndef FAUSTFLOAT
#define FAUSTFLOAT float
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <math.h>

#ifndef FAUSTCLASS
#define FAUSTCLASS mydsp
#endif

#ifdef __APPLE__
#define exp10f __exp10f
#define exp10 __exp10
#endif

#if defined(_WIN32)
#define RESTRICT __restrict
#else
#define RESTRICT __restrict__
#endif

/* --- minimal Faust architecture stubs (espSynth) --- */
struct Meta {
    virtual ~Meta() {}
    virtual void declare(const char* /*key*/, const char* /*value*/) {}
};
struct UI {
    virtual ~UI() {}
    virtual void openHorizontalBox(const char* /*label*/) {}
    virtual void closeBox() {}
    virtual void declare(void* /*zone*/, const char* /*key*/, const char* /*val*/) {}
    virtual void addHorizontalSlider(const char* /*label*/, FAUSTFLOAT* /*zone*/,
                                     FAUSTFLOAT /*init*/, FAUSTFLOAT /*min*/,
                                     FAUSTFLOAT /*max*/, FAUSTFLOAT /*step*/) {}
};
struct dsp {
    virtual ~dsp() {}
};
/* --------------------------------------------------- */


class mydsp : public dsp {

 private:

    FAUSTFLOAT fHslider0;
    int fSampleRate;
    float fConst0;
    float fConst1;
    FAUSTFLOAT fHslider1;
    float fConst2;
    float fRec1[2];
    int IOTA0;
    float fVec0[2];
    float fRec0[131072];

 public:
    mydsp() {
    }

    /* espSynth: drive params from touch */
    void setFeedback(FAUSTFLOAT v)   { fHslider0 = v; }
    void setDurationMs(FAUSTFLOAT v) { fHslider1 = v; }

    void metadata(Meta* m) {
        m->declare("compile_options", "-lang cpp -ct 1 -es 1 -mcd 16 -mdd 1024 -mdy 33 -single -ftz 0");
        m->declare("delays.lib/fdelay4:author", "Julius O. Smith III");
        m->declare("delays.lib/fdelayltv:author", "Julius O. Smith III");
        m->declare("delays.lib/name", "Faust Delay Library");
        m->declare("delays.lib/version", "1.2.0");
        m->declare("filename", "EchoDelay.dsp");
        m->declare("maths.lib/author", "GRAME");
        m->declare("maths.lib/copyright", "GRAME");
        m->declare("maths.lib/license", "LGPL with exception");
        m->declare("maths.lib/name", "Faust Math Library");
        m->declare("maths.lib/version", "2.9.0");
        m->declare("name", "EchoDelay");
        m->declare("platform.lib/name", "Generic Platform Library");
        m->declare("platform.lib/version", "1.3.0");
        m->declare("signals.lib/name", "Faust Signal Routing Library");
        m->declare("signals.lib/version", "1.6.0");
    }

    virtual int getNumInputs() {
        return 1;
    }
    virtual int getNumOutputs() {
        return 1;
    }

    static void classInit(int sample_rate) {
    }

    virtual void instanceConstants(int sample_rate) {
        fSampleRate = sample_rate;
        fConst0 = std::min<float>(1.92e+05f, std::max<float>(1.0f, static_cast<float>(fSampleRate)));
        fConst1 = 0.441f / fConst0;
        fConst2 = 1.0f - 44.1f / fConst0;
    }

    virtual void instanceResetUserInterface() {
        fHslider0 = static_cast<FAUSTFLOAT>(0.5f);
        fHslider1 = static_cast<FAUSTFLOAT>(2e+02f);
    }

    virtual void instanceClear() {
        for (int l0 = 0; l0 < 2; l0 = l0 + 1) {
            fRec1[l0] = 0.0f;
        }
        IOTA0 = 0;
        for (int l1 = 0; l1 < 2; l1 = l1 + 1) {
            fVec0[l1] = 0.0f;
        }
        for (int l2 = 0; l2 < 131072; l2 = l2 + 1) {
            fRec0[l2] = 0.0f;
        }
    }

    virtual void init(int sample_rate) {
        classInit(sample_rate);
        instanceInit(sample_rate);
    }

    virtual void instanceInit(int sample_rate) {
        instanceConstants(sample_rate);
        instanceResetUserInterface();
        instanceClear();
    }

    virtual mydsp* clone() {
        return new mydsp();
    }

    virtual int getSampleRate() {
        return fSampleRate;
    }

    virtual void buildUserInterface(UI* ui_interface) {
        ui_interface->declare(0, "0", "");
        ui_interface->openHorizontalBox("Echof");
        ui_interface->declare(&fHslider1, "knob", "7");
        ui_interface->addHorizontalSlider("Duration", &fHslider1, FAUSTFLOAT(2e+02f), FAUSTFLOAT(1.0f), FAUSTFLOAT(2e+02f), FAUSTFLOAT(1.0f));
        ui_interface->declare(&fHslider0, "knob", "8");
        ui_interface->addHorizontalSlider("Feedback", &fHslider0, FAUSTFLOAT(0.5f), FAUSTFLOAT(0.0f), FAUSTFLOAT(1.0f), FAUSTFLOAT(0.01f));
        ui_interface->closeBox();
    }

    virtual void compute(int count, FAUSTFLOAT** RESTRICT inputs, FAUSTFLOAT** RESTRICT outputs) {
        FAUSTFLOAT* input0 = inputs[0];
        FAUSTFLOAT* output0 = outputs[0];
        float fSlow0 = 0.5f * static_cast<float>(fHslider0);
        float fSlow1 = fConst1 * static_cast<float>(fHslider1);
        for (int i0 = 0; i0 < count; i0 = i0 + 1) {
            fRec1[0] = fSlow1 + fConst2 * fRec1[1];
            float fTemp0 = fConst0 * fRec1[0];
            float fTemp1 = fTemp0 + -2.499995f;
            float fTemp2 = std::floor(fTemp1);
            float fTemp3 = fTemp0 + (-4.0f - fTemp2);
            float fTemp4 = fTemp0 + (-3.0f - fTemp2);
            int iTemp5 = static_cast<int>(fTemp1);
            float fTemp6 = fTemp0 + (-2.0f - fTemp2);
            float fTemp7 = fTemp0 + (-1.0f - fTemp2);
            float fTemp8 = fTemp7 * fTemp6;
            float fTemp9 = fTemp8 * fTemp4;
            float fTemp10 = (fTemp0 + (-5.0f - fTemp2)) * (fTemp3 * (fTemp4 * (0.041666668f * fRec0[(IOTA0 - (std::min<int>(96000, std::max<int>(0, iTemp5)) + 1)) & 131071] * fTemp6 - 0.16666667f * fTemp7 * fRec0[(IOTA0 - (std::min<int>(96000, std::max<int>(0, iTemp5 + 1)) + 1)) & 131071]) + 0.25f * fTemp8 * fRec0[(IOTA0 - (std::min<int>(96000, std::max<int>(0, iTemp5 + 2)) + 1)) & 131071]) - 0.16666667f * fTemp9 * fRec0[(IOTA0 - (std::min<int>(96000, std::max<int>(0, iTemp5 + 3)) + 1)) & 131071]) + 0.041666668f * fTemp9 * fTemp3 * fRec0[(IOTA0 - (std::min<int>(96000, std::max<int>(0, iTemp5 + 4)) + 1)) & 131071];
            fVec0[0] = fTemp10;
            fRec0[IOTA0 & 131071] = static_cast<float>(input0[i0]) + fSlow0 * (fTemp10 + fVec0[1]);
            output0[i0] = static_cast<FAUSTFLOAT>(fRec0[IOTA0 & 131071]);
            fRec1[1] = fRec1[0];
            IOTA0 = IOTA0 + 1;
            fVec0[1] = fVec0[0];
        }
    }

};

#endif
