#pragma once
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

extern "C" {
    #include "esp_timer.h"
    #include "driver/dac_oneshot.h"
}

class SoundManager {
public:
    bool init(int dac_gpio = 25);                 // GPIO25 (DAC1) on M5Stack FIRE
    void beep(uint32_t freq_hz, uint32_t ms);     // enqueue a simple beep (square wave)
    void death_jingle();                           // enqueue a short sequence
    // Guest got hit by actor: short buzzer-like burst (distinct, noticeable)
    void hit_buzzer();
    // Guest got hit by actor: short alarm "whoop" (fast up-down sweep, ~350 ms)
    void hit_alarm_whoop();
    // Actor hit confirmation: two-tone chirp (800Hz -> 1200Hz, ~150ms total)
    void playActorHitConfirm();
    void shutdown();                               // optional cleanup
    
    // --- Soft tone additions ---
    void setMasterGain(float g0_1);                   // 0..1, global volume
    void softBeep(float freq_hz, int dur_ms, float gain0_1 = 0.30f); // sine + envelope
    void dangerChirpSoft();   // 800→600→420 soft chirp
    void successSoft();       // short pleasant confirm
    
    // --- Modern audio API ---
    // Frequency sweep (e.g., red siren). If bidir=true, sweeps there and back.
    void sweepBeep(float f0_hz, float f1_hz, int dur_ms, float gain0_1 = 0.35f, bool bidir = false);
    
    // High-level cues for radiation zones:
    void playGreenTone();       // low soft thump
    void playYellowWarning();   // double blip
    void playRedSiren();        // low industrial sweep

private:
    // command queue
    enum class CmdType { Beep, DeathJingle, Stop, SoftBeep, DangerChirpSoft, SuccessSoft, SweepBeep, PlayGreen, PlayYellow, PlayRed, PlayActorHitConfirm };
    struct Cmd { CmdType type; float f0, f1; int ms; float gain; bool bidir; };

    static void taskEntry(void* arg);
    void taskLoop();
    void playBeepBlocking(uint32_t freq_hz, uint32_t ms);
    void playDeathJingleBlocking();
    void playSoftBeepBlocking(float freq_hz, int dur_ms, float gain);
    void playDangerChirpSoftBlocking();
    void playSuccessSoftBlocking();
    void playActorHitConfirmBlocking();
    void playSweepBeepBlocking(float f0_hz, float f1_hz, int dur_ms, float gain, bool bidir);

    // timer callback that toggles DAC value (square wave)
    static void IRAM_ATTR timerCb(void* arg);
    // soft tone timer callback
    static void IRAM_ATTR timerCbSoft(void* arg);

    // helpers
    bool startTone(uint32_t freq_hz);
    void stopTone();
    bool startSoftEngine(float freq_hz);

private:
    dac_oneshot_handle_t dac_ = nullptr;
    esp_timer_handle_t   timer_ = nullptr;
    volatile uint8_t     level_ = 0;        // toggled by timer ISR
    QueueHandle_t        q_ = nullptr;
    TaskHandle_t         task_ = nullptr;
    
    // Soft tone engine state
    volatile float master_gain_ = 0.35f;
    volatile bool   soft_playing_   = false;
    volatile float  phase_          = 0.0f;
    volatile float  phase_inc_      = 0.0f;
    volatile int    soft_samples_left_ = 0;
    volatile float  note_gain_      = 0.3f;      // per-note gain
    
    // Sweep state
    volatile bool  sweep_on_      = false;
    volatile float sweep_f0_      = 0.0f;
    volatile float sweep_f1_      = 0.0f;
    volatile int   sweep_total_   = 0;
    volatile int   sweep_played_  = 0;
    volatile bool  sweep_bidir_   = false;
};
