#include "SoundManager.hpp"
#include <cstdio>

// ==== Soft tone engine constants ====
static constexpr int   SR = 8000;       // 8 kHz
static constexpr int   ATTACK_SMP  = 32;  // ~4 ms attack
static constexpr int   RELEASE_SMP = 128; // ~16 ms release

// wavetable (64-step sine, 0..255). (exact values not critical)
static const uint8_t SINE64[64] = {
  128,140,153,165,176,186,196,204,211,216,220,223,225,226,225,223,
  220,216,211,204,196,186,176,165,153,140,128,116,103, 91, 80, 70,
   60, 52, 45, 40, 36, 33, 32, 33, 36, 40, 45, 52, 60, 70, 80, 91,
  103,116,128,140,153,165,176,186,196,204,211,216,220,223,225,226
};

bool SoundManager::init(int /*dac_gpio*/)
{
    // Create DAC oneshot channel on DAC1 (GPIO25) for M5Stack FIRE
    // Note: channel is selected by enum, not raw GPIO
    dac_oneshot_config_t cfg = {};
    cfg.chan_id = DAC_CHAN_0; // DAC1 = GPIO25

    if (dac_oneshot_new_channel(&cfg, &dac_) != ESP_OK || !dac_) {
        printf("[Sound] dac_oneshot_new_channel failed\n");
        return false;
    }

    // Command queue + worker task
    q_ = xQueueCreate(8, sizeof(Cmd));
    if (!q_) return false;

    xTaskCreatePinnedToCore(taskEntry, "snd_task", 4096, this, 4, &task_, tskNO_AFFINITY);
    return task_ != nullptr;
}

void SoundManager::shutdown()
{
    if (q_) {
        Cmd c{CmdType::Stop, 0.0f, 0.0f, 0, 0.0f, false};
        xQueueSend(q_, &c, 0);
    }
}


void SoundManager::beep(uint32_t freq_hz, uint32_t ms)
{
    if (!q_) return;
    Cmd c{CmdType::Beep, (float)freq_hz, 0.0f, (int)ms, 0.0f, false};
    xQueueSend(q_, &c, 0);
}


void SoundManager::death_jingle()
{
    if (!q_) return;
    Cmd c{CmdType::DeathJingle, 0.0f, 0.0f, 0, 0.0f, false};
    xQueueSend(q_, &c, 0);
}

void SoundManager::taskEntry(void* arg)
{
    static_cast<SoundManager*>(arg)->taskLoop();
}

void SoundManager::taskLoop()
{
    Cmd cmd{};
    while (xQueueReceive(q_, &cmd, portMAX_DELAY) == pdTRUE) {
        switch (cmd.type) {
            case CmdType::Beep:
                playBeepBlocking((uint32_t)cmd.f0, (uint32_t)cmd.ms);
                break;
            case CmdType::DeathJingle:
                playDeathJingleBlocking();
                break;
            case CmdType::SoftBeep:
                playSoftBeepBlocking(cmd.f0, cmd.ms, cmd.gain);
                break;
            case CmdType::DangerChirpSoft:
                playDangerChirpSoftBlocking();
                break;
            case CmdType::SuccessSoft:
                playSuccessSoftBlocking();
                break;
            case CmdType::SweepBeep:
                playSweepBeepBlocking(cmd.f0, cmd.f1, cmd.ms, cmd.gain, cmd.bidir);
                break;
            case CmdType::PlayGreen:
                playSoftBeepBlocking(200.f, 120, 0.24f); // 🟢 soft thump
                break;
            case CmdType::PlayYellow: // 🟡 double blip
                playSoftBeepBlocking(500.f, 80, 0.28f);
                vTaskDelay(pdMS_TO_TICKS(120));
                playSoftBeepBlocking(700.f, 80, 0.28f);
                break;
            case CmdType::PlayRed:
                playSweepBeepBlocking(400.f, 800.f, 700, 0.33f, false); // 🔴 low siren
                break;
            case CmdType::PlayActorHitConfirm:
                playActorHitConfirmBlocking(); // 🎯 two-tone confirmation chirp
                break;
            case CmdType::Stop:
                stopTone();
                vTaskDelete(nullptr);
                return;
        }
    }
}

// ---- Tone generation via esp_timer toggling DAC (square wave) ----

bool SoundManager::startTone(uint32_t freq_hz)
{
    stopTone(); // ensure clean state

    if (freq_hz == 0 || !dac_) return false;

    // Create periodic timer at half-period to toggle the level
    const int64_t half_period_us = static_cast<int64_t>(500000) / (freq_hz); // 1e6/2 = 500000
    esp_timer_create_args_t args = {};
    args.callback = &SoundManager::timerCb;
    args.arg = this;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "snd_tone";

    if (esp_timer_create(&args, &timer_) != ESP_OK) {
        printf("[Sound] esp_timer_create failed\n");
        timer_ = nullptr;
        return false;
    }

    level_ = 0;
    // Start toggling
    esp_err_t err = esp_timer_start_periodic(timer_, half_period_us);
    if (err != ESP_OK) {
        printf("[Sound] esp_timer_start_periodic failed\n");
        esp_timer_delete(timer_);
        timer_ = nullptr;
        return false;
    }
    return true;
}

void SoundManager::stopTone()
{
    if (timer_) {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }
    if (dac_) {
        // Drive to 0 to avoid lingering DC level (click/pop reduction)
        dac_oneshot_output_voltage(dac_, 0);
    }
}

void IRAM_ATTR SoundManager::timerCb(void* arg)
{
    auto* self = static_cast<SoundManager*>(arg);
    // Toggle between low and high output (simple square wave)
    uint8_t next = self->level_ ? 0 : 255;
    self->level_ = next;
    // Keep ISR short: just write the sample
    dac_oneshot_output_voltage(self->dac_, next);
}

// ---- Blocking sequences (executed in sound task) ----

void SoundManager::playBeepBlocking(uint32_t freq_hz, uint32_t ms)
{
    if (!startTone(freq_hz)) return;
    vTaskDelay(pdMS_TO_TICKS(ms));
    stopTone();
}

void SoundManager::playDeathJingleBlocking()
{
    // Simple descending tri-tone (tweak to taste)
    const struct { uint32_t f; uint32_t d; } seq[] = {
        { 1200, 160 }, { 900, 160 }, { 600, 240 },
    };
    for (auto &s : seq) {
        playBeepBlocking(s.f, s.d);
        vTaskDelay(pdMS_TO_TICKS(40)); // tiny gap
    }
}

// --- NEW: Guest-hit buzzer (Option 1) ---
// A low, short, buzzer-like burst that reads as a "shock/penalty",
// distinct from status tones. Uses existing beep() (square) for grit.
// If you later swap to the soft engine, just change internals here.
void SoundManager::hit_buzzer()
{
    // 140 Hz, 250 ms -> "electrical buzz" feel without being shrill
    // (Square at low freq is perceived as a buzzer. Tweak to 120–160 Hz if needed.)
    beep(140, 250);
}


void SoundManager::hit_alarm_whoop()
{
    // Use the red siren sweep (400Hz → 800Hz, 700ms) for consistent audio theme
    sweepBeep(400.f, 800.f, 700, 0.33f, false); // Same as red siren
}

// --- NEW: Actor hit confirmation ---
// Two-tone chirp for actor when they successfully hit a guest
void SoundManager::playActorHitConfirm()
{
    if (!q_) return;
    Cmd c{CmdType::PlayActorHitConfirm, 0, 0, 0, 0.0f, false};
    xQueueSend(q_, &c, 0);
}

// ---- Soft tone API ----

void SoundManager::setMasterGain(float g){
    if (g < 0) g = 0; 
    if (g > 1) g = 1;
    master_gain_ = g;
}

void SoundManager::softBeep(float freq_hz, int dur_ms, float gain0_1){
    if (!q_) return;
    Cmd c{CmdType::SoftBeep, freq_hz, 0.0f, dur_ms, gain0_1, false};
    xQueueSend(q_, &c, 0);
}

void SoundManager::dangerChirpSoft(){
    if (!q_) return;
    Cmd c{CmdType::DangerChirpSoft, 0.0f, 0.0f, 0, 0.0f, false};
    xQueueSend(q_, &c, 0);
}

void SoundManager::successSoft(){
    if (!q_) return;
    Cmd c{CmdType::SuccessSoft, 0.0f, 0.0f, 0, 0.0f, false};
    xQueueSend(q_, &c, 0);
}

// --- Modern audio API ---
void SoundManager::sweepBeep(float f0_hz, float f1_hz, int dur_ms, float gain0_1, bool bidir){
    if (!q_) return;
    Cmd c{CmdType::SweepBeep, f0_hz, f1_hz, dur_ms, gain0_1, bidir};
    xQueueSend(q_, &c, 0);
}

void SoundManager::playGreenTone(){
    if (!q_) return;
    Cmd c{CmdType::PlayGreen, 0.0f, 0.0f, 0, 0.0f, false};
    xQueueSend(q_, &c, 0);
}

void SoundManager::playYellowWarning(){
    if (!q_) return;
    Cmd c{CmdType::PlayYellow, 0.0f, 0.0f, 0, 0.0f, false};
    xQueueSend(q_, &c, 0);
}

void SoundManager::playRedSiren(){
    if (!q_) return;
    Cmd c{CmdType::PlayRed, 0.0f, 0.0f, 0, 0.0f, false};
    xQueueSend(q_, &c, 0);
}

// ---- Soft tone engine ----

bool SoundManager::startSoftEngine(float freq_hz){
    stopTone();  // stop any previous timer

    if (!dac_ || freq_hz <= 0.f) return false;

    // Configure audio ISR at SR (8kHz)
    esp_timer_create_args_t args = {};
    args.callback = &SoundManager::timerCbSoft;
    args.arg = this;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "snd_soft";
    if (esp_timer_create(&args, &timer_) != ESP_OK) { timer_ = nullptr; return false; }

    phase_ = 0.0f;
    phase_inc_ = freq_hz / (float)SR; // 0..1 wraps
    soft_playing_ = true;

    // 1e6 / SR microseconds per tick
    return esp_timer_start_periodic(timer_, 1000000 / SR) == ESP_OK;
}

void IRAM_ATTR SoundManager::timerCbSoft(void* arg)
{
    auto* self = static_cast<SoundManager*>(arg);
    if (!self->soft_playing_ || self->soft_samples_left_ <= 0) {
        self->soft_playing_ = false;
        dac_oneshot_output_voltage(self->dac_, 0);
        esp_timer_stop(self->timer_);
        return;
    }
    
    // Simple linear attack/decay
    float env = 1.0f;
    int N = self->soft_samples_left_;
    if (N < RELEASE_SMP) env = (float)N / (float)RELEASE_SMP; // decay tail

    // Sweep: linearly adjust phase_inc_ over duration (or bidirectional)
    if (self->sweep_on_ && self->sweep_total_ > 0) {
        int sweep_played = self->sweep_played_;
        self->sweep_played_ = sweep_played + 1;
        float t = (float)(sweep_played + 1) / (float)self->sweep_total_;
        if (t > 1.f) t = 1.f;
        float f;
        if (!self->sweep_bidir_) {
            f = self->sweep_f0_ + (self->sweep_f1_ - self->sweep_f0_) * t;
        } else {
            // Go there (0..0.5) and back (0.5..1.0)
            float tt = (t <= 0.5f) ? (t*2.f) : (2.f - t*2.f);
            f = self->sweep_f0_ + (self->sweep_f1_ - self->sweep_f0_) * tt;
        }
        self->phase_inc_ = f / (float)SR;
    }

    // Oscillator
    self->phase_ += self->phase_inc_;
    if (self->phase_ >= 1.0f) self->phase_ -= 1.0f;
    int idx = (int)(self->phase_ * 64.0f) & 63;

    int centered = (int)SINE64[idx] - 128;                   // -128..127
    float g = self->master_gain_ * self->note_gain_ * env;   // 0..1
    int sample = 128 + (int)(centered * g);                  // 0..255
    if (sample < 0) sample = 0; 
    if (sample > 255) sample = 255;

    dac_oneshot_output_voltage(self->dac_, (uint8_t)sample);
    int samples_left = self->soft_samples_left_;
    self->soft_samples_left_ = samples_left - 1;
}

// ---- Blocking soft wrappers (inside sound task) ----

void SoundManager::playSoftBeepBlocking(float freq_hz, int dur_ms, float gain){
    if (gain < 0) gain = 0; 
    if (gain > 1) gain = 1;
    note_gain_ = gain;
    soft_samples_left_ = (dur_ms * SR) / 1000;
    if (!startSoftEngine(freq_hz)) return;
    // busy-wait in the sound task by polling samples_left_ with short sleeps
    while (soft_playing_) { vTaskDelay(pdMS_TO_TICKS(2)); }
}

void SoundManager::playDangerChirpSoftBlocking(){
    const struct { float f; int d; float g; } seq[] = {
        { 800.f,  70, 0.32f },
        { 600.f,  70, 0.32f },
        { 420.f, 100, 0.32f },
    };
    for (auto &s : seq) {
        playSoftBeepBlocking(s.f, s.d, s.g);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void SoundManager::playSuccessSoftBlocking(){
    const struct { float f; int d; float g; } seq[] = {
        { 1046.f, 40, 0.25f }, // C6
        { 1318.f, 55, 0.25f }, // E6
    };
    for (auto &s : seq) {
        playSoftBeepBlocking(s.f, s.d, s.g);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void SoundManager::playActorHitConfirmBlocking(){
    // Two-tone chirp: 800Hz → 1200Hz for actor hit confirmation
    playBeepBlocking(800, 70);   // first chirp
    vTaskDelay(pdMS_TO_TICKS(10)); // small gap
    playBeepBlocking(1200, 70);  // second chirp (higher, confirmation feel)
}

void SoundManager::playSweepBeepBlocking(float f0_hz, float f1_hz, int dur_ms, float gain, bool bidir){
    if (gain < 0) gain = 0; 
    if (gain > 1) gain = 1;
    if (dur_ms <= 0 || f0_hz <= 0 || f1_hz <= 0) return;
    
    stopTone(); // stop any previous timer
    note_gain_ = gain;
    phase_ = 0.f;
    phase_inc_ = f0_hz / (float)SR;
    soft_samples_left_ = (dur_ms * SR) / 1000;
    sweep_on_ = true; 
    sweep_f0_ = f0_hz; 
    sweep_f1_ = f1_hz; 
    sweep_total_ = soft_samples_left_; 
    sweep_played_ = 0; 
    sweep_bidir_ = bidir;
    soft_playing_ = true;
    
    if (!startSoftEngine(f0_hz)) { 
        soft_playing_ = false; 
        sweep_on_ = false;
        return; 
    }
    
    // busy-wait in the sound task by polling samples_left_ with short sleeps
    while (soft_playing_) { vTaskDelay(pdMS_TO_TICKS(2)); }
}
