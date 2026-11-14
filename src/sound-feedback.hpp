#pragma once

#include "portaudio.h"


inline constexpr double SAMPLE_RATE = 44100.0;
inline constexpr double FRAME_TIME = 1 / SAMPLE_RATE;


struct WaveSound {
    double amplitude = 0.25;
    double frequency = 440;
    double phase = 0;
};


class SoundFeedback {
    public:
        SoundFeedback();
        void cleanUp();
        void start(WaveSound sound);
        void stop();
    
    private:
        static void handleError(PaError err);
        
        static int sinCallback(
            const void* inputBuffer,
            void* outputBuffer,
            unsigned long framesPerBuffer,
            const PaStreamCallbackTimeInfo* timeInfo,
            PaStreamCallbackFlags statusFlags,
            void* userData
        );
        
        WaveSound sound;
        PaStream* stream;
};