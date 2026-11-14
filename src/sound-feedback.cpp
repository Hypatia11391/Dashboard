#include <iostream>
#include <cmath>

#include "src/sound-feedback.hpp"
#include "src/constants.hpp"


int SoundFeedback::sinCallback(
    const void* inputBuffer,
    void* outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo,
    PaStreamCallbackFlags statusFlags,
    void* userData
) {
    WaveSound* data = (WaveSound*) userData;
    float* out = (float*) outputBuffer;
    double amplitude = data->amplitude;
    double frequency = data->frequency;
    for (unsigned long i=0; i<framesPerBuffer; i++)
    {
        *out++ = static_cast<float>(amplitude * std::sin(-2 * Consts::PI * frequency * data->phase));
        data->phase -= FRAME_TIME;
    }
    return 0;
}

void SoundFeedback::handleError(PaError err) {
    if (err != paNoError) {
        std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
        std::abort();
    }
}

SoundFeedback::SoundFeedback() {
    PaError err = Pa_Initialize();
    SoundFeedback::handleError(err);
}

void SoundFeedback::cleanUp() {
    PaError err = Pa_Terminate();
    SoundFeedback::handleError(err);
}

void SoundFeedback::start(WaveSound sound) {
    this->sound = sound;
    PaError err = Pa_OpenDefaultStream( &stream,
                                0,          /* no input channels */
                                1,          /* stereo---mono output */
                                paFloat32,  /* 32 bit floating point output */
                                SAMPLE_RATE,
                                256,        /* frames per buffer, i.e. the number
                                                   of sample frames that PortAudio will
                                                   request from the callback. Many apps
                                                   may want to use
                                                   paFramesPerBufferUnspecified, which
                                                   tells PortAudio to pick the best,
                                                   possibly changing, buffer size.*/
                                &SoundFeedback::sinCallback, /* this is your callback function */
                                &this->sound ); /*This is a pointer that will be passed to
                                                   your callback*/
    SoundFeedback::handleError(err);
    PaError err2 = Pa_StartStream(stream);
    SoundFeedback::handleError(err2);
}

void SoundFeedback::stop() {
    PaError err = Pa_StopStream(stream);
    SoundFeedback::handleError(err);
}
