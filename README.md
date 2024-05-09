# STM32F446ZE Hardware Synth

## Key features
* Five-band equalizer implemented using CMSIS biquads with Direct Memory Access (DMA)
* Optimized keypad circuitry and interrupt code to avoid polling to limit amount of time that the main thread is blocked
* Lightweight reverb filter implemented via delay lines using a ring buffer
* Triangular, Sinusoidal, and Square waveform playback with DMA
