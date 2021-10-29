import pyaudio
import numpy as np
import wave

p = pyaudio.PyAudio()

volume = 0.5
fs = 44100
duration = 0.05
f = 100
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = fs
RECORD_SECONDS = duration
WAVE_OUTPUT_FILENAME = "low.wav"

k = 0
while k < 360:

    samples = (np.sin(2 * np.pi * np.arange(fs * duration) * k /
    fs)).astype(np.float16).tobytes()

    print(samples)

    stream = p.open(format = FORMAT,
                    channels = 1,
                    rate = RATE,
                    output = True)

    stream.write(samples)
    strNm = "waveFiles/wave" + str(k) + ".wav"
    wf = wave.open(strNm, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(samples)
    wf.close()
    stream.stop_stream()
    stream.close()
    k += 1
