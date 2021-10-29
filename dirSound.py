import time
import math
from openal.audio import SoundSink, SoundSource
from openal.loaders import load_wav_file, load_file, load_stream

if __name__ == "__main__":
    sink = SoundSink()
    sink.activate()
    source = SoundSource(position=[0, 0, 0])
    source.looping = True
    data = load_file("./waveFiles/wave234.wav")
    source.queue(data)
    sink.play(source)
    sink.update()
    t = 0
    while t < 360:
        x_pos = 20*math.sin(math.radians(t))
        y_pos = 20*math.cos(math.radians(t))
        source.position = [x_pos, y_pos, source.position[2]]
        sink.update()
        print("playing at %r" % source.position)
        time.sleep(0.1)
        t += 2
