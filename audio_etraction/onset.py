# import modules
import librosa 
import IPython.display as ipd 

# read audio file 
x, sr = librosa.load('LaSolitude.mp3') 
ipd.Audio(x, rate=sr)

# approach 1 - onset detection and dynamic programming
tempo, beat_times = librosa.beat.beat_track(y=x, sr=sr, start_bpm=60, units='time')
onset_frames = librosa.beat.beat_track(y=x, sr=sr, onset_envelope=None, hop_length=512, units='time')
#onset_times = librosa.frames_to_time(onset_frames, sr=sr)
clicks = librosa.clicks(beat_times, sr=sr, length=len(x))
ipd.Audio(x + clicks, rate=sr)



'''
# import modules
import madmom 
import librosa
# approach 2 - dbn tracker
proc = madmom.features.beats.DBNBeatTrackingProcessor(fps=100)
act = madmom.features.beats.RNNBeatProcessor()('starboy.wav')

beat_times = proc(act)

clicks = librosa.clicks(beat_times, sr=sr, length=len(x))
ipd.Audio(x + clicks, rate=sr)'''