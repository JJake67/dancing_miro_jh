import librosa
import librosa.display
import matplotlib.pyplot as plt
import numpy as np

y, sr = librosa.load('smooth (2).wav')
hop_length = 512 

# Compute local onset autocorrelation
oenv = librosa.onset.onset_strength(y=y, sr=sr, hop_length=hop_length)
times = librosa.times_like(oenv, sr=sr, hop_length=hop_length)
tempogram = librosa.feature.tempogram(onset_envelope=oenv, sr=sr,
                                      hop_length=hop_length)

# Estimate the global tempo for display purposes
#    tempo = librosa.beat.tempo(onset_envelope=oenv, sr=sr,
#                            hop_length=hop_length)[0]
tempo = librosa.feature.rhythm.tempo(onset_envelope=oenv, sr=sr,
                           hop_length=hop_length)[0]

fig, ax = plt.subplots(nrows=2, figsize=(15, 6))

ax[0].plot(times, oenv, label='Onset strength')
ax[0].label_outer()
ax[0].legend(frameon=True)
librosa.display.specshow(tempogram, sr=sr, hop_length=hop_length,
                         x_axis='time', y_axis="off", cmap='magma',
                         ax=ax[1])
plt.show()

ax[1].axhline(tempo, color='w', linestyle='--', alpha=1,
            label='Estimated tempo={:g}'.format(tempo))
ax[1].legend(loc='upper right')
ax[1].set(title='Tempogram')
print ("Estimated tempo bpm = " ,tempo)
"""
#compute fundamental freq 
fmin=librosa.note_to_hz('C0')
fmax=librosa.note_to_hz('C7')

f0, voiced_flag, voiced_prob = librosa.pyin(y,fmin=fmin,fmax=fmax,pad_mode='constant',n_thresholds=10,max_transition_rate=100,sr=sr)

times = librosa.times_like(f0)
avgFreq = np.average(times)
print(avgFreq)

avg2 = np.average(f0)
print(avg2)

print("f0 len:", len(f0))
#print("f0 1:", f0(0))
#print(f0)
total = 0
totalLength = 0 
for x in range(len(f0)):
    if not np.isnan(f0[x]):
        total = f0[x] + total
        totalLength = totalLength + 1 
        #print(f0[x])
print(total)
print(totalLength)
total = total / totalLength
print("freq: ",total)

fig, ax = plt.subplots()
D = librosa.amplitude_to_db(np.abs(librosa.stft(y)), ref=np.max)
img = librosa.display.specshow(D, x_axis='time', y_axis='log', ax=ax)
ax.set(title='pYIN fundamental frequency estimation')
fig.colorbar(img, ax=ax, format="%+2.f dB")
ax.plot(times, f0, label='f0', color='cyan', linewidth=3)
ax.legend(loc='upper right')
avgFreq = sum(f0)/len(f0)
#plt.show()
"""

#client_id = os.getenv("CLIENT_ID")
client_id = "acbd6c4e089e4c9cb071ce9d3e4a9583"
#client_secret = os.getenv("CLIENT_SECRET")
client_secret = "a35830533528441f9ae304893a279b38"