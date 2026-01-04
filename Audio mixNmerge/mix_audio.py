from pydub import AudioSegment as AS
import os

file_name = []
for root, dirs, files in os.walk("/Users/devil_edward/Documents/Audio Book/Gokarneshwor Route/"):
    file_name = files
s0 = AS.from_file("/Users/devil_edward/Desktop/attention.wav")
s1 = AS.from_file("/Users/devil_edward/Desktop/Yespachi.wav") 
s3 = AS.from_file("/Users/devil_edward/Desktop/beReady.wav")
for i in file_name:
    try:
        s2 = AS.from_wav("/Users/devil_edward/Documents/Audio Book/Gokarneshwor Route/" + i)
        C_S = s1 + s2 + s3
        C_S.export("/Users/devil_edward/Desktop/mixAudioWidBack/Gokarneshwor Next/" + i, format="wav")
        print i + "  done!!!"
    except:
        pass