from pydub import AudioSegment as AS
import os

file_name = []
for root, dirs, files in os.walk("/Users/devil_edward/Desktop/mixAudioWidBack/Gokarneshwor Next/"):
    file_name = files

s1 = AS.from_file("/Users/devil_edward/Desktop/back.wav") 
for i in file_name:
    #print "/Users/devil_edward/Documents/Audio Book/Ringroad Route/" + i
    try:
        s2 = AS.from_file("/Users/devil_edward/Desktop/mixAudioWidBack/Gokarneshwor Next/" + i)
        s3 = s1.overlay(s2 , position=4000)
        s3.export("/Users/devil_edward/Desktop/mixAudioWidBack/Gokarneshwor Next2/" + i, format="wav")
        print i + "  done!!!"
    except:
        pass