from pydub import AudioSegment as AS
import os

file_name = []
for root, dirs, files in os.walk("/Users/devil_edward/Desktop/mixAudioWidBack/Gokarneyshwor Route2/"):
    file_name = files

s1 = AS.from_file("/Users/devil_edward/Desktop/madal.wav") 
for i in file_name:
    try:
        s2 = AS.from_wav("/Users/devil_edward/Desktop/mixAudioWidBack/Gokarneyshwor Route2/" + i)
        C_S = s1 + s2
        C_S.export("/Users/devil_edward/Desktop/mixAudioWidBackFinal/Gokarneyshwor Route/" + i, format="wav")
        print i + "  done!!!"
    except:
        pass