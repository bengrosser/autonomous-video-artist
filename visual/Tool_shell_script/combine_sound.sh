#cp ./sort_with_H.avi sort_with_H_copy.avi
#cp ./sort_with_I.avi sort_with_I_copy.avi
#cp ./sort_with_S.avi sort_with_S_copy.avi
#cp ./sort_with_V.avi sort_with_V_copy.avi

#ffmpeg -i sort_with_H_copy.avi -i ./src_video/matrix-woman-red.aac sort_with_H_with_audio.avi
#ffmpeg -i sort_with_I_copy.avi -i ./src_video/matrix-woman-red.aac sort_with_I_with_audio.avi
#ffmpeg -i sort_with_S_copy.avi -i ./src_video/matrix-woman-red.aac sort_with_S_with_audio.avi
#ffmpeg -i sort_with_V_copy.avi -i ./src_video/matrix-woman-red.aac sort_with_V_with_audio.avi

#rm sort_with_H_copy.avi
rm sort_with_I_copy.avi
rm sort_with_S_copy.avi
rm sort_with_V_copy.avi

echo finished
