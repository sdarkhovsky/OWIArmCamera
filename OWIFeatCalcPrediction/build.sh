g++ -L/usr/local/lib -I/usr/local/include/opencv2 -O0 -g3 -Wall -fmessage-length=0 -std=c++11 -o OWIFeatCalcPrediction OWIFeatCalcPrediction.cpp ais_history.cpp ais_prediction.cpp  ais_event.cpp ../joint_commands.cpp -lopencv_core -lopencv_objdetect -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs

