PROG = grime2cli
CC = g++
CPPFLAGS = -c -pipe -g -std=gnu++17 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../../GRIME2/grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4
# CPPFLAGS = -c -pipe -O2 -std=gnu++17 -Wall -Wextra -fPIC -I. -I.. -I../../algorithms -I/usr/include/opencv4 -I/usr/include -I/usr/local/include
LDFLAGS = -Wl,-O1 -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_video -lboost_date_time -lboost_system -lboost_filesystem -lboost_chrono
OBJS = animate.o calibbowtie.o calibexecutive.o caliboctagon.o findcalibgrid.o findline.o gifanim.o metadata.o searchlines.o octagonsearch.o visapp.o main.o

$(PROG) : $(OBJS)
	$(CC) -o $(PROG) $(OBJS) $(LDFLAGS)
main.o : ../arghandler.h
	$(CC) $(CPPFLAGS) -c ../main.cpp
animate.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/animate.cpp
calibbowtie.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/calibbowtie.cpp
calibexecutive.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/calibexecutive.cpp
caliboctagon.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/caliboctagon.cpp
findcalibgrid.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/findcalibgrid.cpp
findline.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/findline.cpp
gifanim.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/gifanim/gifanim.cpp
metadata.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/metadata.cpp
searchlines.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/searchlines.cpp
octagonsearch.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/octagonsearch.cpp
visapp.o :
	$(CC) $(CPPFLAGS) -c ../../algorithms/visapp.cpp
clean:
	rm -f core $(PROG) $(OBJS)

# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o animate.o ../algorithms/animate.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o calibbowtie.o ../algorithms/calibbowtie.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o calibexecutive.o ../algorithms/calibexecutive.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o caliboctagon.o ../algorithms/caliboctagon.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o findcalibgrid.o ../algorithms/findcalibgrid.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o findline.o ../algorithms/findline.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o gifanim.o ../algorithms/gifanim/gifanim.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o metadata.o ../algorithms/metadata.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o searchlines.o ../algorithms/searchlines.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o octagonsearch.o ../algorithms/octagonsearch.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o visapp.o ../algorithms/visapp.cpp
# g++ -c -pipe -g -std=gnu++1z -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DQT_QML_DEBUG -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o main.o ../grime2cli/main.cpp
# g++  -o grime2cli animate.o calibbowtie.o calibexecutive.o caliboctagon.o findcalibgrid.o findline.o gifanim.o metadata.o searchlines.o octagonsearch.o visapp.o main.o   -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_video -lboost_date_time -lboost_system -lboost_filesystem -lboost_chrono   

