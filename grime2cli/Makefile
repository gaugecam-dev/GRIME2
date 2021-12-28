PROG = grime2cli
CC = g++
CPPFLAGS = -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4
LDFLAGS = -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_video -lboost_date_time -lboost_system -lboost_filesystem -lboost_chrono
OBJS = calibbowtie.o calibexecutive.o calibstopsign.o findcalibgrid.o findline.o gifanim.o metadata.o visapp.o animate.o main.o

$(PROG) : $(OBJS)
	$(CC) -o $(PROG) $(OBJS) $(LDFLAGS)
main.o : arghandler.h
	$(CC) $(CPPFLAGS) -c main.cpp
gifanim.o :
	$(CC) $(CPPFLAGS) -c ../algorithms/gifanim/gifanim.cpp
calibbowtie.o :
	$(CC) $(CPPFLAGS) -c ../algorithms/calibbowtie.cpp
calibexecutive.o :
	$(CC) $(CPPFLAGS) -c ../algorithms/calibexecutive.cpp
calibstopsign.o :
	$(CC) $(CPPFLAGS) -c ../algorithms/calibstopsign.cpp
findcalibgrid.o :
	$(CC) $(CPPFLAGS) -c ../algorithms/findcalibgrid.cpp
findline.o :
	$(CC) $(CPPFLAGS) -c ../algorithms/findline.cpp
metadata.o :
	$(CC) $(CPPFLAGS) -c ../algorithms/metadata.cpp
visapp.o :
	$(CC) $(CPPFLAGS) -c ../algorithms/visapp.cpp
animate.o :
	$(CC) $(CPPFLAGS) -c ../algorithms/animate.cpp
clean:
	rm -f core $(PROG) $(OBJS)

# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o animate.o ../algorithms/animate.cpp
# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o calibbowtie.o ../algorithms/calibbowtie.cpp
# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o calibexecutive.o ../algorithms/calibexecutive.cpp
# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o calibstopsign.o ../algorithms/calibstopsign.cpp
# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o findcalibgrid.o ../algorithms/findcalibgrid.cpp
# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o findline.o ../algorithms/findline.cpp
# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o gifanim.o ../algorithms/gifanim/gifanim.cpp
# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o metadata.o ../algorithms/metadata.cpp
# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o visapp.o ../algorithms/visapp.cpp
# g++ -c -pipe -g -std=gnu++11 -Wall -Wextra -fPIC -DBOOST_ALL_NO_LIB -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I../grime2cli -I. -I/usr/local/include -I/usr/local/include/opencv4 -I/home/kchapman/Qt/5.15.2/gcc_64/mkspecs/linux-g++ -o main.o ../grime2cli/main.cpp
# g++  -o grime2cli animate.o calibbowtie.o calibexecutive.o calibstopsign.o findcalibgrid.o findline.o gifanim.o metadata.o visapp.o main.o   -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_video -lboost_date_time -lboost_system -lboost_filesystem -lboost_chrono   
