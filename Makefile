CFLAGS+=-O3
OPENCV_INCLUDE=-I/usr/include/opencv -I/usr/local/include
OPENCV_LIBS=-L/usr/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_flann -lopencv_calib3d

# OpenCV2 library is required
CFLAGS+=$(OPENCV_INCLUDE)

LDFLAGS+=$(OPENCV_LIBS)

SHELL_OBJS=test.o 

PROG=test

$(PROG):  $(SHELL_OBJS)
	$(CC) $(SHELL_OBJS) -o $(PROG) $(LDFLAGS)

%.o: %.cpp
	$(CC) $(CFLAGS) -c $<

all: $(PROG)

default: $(PROG)

clean:
	rm -f  $(SHELL_OBJS) $(PROG)



