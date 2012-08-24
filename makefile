#CFLAGS=-Wall -g -pg -O3 $(shell pkg-config --cflags opencv)
CFLAGS=-Wall -O3 $(shell pkg-config --cflags opencv)
CFLAGS+=$(shell pkg-config --cflags opencv)
LDFLAGS=$(shell pkg-config --libs opencv)

tennis_ballbot: tennis_ballbot.cpp
	gcc $(CFLAGS) -o ./$@ $< camera.cpp $(LDFLAGS) -lm -lstdc++ -O3

tennis_ballbot_debug: tennis_ballbot.cpp 
	gcc $(CFLAGS) -o bin/$@ $< camera.cpp $(LDFLAGS) -lm -lstdc++ -fno-omit-frame-pointer -O3

camera_tune: camera_tune.cpp
	gcc $(CFLAGS) -o bin/$@ $< camera.cpp $(LDFLAGS) -lm -lstdc++ -O3

clean:
	rm -f tennis_ballbot tennis_ballbot_debug
