all:
		gcc -Ofast pixelwaterfall.c -o pixelwaterfall -lm -lpulse -lpulse-simple -lfftw3 -lsystemd -march=native -mtune=native 

clean:
		rm pixelwaterfall 

