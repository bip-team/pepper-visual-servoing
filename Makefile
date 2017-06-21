TC?=mytoolchain-remote

all: clean
	qibuild configure -c ${TC}
	qibuild make -j4 -c ${TC}

clean:
	rm -Rf build-*

.PHONY: clean 
