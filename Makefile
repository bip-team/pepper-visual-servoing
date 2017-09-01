TC?=mytoolchain-remote

all: update clean
	qibuild configure -c ${TC}
	qibuild make -j4 -c ${TC}

clean:
	rm -Rf build-*

update:
	git submodule update --init --recursive

.PHONY: clean 
