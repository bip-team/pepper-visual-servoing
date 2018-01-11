TC?=mytoolchain-remote

all: update clean
	qibuild configure -c ${TC}
	qibuild make -j4 -c ${TC}

clean:
	rm -Rf build-*

doc:
	cd doc; ${MAKE}

doc-view:
	cd doc; ${MAKE} view

doc-clean:
	cd doc; ${MAKE} clean

update:
	git submodule update --init --recursive

.PHONY: clean doc
