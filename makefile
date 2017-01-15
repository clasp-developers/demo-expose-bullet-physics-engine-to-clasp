
include $(wildcard local.config)

export DEMO_HOME = $(shell pwd)

export OPTIONS = -I$(CLASP_HOME)/include \
		-I$(CLASP_HOME)/src/main \
		-I$(CLASP_HOME)/build/$(CLASP_RUNTIME) \
		-I/usr/local/include/bullet \
		-c -emit-llvm \
		-std=c++11 \
		-Wno-deprecated-register \
		-Wno-inconsistent-missing-override

export CLANG = $(EXTERNALS_CLASP_DIR)/build/release/bin/clang++

all: exposeBullet.bc

exposeBullet.bc: exposeBullet.cc
	$(CLANG) $(OPTIONS) -o exposeBullet.bc exposeBullet.cc

test:
	$(CLASP_HOME)/build/$(CLASP_RUNTIME)/cclasp-$(CLASP_RUNTIME) -l "hello-world.lisp"

clean:
	rm -f *.bc *~
