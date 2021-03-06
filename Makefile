sifs = pam_base.sif pam_mujoco.sif learning_table_tennis_from_scratch.sif

USE_SUDO = 0

.PHONY: all
all: $(sifs)

.PHONY: clean
clean:
	rm -rf build
	rm -f $(sifs)

build/pam_mujoco_ws:
	mkdir -p build/pam_mujoco_ws
	cd build/pam_mujoco_ws; \
		git clone https://github.com/intelligent-soft-robots/treep_isr.git; \
		treep --clone-https PAM_MUJOCO;

build/cluster_utils:
	mkdir -p build
	cd build; git clone git@gitlab.tuebingen.mpg.de:mrolinek/cluster_utils.git


# Regarding the structure of this Makefile:
# There is a generic "make *.sif" target which simply builds the *.def file of
# the same name.  Images that have dependencies should resolve them in a target
# for the specific def file.

pam_mujoco.def: pam_base.sif build/pam_mujoco_ws

learning_table_tennis_from_scratch.def: pam_mujoco.sif build/cluster_utils

# build arbitrary def file
%.sif: %.def
ifeq ($(USE_SUDO),1)
	sudo singularity build $@ $<
else
	singularity build --fakeroot $@ $<
endif
