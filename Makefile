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
		treep --clone-https O80; \
		treep --clone-https pam_mujoco; \
		treep --clone-https mujoco_interface; \
		treep --clone-https pam_install; \
		treep --clone-https pam_configuration; \
		treep --clone-https pam_interface; \
		treep --clone-https json_helper; \
		treep --clone-https o80_pam; \
		treep --clone-https context; \
		treep --clone-https pam_models; \
		treep --clone-https frequency_monitoring; \
		treep --clone-https cmake_always_do; \
		treep --checkout-branch sguist/ball_vel_filtering; \
		treep --checkout-branch sguist/racket_state_rebase; \
		treep --checkout-branch sguist/originals_extended; \
		treep --checkout-branch sguist/get_fk;

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
