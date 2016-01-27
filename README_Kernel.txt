################################################################################

1. How to Build
	- get Toolchain
		From android git server , codesourcery and etc ..
		 - aarch64-linux-android-4.9

	- edit Makefile
		edit "CROSS_COMPILE" to right toolchain path(You downloaded).
		  EX)   CROSS_COMPILE= $/android/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.6/bin/arm-linux-androideabi-
          Ex)   CROSS_COMPILE=android/prebuilts/gcc/aarch64-linux-android-4.9/bin/arm-linux-androideabi-  // use your toolchain location
		  or
		  Ex)	export CROSS_COMPILE=aarch64-linux-android-
		  Ex)	export PATH=$PATH:<toolchain_parent_dir>/aarch64-linux-android-4.9/bin

		$ export ARCH=arm64
		$ make msm_defconfig
		$ make

2. Output files
	- Kernel : arch/arm64/boot/Image.gz

3. How to Clean
		$ make ARCH=arm64 distclean
################################################################################
