# Cross-compile for Tarragon

This documentation explains how to use a toolchain to cross-compile EVerest for Tarragon.

For this you will need the filesystem generated from an EVerest build for Tarragon, preferably a
developer build.

You will then cross-compile using arm-linux-gnueabihf-gxx against the above-mentioned filesystem fed
to CMake as sysroot.

Remember to adapt `toolchain.cmake` for your needs.

Then, after creating the `build` directory under `everest-chargebyte`, navigate to there and run the
following command:

```bash
cmake -DCMAKE_TOOLCHAIN_FILE=/path/to/everest-chargebyte/toolchain/toolchain.cmake -DCMAKE_SYSROOT=/path/to/everest_sdk ..
```

When this ends successfully, start cross-compiling using `make`:

```bash
make install -j$(nproc)
```

Test that the resulting binaries are compiled for Tarragon as a target:

```bash
file everest-chargebyte/build/dist/libexec/everest/modules/EvseV2G/EvseV2G
```

You should get something that looks like the following:

```bash
build/dist/libexec/everest/modules/EvseV2G/EvseV2G: ELF 32-bit LSB shared object, ARM, EABI5 version 1 (GNU/Linux), dynamically linked, interpreter /lib/ld-linux-armhf.so.3, BuildID[sha1]=9f287c2dbdcacd9ecde770df4820de9218deb439, for GNU/Linux 3.2.0, not stripped
```

# Troubleshoot

You need to install the cross-compilers for Tarragon in case you do not have them:

```bash
sudo apt install build-essential libc6-armhf-cross libc6-dev-armhf-cross binutils-arm-linux-gnueabihf gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf pkg-config-arm-linux-gnueabihf
```

If you get problems regarding nodejs, make sure nodejs is installed:

```bash
sudo apt-get install nodejs-dev
```
