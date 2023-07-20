# Preliminary Basecamp

This is the temporary storage location for basecamp modules until we have a dedicated strategy on how to weave non-public parts into public code.

# How to write new modules and build EVerest

Make sure this repository is checked out in the same directory as everest-core, eg.:

```bash
├── everest-core
└── everest-redis
```

To write a new module create a new directory inside the modules directory of this repository. The name of your directory is the name of your new module.

Create your module's manifest and the interfaces you need inside the interfaces directory.

Use ev-cli to initialize your module from its manifest

```bash
ev-cli mod create <your_module_name>
```

Use ev-cli to create the necessary headers for your interfaces.

```bash
ev-cli if gh <your_interface_name>
```

See the [FutechMqtt](modules/FutechMqtt/) module as an example.

To use the module you created within EVerest, add the name of the module to the [CMakeLists.txt](modules/CMakeLists.txt) file in the modules directory.

```cmake
list(APPEND EVEREST_MODULES_LIST
    <your_module_name>
)

```

Now you can build EVerest with your new module(s). Make sure you are in the root directory of this repository.

```bash
cd build
cmake ..
make
make install
```

Make sure to modify or create your the [EVerest config](config) to include your new module(s).

```bash
.././run_sil-futech.sh
```

# Troubleshoot

You need to install the cross-compilers for Tarragon in case you do not have them

```bash
sudo apt install build-essential libc6-armhf-cross libc6-dev-armhf-cross binutils-arm-linux-gnueabihf gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf pkg-config-arm-linux-gnueabihf
```

If you get No rule to target:
- Remove the build/\_deps folder
- Remove CPM cache

```bash
rm -rf /home/<user>/.cache/CPM
```
