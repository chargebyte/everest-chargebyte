# chargebyte's Hardware EVerest Modules

This repository contains the control logic for chargebyte GmbH's Linux-based hardware products, implemented as modules within [EVerest](https://github.com/EVerest).

This repository includes the following modules:  
- **CbChargeSOMDriver**: Hardware abstraction layer for chargebyte's Charge SOM.  
- **CbTarragonDriver**: Hardware abstraction layer for chargebyte's Tarragon board.  
- **CbTarragonPlugLock**: Driver for plug lock control on chargebyte's Tarragon board.  
- **CbTarragonDIs**: Driver for configuring digital input reference PWM on chargebyte's Tarragon board.  
- **CbSystem**: Implements system wide operations for chargebyte's hardware products.
- **InfypowerDCSupply**: Driver for Infypower's BEC/BEG power modules.

## Compatibility matrix
| Tag    | EVerest release              |
|--------|------------------------------|
| 0.20.0 | 2024.9.0 or newer[^or_newer] |
| 0.19.0 | 2024.9.0 or newer[^or_newer] |
| 0.18.0 | 2024.9.0 or newer[^or_newer] |
| 0.17.0 | 2024.9.0 or newer[^or_newer] |
| 0.16.0 | 2024.8.0                     |
| 0.15.0 | 2024.8.0                     |
| 0.14.0 | 2024.7.1 <br> 2024.7.0       |
| 0.13.0 | 2024.7.1 <br> 2024.7.0       |
| 0.12.0 | 2024.6.0 <br> 2024.5.0       |
| 0.11.0 | 2024.6.0 <br> 2024.5.0       |
| 0.10.0 | 2024.6.0 <br> 2024.5.0       |
| 0.9.0  | 2024.3.0                     |

[^or_newer]: Newer releases or upstream's `main` branch may already contain incompatible changes.
             The compatibility matrix will be updated with each new tag in this repository.

## Usage
To build and use these modules in EVerest, check out this repository in the same directory as everest-core, i.e., your EVerest workspace.

```bash
├── everest-core
└── everest-chargebyte
└── ....
```

Make sure to follow the instructions written [everest-core](https://github.com/EVerest/everest-core).

## Dependencies
Some modules depend on [libgpiod](git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git). However, this cannot be installed on Ubuntu with a package manager because the modules depend on a newer version of libgpiod which Ubuntu does not provide (2.0.1). Therefore, the libgpiod should be installed manually.

```bash
sudo apt-get update
sudo apt-get install autoconf-archive
git clone git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
cd libgpiod/
git checkout v2.0.1
./autogen.sh --enable-bindings-cxx
sudo make install
```

Some other modules depend on [sigslot](https://github.com/palacaze/sigslot). This is a header-only C++ library which is not packaged in Ubuntu.
It must be installed manually, for example with the following steps:

```bash
git clone https://github.com/palacaze/sigslot.git
cd sigslot
git checkout v1.2.1
mkdir build
cd build
cmake -DSIGSLOT_COMPILE_EXAMPLES=OFF -DSIGSLOT_COMPILE_TESTS=OFF ..
sudo make install
```

Some modules depend on [libsocketcan](https://github.com/linux-can/libsocketcan).
The library is included as package in Ubuntu and also available in Yocto (both version 0.0.12).
In case manual installation is needed, use for example the following steps:

```bash
git clone https://github.com/linux-can/libsocketcan.git
cd libsocketcan
git checkout v0.0.12
./autogen.sh
mkdir build
cd build
../configure
sudo make install
```


Remember that, when cross-compiling for the target platforms, all libraries must be included in your SYSROOT environment.

## Build and install
If you need to regenerate the modules using the EVerest ev-cli tool, for example, if there is a change in the EVerest interfaces, execute the following command:

```bash
cd everest-chargebyte
ev-cli mod update <your_module_name> --everest-dir . ../everest-core
```

Note the two arguments given as `everest-dir`. This is because the modules depend on types in the **everest-chargebyte** repository, along with the known dependency on **everest-core**.

Finally, to build the modules, execute the following commands:

```bash
mkdir build # inside everest-chargebyte
cd build
make install -j$(nproc)
```

## Testing

First of all you need to install GTest

```bash

sudo apt update
sudo apt install libgtest-dev

```

You might run into some problems regarding running/compiling the tests. A quick fix is as follows

```bash

sudo apt install libgpiod-dev libgpiod2

```

To run the tests, you need to run cmake and turn on the option of compiling tests

```bash

mkdir build # inside everest-chargebyte

cd build

cmake -DBUILD_CB_TESTING=1 ..

make -j$(nproc)

```

Under the 'build' directory, change the directory to the path where the tests are compiled and use CTest

```bash

cd tests/modules/CbTarragonDriver

ctest -N # To know how many tests are captured

ctest -rerun-failed --output-on-failure # to run the tests and re-run the failed cases verbosely

```