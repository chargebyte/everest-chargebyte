# chargebyte's Hardware EVerest Modules

This repository contains the control logic for chargebyte GmbH's Linux-based hardware products, implemented as modules within [EVerest](https://github.com/EVerest).

This repository includes the following modules:  
- **CbChargeSOMDriver**: Hardware abstraction layer for chargebyte's Charge SOM.  
- **CbTarragonDriver**: Hardware abstraction layer for chargebyte's Tarragon board.  
- **CbTarragonPlugLock**: Driver for plug lock control on chargebyte's Tarragon board.  
- **CbTarragonDIs**: Driver for configuring digital input reference PWM on chargebyte's Tarragon board.  
- **CbSystem**: Implements system wide operations for chargebyte's hardware products.

## Compatibility matrix
| Tag | EVerest release |
|----------|----------|
| 0.16.0 | 2024.8.0 |
| 0.15.0 | 2024.8.0 |
| 0.14.0 | 2024.7.1 <br> 2024.7.0 |
| 0.13.0 | 2024.7.1 <br> 2024.7.0 |
| 0.12.0 | 2024.6.0 <br> 2024.5.0 |
| 0.11.0 | 2024.6.0 <br> 2024.5.0 |
| 0.10.0 | 2024.6.0 <br> 2024.5.0 |
| 0.9.0  | 2024.3.0 |

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

Remember that, when cross-compiling for the target platforms, both libraries must be included in your SYSROOT environment.

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
