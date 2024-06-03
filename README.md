# chargebyte's Hardware EVerest Modules

This repository contains the control logic for chargebyte GmbH's Linux-based hardware products, implemented as modules within [EVerest](https://github.com/EVerest).

This repository includes the following modules:  
- **CbTarragonDriver**: Hardware abstraction layer for chargebyte's Tarragon board.  
- **CbTarragonPlugLock**: Driver for plug lock control on chargebyte's Tarragon board.  
- **CbTarragonDIs**: Driver for configuring digital input reference PWM on chargebyte's Tarragon board.

## Usage
To build and use these modules in EVerest, check out this repository in the same directory as everest-core, i.e., your EVerest workspace.

```bash
├── everest-core
└── everest-chargebyte
└── ....
```

Make sure to follow the instructions written [everest-core](https://github.com/EVerest/everest-core).

## Dependencies
Some modules depend on [libgpiod](git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git). However, this cannot be installed on Ubuntu with a package manager because the modules depend on a newer version of libgpiod. The libgpiod should be installed manually.

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
