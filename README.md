# How to write new modules and build EVerest

Make sure this repository is checked out in the same directory as everest-core, eg.:

```bash
├── everest-core
└── everest-cmake
```

To write a new module create a new directory inside the modules directory of this repository. The name of your directory is the name of your new module.

Create your module's manifest and the interfaces you need inside the interfaces directory.

Use ev-cli to initialize your module from its manifest. Call it from inside your repo:

```bash
cd everest-chargebyte-internal
ev-cli mod create <your_module_name>
```

If your module depends on interfaces/types that are part of another repo e.g. everest-core, the command should be expanded to be the following:

```bash
ev-cli mod create <your_module_name> --everest-dir . /path/to/dependency
```

e.g.,

```bash
cd everest-chargebyte-internal
ev-cli mod create EvseHardwareManager --everest-dir . ../everest-core
```

