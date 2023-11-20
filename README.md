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

#### libgpiod for Ubuntu 22.04
If you want to compile the content of this repository natively and not cross-compile, you need a version of libgpiod newer than the one Ubuntu 22 offers, so we clone the libgpiod repository somewhere local, build and install it ourselves.

```bash
git clone git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
cd libgpiod/
./autogen.sh --enable-bindings-cxx
sudo make install
```

#### Testing
First of all you need to install GTest

```bash
sudo apt install libgtest-dev
```

You might run into some problems regarding running/compiling the tests. A quick fix is as follows

```bash
sudo apt install libgpiod2
```

To run the tests, you need to run cmake and turn on the option of compiling tests

```bash
mkdir build # inside everest-chargebyte-internal
cd build
cmake -DBUILD_CB_TESTING=1 ..
make install -j$(nproc)
```

Under the 'build' directory, change the directory to the path were the tests are compiled and use CTest

```bash
cd modules/CbTarragonDriver/tests
ctest -N # To know how many tests are captured
ctest -rerun-failed --output-on-failure # to run the tests and re-run the failed cases verbosely
```
