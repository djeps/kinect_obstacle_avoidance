[Home](README.md)

# Installing the `libfreenect` driver and library

Installing the driver is probably the more challenging part but definitely not overly difficult.

The Kinect v1 (or the Kinect Xbox 360) is an 'older' sensor (RGB-D or *RGB depth camera*). It used
to be a very popular sensor (still is I guess) of its kind back in the day - especially for home
and DIY projects.

There was an entire wealth of information and a lot of work in general done and maintained by the
[OpenKinect project](http://www.openkinect.org/), but this page seems no longer maintained or available!?

However, most of what's important to get you started on a Linux OS and the entire code base, has been
hosted on [the project's GitHub repo](https://github.com/OpenKinect/libfreenect).

To install the driver and the library, we simply follow the instructions in [the project's
`README.md`](https://github.com/OpenKinect/libfreenect/blob/master/README.md).

For the sake of completeness, we would outline the steps here as well, as confirmation that's the
process to follow and that it actually works.

There are two paths to follow:

- Install a pre-built package, or
- Install from source

We will follow the second path as it's a pretty old package, and we can't be sure it has been built
from the latest release available.

## STEP 1: Install build dependencies

```bash
sudo apt-get install git cmake build-essential libusb-1.0-0-dev freeglut3-dev libxmu-dev libxi-dev
```

Most of these might already be on the host OS.

## STEP 2: Clone the repo and change directory

```bash
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
```

Once you clone the repo, you will see you're on the `master` branch. I have found out experimentally,
you need to be a couple of release tags behind the `master` branch in order to lower the 'version
dependency` with other existing packages.

To see a list of tags:

```bash
git tag
```

At the time of this writing, the last couple of releases were `v0.7.0` and `v.0.7.5`. We need to be
on the `v0.7.0`. To do so, create a new branch based on that specific release tag and to switch to it
in one go:

```bash
git checkout tags/v0.7.0 -b release-v0.7.0
```

## STEP 3: Create a `build` folder and build the library

```bash
mkdir build
cd build
```

## STEP 4: To see a list of all 'build options'

```bash
cmake -L ..
```

Most of what we would expect and need is already enabled by default but if we want to work with the
sensor from within Python, we need to build it with the proper Python binding.

## STEP 5: Build the SW

```bash
cmake .. -DBUILD_PYTHON3=ON
make
```

## STEP 6: Install the SW

```bash
sudo make install
sudo ldconfig
```

## STEP 7: Set up `udev` rules to allow non-root access to Kinect devices

### STEP 7.1: Create a new rules file

```bash
sudo nano /etc/udev/rules.d/51-kinect.rules
```

### STEP 7.2: Add the following content

```
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
```

### STEP 7.3: Reload `udev` rules

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### STEP 7.4: Add your user to the appropriate groups

```bash
sudo adduser $USER video
sudo adduser $USER plugdev
```

> Note: The user might already be part of these groups.

## STEP 8: Test that everything works as expected

```bash
freenect-glview
```

## STEP 9: Python bindings

After you build and install the library, you might need to do an extra step and copy it to the necessary
folder:

```bash
sudo cp wrappers/python/python3/freenect.so /usr/local/lib/python3.8/dist-packages/
```

Then you can test if everything is OK with:

```bash
python3 -c "import freenect; print('import freenect: OK.');"
```

`freenect` has no attribute `__version__` or `version` for that matter. Plus, it was built from source and
not '`pip` installed', so something like: `print(freenect.__version__)` will not work.

[Home](README.md)
