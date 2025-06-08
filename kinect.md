# Notes on the system setup

- The host OS is Ubuntu 20.04 LTS
- Specifially prepared and for the Jetson Nano and not an official one
- This practically is at the limit of what could be deployed and run on it
- Nvidia offers no support (official or otherwise) or any drivers beyond 18.04 LTS 
- Most of the (system) packages are at very specific release numbers
- and have been built from source
- We won't be able to find or install newer releases because there are simply
  no packages maintained for the architecture of the Jetson Nano (ARM64)

We would like to maintain this and thus work in an *isolated* or a virtual Python environment for
any additional packages we might need to install with `pip3`.

# Installing the libfreenect driver and library

The Kinect v1 (or the Kinect XBOX 360) is an 'older' sensor (RGBD or RGB depth camera). It used
to be a very popular (still is I guess) sensor of its kind back in the day - especially for home
and DIY projects.

There was an entire wealth of information and a lot of work in general done and maintained by the
[OpenKinect project](http://www.openkinect.org/) but this page seems not only no longer maintained
but available!?

However, most of what's important to get you going on a Linux OS and the entire code base, has been
hosted on [the project's GitHub repo](https://github.com/OpenKinect/libfreenect).

To install the driver and the library, we simply follow the instructions in the `README.md` page.

For the sake of completeness, we would outline the steps here as well, as a confirmation that is
the process to follow and that it actually works.

There are two paths to follow:

- Install a pre-built package, or
- Install from source

We will follow the second path as it's a pretty old package and we can't be sure it has been built
from the latest version available.

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

At the time of this writting, the last couple of releases were `v0.7.0` and `v.0.7.5`. We need to be
on the `v0.7.0`. To do create a new branch based on that specific tag release and to switch to that
branch:

```bash
git checkout tags/v0.7.0 -b release-v0.7.0
```

This will helps lower the amount of work as we go further.

## STEP 3: Create a `build` folder and build the library

```bash
mkdir build
cd build
```

## STEP 4: To see a list of all 'build options'

```bash
cmake -L ..
```

Most of what we would expect to need is already enabled by default but if we want to work with the
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

## STEP 7: Set up udev rules to allow non-root access to Kinect devices

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

The user might already be part of these groups.

## STEP 8: Test that everything works as expected

```bash
freenect-glview
```

## STEP 9: Python bindings

After you build and install the library, you might need to do an extra step and copy it to the neccessary
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

# Python prerequisites

## Checking your python version

To check which version of Python3 is currently present on the host OS:

```bash
python3 --version
```

On my setup this yields: Python 3.8.10.

We could try installing a newer version but since the image on the SD card is as we already pointed out
**very** specific to the Jetson Nano, we'll just stick to what we currently have.

## Creating a Python virtual environment

We could be needing the `python3-venv` package which could be installed by simply executing:

```bash
sudo apt install python3.8-venv
```

To create a virtual Python environment:

```bash
python3 -m venv --system-site-packages ~/venv/freenect-venv
```

This will create the virtual environment under `freenect-venv` and it will also create the parent folder
`venv` if it's not already created.

Furthermore, it will 'import' all of the existing system packages into the virtual environment. If you
want to create a 'clean' or essentially an 'empty' virtual environment:

```bash
python3 -m venv ~/venv/freenect-venv
```

If you don't plan on doing any ML processing tasks which rely on TensorFlow and/or PyTorch, then you might
continue with a clean environment but should you install any other package that depends on any of the two,
chances are it won't work as these (along with OpenCV, TensorRT, TorchVision and CUDA) were specifically
built (from source!) for the Jetson Nano running Ubuntu 20.04 LTS.

For these reasons, I would suggest going the first (more difficult) route.

Anyway, to activate the virtual environment:

```bash
source ~/venv/freenect-venv/bin/activate
```

To deactivate it:

```bash
deactivate
```

## Installing the Open3D package

At the time of this writting, Open3D is released in three versions: `0.14.1`, `0.15.1` and `0.16.0`. If we
install it without specifying a release version, by default it will go with the latest one.

In order to see the package dependancy of Open3D before installing it, we first install the `pipdeptree` package:

```bash
pip3 install pipdeptree
```

To actually see the package dependancy of `open3d`:

```bash
pipdeptree --packages open3d
```

We will see 'some' warnings, out of which most are not critical. Two of them however, should be addressed:

```
* tensorflow==2.4.1
 - numpy [required: ~=1.19.2, installed: 1.18.5]
 - wheel [required: ~=0.35, installed: 0.34.2]
```

This tells us that the `tensorflow` package which was 'imported' from the existing system packages depends
on two packages which are not at the required version number.

Let's address both of them:

```bash
pip3 install wheel==0.35
pip3 show wheel

pip3 install numpy==1.19.2
pip3 show numpy
```

Next, you could execute a 'dry run' and see what's going to be installed without actually installing
anything (and in the process get a false sense that everything will be OK):

```bash
pip3 install open3d==0.16.0 --dry-run
```

Then you could install the package for real and use `pipdeptree` again to see that there are 'broken'
'package version number dependencies' which are broken.

To save you the trouble, this is the process to follow:

```bash
pip3 install importlib-metadata==3.6.0
pip3 show importlib-metadata

pip3 install dash==2.8
pip3 show dash

pip3 install typing-extensions==3.7.4
pip3 show typing-extensions

pip3 uninstall Werkzeug
pip3 install Werkzeug==3.0.0
pip3 show Werkzeug
```

Then you can install `open3d` and use `pipdeptree` to go over the list and confirm all is good:

```bash
pip3 install open3d==0.16.0
pip3 show open3d
pipdeptree --packages open3d
```

As a final check:

```bash
python3 -c 'import open3d; print(f"open3d version: {open3d.__version__}")'
```

## Installing additional packages and libraries

### `pyautogui`

```bash
pip3 install pyautogui
```

This library is useful for auto-detecting the screen resolution.
