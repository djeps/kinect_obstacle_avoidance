[Home](README.md)

# Python prerequisites

## Checking your python version

To check which version of Python3 is currently present on the host OS:

```bash
python3 --version
```

On my setup this yields: Python 3.8.10.

You should get the same output with:

```bash
python --version
```

If you do, it means `python` and `python3` are *sym-linked*. Otherwise, you need to use `python3` when
invoking the Python interpreter or sym-link them yourself.

We could try installing a newer version but since the image on the SD card is as we already pointed out
**very** specific to the Jetson Nano, we'll just stick to what we currently have.

## Creating a Python virtual environment

> Note: We could be needing the `python3-venv` package, and to install it which could be installed by
simply executing:

```bash
sudo apt install python3.8-venv
```

To create a virtual Python environment:

```bash
python -m venv --system-site-packages ~/venv/freenect-venv
```

This will create the virtual environment under `freenect-venv` and it will also create the parent folder
`venv` if it was not already created prior.

Furthermore, it will 'import' all the existing system packages into the virtual environment. If you
want to create a 'clean' or essentially an 'empty' virtual environment (just the bare essentials):

```bash
python -m venv ~/venv/freenect-venv
```

To activate and work within the virtual environment:

```bash
source ~/venv/freenect-venv/bin/activate
```

To deactivate it:

```bash
deactivate
```

> Note: Should you go with a 'clean' environment, the following steps are pretty much optional.

If you don't plan on doing any ML processing tasks which rely on TensorFlow and/or PyTorch, then you might
continue with a clean environment but should you install any other package that depends on any of the two,
chances are it won't work as these (along with OpenCV, TensorRT, TorchVision and CUDA) were specifically
built (from source!) for the Jetson Nano running Ubuntu 20.04 LTS.

For these reasons, I would suggest going the first (more difficult) route.

## Installing the Open3D package

> Note: As mentioned above, this is an optional step. My initial idea was to use the
[Open 3D library](https://www.open3d.org/) for visualizing point cloud data in my demo application, but it
turned out not really applicable in my case - at least not in the way I had initially imagined it.
Nevertheless, I kept the steps on installing it because it's a useful tool to have.

Looking at [the project's GitHub repos](https://github.com/isl-org/Open3D), and at the time of this writing,
Open3D is released in three versions: `0.14.1`, `0.15.1` and `0.16.0`. If we install it without specifying
a release version, by default it will go with the latest one.

In order to see the package dependency of Open3D before actually installing it, we first install the
`pipdeptree` package:

```bash
pip install pipdeptree
```

To actually see the package dependency of `open3d`:

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
pip install wheel==0.35
pip show wheel

pip install numpy==1.19.2
pip show numpy
```

Next, you could execute a 'dry run' and see what's going to be installed without actually installing
anything (and in the process get a false sense that everything will be OK):

```bash
pip install open3d==0.16.0 --dry-run
```

Then you could install the package for real and use `pipdeptree` again to see that there are 'broken'
'package version number dependencies'.

To address all of these and *save you the trouble* should you decide to go down this route, this is the
process to follow:

```bash
pip install importlib-metadata==3.6.0
pip show importlib-metadata

pip install dash==2.8
pip show dash

pip install typing-extensions==3.7.4
pip show typing-extensions

pip uninstall Werkzeug
pip install Werkzeug==3.0.0
pip show Werkzeug
```

Then you can install `open3d` and use `pipdeptree` to go over the list and confirm all is good:

```bash
pip install open3d==0.16.0
pip show open3d
pipdeptree --packages open3d
```

As a final check:

```bash
python -c 'import open3d; print(f"open3d version: {open3d.__version__}")'
```

## Installing additional packages and libraries

### `pyautogui`

```bash
pip install pyautogui
```

This library is useful for auto-detecting the screen resolution, and it's actually a required one.

In case I have missed anything and your get an error when launching the demo:

```
ModuleNotFoundError: No module named '...'
```

... you install it the same way as `pyautogui` and within an active virtual environment.

[Home](README.md)
