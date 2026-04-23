# Hand Demo Tool Setup

## Prebuild Binaries:
We offer prebuild binaries build via PyInstaller which package the dependencies and code into a single file.
The executables come with a config.yaml file in which Comports and servo ID's are configured.
While servo ID's are pre configured, the Comports will likely have to be set by you.

[Windows](https://storage.seedrobotics.com/tools/Handdemo/Windows.zip),
[Linux](https://storage.seedrobotics.com/tools/Handdemo/Linux.zip)

## Running it as a Python Script: 

### Linux Setup
1) Create and activate a virtual environment:

```bash
python3.12 -m venv env
source env/bin/activate
```

2) Install dependencies:

```bash
pip install -r requirements.txt
```

3) Reduce USB latency (requires root):

```bash
sudo bash set_usb_latency.sh
```

4) Update ports in RH8D.yaml (example):
- control_port_linux: "/dev/ttyUSB0"
- sensor_port_linux: "/dev/ttyUSB1"


### Windows Setup
1) Install Python 3.12 and create a virtual environment:

```powershell
py -3.12 -m venv env
.\env\Scripts\Activate.ps1
```

2) Install dependencies:

```powershell
pip install -r requirements.txt
```

3) Configure COM ports in RH8D_L.yaml (example):
- control_port_windows: "COM9"
- sensor_port_windows: "COM10"

4) Set USB latency via Device Manager (FTDI adapters):
- Open Device Manager
- Expand "Ports (COM & LPT)"
- Right-click the adapter -> Properties -> Port Settings -> Advanced
- Set "Latency Timer (msec)" to 1
- Repeat for each adapter

## Run

```bash
python main.py
```

## Troubleshooting
- Linux serial permissions: add your user to the dialout group and re-login:

```bash
sudo usermod -aG dialout $USER
```

- Linux OpenGL issues: install or update GPU drivers (Mesa/NVIDIA) and reboot.
- Windows OpenGL issues: update your GPU driver from the vendor (NVIDIA/AMD/Intel), then restart.

## Build Executable
You can build an executable of this including all dependencies by first sourcing the environment, afterwards run the build.sh or with this command. repalce : with ; if you run it on windows.

``` bash
pyinstaller --onefile main.py --collect-all roboticstoolbox --collect-all rtbdata --collect-all spatialgeometry --collect-all spatialmath --add-data "src/urdf/:src/urdf"
```
