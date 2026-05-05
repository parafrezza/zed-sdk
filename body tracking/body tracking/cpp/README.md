# ZED SDK - Body Tracking

This sample shows how to detect and track human bodies in space.

## Getting Started
 - Get the latest [ZED SDK](https://www.stereolabs.com/developers/release/)
 - Check the [Documentation](https://www.stereolabs.com/docs/)

## Build the program
 - Build for [Windows](https://www.stereolabs.com/docs/app-development/cpp/windows/)
 - Build for [Linux/Jetson](https://www.stereolabs.com/docs/app-development/cpp/linux/)

### Windows commands

Open a terminal in this folder and run:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

If you only changed `.cpp` or `.hpp` files and want to recompile after parameter changes, you can usually run just:

```powershell
cmake --build build --config Release
```

If you changed `CMakeLists.txt` or want to regenerate the project files, run configure again first:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

## Run the program
*NOTE: The ZED v1 is not compatible with this module*
- Navigate to the build directory and launch the executable
- Or open a terminal and run the sample directly from this folder:

```powershell
.\build\Release\ZED_Body_Tracking_Viewer.exe
```

- To force a specific camera resolution at startup:

```powershell
.\build\Release\ZED_Body_Tracking_Viewer.exe HD720
```

## Features
 - Display bodies bounding boxes by pressing the `b` key.

## Support
If you need assistance go to our Community site at https://community.stereolabs.com/

## Zed18 Mapping

- `0`: `nose`
- `1`: `neck`
- `2`: `right_shoulder`
- `3`: `right_elbow`
- `4`: `right_wrist`
- `5`: `left_shoulder`
- `6`: `left_elbow`
- `7`: `left_wrist`
- `8`: `right_hip`
- `9`: `right_knee`
- `10`: `right_ankle`
- `11`: `left_hip`
- `12`: `left_knee`
- `13`: `left_ankle`
- `14`: `right_eye`
- `15`: `left_eye`
- `16`: `right_ear`
- `17`: `left_ear`