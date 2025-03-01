# About Field Calibration

Field calibration is a process of recording the position of all tags on each specific field so that the error in tag placement can be accounted for.

The process of field calibration includes:

1. Record a video of a Chauco calibration board with your cellphone camera to calibrate the camera
1. Record a video of the field with the same camera and camera settings including all tags.  The video should try to include as many field tags in the same frame as possible.
1. Run the camera calibration through wpical to convert the video to a camera calibration.json file.
1. Run the field calibration video(s) through wpical to convert them to a field calibration .json file.
1. Upload the field calibration files to each limelight camera using the cameras web interface.

## Additional Tools

### FieldCal Python Script

This tool will print the error between the ideal field and the measured field.

Usage:

This tool depends on a non-standard python library to perform the quaternian to degrees conversion.

1. To obtain access to this library, you need to run the setup.bat file to create a virtual environment
1. VsCode must be told to use this new virtual environment.  In VsCode run Python:Select Interpreter, and select the newly created virtual environment.
1. There is a launch configuration in /.vscode/launch.json that configures the ideal and field calibration json files to use when running the script.  Edit this script to specify the correct name for the field being measured.

Run the FieldCal launch configuration.  The script will print to stdOut a list of errors for each tag.
