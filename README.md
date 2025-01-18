# 2025-Robot

[![CI](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/ci.yml/badge.svg)](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/ci.yml) [![Format](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/format.yml/badge.svg)](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/format.yml) [![Documentation](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/doxygen.yml/badge.svg)](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/doxygen.yml)\
[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/FRC1756-Argos/2025-Robot?quickstart=1)


Robot code for 2025 FRC season

[Doxygen documentation](https://frc1756-argos.github.io/2025-Robot/)

## Key Features

### LED Color Codes

| Color | Pattern | Strips | Meaning | Trigger |
| ----- | ------- | ------ | ------- | ------- |

### Controller Vibration Feedback

| Pattern | Controller | Meaning |
| ------- | ---------- | ------- |
| Continuous 100% | Both | Swapping controllers activated.  Swap occurs after vibration ends |

## FTP

* ftp://172.22.11.2 -USB only
* ftp://roborio-1756-frc.local -USB, Wifi, or Ethernet
* ftp://10.17.56.2 -Wifi or Ethernet

## Commissioning A Robot

When commissioning a new robot, you should set the instance type to either "Competition" or "Practice" by creating a text file using FTP readable by `lvuser` on the RoboRIO at path `/home/lvuser/robotInstance`.  The content of this file should be just the text `Competition` or `Practice` with no whitespace preceding.  If no valid instance is found at runtime, competition instance will be used and an error will be generated.

### Homing Swerve Drive

1. When homing swerve modules figure out what is the front of the robot. The intake is the front.
2. Rotate each swerve module to where each bevel gear is to the left side of the robot.
3. Use something flat that is the length of the robot and line the swerve wheels up.
4. Power on the robot and connect your computer. Open Phoenix Tuner X. Select each drive CANcoder and click the "Zero CANcoder" button.

### Vision

See [vision readme](vision/README.md) for information on which pipelines to use and which indices to install these pipelines on.

### Playing With Fusion Addressing

Go to http://10.17.56.2:5812 while connected to the robot to update firmware or change addresses.

## Project Setup

### Pre-Commit

This project uses [pre-commit](https://pre-commit.com/) to check code formatting before accepting commits.

First install the prerequisites:

* python3 (with pip) - [instructions](https://realpython.com/installing-python/)
  * Python 3.12.x from the [Python website](https://www.python.org/downloads/) works well.  Make sure to check the add to path option in the installer.
* pip packages:
  * You may need to add the pip install path to your shell's path if you're using Git Bash.  In git bash:
    1. Open (or create) new file `~/.bashrc` by running `vim ~/.bashrc`
    2. Add this to the end: `PATH=$PATH:$LOCALAPPDATA/Programs/Python/Python39/Scripts/` (change `Python39` to match your python version)
       * **Note**: The actual path you need to add (`$LOCALAPPDATA/Programs/Python/Python39/Scripts/` in the above example) depends on your Python installation.  If y ou do the `pip install` steps first, pip will print the path you need to add.
       * To type in Vim, type <kbd>i</kbd> and you should see `INSERT` at the bottom of the window to indicate you're editing in insert mode
    3. Exit by pressing <kbd>Esc</kbd> then type `:wq` and press <kbd>Enter</kbd>
    4. Run `source ~/.bashrc` to update your session
  * wpiformat - `pip install wpiformat`
  * clang-format - `pip install clang-format`
  * pre-commit - `pip install pre-commit`

  Make sure to run `pip install <package>` commands in an administrator terminal if installing in windows

Then initialize:

```
pre-commit install
pre-commit run
```

The first run may take a moment, but subsequent automatic runs are very fast.

You'll now have the linter run before each commit!  For compatibility with Windows, we recommend the pip version of clang-format, but wpi-format will find any installed `clang-format` binary in the system path.

## Controls

**Driver:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | Drive |
| Left JS Y       | Drive |
| Right JS X      | Turn |
| Right JS Y      | Unused |
| DPad Up         | Unused |
| DPad Right      | Unused |
| DPad Down       | Unused |
| DPad Left       | Unused |
| A               | Unused |
| B               | Unused |
| X               | Unused |
| Y               | Field Home (hold) |
| LB              | Outtake |
| RB              | Intake |
| LT              | Unused |
| RT              | Unused |
| Back            | Swap (hold with <kbd>Start</kbd>) |
| Start           | Swap (hold with <kbd>Back</kbd>) |
| Left JS Button  | Unused |
| Right JS Button | Unused |

**Operator:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | Unused |
| Left JS Y       | Unused |
| Right JS X      | Unused |
| Right JS Y      | Unused |
| A               | Unused |
| B               | Unused |
| X               | Unused |
| Y               | Unused |
| DPad Up         | Unused |
| DPad Right      | Unused |
| DPad Down       | Unused |
| DPad Left       | Unused |
| LB              | Unused |
| RB              | Unused |
| LT              | Unused |
| RT              | Unused |
| Back            | Swap (hold with <kbd>Start</kbd>) |
| Start           | Unused |
| Left JS Button  | Unused |
| Right JS Button | Unused |

## Software Checkout

1. Swerve Drive
    * Forward/reverse
    * Left/right strafe
    * Clockwise/counterclockwise rotation
7. Shut down robot, swap battery

## Software Versions

We're using the following dependencies:

 * [CTRE Phoenix 25.2.0](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v25.2.0)
 * [WPILib 2025.2.1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2025.2.1)
 * [Choreo v2025.0.1](https://github.com/SleipnirGroup/Choreo/releases/tag/v2025.0.1)
 * [Limelightlib v1.15](https://github.com/LimelightVision/limelightlib-wpicpp/tree/dbe2537a99c805446e015f76ad1d02109b1970bf)

## Special Thanks

 * Our sponsors for the 2025 season.  Thank you for your continued support!
   * [Caterpillar](https://www.caterpillar.com/)
   * [Gene Haas Foundation](https://ghaasfoundation.org/)
   * [Boeing](https://www.boeing.com/)
   * [Limestone Community High School](https://www.limestone310.org/)
   * [LCHS Booster Club](https://www.facebook.com/LCHSBoosterClub/)
   * [Playing With Fusion](https://www.playingwithfusion.com/)
   * [Sergison Machine](https://www.sergisonmachine.net/)
   * [Bean's Best LLC](https://beansbestllc.com/)
   * [GRIDLIFE](https://www.grid.life/)
   * Caterpillar employees & [The Caterpillar Foundation](https://www.caterpillar.com/en/company/caterpillar-foundation.html)
 * [Doxygen Awesome](https://jothepro.github.io/doxygen-awesome-css/) - for making our [documentation](https://frc1756-argos.github.io/2025-Robot/) look great

## License
This software is licensed under the [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause). If you would like to use this software under the terms of a different license agreement, please [contact us](mailto:1756argos1756@limestone310.org).
