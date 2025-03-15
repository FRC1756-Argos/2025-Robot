# 2025-Robot

[![CI](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/ci.yml/badge.svg)](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/ci.yml) [![Format](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/format.yml/badge.svg)](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/format.yml) [![Documentation](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/doxygen.yml/badge.svg)](https://github.com/FRC1756-Argos/2025-Robot/actions/workflows/doxygen.yml)\
[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/FRC1756-Argos/2025-Robot?quickstart=1)


Robot code for 2025 FRC season

[Doxygen documentation](https://frc1756-argos.github.io/2025-Robot/)

## Key Features

### LED Color Codes

| Color | Pattern | Strips | Meaning | Trigger |
| ----- | ------- | ------ | ------- | ------- |
| Green | Solid | All | Reef AprilTag detected | N/A |

### Controller Vibration Feedback

| Pattern | Controller | Meaning |
| ------- | ---------- | ------- |
| Continuous 100% | Both | Swapping controllers activated.  Swap occurs after vibration ends |
| 500 ms pulse | Driver | Aligned to reef for scoring coral |

## FTP

* ftp://172.22.11.2 -USB only
* ftp://roborio-1756-frc.local -USB, Wifi, or Ethernet
* ftp://10.17.56.2 -Wifi or Ethernet

## Commissioning A Robot

When commissioning a new robot, you should set the instance type to either "Competition" or "Practice" by creating a text file using FTP readable by `lvuser` on the RoboRIO at path `/home/lvuser/robotInstance`.  The content of this file should be just the text `Competition` or `Practice` with no whitespace preceding.  If no valid instance is found at runtime, competition instance will be used and an error will be generated.

### Homing Swerve Drive

1. When homing swerve modules figure out what is the front of the robot. The battery is the front.
2. Rotate each swerve module to where each bevel gear is to the left side of the robot.
3. Use something flat that is the length of the robot and line the swerve wheels up.
4. Power on the robot and connect your computer. Open Phoenix Tuner X. Select each drive CANcoder and click the "Zero CANcoder" button.

### Homing arm
1. Move arm so it's parallel to the ground in the left position. The battery is the front.
2. Power on the robot and connect your computer. Open Phoenix Tuner X. Select arm CANcoder and click the "Zero CANcoder" button.
  > :memo: **Note:** Positive direction is rotating toward upward position

### Homing wrist
1. Move wrist so wheels are toward the elevator and the rotating wrist tube aligns to the stationary arm tube.
2. Power on the robot and connect your computer. Open Phoenix Tuner X. Select arm CANcoder and click the "Zero CANcoder" button.
  > :memo: **Note:** Positive direction is rotating toward wheels left while arm is up.  The battery is the front

### Homing climber
1. Move climber so it's parallel to the ground extended toward front of robot. The battery is the front.
2. Power on the robot and connect your computer. Open Phoenix Tuner X. Select climber CANcoder and click the "Zero CANcoder" button.
  > :memo: **Note:** Positive direction is rotating inward toward elevator


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
| A               | Outtake |
| B               | Align Right |
| X               | Align Left |
| Y               | Unused |
| LB              | Left Intake |
| RB              | Right Intake |
| LT              | Left Place |
| RT              | Right Place |
| Back            | Field Home |
| Start           | Unused |
| Left JS Button  | Unused |
| Right JS Button | Unused |

**Operator Coral Mode:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | Unused |
| Left JS Y       | Elevator |
| Right JS X      | Wrist |
| Right JS Y      | Unused |
| DPad Up         | Climber Arm Up Manual |
| DPad Right      | Climber Winch Out |
| DPad Down       | Climber Arm Down Manual |
| DPad Left       | Unused |
| A               | Preclimb |
| B               | Climber Arm Down |
| X               | Pressed - HP Pickup, Released - Floor Intake |
| Y               | Climber Arm Up |
| LB              | Unused |
| RB              | Run Intake |
| LT              | Shoulder Left |
| RT              | Shoulder Right |
| Back            | Unused |
| Start           | Unused |
| Left JS Button  | Unused |
| Right JS Button | Unused |

**Operator Algae Mode:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | Unused |
| Left JS Y       | Elevator |
| Right JS X      | Wrist |
| Right JS Y      | Unused |
| DPad Up         | Climber Arm Up Manual |
| DPad Right      | Climber Winch Out |
| DPad Down       | Climber Arm Down Manual |
| DPad Left       | Unused |
| A               | Preclimb |
| B               | Climber Arm Down |
| X               | Pressed - HP Pickup, Released - Floor Intake |
| Y               | Climber Arm Up |
| LB              | Unused |
| RB              | Run Intake |
| LT              | Shoulder Left |
| RT              | Shoulder Right |
| Back            | Unused |
| Start           | Unused |
| Left JS Button  | Unused |
| Right JS Button | Unused |

**Button Box (Coral):**
| Button          | Function |
| --------------- | -------- |
| 1               | Unused |
| 2               | Unused |
| 3               | Coral L4 |
| 4               | Unused |
| 5               | Unused |
| 6               | Coral L3 |
| 7               | Reef Left Position |
| 8               | Reef Right Position |
| 9               | Coral L2 |
| 10              | Coral/Algae Mode Toggle |
| 11              | Stow |
| 12              | Coral L1 |

**Button Box (Algae):**
| Button          | Function |
| --------------- | -------- |
| 1               | Unused |
| 2               | Unused |
| 3               | Algae Net |
| 4               | Unused |
| 5               | Unused |
| 6               | Algae L3 |
| 7               | Reef Left Position |
| 8               | Reef Right Position |
| 9               | Algae L2 |
| 10              | Coral/Algae Mode Toggle |
| 11              | Stow |
| 12              | Algae Processor |

## Software Checkout

1. Swerve Drive
    * Forward/reverse
    * Left/right strafe
    * Clockwise/counterclockwise rotation
2. Coral Floor intake
    * Left/Right
3. Coral manual intake controls
    * Manual intake
    * Manual outtake
4. Coral Station intake
    * Left/Right
5. Coral Place
    * Left/Right
    * L1, L2, L3, L4
6. Algae floor pickup
    * Left/Right
7. Algae Processor placement
    * Left/Right
8. Algae net
    * Left/Right
9. Climber
    * Manual controls
    * Auto sequence
10. Vision
    * Detect targets Left/Right
    * Aim Left/Right reef pole on left and right cameras
11. Shut down robot, swap battery

## Software Versions

We're using the following dependencies:

 * [CTRE Phoenix 25.2.2](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v25.2.2)
 * [WPILib 2025.3.1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2025.3.1)
 * [Choreo v2025.0.3](https://github.com/SleipnirGroup/Choreo/releases/tag/v2025.0.3)
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
   * Caterpillar employees & [The Caterpillar Foundation](https://www.caterpillar.com/en/company/caterpillar-foundation.html)
 * [Doxygen Awesome](https://jothepro.github.io/doxygen-awesome-css/) - for making our [documentation](https://frc1756-argos.github.io/2025-Robot/) look great

## License
This software is licensed under the [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause). If you would like to use this software under the terms of a different license agreement, please [contact us](mailto:1756argos1756@limestone310.org).
