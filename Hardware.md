# Hardware

This document details the hardware IDs currently defined in the robot code and swerve deploy config.

## Motors and Motor Controllers
(Key: `F` = Front, `B` = Back, `R` = Right, `L` = Left)

| ID | Mechanism | Motor | Motor Controller | PDH Port |
| --- | --- | --- | --- | --- |
| 1 | Drivetrain BL Steer | NEO | SPARK MAX | ? |
| 2 | Drivetrain BL Drive | NEO | SPARK MAX | ? |
| 3 | Drivetrain BR Steer | NEO | SPARK MAX | ? |
| 4 | Drivetrain BR Drive | NEO | SPARK MAX | ? |
| 5 | Drivetrain FL Steer | NEO | SPARK MAX | ? |
| 6 | Drivetrain FL Drive | NEO | SPARK MAX | ? |
| 7 | Drivetrain FR Steer | NEO | SPARK MAX | ? |
| 8 | Drivetrain FR Drive | NEO | SPARK MAX | ? |
| 9 | Intake Extension | Brushless (unspecified in code) | SPARK MAX | ? |
| 10 | Intake Roller | Brushless (unspecified in code) | SPARK MAX | ? |
| 11 | Indexer Leader | Brushless (unspecified in code) | SPARK MAX | ? |
| 12 | Indexer Follower | Brushless (unspecified in code) | SPARK MAX | ? |
| 13 | Turret Rotation | Brushless (unspecified in code) | SPARK MAX | ? |
| 14 | Flywheel Leader | Brushless (unspecified in code) | SPARK MAX | ? |
| 15 | Flywheel Follower | Brushless (unspecified in code) | SPARK MAX | ? |
| 16 | Hood | Brushless (unspecified in code) | SPARK MAX | ? |

## Sensors, IMU, and Vision
| ID | Sensor | Type | PDH Port |
| --- | --- | --- | --- |
| 17 | Pigeon2 | IMU / gyro | ? |
| 21 | BR Swerve CANcoder | Absolute encoder | ? |
| 22 | BL Swerve CANcoder | Absolute encoder | ? |
| 23 | FL Swerve CANcoder | Absolute encoder | ? |
| 24 | FR Swerve CANcoder | Absolute encoder | ? |
| - | Turret Through-Bore Encoder | REV absolute encoder on the turret rotation SPARK MAX data port | ? |
| - | Limelight `limelight` | Vision camera / NetworkTables device | ? |

## Swerve Module Summary
| Module | Drive ID | Steer ID | Encoder ID | Encoder Offset |
| --- | --- | --- | --- | --- |
| Front Left | 6 | 5 | 23 | 229.48 |
| Front Right | 8 | 7 | 24 | 105.82 |
| Back Left | 2 | 1 | 22 | 65.47 |
| Back Right | 4 | 3 | 21 | 92.28 |

## Operator Interfaces
| Port | Device |
| --- | --- |
| 1 | Driver Xbox controller |
| 2 | Operator Xbox controller |

## Source Summary
- Drivetrain motor, CANcoder, and Pigeon2 IDs come from `src/main/deploy/swerve/swervedrive.json` and the four files under `src/main/deploy/swerve/modules/`.
- Intake IDs come from `src/main/java/frc/robot/subsystems/Intake.java`.
- Indexer IDs come from `src/main/java/frc/robot/subsystems/Indexer.java`.
- Turret motor IDs and the REV Through-Bore absolute encoder usage come from `src/main/java/frc/robot/subsystems/Turret.java`.
- Limelight naming comes from `src/main/java/frc/robot/subsystems/VisionSubsystem.java`.
- Xbox controller USB ports come from `src/main/java/frc/robot/RobotContainer.java`.
