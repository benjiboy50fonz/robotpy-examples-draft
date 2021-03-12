"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

from wpimath.kinematics import DifferentialDriveKinematics

import math

# ID for the driver's joystick.
kDriverControllerPort = 0

# Below are constants for the drivetrain.

# The PWM IDs for the drivetrain motor controllers.
kLeftMotor1Port = 0
kLeftMotor2Port = 1
kRightMotor1Port = 2
kRightMotor2Port = 3

# Encoders and their respective motor controllers.
kLeftEncoderPorts = (0, 1)
kRightEncoderPorts = (2, 3)
kLeftEncoderReversed = False
kRightEncoderReversed = True

# In meters, distance between wheels on each side of robot.
kTrackWidthMeters = 0.69

# Encoder counts per revolution/rotation.
kDriveEncoderCPR = 1024
kWheelDiameterMeters = 0.15

# The following works assuming the encoders are directly mounted to the wheel shafts.
kDriveEncoderDistancePerPulse = (kWheelDiameterMeters * math.pi) / kDriveEncoderCPR

# NOTE: Please do NOT use these values on your robot. Rather, characterize your
# drivetrain using the FRC Characterization tool. These are for demo purposes
# only!
ksVolts = 0.22
kvVoltSecondsPerMeter = 1.98
kaVoltSecondsSquaredPerMeter = 0.2

# The slow speed for the drivetrain. It's a percentage.
kSlowSpeedPercent = 0.5

# Below are the values for the arm subsystem.

# PWM ID for the motor controller.
kMotorPort = 4

# P gain for the arm's PID controller.
kP = 1

# More gains for the arm subsystem. Please note that you shouldn't use these exact values!
# Rather, use the FRC Characterization Tool to calculate your own. These are for demonstrative
# purposes only.

kSVolts = 1
kCosVolts = 1
kVVoltSecondPerRad = 0.5
kAVoltSecondSquaredPerRad = 0.1

# Max angular velocity and accleration for the arm.
kMaxVelocityRadPerSecond = 3
kMaxAccelerationRadPerSecSquared = 10

# The arm's encoder ports, as well as the encoder's details.
kEncoderPorts = (4, 5)
kArmEncoderPPR = 256
kArmEncoderDistancePerPulse = 2 * math.pi / kArmEncoderPPR

# Number of motors for the arm.
kArmMotorCount = 1

# The offset of the arm from the horizontal in its neutral position. Measured
# from the horizontal.
kArmOffsetRads = 0.5

# Below are the constants for the autonomous.

# Timeouts for the auto phase as well as the time we spend shooting. Measured in seconds.
kAutoTimeoutSeconds = 12
kAutoShootTimeSeconds = 7
