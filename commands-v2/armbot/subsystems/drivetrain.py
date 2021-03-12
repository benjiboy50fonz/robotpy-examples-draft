from commands2 import SubsystemBase

from wpilib import PWMSparkMax, SpeedControllerGroup, Encoder

from wpilib.drive import DifferentialDrive

import constants


class DriveSubsystem(SubsystemBase):
    def __init__(self):

        """
        The constructor for the drivetrain subsystem.
        Create motors, sensors, and most variables here.
        """

        super().__init__()

        # Create the motor controllers and their respective speed controllers.
        self.leftMotors = SpeedControllerGroup(
            PWMSparkMax(constants.kLeftMotor1Port),
            PWMSparkMax(constants.kLeftMotor2Port),
        )

        self.rightMotors = SpeedControllerGroup(
            PWMSparkMax(constants.kRightMotor1Port),
            PWMSparkMax(constants.kRightMotor2Port),
        )

        # Create the differential drivetrain object, allowing for easy motor control.
        self.drive = DifferentialDrive(self.leftMotors, self.rightMotors)

        # Create the encoder objects.
        self.leftEncoder = Encoder(
            constants.kLeftEncoderPorts[0],
            constants.kLeftEncoderPorts[1],
            constants.kLeftEncoderReversed,
        )

        self.rightEncoder = Encoder(
            constants.kRightEncoderPorts[0],
            constants.kRightEncoderPorts[1],
            constants.kRightEncoderReversed,
        )

        # Configure the encoder so it knows how many encoder units are in one rotation.
        self.leftEncoder.setDistancePerPulse(constants.kDriveEncoderDistancePerPulse)
        self.rightEncoder.setDistancePerPulse(constants.kDriveEncoderDistancePerPulse)

    def arcadeDrive(self, fwd, rot):
        """
        Operate the robot with the controller and drive using standard
        arcade-style controls.
        """

        # Use the left stick's Y-axis as forward, and the right stick's X-axis for rotation.
        self.drive.arcadeDrive(fwd, rot)

    def resetEncoders(self):
        """
        Reset the left and right encoder so they read a current position of
        0. This won't wipe their settings however.
        """

        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getAverageEncoderDistance(self):
        """
        Take the sum of each encoder's traversed distance and divide it by two,
        since we have two encoder values, to find the average value of the two.
        """

        return (self.leftEncoder.getDistance() + self.rightEncoder.getDistance()) / 2

    def getLeftEncoder(self):
        """Returns the left encoder object."""

        return self.leftEncoder

    def getRightEncoder(self):
        """Returns the right encoder object."""
        return self.rightEncoder

    def setMaxOutput(self, maxOutput):
        """
        Set the max percent output of the drivetrain, allowing for slower control.
        The default argument allows us to easily set the max to the motor's max.
        """
        self.drive.setMaxOutput(maxOutput)
