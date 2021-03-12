from commands2 import ProfiledPIDSubsystem

from wpilib import PWMSparkMax, Encoder

from wpimath.trajectory import TrapezoidProfile

from wpilib.controller import ProfiledPIDController

from wpimath.controller import ArmFeedforward

import constants


class ArmSubsystem(ProfiledPIDSubsystem):

    """
    Subsystem to control and monitor the arm's motor controller
    and encoders. All the nitty gritty work should take place
    here in a command-based system.
    """

    def __init__(self):
        """
        The constructor for the arm subsystem. Create motors, sensors, and most variables here.
        """

        # Create the arm motor, which will be controlled by a SparkMAX. Also create its encoder.
        self.motor = PWMSparkMax(constants.kMotorPort)
        self.encoder = Encoder(constants.kEncoderPorts[0], constants.kEncoderPorts[1])

        # Create the feedforward controller for the arm subsystem.
        self.feedforward = ArmFeedforward(
            constants.kSVolts,
            constants.kCosVolts,
            constants.kVVoltSecondPerRad,
            constants.kAVoltSecondSquaredPerRad,
        )

        # Call the parent class' constructor.
        super().__init__(
            ProfiledPIDController(
                constants.kP,
                0,
                0,
                TrapezoidProfile.Constraints(
                    constants.kMaxVelocityRadPerSecond,
                    constants.kMaxAccelerationRadPerSecSquared,
                ),
            ),
            0,
        )

        # Configure the arm's encoder so it knows how to convert encoder units to radians.
        self.encoder.setDistancePerPulse(constants.kArmEncoderDistancePerPulse)

        # Start the arm at rest in its neutral position.
        self.setGoal(constants.kArmOffsetRads)

    def _useOutput(self, output, setpoint: TrapezoidProfile.State):
        """
        Calculate the feedforward and use that feedforward with the PID to calculate and set
        the desired motor output.
        """

        # Calculate the feedforward for the arm using it's goal position and speed.
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # Add the feedforward to the desired output.
        self.motor.setVoltage(output + feedforward)

    def _getMeasurement(self):
        """
        Returns the arm's current position using the starting position and the encoder's position.
        """

        return self.encoder.getDistance() + constants.kArmOffsetRads
