#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

from wpilib import RobotController, ADXRS450_Gyro
from wpilib.simulation import (
    PWMSim,
    DifferentialDrivetrainSim,
    SingleJointedArmSim,
    EncoderSim,
    ADXRS450_GyroSim,
)
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor

import constants

import os
import math

from pyfrc.physics.core import PhysicsInterface


class PhysicsEngine:
    """
    Simulates a motor moving something that strikes two limit switches,
    one on each end of the track. Obviously, this is not particularly
    realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface):

        self.physics_controller = physics_controller

        # Motor simulation definitions. Each correlates to a motor defined in
        # the drivetrain subsystem.
        self.frontLeftMotor = PWMSim(constants.kLeftMotor1Port)
        self.backLeftMotor = PWMSim(constants.kLeftMotor2Port)
        self.frontRightMotor = PWMSim(constants.kRightMotor1Port)
        self.backRightMotor = PWMSim(constants.kRightMotor2Port)

        self.driveSystem = LinearSystemId.identifyDrivetrainSystem(
            # The linear velocity gain in volt seconds per distance.
            constants.kvVoltSecondsPerMeter,
            # The linear acceleration gain, in volt seconds^2 per distance.
            constants.kaVoltSecondsSquaredPerMeter,
            # The angular velocity gain, in volt seconds per angle.
            1.5,
            # The angular acceleration gain, in volt seconds^2 per angle.
            0.3,
        )

        # The simulation model of the drivetrain.
        self.drivesim = DifferentialDrivetrainSim(
            # The state-space model for a drivetrain.
            self.driveSystem,
            # The robot's trackwidth, which is the distance between the
            # wheels on the left side and those on the right side. The
            # units is meters.
            constants.kTrackWidthMeters,
            # Four NEO drivetrain setup.
            DCMotor.NEO(4),
            # The gear ratio between the motor and the output shaft for the wheels.
            1,
            # The radius of the drivetrain wheels in meters.
            (constants.kWheelDiameterMeters / 2),
        )

        self.leftEncoderSim = EncoderSim.createForChannel(
            constants.kLeftEncoderPorts[0]
        )
        self.rightEncoderSim = EncoderSim.createForChannel(
            constants.kRightEncoderPorts[0]
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.frontLeftMotor.getSpeed()
        r_motor = self.frontRightMotor.getSpeed()

        voltage = RobotController.getInputVoltage()
        self.drivesim.setInputs(l_motor * voltage, -r_motor * voltage)
        self.drivesim.update(tm_diff)

        # Set the values of the encoders to update them in the sim.
        self.leftEncoderSim.setDistance(self.drivesim.getLeftPosition() * 39.37)
        self.leftEncoderSim.setRate(self.drivesim.getLeftVelocity() * 39.37)
        self.rightEncoderSim.setDistance(self.drivesim.getRightPosition() * 39.37)
        self.rightEncoderSim.setRate(self.drivesim.getRightVelocity() * 39.37)

        # Update the robot's position on the field.
        self.physics_controller.field.setRobotPose(self.drivesim.getPose())
