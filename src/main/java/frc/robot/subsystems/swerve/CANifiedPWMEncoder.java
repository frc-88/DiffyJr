package frc.robot.subsystems.swerve;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.CANifierStatusFrame;

/** Represents a PWM-based encoder that is connected to a CANifier. */
public class CANifiedPWMEncoder {

  // The CANifier that the encoder is plugged into
  private CANifier canifier;

  // The channel that the encoder is plugged into
  private PWMChannel channel;

  /**
   * Constructor.
   *
   * @param canifier The CANifier that the encoder is plugged into
   * @param channel The channel that the encoder is plugged into
   */
  public CANifiedPWMEncoder(int canifierID, int pwmChannel) {
    PWMChannel channel;
    switch (pwmChannel) {
      case 0:
        channel = PWMChannel.PWMChannel0;
        break;
      case 1:
        channel = PWMChannel.PWMChannel1;
        break;
      case 2:
        channel = PWMChannel.PWMChannel2;
        break;
      case 3:
        channel = PWMChannel.PWMChannel3;
        break;
      default:
        throw new RuntimeException(
            String.format(
                "%d is not a valid PWM channel. It must be between 0 and 3, inclusive",
                pwmChannel));
    }

    this.canifier = new CANifier(canifierID);
    this.channel = channel;
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 5);
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 5);
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, 5);
    this.canifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 5);
  }

  // Returns position in radians
  public double getPosition() {
    double[] dutyAndPeriod = new double[2];
    this.canifier.getPWMInput(channel, dutyAndPeriod);
    if (dutyAndPeriod[1] == 0.) {
      // Sensor is unplugged
      return 0;
    }
    return 2.0 * Math.PI * dutyAndPeriod[0] / dutyAndPeriod[1];
  }
}
