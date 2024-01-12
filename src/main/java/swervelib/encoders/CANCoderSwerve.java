package swervelib.encoders;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.wpilibj.DriverStation;
import unfuckthelibraries.CANCODER;

/**
 * Swerve Absolute Encoder for CTRE CANCoders.
 */
public class CANCoderSwerve extends SwerveAbsoluteEncoder {

    /**
     * CANCoder with WPILib sendable and support.
     */
    public CANCODER encoder;

    /**
     * Initialize the CANCoder on the standard CANBus.
     *
     * @param id CAN ID.
     */
    public CANCoderSwerve(int id) {
        encoder = new CANCODER(id);
    }

    /**
     * Initialize the CANCoder on the CANivore.
     *
     * @param id     CAN ID.
     * @param canbus CAN bus to initialize it on.
     */
    public CANCoderSwerve(int id, String canbus) {
        encoder = new CANCODER(id, canbus);
    }

    /**
     * Reset the encoder to factory defaults.
     */
    @Override
    public void factoryDefault() {
        //encoder.configFactoryDefault();
    }

    /**
     * Clear sticky faults on the encoder.
     */
    @Override
    public void clearStickyFaults() {
        encoder.clearStickyFaults();
    }

    /**
     * Configure the absolute encoder to read from [0, 360) per second.
     *
     * @param inverted Whether the encoder is inverted.
     */
    @Override
    public void configure(boolean inverted) {
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        // TODO: Unfuck
        /*canCoderConfiguration.absoluteSensorRange = Abs.Unsigned_0_to_360;
        canCoderConfiguration.sensorDirection = inverted;
        canCoderConfiguration.initializationStrategy =
                SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
        encoder.configAllSettings(canCoderConfiguration); */
    }

    /**
     * Get the absolute position of the encoder. Sets {@link SwerveAbsoluteEncoder#readingError} on erroneous readings.
     *
     * @return Absolute position in degrees from [0, 360).
     */
    @Override
    public double getAbsolutePosition() {
        readingError = false;
        MagnetHealthValue strength = encoder.getMagnetHealth();

        if (strength != MagnetHealthValue.Magnet_Green) {
            DriverStation.reportWarning(
                    "CANCoder " + encoder.getDeviceID() + " magnetic field is less than ideal.\n", false);
        }
        if (strength == MagnetHealthValue.Magnet_Invalid || strength == MagnetHealthValue.Magnet_Red) {
            readingError = true;
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " reading was faulty.\n", false);
            return 0;
        }
        double angle = encoder.getPosition();

        // Taken from democat's library.
        // Source: https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
        /*ErrorCode code = encoder.getLastError();
        for (int i = 0; i < maximumRetries; i++) {
            if (code == ErrorCode.OK) {
                break;
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
            angle = encoder.getPosition().getValue();
            code = encoder.getLastError();
        }
        if (code != ErrorCode.OK) {
            readingError = true;
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " reading was faulty, ignoring.\n", false);
        }*/

        return angle;
    }

    /**
     * Get the instantiated absolute encoder Object.
     *
     * @return Absolute encoder object.
     */
    @Override
    public Object getAbsoluteEncoder() {
        return encoder;
    }

    /**
     * Sets the Absolute Encoder Offset inside of the CANcoder's Memory.
     *
     * @param offset the offset the Absolute Encoder uses as the zero point.
     * @return if setting Absolute Encoder Offset was successful or not.
     */
    @Override
    public boolean setAbsoluteEncoderOffset(double offset) {
        ErrorCode error = encoder.configMagnetOffset(offset);
        if (error == ErrorCode.OK) {
            return true;
        }
        DriverStation.reportWarning(
                "Failure to set CANCoder " + encoder.getDeviceID() + " Absolute Encoder Offset Error: " + error, false);
        return false;
    }

    /**
     * Get the velocity in degrees/sec.
     *
     * @return velocity in degrees/sec.
     */
    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
}
