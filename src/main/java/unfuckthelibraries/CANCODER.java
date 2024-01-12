package unfuckthelibraries;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;

public class CANCODER {
    public CANcoder encoder;
    private double offset;

    public CANCODER(int deviceId) {
        encoder = new CANcoder(deviceId);
        offset = 0;
    }

    public CANCODER(int deviceId, String canbus) {
        encoder = new CANcoder(deviceId, canbus);
        offset = 0;
    }

    public ErrorCode getLastError() {
        return ErrorCode.GENERAL_ERROR; // TODO: Actually make this do things
    }

    public ErrorCode configMagnetOffset(double offset) {
        this.offset = offset;
        return ErrorCode.OK;
    }

    public void clearStickyFaults() {
        encoder.clearStickyFaults();
    }

    public MagnetHealthValue getMagnetHealth() {
        return encoder.getMagnetHealth().getValue();
    }

    public int getDeviceID() {
        return encoder.getDeviceID();
    }

    public double getPosition() {
        return encoder.getPosition().getValue() * 360;
    }

    public double getVelocity() {
        return encoder.getVelocity().getValue();
    }
}