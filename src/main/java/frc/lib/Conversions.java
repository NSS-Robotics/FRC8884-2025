package frc.lib;

public class Conversions {
    public static double rotationsToMeters(double wheelRotations, double circumference) {
        return wheelRotations * circumference; 
    }

    public static double rpsToMPS(double rps, double circumference) {
        return rps * circumference;
    }

    public static double mpsToRPS(double mps, double circumference) {
        return mps / circumference;
    }
}
