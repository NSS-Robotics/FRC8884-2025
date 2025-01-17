package frc.lib;

public class Conversions {
    public static double rotationsToMeters(double wheelRotations, double circumference) {
        return wheelRotations * circumference; 
    }
}
