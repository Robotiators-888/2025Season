package frc.robot.utils;

public class ScoringSetpoint {
    public double pivotLow;
    public double pivotHigh;

    public double elevatorLow;
    public double elevatorHigh;

    public ScoringSetpoint(double pivotLow, double pivotHigh, double elevatorLow, double elevatorHigh) {
        this.pivotLow = pivotLow;
        this.pivotHigh = pivotHigh;
        this.elevatorLow = elevatorLow;
        this.elevatorHigh = elevatorHigh;
    }

    public boolean atSetpoint(double pivotReading, double elevatorReading){
        return pivotReading < pivotHigh && pivotReading > pivotLow && elevatorReading < elevatorHigh && elevatorReading > elevatorLow;
    }

    @Override 
    public boolean equals(Object o){
        
        if (!(o instanceof ScoringSetpoint)){
            return false;
        }

        ScoringSetpoint s = (ScoringSetpoint) o;
        return pivotLow == s.pivotLow && pivotHigh == s.pivotHigh && elevatorLow == s.elevatorLow && elevatorHigh == s.elevatorHigh;
    }
}