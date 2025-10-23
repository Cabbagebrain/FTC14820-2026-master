package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.MAX_PID_OUTPUT;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kp; //proportional gain
    private double ki; //integral gain
    private double kd; //derivative gain
    private double target;
    private double integral;
    private double previousError;
    private double previousDerivative;
    private static final double ALPHA_FILTER = 0.2;    // Low-Pass Filter Factor: 0.0 means no filtering, 1.0 means infinite filtering.

    public PIDController(double initkp, double initki, double initkd) {
        kp = initkp;
        ki = initki;
        kd = initkd;
        target = 0;
        integral = 0;
        previousError = 0;
        previousDerivative = 0;
    }

    public void setTarget(double initTarget) {
        // Normalizes the target to [-180, 180] degrees
        target = normalizeAngle(initTarget);
        integral = 0;
        previousError = 0;
        previousDerivative = 0;
    }
    public void setPID(double initkp, double initki, double initkd) {
        kp = initkp;
        ki = initki;
        kd = initkd;
    }

    public double getTarget() {
        return target;
    }

    //helper method to ensure angles are within the standard [-180, 180] range
    private double normalizeAngle(double angle) {
        double mod = angle % 360.0;
        if (mod > 180.0) {
            mod -= 360.0;
        } else if (mod <= -180.0) {
            mod += 360.0;
        }
        return mod;
    }

    //calculates PID output power
    public double calculateOutput(double currentValue, double deltaTime) {
        double error = target - currentValue;
        error = normalizeAngle(error);

        //Proportional Term
        double proportionalTerm = kp * error;

        //Integral Term
        if (Math.abs(proportionalTerm) < MAX_PID_OUTPUT) {
            integral += error * deltaTime;
        }
        double integralTerm = ki * integral;

        //Derivative Term (D)
        double rawDerivative = 0;
        if (deltaTime > 0) {
            // Calculate the instantaneous rate of error change
            rawDerivative = (error - previousError) / deltaTime;
        }
        // Low-Pass Filter: Blends the raw derivative with the previous filtered value.
        // This removes high-frequency noise spikes.
        double filteredDerivative = (ALPHA_FILTER * rawDerivative) +
                ((1.0 - ALPHA_FILTER) * previousDerivative);

        double derivativeTerm = kd * filteredDerivative;

        double output = proportionalTerm + integralTerm + derivativeTerm;
        previousError = error;
        previousDerivative = filteredDerivative; // Save the filtered value for the next loop

        // Clamp output to the [-MAX_PID_OUTPUT, MAX_PID_OUTPUT] range
        return Math.min(Math.max(output, -MAX_PID_OUTPUT), MAX_PID_OUTPUT);
    }


}
