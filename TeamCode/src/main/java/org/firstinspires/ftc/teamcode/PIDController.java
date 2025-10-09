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

    public PIDController(double initkp, double initki, double initkd) {
        kp = initkp;
        ki = initki;
        kd = initkd;
        target = 0;
        integral = 0;
        previousError = 0;
    }

    public void setTarget(double initTarget) {
        target = initTarget;
        integral = 0;
        previousError = 0;
    }

    public double calculateOutput(double currentValue, double deltaTime) {
        double error = target - currentValue;

        //Proportional Term
        double proportionalTerm = kp * error;

        //Integral Term
        if (deltaTime > 0) {
            //anti-windup logic
            // Calculate the total PID output *without* the I-term.
            // This is used to determine if the I-term is needed or if P+D already saturates the output.
            double derivativeTerm = kd * ((error - previousError) / deltaTime);
            double outputEstimate = proportionalTerm + derivativeTerm;

            // Only accumulate the integral if the motor output is NOT saturated
            // and the ki gain is non-zero.
            if (ki != 0 && Math.abs(outputEstimate) < MAX_PID_OUTPUT) {
                integral += error * deltaTime;
            }
        }
        double integralTerm = ki * integral;

        //Derivative Term (D)
        double derivativeTerm = 0;
        if (deltaTime > 0) {
            derivativeTerm = kd * ((error - previousError) / deltaTime);
        }

        // 4. Calculate Final Output
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // 5. Update state and limit output
        previousError = error;

        // Clamp output to the [-MAX_PID_OUTPUT, MAX_PID_OUTPUT] range
        return Math.min(Math.max(output, -MAX_PID_OUTPUT), MAX_PID_OUTPUT);
    }

}
