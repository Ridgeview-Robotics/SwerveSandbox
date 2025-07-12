package org.firstinspires.ftc.teamcode.assets;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDC {

    private final double kp;
    private final double ki;
    private final double kd;

    private double setPoint;

    private double integral;
    private double previousError;
    private double lastTimeMS;
    private ElapsedTime timer;

    public PIDC(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        setPoint = 0.0;
        integral = 0.0;
        previousError = 0.0;
        timer = new ElapsedTime();
        this.lastTimeMS = timer.milliseconds();

    }

    public void setTarg(double targ) {
        setPoint = targ;
    }

    public void reset() {
        this.integral = 0.0;
        this.previousError = 0.0;
        timer.reset();
    }

    public double calculate(double measuredValue) {
        double now = timer.milliseconds();
        double dt = (now - lastTimeMS)/ 1.0e9;

        double error = setPoint - measuredValue;
        integral += error * dt;
        double derivative = dt > 0 ? (error - previousError) / dt : 0.0;

        double output = (kp * error) + (ki * integral) + (kd * derivative);

        previousError = error;
        lastTimeMS = now;
        timer.reset();

        return output;
    }

}
