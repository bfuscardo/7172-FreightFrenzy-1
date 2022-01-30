package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class ServoX {

    public Servo srv = null;
    public double speed = 1;
    public double targetPos = 0.5;
    public double currentPos = 0.5;
    public static ElapsedTime currTime = new ElapsedTime();
    public double lastTime = 0;
    public double timeout = 0;

    public ServoX(Servo s) {
        this.attach(s);
    }

    public ServoX(HardwareMap hwMap, String name) {
        this.attach(hwMap.get(Servo.class, name));
    }

    public void update() {
        double now = currTime.seconds();
        double dt = now - lastTime;
        if (dt < 0.005) return;                 // wait 5ms to update (200Hz)
        double dx = targetPos - currentPos;
        if (dx > -0.001 && dx < 0.001) {        // target reached within 0.1% ?
            currentPos = targetPos;
            lastTime = now;
            return;
        }
        double maxdx = dt * speed;
        dx = Range.clip(dx, -maxdx, maxdx);
        currentPos = Range.clip(currentPos + dx, 0, 1);
        srv.setPosition(currentPos);
        lastTime = now;
        if (now > timeout) targetPos = currentPos;
    }

    public void setSpeed(double spd) { speed = Math.abs(spd); }
    public void setTimeout(double sec) { timeout = currTime.seconds() + sec; }

    // use this to go to a position at the last set speed
    public void setPosition(double target) {
        setPosition(target, speed, 3600);
    }

    // use this to go to a position at a given speed
    public void setPosition(double target, double spd) {
        setPosition(target, spd, 3600);
    }

    // use this to go to a position at a given speed but stop after a timeout
    public void setPosition(double target, double spd, double sec) {
        setSpeed(spd);
        setTimeout(sec);
        targetPos = target;
    }

    public ServoX attach(Servo s) {
        srv = s;
        targetPos = srv.getPosition();
        currentPos = targetPos;
        return this;
    }

}
