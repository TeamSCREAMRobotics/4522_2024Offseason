package com.team4522.lib.sim;

public interface SimInterface {
    void update(double deltaTime);
    void setInputVoltage(double voltage);
    double getPosition();
    double getVelocity();
}

