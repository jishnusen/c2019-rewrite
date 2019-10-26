package com.frc1678.c2019.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getVision();

    boolean getStartVision();
}
