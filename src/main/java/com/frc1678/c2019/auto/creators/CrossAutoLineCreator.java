package com.frc1678.c2019.auto.creators;

import com.frc1678.c2019.auto.AutoModeBase;
import com.frc1678.c2019.auto.modes.CrossAutoLineMode;

public class CrossAutoLineCreator implements AutoModeCreator {

    // Pre build trajectories to go left and right
    private CrossAutoLineMode mCrossAutoLineMode = new CrossAutoLineMode();

    @Override
    public AutoModeBase getStateDependentAutoMode() {
        return mCrossAutoLineMode;
    }
}
