package com.frc1678.c2019.auto.creators;

import com.frc1678.c2019.auto.AutoModeBase;
import com.frc1678.c2019.auto.modes.CharacterizeHighGearStraight;

public class CharacterizeHighGearStraightCreator implements AutoModeCreator {

    // Pre build trajectories to go left and right
    private CharacterizeHighGearStraight mCharacterizeHighGearStraightMode = new CharacterizeHighGearStraight();

    @Override
    public AutoModeBase getStateDependentAutoMode() {
        return mCharacterizeHighGearStraightMode;
    }
}
