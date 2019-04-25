package com.frc1678.c2019.auto.creators;

import com.frc1678.c2019.auto.AutoModeBase;

public interface AutoModeCreator {
    AutoModeBase getStateDependentAutoMode();
}