#pragma once

#include "gtest/gtest.h"
#include "cpu.h"

class CPUTest : public ::testing::Test {
protected:
    void SetUp() override {
        cpu = new CPU();
        cpu->PC = 0x0;

    }

    void TearDown() override {
        delete cpu;
    }

    CPU* cpu;
};
