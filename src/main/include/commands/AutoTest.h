#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/AnalogInput.h>
#include "Constants.h"


class AutoTest : public frc2::CommandHelper<frc2::Command, AutoTest> {
	public:
		void Initialize() override;
		void Execute() override;
  		void End(bool interrupted) override;
};