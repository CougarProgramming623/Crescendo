#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/AnalogInput.h>
#include "Constants.h"


class PivotToPos : public frc2::CommandHelper<frc2::Command, PivotToPos> {
	public:
		explicit PivotToPos(int target);
		void Initialize() override;
		void Execute() override;
  		void End(bool interrupted) override;
		bool IsFinished() override;	

		// double targetDegrees;
		int targetValue;
		int stringpot;
		double targetRotations;
		// double startingDegrees;
		// double ticksToMove;
};