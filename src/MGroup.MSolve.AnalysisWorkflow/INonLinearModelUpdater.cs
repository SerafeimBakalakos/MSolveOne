using System.Collections.Generic;
using System;
using MGroup.LinearAlgebra.Distributed;

namespace MGroup.MSolve.AnalysisWorkflow
{
	public interface INonLinearModelUpdater
	{
		void ScaleConstraints(double scalingFactor);

		void UpdateState();

		IGlobalVector GetRhsFromSolution(IGlobalVector solution);
	}
}
