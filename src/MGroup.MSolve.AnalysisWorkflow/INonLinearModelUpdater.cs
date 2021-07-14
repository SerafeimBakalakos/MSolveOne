using MGroup.LinearAlgebra.Vectors;

using MGroup.MSolve.Discretization;
using System.Collections.Generic;
using MGroup.MSolve.Solution.LinearSystem;
using System;
using MGroup.MSolve.Solution;

namespace MGroup.MSolve.AnalysisWorkflow
{
	public interface INonLinearModelUpdater
	{
		void ScaleConstraints(double scalingFactor);

		void UpdateState();

		IGlobalVector GetRhsFromSolution(IGlobalVector solution);
	}
}
