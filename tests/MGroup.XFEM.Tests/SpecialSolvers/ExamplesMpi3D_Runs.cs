using System;
using System.Collections.Generic;
using System.Text;
using static MGroup.XFEM.Tests.SpecialSolvers.ExamplesMpi3D;

namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public class ExamplesMpi3D_Runs
	{
		public static void RunTestAnalysis()
		{
			var exampleOptions = new ExampleImpactOptions();
			exampleOptions.heavisideTol = 1E-3;
			exampleOptions.maxSteps = 3;

			var meshOptions = new MeshOptions(15, 3);

			var solverOptions = new SolverOptions(SolverChoice.PFETI_DP_I);
			solverOptions.environment = EnvironmentChoice.TPL;

			var outputOptions = new OutputOptions(false, "Test");

			RunSingleAnalysis(exampleOptions, meshOptions, solverOptions, outputOptions);
		}
	}
}
