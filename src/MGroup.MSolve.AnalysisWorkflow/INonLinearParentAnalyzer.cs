using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.MSolve.AnalysisWorkflow
{
	public interface INonLinearParentAnalyzer : IParentAnalyzer
	{
		IGlobalVector GetOtherRhsComponents(IGlobalVector currentSolution);
	}
}
