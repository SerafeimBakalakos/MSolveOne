using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution.LinearSystem;

namespace MGroup.MSolve.AnalysisWorkflow.Providers
{
	public interface IAnalyzerProvider
	{
		void AssignRhs();

		void ClearMatrices();

		void Reset();
	}
}
