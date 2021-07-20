using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public interface IFetiDPCoarseProblem
	{
		void FindCoarseProblemDofs();

		void PrepareMatricesForSolution();

		//TODOMPI: Perhaps I should have dedicated classes for these distributed vectors. DistributedOverlappingVector was 
		//		cumbersome, because it required an indexer. Having an indexer for coarse-problem vectors makes sense only in 
		//		distributed implementations.
		void SolveCoarseProblem(IDictionary<int, Vector> coarseProblemRhs, IDictionary<int, Vector> coarseProblemSolution);
	}
}
