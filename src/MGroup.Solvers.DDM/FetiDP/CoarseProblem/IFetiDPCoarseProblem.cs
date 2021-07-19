using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed.Overlapping;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public interface IFetiDPCoarseProblem
	{
		void FindCoarseProblemDofs();

		void PrepareMatricesForSolution();

		void SolveCoarseProblem(DistributedOverlappingVector coarseProblemRhs, DistributedOverlappingVector coarseProblemSolution);
	}
}
