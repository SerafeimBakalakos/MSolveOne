using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed.Overlapping;

namespace MGroup.Solvers.DDM.FetiDP.Reanalysis
{
	/// <summary>
	/// Will always start the new solution from 0.
	/// </summary>
	public class ZeroInitialSolutionGuess : IInitialSolutionGuessStrategy
	{
		public (DistributedOverlappingVector guess, bool isZero) GuessFirstSolution(
			DistributedOverlappingIndexer currentLagrangeVectorIndexer)
		{
			return (new DistributedOverlappingVector(currentLagrangeVectorIndexer), true);
		}

		public (DistributedOverlappingVector guess, bool isZero) GuessNextSolution(
			DistributedOverlappingIndexer currentLagrangeVectorIndexer, DistributedOverlappingVector previousSolution)
		{
			return (new DistributedOverlappingVector(currentLagrangeVectorIndexer), true);
		}
	}
}