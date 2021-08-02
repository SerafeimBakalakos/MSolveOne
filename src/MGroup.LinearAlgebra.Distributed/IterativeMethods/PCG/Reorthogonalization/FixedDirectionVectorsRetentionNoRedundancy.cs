using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.LinearAlgebra.Distributed.IterativeMethods.PCG.Reorthogonalization
{
	public class FixedDirectionVectorsRetentionNoRedundancy : IDirectionVectorsRetention
	{
		private readonly int numVectorsToKeep;

		private ReorthogonalizedPcg_v2 pcg;

		public FixedDirectionVectorsRetentionNoRedundancy(int numVectorsToKeep)
		{
			this.numVectorsToKeep = numVectorsToKeep;
		}

		public void DiscardDirectionVectors() { }

		public void Intialize(ReorthogonalizedPcg_v2 pcg)
		{
			this.pcg = pcg;
		}

		public bool KeepUsingReorthogonalization() => pcg.ReorthoCache.Directions.Count < numVectorsToKeep;
	}
}
