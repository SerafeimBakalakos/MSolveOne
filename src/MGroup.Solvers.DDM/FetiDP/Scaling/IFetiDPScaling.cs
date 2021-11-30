using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.FetiDP.Scaling
{
	public interface IFetiDPScaling
	{
		IDictionary<int, DiagonalMatrix> SubdomainMatricesWbr { get; }

		void CalcScalingMatrices();

		void ScaleSubdomainRhsVector(int subdomainID, Vector rhsAtFreeDofs);

		void ScaleSubdomainSolutionVector(int subdomainID, Vector solutionAtFreeDofs);
	}
}
