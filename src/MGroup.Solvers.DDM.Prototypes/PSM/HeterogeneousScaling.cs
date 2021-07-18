using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
	public class HeterogeneousScaling : IPsmScaling
	{
		private readonly IModel model;

		public HeterogeneousScaling(IModel model)
		{
			this.model = model;
		}

		public void CalcScalingMatrices(Func<int, Matrix> getKff)
		{
			throw new NotImplementedException();
		}

		public void ScaleRhsVector(int subdomainID, Vector Fb)
		{
			throw new NotImplementedException();
		}
	}
}
