using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
    public class HeterogeneousScaling : IPrimalScaling
    {
		private readonly IModel model;

		public HeterogeneousScaling(IModel model)
		{
			this.model = model;
		}

		public Dictionary<int, SparseVector> DistributeNodalLoads(Table<INode, IDofType, double> nodalLoads)
        {
			throw new NotImplementedException();
		}
    }
}
