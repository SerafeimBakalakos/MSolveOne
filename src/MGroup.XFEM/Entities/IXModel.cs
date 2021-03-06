using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Elements;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.XFEM.Entities
{
	public interface IXModel : IModel
	{
		int Dimension { get; }

		Dictionary<int, XNode> Nodes { get; }

		IEnumerable<IXFiniteElement> EnumerateElements();

		void Initialize();

		void Update(IAlgebraicModel algebraicModel, IGlobalVector solutionFreeDofs);
	}
}
