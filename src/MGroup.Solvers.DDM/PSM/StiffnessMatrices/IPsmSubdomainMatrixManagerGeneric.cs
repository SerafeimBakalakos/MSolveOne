using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.PSM.StiffnessMatrices
{
	public interface IPsmSubdomainMatrixManagerGeneric<TMatrix> : IPsmSubdomainMatrixManager
		where TMatrix : class, IMatrix
	{
		void ExtractKiiKbbKib(TMatrix Kff);

		void ReorderInternalDofs(TMatrix Kff);
	}
}
