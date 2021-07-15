using System;
using System.Collections.Generic;

using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Solution.LinearSystem;

namespace MGroup.Solvers.LinearSystem
{
	public class DistributedLinearSystem<TMatrix> : IGlobalLinearSystem, IDistributedLinearSystem
		where TMatrix : class, IMatrix
	{
		private readonly Func<IGlobalVector, DistributedVector> checkCompatibleVector;
		private readonly Func<IGlobalMatrix, DistributedMatrix<TMatrix>> checkCompatibleMatrix;

		public DistributedLinearSystem(Func<IGlobalVector, DistributedVector> checkCompatibleVector,
			Func<IGlobalMatrix, DistributedMatrix<TMatrix>> checkCompatibleMatrix)
		{
			this.checkCompatibleVector = checkCompatibleVector;
			this.checkCompatibleMatrix = checkCompatibleMatrix;
			Observers = new HashSet<ILinearSystemObserver>();
		}

		IGlobalMatrix IGlobalLinearSystem.Matrix
		{
			get => Matrix;
			set
			{
				DistributedMatrix<TMatrix> globalMatrix = checkCompatibleMatrix(value);
				foreach (var observer in Observers)
				{
					observer.HandleMatrixWillBeSet();
				}
				Matrix = globalMatrix;
			}
		}

		public DistributedMatrix<TMatrix> Matrix { get; set; }

		public HashSet<ILinearSystemObserver> Observers { get; }

		IGlobalVector IGlobalLinearSystem.RhsVector
		{
			get => RhsVector;
			set
			{
				DistributedVector globalVector = checkCompatibleVector(value);
				RhsVector = globalVector;
			}
		}

		public DistributedVector RhsVector { get; set; }

		IGlobalVector IGlobalLinearSystem.Solution => Solution;

		public DistributedVector Solution { get; set; }
	}
}
