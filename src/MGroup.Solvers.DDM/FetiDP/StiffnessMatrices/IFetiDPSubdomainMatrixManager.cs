using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.FetiDP.StiffnessMatrices
{
	public interface IFetiDPSubdomainMatrixManager
	{
		bool IsEmpty { get; }

		IMatrix SchurComplementOfRemainderDofs { get; }

		void CalcSchurComplementOfRemainderDofs();

		void ClearSubMatrices();

		void ExtractKrrKccKrc();

		void HandleDofsWereModified();

		void InvertKrr();

		Vector MultiplyInverseKrrTimes(Vector vector);

		Vector MultiplyKccTimes(Vector vector);

		Vector MultiplyKcrTimes(Vector vector);

		Vector MultiplyKrcTimes(Vector vector);

		/// <summary>
		/// S[s] * x = (Kcc[s] - Kcr[s] * inv(Krr[s]) * Krc[s]) * x, where s is a subdomain and x is the <paramref name="input"/>.
		/// </summary>
		/// <param name="input">The displacements that correspond to corner dofs of this subdomain.</param>
		/// <param name="output">The forces that correspond to corner dofs of this subdomain.</param>
		public void MultiplySchurComplementImplicitly(Vector input, Vector output)
		{
			output.CopyFrom(MultiplyKccTimes(input));
			Vector temp = MultiplyKrcTimes(input);
			temp = MultiplyInverseKrrTimes(temp);
			temp = MultiplyKcrTimes(temp);
			output.SubtractIntoThis(temp);
		}

		void ReorderRemainderDofs();
	}
}
