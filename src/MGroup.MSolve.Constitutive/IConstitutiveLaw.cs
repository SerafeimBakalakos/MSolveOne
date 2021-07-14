using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.DataStructures;

namespace MGroup.MSolve.Constitutive
{
	/// <summary>
	/// Interface for constitutive laws to be adhered to, for spatial discretization (e.g.: FEM) 
	/// </summary>
	public interface IConstitutiveLaw : ICreateState
	{
		IMatrixView ConstitutiveMatrix { get; }
		double[] UpdateConstitutiveMatrixAndEvaluateResponse(double[] stimuli);

		#region First implementation
		//IReadOnlyDictionary<string, double> Properties { get; }
		//IMatrixView ConstitutiveMatrix { get; }
		//double[] EvaluateResponse(double[] stimuli);

		//bool IsModified { get; }
		//void ResetResponseHistory();
		//void SaveState();
		//void ResetState();

		//IConstitutiveLaw Clone();
		#endregion

	}
}
