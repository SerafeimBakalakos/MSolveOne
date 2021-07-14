using System;
using MGroup.LinearAlgebra.Matrices;

namespace MGroup.MSolve.Constitutive
{
	/// <summary>
	/// Interface for materials laws implementations to be used in finite elements 
	/// </summary>
	public interface IFiniteElementMaterial : ICloneable
	{
		IMatrixView ConstitutiveMatrix { get; }
		double[] UpdateConstitutiveMatrixAndEvaluateResponse(double[] strains);
		bool IsCurrentStateDifferent();
		void ResetModified();
		void CreateState();
	}
}
