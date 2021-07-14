using System.Collections.Generic;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.AnalysisWorkflow.Providers;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.FEM.Entities;
using System.Linq;

using MGroup.MSolve.Solution.AlgebraicModel;


//TODO: Usually the LinearSystem is passed in, but for GetRHSFromHistoryLoad() it is stored as a field. Decide on one method.
//TODO: Right now this class decides when to build or rebuild the matrices. The analyzer should decide that.
namespace MGroup.Constitutive.Structural
{
	public class ProblemStructural : IImplicitIntegrationProvider, IStaticProvider, INonLinearProvider
	{
		private IGlobalMatrix mass, damping, stiffness;
		private readonly IModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ISolver solver;
		private ElementStructuralStiffnessProvider stiffnessProvider = new ElementStructuralStiffnessProvider();
		private ElementStructuralMassProvider massProvider = new ElementStructuralMassProvider();
		private ElementStructuralDampingProvider dampingProvider = new ElementStructuralDampingProvider();
		private readonly IElementMatrixPredicate rebuildStiffnessPredicate = new MaterialModifiedElementMarixPredicate();

		public ProblemStructural(IModel model, IAlgebraicModel algebraicModel, ISolver solver)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
		}

		private IGlobalMatrix Mass
		{
			get
			{
				if (mass == null) BuildMass();
				return mass;
			}
		}

		private IGlobalMatrix Damping
		{
			get
			{
				if (damping == null) BuildDamping();
				return damping;
			}
		}

		private IGlobalMatrix Stiffness
		{
			get
			{
				if (stiffness == null)
					BuildStiffness();
				else
				{
					RebuildStiffness(); // This is the same but also resets the material modified properties. 
				}
				return stiffness;
			}
		}

		private void BuildStiffness() 
			=> stiffness = algebraicModel.BuildGlobalMatrix(model.EnumerateElements, stiffnessProvider);

		private void RebuildStiffness()
		{
			algebraicModel.RebuildGlobalMatrixPartially(
				stiffness, model.EnumerateElements, stiffnessProvider, rebuildStiffnessPredicate);

			// Original code kept, in case we need to reproduce its behavior.
			//foreach (ISubdomain subdomain in model.Subdomains)
			//{
			//    if (subdomain.MaterialsModified)
			//    {
			//        stiffness[subdomain.ID] = solver.BuildGlobalMatrix(subdomain, stiffnessProvider);
			//        subdomain.ResetMaterialsModifiedProperty();
			//    }
			//}
		}

		private void BuildMass() 
			=> mass = algebraicModel.BuildGlobalMatrix(model.EnumerateElements, massProvider);

		//TODO: With Rayleigh damping, C is more efficiently built using linear combinations of global K, M, 
		//      instead of building and assembling element k, m matrices.
		private void BuildDamping() 
			=> damping = algebraicModel.BuildGlobalMatrix(model.EnumerateElements, dampingProvider);

		#region IAnalyzerProvider Members
		public void ClearMatrices()
		{
			damping = null;
			stiffness = null;
			mass = null;
		}

		public void Reset()
		{
			// TODO: Check if we should clear material state - (goat) removed that, seemed erroneous
			//foreach (ISubdomain subdomain in model.Subdomains)
			//	foreach (IElement element in subdomain.Elements)
			//		element.ElementType.ClearMaterialState();

			damping = null;
			stiffness = null;
			mass = null;
		}
		#endregion

		#region IImplicitIntegrationProvider Members

		public void LinearCombinationOfMatricesIntoStiffness(ImplicitIntegrationCoefficients coefficients)
		{
			//TODO: when the matrix is mutated, the solver must be informed via observers (or just flags).
			IGlobalMatrix matrix = Stiffness;
			matrix.LinearCombinationIntoThis(coefficients.Stiffness, Mass, coefficients.Mass);
			matrix.AxpyIntoThis(Damping, coefficients.Damping);
			solver.LinearSystem.Matrix = matrix;
		}

		public void ProcessRhs(ImplicitIntegrationCoefficients coefficients, IGlobalVector rhs)
		{
			// Method intentionally left empty.
		}

		public IGlobalVector GetAccelerationsOfTimeStep(int timeStep)
		{
			var femModel = (Model)model;
			IGlobalVector d = algebraicModel.CreateZeroVector();
			if (femModel.MassAccelerationHistoryLoads.Count > 0)
			{
				List<MassAccelerationLoad> m = new List<MassAccelerationLoad>(femModel.MassAccelerationHistoryLoads.Count);
				foreach (IMassAccelerationHistoryLoad l in femModel.MassAccelerationHistoryLoads)
				{
					m.Add(new MassAccelerationLoad() { Amount = l[timeStep], DOF = l.DOF });
				}
				algebraicModel.AddToGlobalVector(m, d);
			}
			return d;
		}

		public IGlobalVector GetVelocitiesOfTimeStep(int timeStep) => algebraicModel.CreateZeroVector();

		public IGlobalVector GetRhsFromHistoryLoad(int timeStep)
		{
			var femModel = (Model)model;
			femModel.TimeStep = timeStep;

			solver.LinearSystem.RhsVector.Clear(); //TODO: this is also done by model.AssignLoads()
			AssignRhs();
			algebraicModel.AddToGlobalVector(femModel.EnumerateMassAccelerationHistoryLoads, solver.LinearSystem.RhsVector);

			return solver.LinearSystem.RhsVector.Copy();
		}

		public IGlobalVector MassMatrixVectorProduct(IGlobalVector vector)
		{
			IGlobalVector result = algebraicModel.CreateZeroVector();
			Mass.MultiplyVector(vector, result);
			return result;
		}

		public IGlobalVector DampingMatrixVectorProduct(IGlobalVector vector)
		{
			IGlobalVector result = algebraicModel.CreateZeroVector();
			Damping.MultiplyVector(vector, result);
			return result;
		}

		#endregion

		#region IStaticProvider Members
		public void CalculateMatrix()
		{
			if (stiffness == null) BuildStiffness();
			solver.LinearSystem.Matrix = stiffness;
		}
		#endregion

		#region INonLinearProvider Members
		public double CalculateRhsNorm(IGlobalVector rhs) => rhs.Norm2();

		public void ProcessInternalRhs(IGlobalVector solution, IGlobalVector rhs) { }
		#endregion

		public void AssignRhs()
		{//TODO: Remove type casting
			var femModel = (Model)model;
			solver.LinearSystem.RhsVector.Clear();
			algebraicModel.AddToGlobalVector(femModel.EnumerateNodalLoads, solver.LinearSystem.RhsVector);
			algebraicModel.AddToGlobalVector(femModel.EnumerateElementMassLoads, solver.LinearSystem.RhsVector);
			algebraicModel.AddToGlobalVector(femModel.EnumerateMassAccelerationLoads, solver.LinearSystem.RhsVector);
		}
	}
}
