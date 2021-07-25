using System;
using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.PlanarElements;
using MGroup.Environments;
using MGroup.FEM.Entities;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.Tests.Commons;
using MGroup.Solvers.Results;

// Subdomains:
// /|
// /||-------|-------|-------|-------|
// /||  (4)  |  (5)  |  (6)  |  (7)  |
// /||   E1  |   E0  |   E0  |   E0  |
// /||-------|-------|-------|-------|
// /||  (0)  |  (1)  |  (2)  |  (3)  |
// /||   E1  |   E0  |   E0  |   E0  |
// /||-------|-------|-------|-------|
// /|
namespace MGroup.Solvers.DDM.Tests.ExampleModels
{
	/// <summary>
	/// See Papagiannakis Bachelor thesis, pages 134-147
	/// </summary>
	public static class PapagiannakisExample_9_1
	{
		private const double E0 = 2.1E7, v = 0.3, thickness = 0.3;
		private const double load = 100;

		public static double[] MinCoords => new double[] { 0, 0 };

		public static double[] MaxCoords => new double[] { 3, 1.5 };

		public static int[] NumElements => new int[] { 20, 20 };

		public static int[] NumSubdomains => new int[] { 4, 2 };

		public static int[] NumClusters => new int[] { 1, 1 };

		public static int NumTotalDofs => 882;

		public static Model CreateSingleSubdomainModel(double stiffnessRatio)
		{
			UniformDdmModelBuilder2D builder = CreateDefaultModelBuilder(stiffnessRatio);
			builder.NumSubdomains = new int[] { 1, 1 };
			builder.NumClusters = new int[] { 1, 1 };
			return builder.BuildSingleSubdomainModel();
		}

		public static (Model model, ComputeNodeTopology topology) CreateMultiSubdomainModel(double stiffnessRatio)
		{
			UniformDdmModelBuilder2D builder = CreateDefaultModelBuilder(stiffnessRatio);
			builder.NumSubdomains = NumSubdomains;
			builder.NumClusters = NumClusters;
			return builder.BuildMultiSubdomainModel();
		}

		public static ICornerDofSelection GetCornerDofs(IModel model) => UniformDdmModelBuilder2D.FindCornerDofs(model, 2);

		public static NodalResults SolveWithSkylineSolver(Model model) => PapagiannakisExample_8.SolveWithSkylineSolver(model);

		private static UniformDdmModelBuilder2D CreateDefaultModelBuilder(double stiffnessRatio)
		{
			var builder = new UniformDdmModelBuilder2D();
			builder.MinCoords = MinCoords;
			builder.MaxCoords = MaxCoords;
			builder.NumElementsTotal = NumElements;

			double E1 = stiffnessRatio * E0;
			builder.GetMaterialPerElementIndex = elementIdx =>
			{
				if (elementIdx[0] < 5)
				{
					return new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = E1, PoissonRatio = v };
				}
				else
				{
					return new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = E0, PoissonRatio = v };
				}
			};
			builder.Thickness = thickness;

			builder.PrescribeDisplacement(UniformDdmModelBuilder2D.BoundaryRegion.LeftSide, StructuralDof.TranslationX, 0.0);
			builder.PrescribeDisplacement(UniformDdmModelBuilder2D.BoundaryRegion.LeftSide, StructuralDof.TranslationY, 0.0);
			builder.DistributeLoadAtNodes(UniformDdmModelBuilder2D.BoundaryRegion.RightSide, StructuralDof.TranslationY, load);

			return builder;
		}
	}
}
