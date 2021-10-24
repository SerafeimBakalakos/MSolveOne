using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.Constitutive.Structural;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Boundaries;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Output.Writers;
using Xunit;

namespace MGroup.XFEM.Tests.Fracture.Khoei
{
	public static class Example_7_6_2
	{
		private const double w = 50, a = 1.0;
		private const double thickness = 1.0;
		private static readonly double[] minCoords = new double[] { -w / 2.0, -w / 2.0 };
		private static readonly double[] maxCoords = new double[] { +w / 2.0, +w / 2.0 };
		private static readonly int[] numElements = new int[] { 35, 35 };
		private const double E = 2.1E6, v = 0.3;
		private const double syy = 2000, sxx = 0, sxy = 0;
		private const int subdomainID = 0;

		[Fact]
		public static void PlotSolution()
		{
			// Create and analyze model, in order to get the solution vector
			XModel<IXCrackElement> model = CreateModel();
			model.Initialize();
			(IAlgebraicModel algebraicModel, IGlobalVector globalU, _) = RunAnalysis(model);

			// Plot
			string outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\plots\khoei_7_6_2";
			var writer = new StructuralFieldWriter(model, outputDirectory, false, false, true);
			writer.WriteResults(algebraicModel, globalU);
		}

		private static void ApplyLinearBoundaryConditions(XModel<IXCrackElement> model)
		{
			// Equivalent displacements for plane stress
			double exx = Math.Abs(-v / E * syy);
			double eyy = Math.Abs(1.0 / E * syy);
			double ux = exx * w;
			double uy = eyy * w;


			// Find boundary nodes of interest
			double tol = 1E-6;
			var nodesXMin = new List<XNode>();
			var nodesXMax = new List<XNode>();
			var nodesYMin = new List<XNode>();
			var nodesYMax = new List<XNode>();
			foreach (XNode node in model.Nodes.Values)
			{
				if (Math.Abs(node.X - minCoords[0]) <= tol)
				{
					nodesXMin.Add(node);
				}
				if (Math.Abs(node.X - maxCoords[0]) <= tol)
				{
					nodesXMax.Add(node);
				}
				if (Math.Abs(node.Y - minCoords[1]) <= tol)
				{
					nodesYMin.Add(node);
				}
				if (Math.Abs(node.Y - maxCoords[1]) <= tol)
				{
					nodesYMax.Add(node);
				}
			}

			// Prescribe displacements to boundary nodes
			foreach (XNode node in nodesYMax)
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = +uy });
			}
			foreach (XNode node in nodesYMin)
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = -uy });
			}
			foreach (XNode node in nodesXMax)
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = +ux });
			}
			foreach (XNode node in nodesXMin)
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = -ux });
			}
		}

		//private static void ApplySimpleBoundaryConditions(XModel<IXCrackElement> model)
		//{
		//	HERE
		//}

		private static TempEdgeCrack2D CreateCrack(XModel<IXCrackElement> model, IPropagatorOLD propagator = null)
		{
			double crackHeight = 0.5 * (minCoords[1] + maxCoords[1]);
			var crackStart = new double[] { -a, crackHeight };
			var crackEnd = new double[] { +a, crackHeight };
			var initialGeom = new PolyLine2D(crackStart, crackEnd);

			double domainDimension = maxCoords[0] - minCoords[0];
			var crackVertices = new List<Vertex2D>();
			for (int i = 0; i < initialGeom.Vertices.Count; ++i)
			{
				double[] point = initialGeom.Vertices[i];
				crackVertices.Add(new Vertex2D(i, point));
			}
			var hybridGeometry = new CrackCurve2D(0, domainDimension, crackVertices);
			hybridGeometry.CrackFront = new CrackFront2D(hybridGeometry, new RectangularDomainBoundary(minCoords, maxCoords));

			var crack = new TempEdgeCrack2D(0, initialGeom, hybridGeometry, model, propagator, false);

			#region plot
			//string outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\matrices";
			//crack.Observers.Add(new CrackBody2DObserver(crack.hybridGeometry, outputDirectory));
			//crack.Observers.Add(new CrackFront2DObserver(crack.hybridGeometry, outputDirectory));
			//crack.Observers.Add(new CrackLevelSetPlotter_v2(model, crack.hybridGeometry, outputDirectory));
			//model.ModelObservers.Add(new CoordinatesAtGaussPointsPlotter(
			//		model, crack.hybridGeometry, crack.hybridFrontCoordinateSystem, outputDirectory));
			#endregion

			return crack;
		}

		private static XModel<IXCrackElement> CreateModel()
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);

			// Materials, integration
			var material = new HomogeneousFractureMaterialField2D(E, v, thickness, false);
			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory2D(material, thickness, bulkIntegration);

			// Mesh
			var mesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			Utilities.Models.AddNodesElements(model, mesh, factory);

			ApplyLinearBoundaryConditions(model);

			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(geometryModel, new NullSingularityResolver());
			//var crack = new Crack(model);
			var crack = CreateCrack(model, null);
			geometryModel.Cracks[crack.ID] = crack;

			return model;
		}

		private static (IAlgebraicModel algebraicModel, IGlobalVector globalU, IMatrixView globalK)
			RunAnalysis(XModel<IXCrackElement> model)
		{
			// Solver
			var factory = new SkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);
			solver.PreventFromOverwrittingSystemMatrices(); // Necessary to extract the stiffness matrix.

			// Problem type
			var provider = new ProblemStructural(model, algebraicModel, solver);

			// Analyzers
			var childAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, provider);
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, provider, childAnalyzer);

			// Run the anlaysis 
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			return (algebraicModel, algebraicModel.LinearSystem.Solution, algebraicModel.LinearSystem.Matrix.SingleMatrix);
		}
	}
}
