using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Meshes.Manifolds;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Analysis;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.FriesPropagation;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Cracks.Jintegral;
using MGroup.XFEM.Cracks.PropagationCriteria;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.Observers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Boundaries;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Output.FriesHybridCrack;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Writers;
using Xunit;

namespace MGroup.XFEM.Tests.Fracture.HybridFries
{
	public static class EdgeCrack3D
	{
		private static string outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\plots\edge_crack_3D";
		private const int subdomainID = 0;

		private static readonly double[] minCoords = new double[] { 0, 0, 0 };
		private static readonly double[] maxCoords = new double[] { 4, 4, 4 }; // in
		private static readonly int[] numElements = new int[] { 24, 24, 24 }; // in

		private const double E = 2E7;
		private const double v = 0.3;
		private const double load = 197; // lbs
		private const double a = 1.05, growthLength = 0.3, growthAngle = -(10.0 / 180.0) * Math.PI;

		private const bool useFixedPropagator = false;
		private const int numTrialPoints = 100;
		private const double trialPointRadius = 1.5 * growthLength;
		private const double zeroStresRThetaTolerance = 5E-2;

		private const double heavisideTol = 1E-4;
		private const double tipEnrichmentArea = 0.0;
		private const int maxIterations = 8;
		private const double fractureToughness = double.MaxValue;

		private static HomogeneousFractureMaterialField3D Material 
			=> new HomogeneousFractureMaterialField3D(E, v);

		//[Fact]
		public static void RunExample()
		{
			//HERE: debug this, plot strains/stresses and make sure displacement plots are correct.
			XModel<IXCrackElement> model = CreatePhysicalModel();
			CreateGeometryModel(model);
			SetupEnrichmentOutput(model);
			RunAnalysis(model);
		}

		private static IPropagator ChooseCrackPropagator(XModel<IXCrackElement> model)
		{
			if (useFixedPropagator)
			{
				return new MockPropagator();
			}
			else
			{
				var growthAngleCriterion = new DefaultGrowthAngleCriterion(zeroStresRThetaTolerance);
				var mesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElements).BuildMesh();
				var propagator = new FriesPropagator(
					model, mesh, growthAngleCriterion, growthLength, trialPointRadius, numTrialPoints);
				propagator.plotter = new TrialPointsPlotter(outputDirectory);
				return propagator;
			}
		}

		private static void CreateGeometryModel(XModel<IXCrackElement> model)
		{
			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(
				geometryModel, new RelativeAreaSingularityResolver(heavisideTol), tipEnrichmentArea);

			IPropagator propagator = ChooseCrackPropagator(model);
			HybridFriesCrack3D crack = CreateCrackGeometry(model, propagator);
			crack.Observers.Add(new LevelSetObserver(model, crack.CrackGeometry_v2, outputDirectory));
			crack.Observers.Add(new CrackLevelSetPlotter_v2(model, crack.CrackGeometry_v2, outputDirectory));
			crack.Observers.Add(new CrackInteractingElementsPlotter(crack, outputDirectory));
			crack.Observers.Add(new CrackBody3DObserver(crack.CrackGeometry_v2, outputDirectory));
			crack.Observers.Add(new CrackFront3DObserver(crack.CrackGeometry_v2, outputDirectory));
			geometryModel.Cracks[crack.ID] = crack;
		}

		private static HybridFriesCrack3D CreateCrackGeometry(XModel<IXCrackElement> model, IPropagator propagator)
		{
			double crackHeight = /*0.40*/ 0.45 * (maxCoords[1] - minCoords[1]);

			double[] pointA = { minCoords[0], crackHeight, minCoords[2] };
			double[] pointB = { minCoords[0], crackHeight, maxCoords[2] };
			double[] pointC = { minCoords[0] + a, crackHeight, minCoords[2] };
			int numVerticesAlongAB = 7;
			int numVerticesAlongAC = 3;
			var initialCrackMesh = TriangleMesh3D.CreateForRectangle(pointA, pointB, pointC, numVerticesAlongAB, numVerticesAlongAC);

			var crackGeometry = CrackSurface3D.CreateFromMesh(0, maxCoords[0] - minCoords[0], initialCrackMesh);
			var domainBoundary = new BoxDomainBoundary3D(minCoords, maxCoords, 1E-6);
			crackGeometry.CrackFront = new CrackFront3D(crackGeometry, domainBoundary);
			var crack = new HybridFriesCrack3D(model, crackGeometry, propagator);

			return crack;
		}

		private static XModel<IXCrackElement> CreatePhysicalModel()
		{
			var model = new XModel<IXCrackElement>(3);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);
			model.FindConformingSubcells = true;

			// Materials, integration
			var material = new HomogeneousFractureMaterialField3D(E, v);
			var enrichedIntegration = new IntegrationWithNonconformingHexa3D(8, GaussLegendre3D.GetQuadratureWithOrder(2, 2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory3D(material, bulkIntegration);

			// Mesh
			var mesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			Utilities.Models.AddNodesElements(model, mesh, factory);

			// Boundary conditions
			double tol = 1E-6;
			XNode[] maxXNodes = model.Nodes.Values.Where(n => Math.Abs(n.X - maxCoords[0]) <= tol).ToArray();
			foreach (XNode node in maxXNodes)
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0.0 });
			}

			XNode[] topLeftNodes = model.Nodes.Values.Where(
				n => Math.Abs(n.X - minCoords[0]) <= tol && Math.Abs(n.Y - maxCoords[1]) <= tol).ToArray();
			foreach (XNode node in topLeftNodes)
			{
				model.NodalLoads.Add(
					new Load() { Node = node, DOF = StructuralDof.TranslationY, Amount = +load / topLeftNodes.Length });
			}
			
			XNode[] bottomLeftNodes = model.Nodes.Values.Where(
				n => Math.Abs(n.X - minCoords[0]) <= tol && Math.Abs(n.Y - minCoords[1]) <= tol).ToArray();
			foreach (XNode node in bottomLeftNodes)
			{
				model.NodalLoads.Add(
					new Load() { Node = node, DOF = StructuralDof.TranslationY, Amount = -load / bottomLeftNodes.Length });
			}

			return model;
		}

		private static void RunAnalysis(XModel<IXCrackElement> model)
		{
			//for (int t = 0; t < maxIterations; ++t)
			//{
			//	if (t == 0)
			//	{
			//		model.Initialize();
			//	}
			//	else
			//	{
			//		model.Update(null, null);
			//	}
			//}

			// Solver
			//var factory = new SkylineSolver.Factory();
			var factory = new SuiteSparseSolver.Factory();
			var algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);


			var domainBoundary = new RectangularDomainBoundary(minCoords, maxCoords);
			var termination = new TerminationLogic.Or(
				new FractureToughnessTermination(fractureToughness),
				new CrackExitsDomainTermination(domainBoundary));
			var analyzer = new QuasiStaticLefmAnalyzer(model, algebraicModel, solver, maxIterations, termination);
			analyzer.Results.Add(new StructuralFieldWriter(model, outputDirectory));

			analyzer.Analyze();
		}

		private static void SetupEnrichmentOutput(XModel<IXCrackElement> model)
		{
			// Enrichments
			var allCrackStepNodes = new AllCrackStepNodesObserver();
			var newCrackTipNodes = new NewCrackTipNodesObserver();
			var enrichmentPlotter = new CrackEnrichmentPlotterBasic(outputDirectory, newCrackTipNodes, allCrackStepNodes);

			var compositeObserver = new CompositeEnrichmentObserver();
			compositeObserver.AddObservers(allCrackStepNodes, newCrackTipNodes, enrichmentPlotter);
			model.GeometryModel.Enricher.Observers.Add(compositeObserver);
		}

		private class MockPropagator : IPropagator
		{
			public (double[] growthAngles, double[] growthLengths) Propagate(
				IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements, ICrackTipSystem[] crackTipSystems)
			{
				var growthAngles = new double[crackTipSystems.Length];
				var growthLengths = new double[crackTipSystems.Length];
				for (int i = 0; i < crackTipSystems.Length; ++i)
				{
					growthAngles[i] = growthAngle;
					growthLengths[i] = growthLength;
				}
				return (growthAngles, growthLengths);
			}
		}
	}
}
