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
using MGroup.Solvers.Iterative;
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
	public static class FriesExample_7_1_2
	{
		private static string outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\plots\example_7_1_2";
		private const int subdomainID = 0;

		private static readonly double[] minCoords = new double[] { 0, 0 };
		private static readonly double[] maxCoords = new double[] { 675, 150 };
		//private static readonly int[] numElements = new int[] { 45, 10 };
		private static readonly int[] numElements = new int[] { 135, 30 };

		private const double thickness = 75.0, E = 3E7, v = 0.300, load = -1000;
		private static readonly double[] loadCoords = { 487.5, 150 };
		private static readonly double[] support0Coords = { 257.5, 0 };
		private static readonly double[] support1Coords = { 637.5, 0 };
		private static readonly double[] support2Coords = { 37, 150 };
		private static readonly double[] crackMouthCoords = { 337.5, 0 };
		private static readonly double[] crackFrontCoords = { 337.5, 74 };

		//private const double da = 8, rc = 1.5;
		private const double da = 4, rc = 1.5;
		private const int numTrialPoints = 100;
		private const double zeroStresRThetaTolerance = 5E-2;

		private const double growthAngle = -(40.0 / 180.0) * Math.PI;
		
		private const double heavisideTol = 1E-4;
		private const double tipEnrichmentArea = 0.0;
		//private const int maxIterations = 13;
		private const int maxIterations = 26;
		private const double fractureToughness = double.MaxValue;

		private const double jIntegralRadiusRatio = 2.0;


		private static HomogeneousFractureMaterialField2D Material
			=> new HomogeneousFractureMaterialField2D(E, v, thickness, true);

		//[Fact]
		public static void RunExample()
		{
			//HERE: debug this, plot strains/stresses and make sure displacement plots are correct.
			XModel<IXCrackElement> model = CreatePhysicalModel();
			CreateGeometryModel(model);
			//SetupEnrichmentOutput(model);
			RunAnalysis(model);
		}

		private static void CreateGeometryModel(XModel<IXCrackElement> model)
		{
			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(
				geometryModel, new RelativeAreaSingularityResolver(heavisideTol), tipEnrichmentArea);

			ExteriorLsmCrack2D crack = CreateCrackGeometry(model);
			var outputMesh = new ContinuousOutputMesh(model.Nodes.Values.OrderBy(n => n.ID).ToList(), model.EnumerateElements());
			crack.Observers.Add(new CrackLevelSetPlotter(crack, outputMesh, outputDirectory));
			geometryModel.Cracks[crack.ID] = crack;
		}

		private static ExteriorLsmCrack2D CreateCrackGeometry(XModel<IXCrackElement> model)
		{
			var point0 = new double[] { crackMouthCoords[0], crackMouthCoords[1] };
			var point1 = new double[] { crackFrontCoords[0], crackFrontCoords[1] };
			var initialGeom = new PolyLine2D(point0, point1);

			var jIntegrationRule = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(4, 4));
			var propagator = new JintegralPropagator2D(model, jIntegralRadiusRatio, jIntegrationRule, Material,
				new MaximumCircumferentialTensileStressCriterion(), new ConstantIncrement2D(da));
			var crack = new ExteriorLsmCrack2D(0, initialGeom, model, propagator);

			return crack;
		}

		private static XModel<IXCrackElement> CreatePhysicalModel()
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);
			model.FindConformingSubcells = true;

			// Materials, integration
			var material = new HomogeneousFractureMaterialField2D(E, v, thickness, true);
			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory2D(material, thickness, bulkIntegration);

			// Mesh
			var mesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			Utilities.Models.AddNodesElements(model, mesh, factory);

			// Boundary conditions
			List<XNode> loadedNodes = FindNodesNear(loadCoords[0], loadCoords[1], model);
			foreach (XNode node in loadedNodes)
			{
				model.NodalLoads.Add(new Load() 
				{ 
					Node = node, DOF = StructuralDof.TranslationY, Amount = load / loadedNodes.Count 
				});
			}

			foreach (XNode node in FindNodesNear(support0Coords[0], support0Coords[1], model))
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
			}

			foreach (XNode node in FindNodesNear(support1Coords[0], support1Coords[1], model))
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
			}

			foreach (XNode node in FindNodesNear(support2Coords[0], support2Coords[1], model))
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
			} 

			return model;
		}

		private static List<XNode> FindNodesNear(double x, double y, XModel<IXCrackElement> model)
		{
			double minDistance = double.MaxValue;
			double[] point = { x, y };
			var results = new List<XNode>();
			foreach (XNode node in model.Nodes.Values)
			{
				double distance = XFEM.Geometry.Utilities.Distance2D(node.Coordinates, point);
				if (distance < minDistance)
				{
					minDistance = distance;
					results.Clear();
					results.Add(node);
				}
				else if (distance == minDistance)
				{
					results.Add(node);
				}
			}
			return results;
		}

		private static void RunAnalysis(XModel<IXCrackElement> model)
		{
			// Solver
			//var factory = new SkylineSolver.Factory();

			//var factory = new SuiteSparseSolver.Factory();
			//var algebraicModel = factory.BuildAlgebraicModel(model);
			//var solver = factory.BuildSolver(algebraicModel);

			var factory = new PcgSolver.Factory();
			var algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);


			var domainBoundary = new RectangularDomainBoundary(minCoords, maxCoords);
			var termination = new TerminationLogic.Or(
				new FractureToughnessTermination(fractureToughness),
				new CrackExitsDomainTermination(domainBoundary));
			var analyzer = new QuasiStaticLefmAnalyzer(model, algebraicModel, solver, maxIterations, termination);
			analyzer.Results.Add(new StructuralFieldWriter(model, outputDirectory));
			analyzer.Results.Add(new SolutionNormLogger(outputDirectory + "\\solution_norm.txt"));

			analyzer.Analyze();

			solver.Logger.ExtraInfo = $"PCG Solver, v={v}";
			solver.Logger.WriteAggregatesToFile(outputDirectory + "\\other_results\\performance.txt", true);
		}

		private static void SetupEnrichmentOutput(XModel<IXCrackElement> model)
		{
			model.ModelObservers.Add(new StructuralBoundaryConditionsPlotter(outputDirectory, model));

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
			private int iteration = 0;

			public (double[] growthAngles, double[] growthLengths) Propagate(
				IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements, ICrackTipSystem[] crackTipSystems)
			{
				double theta = iteration == 0 ? growthAngle : 0;

				var growthAngles = new double[crackTipSystems.Length];
				var growthLengths = new double[crackTipSystems.Length];
				for (int i = 0; i < crackTipSystems.Length; ++i)
				{
					growthAngles[i] = theta;
					growthLengths[i] = da;
				}

				++iteration;
				return (growthAngles, growthLengths);
			}
		}
	}
}
