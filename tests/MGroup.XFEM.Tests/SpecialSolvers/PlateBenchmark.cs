using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Analysis;
using MGroup.XFEM.Cracks;
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
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Solvers.PFetiDP;
using Xunit;

namespace MGroup.XFEM.Tests.SpecialSolvers
{
	public static class PlateBenchmark
	{
		private static readonly double[] minCoords = new double[] { 0, 0 };
		private static readonly double[] maxCoords = new double[] { 4, 4 }; // in
		private const double thickness = 1.0;
		private const double E = 2E7;
		private const double v = 0.3;
		private const bool planeStress = true;
		private const double load = 197; // lbs
		private const double a = 1.05, growthLength = 0.3; 

		private const double jIntegralRadiusRatio = 2.0;
		private const double heavisideTol = 1E-4;
		private const double tipEnrichmentArea = 0.0;
		private const int maxIterations = 8;
		private const double fractureToughness = double.MaxValue;

		private static HomogeneousFractureMaterialField2D Material 
			=> new HomogeneousFractureMaterialField2D(E, v, thickness, planeStress);

		public static void CreateGeometryModel(XModel<IXCrackElement> model)
		{
			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(
				geometryModel, new RelativeAreaSingularityResolver(heavisideTol), tipEnrichmentArea);
			double crackHeight = /*0.40*/ 0.45 * (maxCoords[1] - minCoords[1]);
			var point0 = new double[] { minCoords[0], crackHeight };
			var point1 = new double[] { minCoords[0] + a, crackHeight };
			var initialGeom = new PolyLine2D(point0, point1);

			//TODO: This is probably better, but my expected results are from the next one
			//var jIntegrationRule = new JintegrationStrategy(
			//    GaussLegendre2D.GetQuadratureWithOrder(4, 4),
			//    new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(4, 4)));
			var jIntegrationRule = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(4, 4));
			var propagator = new JintegralPropagator2D(model, jIntegralRadiusRatio, jIntegrationRule, Material,
				new MaximumCircumferentialTensileStressCriterion(), new ConstantIncrement2D(growthLength));
			var crack = new ExteriorLsmCrack2D(0, initialGeom, model, propagator);
			geometryModel.Cracks[crack.ID] = crack;
		}


		public static UniformDdmCrackModelBuilder2D DescribePhysicalModel(int[] numElements, 
			int[] numSubdomains = null, int[] numClusters = null)
		{
			if (numSubdomains == null)
			{
				numSubdomains = new int[] { 1, 1 };
				numClusters = new int[] { 1, 1 };
			}

			var modelBuilder = new UniformDdmCrackModelBuilder2D();
			modelBuilder.MinCoords = minCoords;
			modelBuilder.MaxCoords = maxCoords;
			modelBuilder.Thickness = thickness;

			modelBuilder.NumElementsTotal = numElements;
			modelBuilder.NumSubdomains = numSubdomains;
			modelBuilder.NumClusters = numClusters;

			modelBuilder.MaterialField = Material;
			//var enrichedIntegration = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var enrichedIntegration = new IntegrationWithConformingSubtriangles2D(
				TriangleQuadratureSymmetricGaussian.Order2Points3);
			modelBuilder.BulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);

			modelBuilder.PrescribeDisplacement(
				UniformDdmCrackModelBuilder2D.BoundaryRegion.RightSide, StructuralDof.TranslationX, 0.0); 
			modelBuilder.PrescribeDisplacement(
				UniformDdmCrackModelBuilder2D.BoundaryRegion.RightSide, StructuralDof.TranslationY, 0.0);
			modelBuilder.DistributeLoadAtNodes(
				UniformDdmCrackModelBuilder2D.BoundaryRegion.UpperLeftCorner, StructuralDof.TranslationY, +load);
			modelBuilder.DistributeLoadAtNodes(
				UniformDdmCrackModelBuilder2D.BoundaryRegion.LowerLeftCorner, StructuralDof.TranslationY, -load);

			modelBuilder.FindConformingSubcells = true;

			return modelBuilder;
		}

		public static void RunAnalysis(XModel<IXCrackElement> model, IAlgebraicModel algebraicModel, ISolver solver)
		{
			var domainBoundary = new RectangularDomainBoundary(minCoords, maxCoords);
			var termination = new TerminationLogic.Or(
				new FractureToughnessTermination(fractureToughness), 
				new CrackExitsDomainTermination(domainBoundary));
			var analyzer = new QuasiStaticLefmAnalyzer(model, algebraicModel, solver, maxIterations, termination);

			analyzer.Analyze();
		}

		public static void SetupModelOutput(XModel<IXCrackElement> model, string outputDirectory)
		{
			// Crack geometry and interactions
			var crack = (ExteriorLsmCrack2D)model.GeometryModel.GetDiscontinuity(0);
			var outputMesh = new ContinuousOutputMesh(model.Nodes.Values.OrderBy(n => n.ID).ToList(), model.EnumerateElements());
			crack.Observers.Add(new CrackLevelSetPlotter(crack, outputMesh, outputDirectory));
			crack.Observers.Add(new CrackInteractingElementsPlotter(crack, outputDirectory));

			// Enrichments
			var allCrackStepNodes = new AllCrackStepNodesObserver();
			var newCrackStepNodes = new NewCrackStepNodesObserver();
			var newCrackTipNodes = new NewCrackTipNodesObserver();
			var previousCrackTipNodes = new PreviousCrackTipNodesObserver();
			var rejectedCrackStepNodes = new RejectedCrackStepNodesObserver(
				crack, newCrackTipNodes, allCrackStepNodes);
			var nodesWithModifiedLevelSet = new CrackStepNodesWithModifiedLevelSetObserver(model);
			var nodesWithModifiedEnrichments = new NodesWithModifiedEnrichmentsObserver(
				newCrackStepNodes, nodesWithModifiedLevelSet, newCrackTipNodes, previousCrackTipNodes);
			var elementsWithModifiedNodes = new ElementsWithModifiedNodesObserver(nodesWithModifiedEnrichments);
			var nodesNearModifiedNodes = new NodesNearModifiedNodesObserver(
				nodesWithModifiedEnrichments, elementsWithModifiedNodes);
			var enrichmentPlotter = new CrackEnrichmentPlotterFull(crack, outputDirectory, newCrackTipNodes, previousCrackTipNodes,
				allCrackStepNodes, newCrackStepNodes, rejectedCrackStepNodes, nodesNearModifiedNodes);

			var compositeObserver = new CompositeEnrichmentObserver();
			compositeObserver.AddObservers(
				allCrackStepNodes, newCrackStepNodes, newCrackTipNodes, previousCrackTipNodes, rejectedCrackStepNodes,
				nodesWithModifiedLevelSet, nodesWithModifiedEnrichments, elementsWithModifiedNodes, nodesNearModifiedNodes,
				enrichmentPlotter);
			model.GeometryModel.Enricher.Observers.Add(compositeObserver);
		}

		public static void SetupPartitioningOutput(IComputeEnvironment environment, XModel<IXCrackElement> model, 
			CrackFetiDPCornerDofsPlusLogging cornerDofs, string outputDirectory)
		{
			model.ModelObservers.Add(new PartitioningPlotter(outputDirectory, model, 2));
			model.ModelObservers.Add(new CornerNodesPlotter(environment, model, cornerDofs, outputDirectory));
		}

		public static void WriteCrackPath(XModel<IXCrackElement> model)
		{
			var crack = (ExteriorLsmCrack2D)model.GeometryModel.GetDiscontinuity(0);

			Debug.WriteLine("Crack path:");
			for (int i = 0; i < crack.CrackPath.Count; ++i)
			{
				Debug.WriteLine($"{crack.CrackPath[i][0]} \t {crack.CrackPath[i][1]}");
			}
		}
	}
}
