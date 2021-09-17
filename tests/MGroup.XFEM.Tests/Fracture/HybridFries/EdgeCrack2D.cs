using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
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
	public static class EdgeCrack2D
	{
		private const bool useHybridCrack = true;
		private static string OutputDirectory 
		{ 
			get
			{
				string dir = @"C:\Users\Serafeim\Desktop\xfem 3d\plots\edge_crack_2D";
				if (useHybridCrack)
				{
					return dir + "_hybrid";
				}
				else
				{
					return dir;
				}
			}
		}
			 
		private const int subdomainID = 0;

		private static readonly double[] minCoords = new double[] { 0, 0 };
		private static readonly double[] maxCoords = new double[] { 4, 4 }; // in
		private static readonly int[] numElements = new int[] { 48, 48 }; // in
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

		[Fact]
		public static void RunExample()
		{
			XModel<IXCrackElement> model = CreatePhysicalModel();
			CreateGeometryModel(model);
			SetupEnrichmentOutput(model);
			RunAnalysis(model);
			WriteCrackPath(model);
		}

		public static void CreateGeometryModel(XModel<IXCrackElement> model)
		{
			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(
				geometryModel, new RelativeAreaSingularityResolver(heavisideTol), tipEnrichmentArea);

			//TODO: This is probably better, but my expected results are from the next one
			//var jIntegrationRule = new JintegrationStrategy(
			//    GaussLegendre2D.GetQuadratureWithOrder(4, 4),
			//    new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(4, 4)));
			var jIntegrationRule = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(4, 4));
			var propagator = new JintegralPropagator2D(model, jIntegralRadiusRatio, jIntegrationRule, Material,
				new MaximumCircumferentialTensileStressCriterion(), new ConstantIncrement2D(growthLength));

			double crackHeight = /*0.40*/ 0.45 * (maxCoords[1] - minCoords[1]);
			var point0 = new double[] { minCoords[0], crackHeight };
			var point1 = new double[] { minCoords[0] + a, crackHeight };
			var initialGeom = new PolyLine2D(point0, point1);

			if (!useHybridCrack)
			{
				ExteriorLsmCrack2D crack = new ExteriorLsmCrack2D(0, initialGeom, model, propagator);
				var outputMesh = new ContinuousOutputMesh(model.Nodes.Values.OrderBy(n => n.ID).ToList(), model.EnumerateElements());
				crack.Observers.Add(new CrackLevelSetPlotter(crack, outputMesh, OutputDirectory));
				crack.Observers.Add(new CrackInteractingElementsPlotter(crack, OutputDirectory));
				geometryModel.Cracks[crack.ID] = crack;
			}
			else
			{
				TempEdgeCrack2D crack = CreateCrackHybrid(model, propagator, initialGeom);
				crack.Observers.Add(new CrackLevelSetPlotter_v2(model, crack.hybridGeometry, OutputDirectory));
				crack.Observers.Add(new CrackInteractingElementsPlotter(crack, OutputDirectory));
				geometryModel.Cracks[crack.ID] = crack;
			}
		}

		public static TempEdgeCrack2D CreateCrackHybrid(XModel<IXCrackElement> model, JintegralPropagator2D propagator,
			PolyLine2D initialGeom)
		{
			double domainDimension = maxCoords[0] - minCoords[0];
			var crackVertices = new List<Vertex2D>();
			for (int i = 0; i < initialGeom.Vertices.Count; ++i)
			{
				double[] point = initialGeom.Vertices[i];
				crackVertices.Add(new Vertex2D(i, point));
			}
			var hybridGeometry = new CrackCurve2D(0, domainDimension, crackVertices);
			hybridGeometry.CrackFront = new CrackFront2D(hybridGeometry, new RectangularDomainBoundary(minCoords, maxCoords));

			var crack = new TempEdgeCrack2D(0, initialGeom, hybridGeometry, model, propagator);
			return crack;
		}

		public static XModel<IXCrackElement> CreatePhysicalModel()
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);
			model.FindConformingSubcells = true;

			// Materials, integration
			var material = new HomogeneousFractureMaterialField2D(E, v, thickness, planeStress);
			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(16, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory2D(material, thickness, bulkIntegration);

			// Mesh
			var mesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			Utilities.Models.AddNodesElements(model, mesh, factory);

			// Boundary conditions
			double tol = 1E-6;
			XNode topLeft = model.Nodes.Values.Where(n => Math.Abs(n.X - minCoords[0]) <= tol && Math.Abs(n.Y - maxCoords[1]) <= tol).First();
			XNode bottomLeft = model.Nodes.Values.Where(n => Math.Abs(n.X - minCoords[0]) <= tol && Math.Abs(n.Y - minCoords[1]) <= tol).First();
			model.NodalLoads.Add(new Load() { Node = topLeft, DOF = StructuralDof.TranslationY, Amount = +load });
			model.NodalLoads.Add(new Load() { Node = bottomLeft, DOF = StructuralDof.TranslationY, Amount = -load });
			foreach (XNode node in model.Nodes.Values.Where(n => Math.Abs(n.X - maxCoords[0]) <= tol))
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
			}

			return model;
		}

		public static void RunAnalysis(XModel<IXCrackElement> model)
		{
			// Solver
			var factory = new SkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);

			var domainBoundary = new RectangularDomainBoundary(minCoords, maxCoords);
			var termination = new TerminationLogic.Or(
				new FractureToughnessTermination(fractureToughness),
				new CrackExitsDomainTermination(domainBoundary));
			var analyzer = new QuasiStaticLefmAnalyzer(model, algebraicModel, solver, maxIterations, termination);
			analyzer.Results.Add(new DisplacementFieldWriter(model, OutputDirectory));
			analyzer.Results.Add(new StrainStressFieldWriter(model, OutputDirectory));

			analyzer.Analyze();
		}

		public static void SetupEnrichmentOutput(XModel<IXCrackElement> model)
		{
			// Enrichments
			var allCrackStepNodes = new AllCrackStepNodesObserver();
			var newCrackTipNodes = new NewCrackTipNodesObserver();
			var enrichmentPlotter = new CrackEnrichmentPlotterBasic(OutputDirectory, newCrackTipNodes, allCrackStepNodes);

			var compositeObserver = new CompositeEnrichmentObserver();
			compositeObserver.AddObservers(allCrackStepNodes, newCrackTipNodes, enrichmentPlotter);
			model.GeometryModel.Enricher.Observers.Add(compositeObserver);
		}

		public static void WriteCrackPath(XModel<IXCrackElement> model)
		{
			IReadOnlyList<double[]> crackPath = null;
			IXDiscontinuity discontinuity = model.GeometryModel.GetDiscontinuity(0);
			if (discontinuity is ExteriorLsmCrack2D lsmCrack)
			{
				crackPath = lsmCrack.CrackPath;
			}
			else if (discontinuity is TempEdgeCrack2D hybridCrack)
			{
				crackPath = hybridCrack.CrackPath;
			}

			Debug.WriteLine("Crack path:");
			for (int i = 0; i < crackPath.Count; ++i)
			{
				Debug.WriteLine($"{crackPath[i][0]} \t {crackPath[i][1]}");
			}
		}
	}
}
