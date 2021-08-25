using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Meshes.Structured;
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
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Boundaries;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using Xunit;

namespace MGroup.XFEM.Tests.Fracture.Benchmarks
{
	public static class DcbBelytschkoBenchmark
	{
		private static readonly double[] minCoords = new double[] { 0, 0 };
		private static readonly double[] maxCoords = new double[] { 3 * 3.94, 3.94 }; // in
		private const double thickness = 1.0;
		private const double E = 3E7; // psi=lbs/in^2
		private const double v = 0.3;
		private const bool planeStress = true;
		private const double load = 197; // lbs
		private const double a = 3.95, da = 0.5, growthLength = 0.3; // in 
		private const double dTheta = 5.71 * Math.PI / 180; // initial crack angle
		private const int subdomainID = 0;

		private const double jIntegralRadiusRatio = 2.0;
		private const double heavisideTol = 1E-4;
		private const double tipEnrichmentArea = 0.0;
		private const int maxIterations = 7;
		private const double fractureToughness = double.MaxValue;

		[Fact]
		public static void TestCrackPropagationPath()
		{
			int[] numElements = { 60, 20 };
			XModel<IXCrackElement> model = CreateModel(numElements);
			RunAnalysis(model);
			var crack = (ExteriorLsmCrack2D)model.GeometryModel.GetDiscontinuity(0);

			// Check propagation path
			var expectedPath = new List<double[]>();
			expectedPath.Add(new double[] { 0, 1.97 });
			expectedPath.Add(new double[] { 3.95, 1.97 });
			expectedPath.Add(new double[] { 4.44751911011965, 1.9202532909053 } );
			expectedPath.Add(new double[] { 4.7448405085728, 1.88025346563242 });
			expectedPath.Add(new double[] { 5.03813895313486, 1.81719775679113 });
			expectedPath.Add(new double[] { 5.31951810803721, 1.7131507905382 } );
			expectedPath.Add(new double[] { 5.57293125400903, 1.55258374084037 });
			expectedPath.Add(new double[] { 5.77034730849673, 1.32669239211158 });
			expectedPath.Add(new double[] { 5.89862696520051, 1.05550181396411 });
			//expectedPath.Add(new double[] { 5.97090805835735, 0.764339585005661 }); // The model is not analyzed for this propagation step will not be 

			Assert.Equal(expectedPath.Count, crack.CrackPath.Count);
			int precision = 10;
			for (int i = 0; i < expectedPath.Count; ++i)
			{
				Assert.Equal(expectedPath[i][0], crack.CrackPath[i][0], precision);
				Assert.Equal(expectedPath[i][1], crack.CrackPath[i][1], precision);
			}
		}

		private static void ApplyBoundaryConditions(XModel<IXCrackElement> model)
		{
			// Boundary conditions
			double tol = 1E-6;
			double L = maxCoords[0];
			double H = maxCoords[1];
			XNode topLeft = model.Nodes.Values.Where(n => Math.Abs(n.X) <= tol && Math.Abs(n.Y - H) <= tol).First();
			XNode bottomLeft = model.Nodes.Values.Where(n => Math.Abs(n.X) <= tol && Math.Abs(n.Y) <= tol).First();
			model.NodalLoads.Add(new Load() { Node = topLeft, DOF = StructuralDof.TranslationY, Amount = +load });
			model.NodalLoads.Add(new Load() { Node = bottomLeft, DOF = StructuralDof.TranslationY, Amount = -load });
			foreach (XNode node in model.Nodes.Values.Where(n => Math.Abs(n.X - L) <= tol))
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
			}
		}

		private static XModel<IXCrackElement> CreateModel(int[] numElements)
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);
			model.FindConformingSubcells = true;

			// Materials, integration
			var material = new HomogeneousFractureMaterialField2D(E, v, thickness, planeStress);
			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory2D(material, thickness, bulkIntegration);

			// Mesh
			var mesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			Utilities.Models.AddNodesElements(model, mesh, factory);

			ApplyBoundaryConditions(model);

			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(
				geometryModel, new RelativeAreaSingularityResolver(heavisideTol), tipEnrichmentArea);
			var point0 = new double[] { minCoords[0], 0.5 * (maxCoords[1] - minCoords[1]) };
			var point1 = new double[] { minCoords[0] + a, 0.5 * (maxCoords[1] - minCoords[1]) };
			var initialGeom = new PolyLine2D(point0, point1);
			initialGeom.UpdateGeometry(-dTheta, da);

			//TODO: This is probably better, but my expected results are from the next one
			//var jIntegrationRule = new JintegrationStrategy(
			//    GaussLegendre2D.GetQuadratureWithOrder(4, 4),
			//    new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(4, 4)));
			var jIntegrationRule = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(4, 4));
			var propagator = new JintegralPropagator2D(model, jIntegralRadiusRatio, jIntegrationRule, material,
				new MaximumCircumferentialTensileStressCriterion(), new ConstantIncrement2D(growthLength));
			var crack = new ExteriorLsmCrack2D(0, initialGeom, model, propagator);
			geometryModel.Cracks[crack.ID] = crack;

			return model;
		}

		private static void RunAnalysis(XModel<IXCrackElement> model)
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

			analyzer.Analyze();
		}
	}
}
