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
	public static class FriesExample_7_2_3
	{
		private static string outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\plots\example_7_2_3";
		private const int subdomainID = 0;

		private static readonly double[] minCoords = new double[] { 0, 0, 0 };
		private static readonly double[] maxCoords = new double[] { 200, 100, 200 };
		private static readonly int[] numElements = new int[] { 20, 10, 20 };

		private const double E = 3E7, v = 0.3;
		private const double crackLength = 49, crackRadius = 25;
		private const double uPrescribed = crackLength, radiusPrescribed = crackRadius;

		private const double da = 5, rc = 1.0;
		private const int numTrialPoints = 100;
		private const double zeroStresRThetaTolerance = 5E-2;

		private const bool useFixedPropagator = true;
		private const double growthAngle = +(70.0 / 180.0) * Math.PI;
		
		private const double heavisideTol = 1E-4;
		private const double tipEnrichmentArea = 0.0;
		private const int maxIterations = 17;
		private const double fractureToughness = double.MaxValue;

		private static HomogeneousFractureMaterialField3D Material
			=> new HomogeneousFractureMaterialField3D(E, v);

		[Fact]
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
				var propagator = new FriesPropagator(model, mesh, growthAngleCriterion, da, rc, numTrialPoints);
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

			double offset = 0.25 * crackLength;
			double[] cylinderStart = { 
				0.5 * (minCoords[0] + maxCoords[0]), minCoords[1] - offset, 0.5 * (minCoords[2] + maxCoords[2]) 
			};
			TriangleMesh3D initialCrackMesh = CreateCrackGeometry(cylinderStart, crackLength + offset, crackRadius, 10, 2);

			IPropagator propagator = ChooseCrackPropagator(model);
			var crackGeometry = CrackSurface3D.CreateFromMesh(0, maxCoords[0] - minCoords[0], initialCrackMesh);
			var domainBoundary = new BoxDomainBoundary3D(minCoords, maxCoords, 1E-6);
			crackGeometry.CrackFront = new CrackFront3D(crackGeometry, domainBoundary);
			var crack = new HybridFriesCrack3D(model, crackGeometry, propagator);

			//crack.Observers.Add(new LevelSetObserver(model, crack.CrackGeometry_v2, outputDirectory));
			crack.Observers.Add(new CrackLevelSetPlotter_v2(model, crack.CrackGeometry_v2, outputDirectory));
			crack.Observers.Add(new CrackInteractingElementsPlotter(crack, outputDirectory));
			crack.Observers.Add(new CrackBody3DObserver(crack.CrackGeometry_v2, outputDirectory));
			crack.Observers.Add(new CrackFront3DObserver(crack.CrackGeometry_v2, outputDirectory));
			geometryModel.Cracks[crack.ID] = crack;
		}

		/// <summary>
		/// The normals of the triangles will point outside the cylinder.
		/// </summary>
		public static TriangleMesh3D CreateCrackGeometry(double[] axisStart, double axisLength, double radius,
			int numVerticesAlongCircle, int numVerticesAlongAxis)
		{
			if (numVerticesAlongAxis != 2)
			{
				throw new NotImplementedException();
			}
			Debug.Assert(numVerticesAlongCircle >= 3);
			Debug.Assert(numVerticesAlongAxis >= 2);

			// Vertices
			var mesh = new TriangleMesh3D();
			double dy = axisLength / (numVerticesAlongAxis - 1);
			double dTheta = 2.0 * Math.PI / numVerticesAlongCircle;
			for (int i = 0; i < numVerticesAlongAxis; ++i)
			{
				for (int j = 0; j < numVerticesAlongCircle; ++j)
				{
					double x = axisStart[0] + radius * Math.Cos(j * dTheta);
					double y = axisStart[1] + i * dy;
					double z = axisStart[2] + radius * Math.Sin(j * dTheta);
					mesh.Vertices.Add(new double[] { x, y, z }); // index = i * numVerticesAlongCircle + j
				}
			}

			// Cells
			for (int i = 0; i < numVerticesAlongAxis - 1; ++i)
			{
				for (int j = 0; j < numVerticesAlongCircle; ++j)
				{
					// Indices of vertices belong to the 2 triangles
					int vA = i * numVerticesAlongCircle + j;
					int vB = (i + 1) * numVerticesAlongCircle + j;
					int vC = (i + 1) * numVerticesAlongCircle + (j + 1) % numVerticesAlongCircle;
					int vD = i * numVerticesAlongCircle + (j + 1) % numVerticesAlongCircle;

					mesh.Cells.Add(new int[] { vA, vB, vC });
					mesh.Cells.Add(new int[] { vA, vC, vD });
				}
			}

			// Cover one base of the cylinder
			mesh.Vertices.Add(new double[] { axisStart[0], axisStart[1], axisStart[2] });
			for (int j = 0; j < numVerticesAlongCircle; ++j)
			{
				// Indices of vertices belong to the 2 triangles
				int vA = j;
				int vB = (j + 1) % numVerticesAlongCircle;
				int vC = mesh.Vertices.Count -1;
				mesh.Cells.Add(new int[] { vA, vB, vC });
			}

			return mesh;
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
			var supportedNodes = new List<XNode>();
			supportedNodes.Add(FindNodeAt(new double[] { minCoords[0], maxCoords[1], minCoords[2] }, model));
			supportedNodes.Add(FindNodeAt(new double[] { maxCoords[0], maxCoords[1], minCoords[2] }, model));
			supportedNodes.Add(FindNodeAt(new double[] { minCoords[0], maxCoords[1], maxCoords[2] }, model));
			supportedNodes.Add(FindNodeAt(new double[] { maxCoords[0], maxCoords[1], maxCoords[2] }, model));
			foreach (XNode node in supportedNodes)
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0.0 });
			}

			var impactedNodes = new List<XNode>();
			var xCenter = 0.5 * (minCoords[0] + maxCoords[0]); 
			var zCenter = 0.5 * (minCoords[2] + maxCoords[2]);
			double r = radiusPrescribed;
			foreach (XNode node in model.Nodes.Values)
			{
				if (node.Y == minCoords[1])
				{
					double dx = node.X - xCenter;
					double dz = node.Z - zCenter;
					double distance = Math.Sqrt(dx * dx + dz * dz);
					if (distance < r)
					{
						impactedNodes.Add(node);
					}
				}
			}
			foreach (XNode node in impactedNodes)
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = uPrescribed });
			}

			return model;
		}

		private static XNode FindNodeAt(double[] x, XModel<IXCrackElement> model)
		{
			double[] dx = { maxCoords[0] - minCoords[0], maxCoords[1] - minCoords[1], maxCoords[2] - minCoords[2] };
			double tol = 1E-6 * Math.Min(dx[0], Math.Min(dx[1], dx[2]));
			var targetNodes = model.Nodes.Values.Where(n =>
			{
				return (Math.Abs(n.X - x[0]) < tol) && (Math.Abs(n.Y - x[1]) < tol) && (Math.Abs(n.Z - x[2]) < tol);
			}).ToArray();
			if (targetNodes.Length != 1)
			{
				throw new Exception("There should be exactly 1 node that fits these criteria");
			}
			return targetNodes[0];
		}

		private static void RunAnalysis(XModel<IXCrackElement> model)
		{
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
			analyzer.Results.Add(new StructuralFieldWriter(model, outputDirectory, true, false));

			analyzer.Analyze();
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
