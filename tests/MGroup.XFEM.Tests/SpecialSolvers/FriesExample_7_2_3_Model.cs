using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Meshes.Manifolds;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.FriesPropagation;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.Observers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Boundaries;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Output.FriesHybridCrack;
using MGroup.XFEM.Output.Writers;

namespace MGroup.XFEM.Tests.SpecialSolvers.HybridFries
{
	public static class FriesExample_7_2_3_Model
	{
		public static readonly double[] minCoords = new double[] { 0, 0, 0 };
		public static readonly double[] maxCoords = new double[] { 200, 100, 200 };

		private const double E = 3E7, v = 0.3;
		public static double crackLength = 49, crackRadius = 25;
		public static double uPrescribed = crackLength, radiusPrescribed = crackRadius;

		public static double da = 5, rc = 1.0;
		private const int numTrialPoints = 100;
		private const double zeroStresRThetaTolerance = 5E-2;

		public static double heavisideTol = 1E-4;
		public static double tipEnrichmentArea = 0.0;

		public static ICrack CreateFullCrack(XModel<IXCrackElement> model, int maxIterations)
		{
			double offset = 0.25 * crackLength;
			double[] cylinderStart = {
				0.5 * (minCoords[0] + maxCoords[0]), minCoords[1] - offset, 0.5 * (minCoords[2] + maxCoords[2])
			};
			TriangleMesh3D initialCrackMesh = CreateCrackGeometry(cylinderStart, crackLength + offset, crackRadius, 10, 2);

			var crackGeometry = CrackSurface3D.CreateFromMesh(0, maxCoords[0] - minCoords[0], initialCrackMesh);
			var domainBoundary = new BoxDomainBoundary3D(minCoords, maxCoords, 1E-6);
			crackGeometry.CrackFront = new CrackFront3D(crackGeometry, domainBoundary);
			crackGeometry.InitializeGeometry(model.Nodes.Values);

			// Expected crack path
			for (int t = 0; t < maxIterations; ++t)
			{
				ICrackFront3D crackFront = crackGeometry.CrackFront;
				int numTips = crackFront.ActiveTips.Count;
				double growthAngle = +(70.0 / 180.0) * Math.PI;
				var frontGrowth = new CrackFrontPropagation();
				frontGrowth.AnglesAtTips = new double[numTips];
				frontGrowth.LengthsAtTips = new double[numTips];
				for (int i = 0; i < numTips; ++i)
				{
					frontGrowth.AnglesAtTips[i] = (t == 0) ? growthAngle : 0;
					frontGrowth.LengthsAtTips[i] = da;
				}
				crackGeometry.PropagateCrack(model.Nodes.Values, frontGrowth);
			}

			return new HybridFriesCrack3D(model, crackGeometry, null);
		}

		public static void CreateGeometryModel(XModel<IXCrackElement> model, int[] numElements,
			string outputDirectory = null)
		{
			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(
				geometryModel, new RelativeAreaSingularityResolver(heavisideTol), tipEnrichmentArea);

			double offset = /*0.25*/ 0.05 * crackLength;
			double[] cylinderStart = {
				0.5 * (minCoords[0] + maxCoords[0]), minCoords[1] - offset, 0.5 * (minCoords[2] + maxCoords[2])
			};
			TriangleMesh3D initialCrackMesh = CreateCrackGeometry(cylinderStart, crackLength + offset, crackRadius, 10, 2);

			IPropagator propagator = ChooseCrackPropagator(model, numElements);
			var crackGeometry = CrackSurface3D.CreateFromMesh(0, maxCoords[0] - minCoords[0], initialCrackMesh);
			var domainBoundary = new BoxDomainBoundary3D(minCoords, maxCoords, 1E-6);
			crackGeometry.CrackFront = new CrackFront3D(crackGeometry, domainBoundary);
			var crack = new HybridFriesCrack3D(model, crackGeometry, propagator);
			geometryModel.Cracks[crack.ID] = crack;

			if (outputDirectory != null)
			{
				//crack.Observers.Add(new LevelSetObserver(model, crack.CrackGeometry_v2, outputDirectory));
				crack.Observers.Add(new CrackLevelSetPlotter_v2(model, crack.CrackGeometry_v2, outputDirectory));
				//crack.Observers.Add(new CrackInteractingElementsPlotter(crack, outputDirectory));
				crack.Observers.Add(new CrackBody3DObserver(crack.CrackGeometry_v2, outputDirectory));
				//crack.Observers.Add(new CrackFront3DObserver(crack.CrackGeometry_v2, outputDirectory));
			}
		}

		public static UniformDdmCrackModelBuilder3D DescribePhysicalModel(
			int[] numElements, int[] numSubdomains = null, int[] numClusters = null, int[][] numElementsPerSubdomain = null)
		{
			if (numSubdomains == null)
			{
				numSubdomains = new int[] { 1, 1 };
				numClusters = new int[] { 1, 1 };
			}

			var modelBuilder = new UniformDdmCrackModelBuilder3D();
			modelBuilder.MinCoords = minCoords;
			modelBuilder.MaxCoords = maxCoords;

			modelBuilder.NumElementsTotal = numElements;
			modelBuilder.NumSubdomains = numSubdomains;
			modelBuilder.NumClusters = numClusters;
			modelBuilder.NumElementsPerSubdomainPerAxis = numElementsPerSubdomain;

			// Materials, integration
			modelBuilder.MaterialField = new HomogeneousFractureMaterialField3D(E, v);
			var enrichedIntegration = new IntegrationWithNonconformingHexa3D(8, GaussLegendre3D.GetQuadratureWithOrder(2, 2, 2));
			modelBuilder.BulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);

			modelBuilder.FindConformingSubcells = true;
			modelBuilder.ApplyBoundaryConditions = ApplyBoundaryConditions;

			return modelBuilder;
		}

		private static void ApplyBoundaryConditions(XModel<IXCrackElement> model)
		{
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
		}

		private static IPropagator ChooseCrackPropagator(XModel<IXCrackElement> model, int[] numElements)
		{
			bool useFixedPropagator = true;
			if (useFixedPropagator)
			{
				return new MockPropagator();
			}
			else
			{
				var growthAngleCriterion = new DefaultGrowthAngleCriterion(zeroStresRThetaTolerance);
				var mesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElements).BuildMesh();
				var propagator = new FriesPropagator(model, mesh, growthAngleCriterion, da, rc, numTrialPoints);
				//propagator.plotter = new TrialPointsPlotter(outputDirectory);
				return propagator;
			}
		}

		/// <summary>
		/// The normals of the triangles will point outside the cylinder.
		/// </summary>
		private static TriangleMesh3D CreateCrackGeometry(double[] axisStart, double axisLength, double radius,
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
				int vC = mesh.Vertices.Count - 1;
				mesh.Cells.Add(new int[] { vA, vB, vC });
			}

			return mesh;
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

		public static void SetupEnrichmentOutput(XModel<IXCrackElement> model, string outputDirectory)
		{
			//model.ModelObservers.Add(new StructuralBoundaryConditionsPlotter(outputDirectory, model));

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

			public (double[] growthAngles, double[] growthLengths) PropagateNoNoise(
				IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements, ICrackTipSystem[] crackTipSystems)
			{
				double growthAngle = +(70.0 / 180.0) * Math.PI;
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

			public (double[] growthAngles, double[] growthLengths) Propagate(
				IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements, ICrackTipSystem[] crackTipSystems)
			{
				//double growthAngle = +(70.0 / 180.0) * Math.PI;

				var growthAngles = new double[crackTipSystems.Length];
				var growthLengths = new double[crackTipSystems.Length];
				double theta0;
				for (int i = 0; i < crackTipSystems.Length; ++i)
				{
					if (iteration == 0)
					{
						theta0 = +(55.0 / 180.0) * Math.PI;
					}
					else if (iteration == 1)
					{
						theta0 = +(16 / 180.0) * Math.PI;
					}
					else if (iteration >= 10)
					{
						theta0 = +(2 / 180.0) * Math.PI;
					}
					else
					{
						theta0 = 0;
					}

					if (i == 0 || i == 5)
					{
						growthAngles[i] = 1.12 * theta0;
					}
					else if (i == 1 || i == 4 || i == 6 || i == 9)
					{
						growthAngles[i] = 0.95 * theta0;
					}
					else
					{
						growthAngles[i] = 1.0 * theta0;
					}
					growthLengths[i] = da;
				}


				++iteration;
				return (growthAngles, growthLengths);
			}
		}
	}
}
