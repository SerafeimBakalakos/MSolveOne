using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Meshes.Manifolds;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.Observers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Boundaries;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Output.FriesHybridCrack;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Tests.Fracture;
using MGroup.XFEM.Tests.Utilities;
using Xunit;

namespace MGroup.XFEM.Tests.Geometry.FriesHybridCrack
{
	public class CircleCrack3DTests
	{
		private static readonly string outputDirectory = @"C:\Users\Serafeim\Desktop\xfem 3d\paper\Theory\XFEM simple crack";

		private static readonly double[] minCoords = { -1.0, -1.0, -1.0 };
		private static readonly double[] maxCoords = { +1.0, +1.0, +1.0 };
		private static readonly int[] numElements = { 29, 29, 29 };
		private const int subdomainID = 0;
		private const double E = 1, v = 0.3;
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;

		private const double tipEnrichmentArea = 0; //0.15 for some plots
		private const int numPointsBoundary = 60;
		private const double radius = 0.25, growthAngle = Math.PI / 12.0, growthLength = 0.1;

		[Fact]
		public static void TestModel()
		{
			if (!Directory.Exists(outputDirectory))
			{
				Directory.CreateDirectory(outputDirectory);
			}

			// Create model and LSM
			XModel<IXCrackElement> model = CreateModel();

			var crack = (HybridFriesCrack3D)model.GeometryModel.GetDiscontinuity(0);
			CrackSurface3D crackGeometry = crack.CrackGeometry_v2;

			// Plot the FE mesh
			var outputMesh = new ContinuousOutputMesh(model.Nodes.Values, model.Elements.Values);
			string meshPath = outputDirectory + "\\mesh.vtk";
			using (var writer = new VtkFileWriter(meshPath))
			{
				writer.WriteMesh(outputMesh);
			}

			// Crack geometry observers
			crack.Observers.Add(new CrackBody3DObserver(crackGeometry, outputDirectory));
			crack.Observers.Add(new CrackBodyNormals3DObserver(crackGeometry, outputDirectory, true));
			crack.Observers.Add(new CrackFront3DObserver(crackGeometry, outputDirectory));
			crack.Observers.Add(new CrackExtension3DObserver(crackGeometry, outputDirectory));
			crack.Observers.Add(new CrackExtensionNormals3DObserver(crackGeometry, outputDirectory, true));
			//crack.Observers.Add(new CrackPathPlotter(crack, outputDirectory));
			crack.Observers.Add(new CrackInteractingElementsPlotter(crack, outputDirectory));

			// Level set observers
			crack.Observers.Add(new LevelSetObserver(model, crackGeometry, outputDirectory));
			crack.Observers.Add(new CrackLevelSetPlotter_v2(model, crackGeometry, outputDirectory));

			// Enrichment observers
			var allCrackStepNodes = new AllCrackStepNodesObserver();
			//var newCrackStepNodes = new NewCrackStepNodesObserver();
			var newCrackTipNodes = new NewCrackTipNodesObserver();
			//var previousCrackTipNodes = new PreviousCrackTipNodesObserver();
			//var rejectedCrackStepNodes = new RejectedCrackStepNodesObserver(
			//	crack, newCrackTipNodes, allCrackStepNodes);
			//var nodesWithModifiedLevelSet = new CrackStepNodesWithModifiedLevelSetObserver(crack);
			//var nodesWithModifiedEnrichments = new NodesWithModifiedEnrichmentsObserver(nodesWithModifiedLevelSet);
			//var elementsWithModifiedNodes = new ElementsWithModifiedNodesObserver(nodesWithModifiedEnrichments);
			//var nodesNearModifiedNodes = new NodesNearModifiedNodesObserver(
			//nodesWithModifiedEnrichments, elementsWithModifiedNodes);
			var enrichmentPlotter = new CrackEnrichmentPlotterBasic(outputDirectory, newCrackTipNodes, allCrackStepNodes);

			var compositeObserver = new CompositeEnrichmentObserver();
			compositeObserver.AddObservers(allCrackStepNodes, newCrackTipNodes, enrichmentPlotter);
			model.GeometryModel.Enricher.Observers.Add(compositeObserver);

			// Plot element subcells
			crack.Observers.Add(new CrackInteractingElementsPlotter(crack, outputDirectory));
			model.ModelObservers.Add(new ConformingMeshPlotter(outputDirectory, model));

			//// Plot bulk and boundary integration points of each element
			//model.ModelObservers.Add(new IntegrationPointsPlotter(outputDirectory, model));

			model.Initialize();
		}


		private static XModel<IXCrackElement> CreateModel()
		{
			var model = new XModel<IXCrackElement>(3);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);
			model.FindConformingSubcells = true;

			// Materials, integration
			var material = new HomogeneousFractureMaterialField3D(E, v);
			var enrichedIntegration = new IntegrationWithConformingSubtetrahedra3D(TetrahedronQuadrature.Order2Points4);
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory3D(material, bulkIntegration);

			// Mesh
			var mesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			Utilities.Models.AddNodesElements(model, mesh, factory);
			//ApplyBoundaryConditions(model);

			// Crack, enrichments
			model.GeometryModel = CreateGeometryModel(model);

			return model;
		}

		private static CrackGeometryModel CreateGeometryModel(XModel<IXCrackElement> model)
		{
			var geometryModel = new CrackGeometryModel(model);
			geometryModel.Enricher = new NodeEnricherIndependentCracks(
				geometryModel, new NullSingularityResolver(), tipEnrichmentArea, false);
			//geometryModel.Enricher = new NullEnricher();

			CrackSurface3D crackGeometry = CreateCircleCrack();
			var crack = new HybridFriesCrack3D(model, crackGeometry, new MockPropagator());
			geometryModel.Cracks[crack.ID] = crack;

			return geometryModel;
		}

		private static CrackSurface3D CreateCircleCrack()
		{
			// Plane xy, center = 0
			double dTheta = 2.0 * Math.PI / numPointsBoundary;

			var vertices = new List<Vertex3D>();
			Vertex3D center = new Vertex3D(0, new double[] { 0, 0, 0 });
			vertices.Add(center);

			// Boundary vertices
			var boundaryVertices = new Vertex3D[numPointsBoundary];
			for (int v = 0; v < numPointsBoundary; ++v)
			{
				double theta = v * dTheta;
				double[] coords = { radius * Math.Cos(theta), radius * Math.Sin(theta), 0.0 };
				Vertex3D vertex = new Vertex3D(v + 1, coords);
				vertices.Add(vertex);
				boundaryVertices[v] = vertex;
			}

			// Divide each triangles of boundary vertices and center into 3 more
			var cells = new List<TriangleCell3D>();
			for (int v = 0; v < numPointsBoundary; ++v)
			{
				Vertex3D current = boundaryVertices[v];
				Vertex3D next = boundaryVertices[(v + 1) % numPointsBoundary];

				// Find barycenter of this triangle
				var barycenter = new Vertex3D(vertices.Count, new double[] { 0, 0, 0 });
				for (int d = 0; d < 3; ++d)
				{
					barycenter.CoordsGlobal[d] += center.CoordsGlobal[d];
					barycenter.CoordsGlobal[d] += current.CoordsGlobal[d];
					barycenter.CoordsGlobal[d] += next.CoordsGlobal[d];
					barycenter.CoordsGlobal[d] /= 3;
				}
				vertices.Add(barycenter);

				// Create 3 new triangles. The order of vertices should produce a positive normal towards the half-space z > 0.
				//               x (center)
				//              /|\
				//             / | \
				//            /  x  \ (barycenter)
				//           / /   \ \
				//          x---------x    
				//  (current)         (next)
				cells.Add(new TriangleCell3D(center, current, barycenter));
				cells.Add(new TriangleCell3D(current, next, barycenter));
				cells.Add(new TriangleCell3D(next, center, barycenter));
			}

			double domainDimension = maxCoords[0] - minCoords[0];
			var crack = new CrackSurface3D(0, domainDimension, vertices, cells/*, true*/);
			var domainBoundary = new BoxDomainBoundary3D(minCoords, maxCoords);
			crack.CrackFront = new CrackFront3D(crack, domainBoundary);

			return crack;
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
