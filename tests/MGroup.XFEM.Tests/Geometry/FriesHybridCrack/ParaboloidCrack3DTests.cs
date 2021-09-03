using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
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
	public class ParaboloidCrack3DTests
	{
		private static readonly string outputDirectory = Path.Combine(
			Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "paraboloid_crack_3D_geometry_temp");
		private static readonly string expectedDirectory = Path.Combine(
			Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "paraboloid_crack_3D_geometry");

		private static readonly double[] minCoords = { -1.0, -1.0, -1.0 };
		private static readonly double[] maxCoords = { +1.0, +1.0, +1.0 };
		private static readonly int[] numElements = { 23, 23, 23 };
		private const int subdomainID = 0;
		private const double E = 1, v = 0.3;
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;

		private const double tipEnrichmentArea = 0.0;
		private const int numPointsBoundary = 8;
		private const double radius = 0.2, growthAngle = Math.PI / 12.0, growthLength = 0.1;
		private const int numGrowthSteps = 7;

		[Fact]
		public static void TestModel()
		{
			try
			{
				if (!Directory.Exists(outputDirectory))
				{
					Directory.CreateDirectory(outputDirectory);
				}

				// Create model and LSM
				XModel<IXCrackElement> model = CreateModel();

				var crack = (HybridFriesCrack3D)model.GeometryModel.GetDiscontinuity(0);
				CrackSurface3D crackGeometry = crack.CrackSurface;

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

				// Level set observers
				crack.Observers.Add(new LevelSetObserver(model, crackGeometry, outputDirectory));
				//crack.Observers.Add(new CrackLevelSetPlotter(crack, outputMesh, outputDirectory));
				crack.Observers.Add(new CrackInteractingElementsPlotter(crack, outputDirectory));

				//var newTipNodes = new NewCrackTipNodesObserver(crack);
				//model.RegisterEnrichmentObserver(newTipNodes);
				//var allBodyNodes = new CrackBodyNodesObserver(crack);
				//model.RegisterEnrichmentObserver(allBodyNodes);
				//var rejectedBodyNodes = new RejectedCrackBodyNodesObserver(crack, newTipNodes, allBodyNodes);
				//model.RegisterEnrichmentObserver(rejectedBodyNodes);

				//var enrichmentPlotter = new CrackEnrichmentPlotter(crack, outputDir, newTipNodes, previousTipNodes, allBodyNodes,
				//    newBodyNodes, rejectedBodyNodes, nearModifiedNodes);
				//model.RegisterEnrichmentObserver(enrichmentPlotter);


				//// Plot element - phase boundaries interactions
				//model.ModelObservers.Add(new LsmElementIntersectionsPlotter(outputDirectory, model));

				// Plot element subcells
				//model.ModelObservers.Add(new ConformingMeshPlotter(outputDirectory, model, false));
				//model.ModelObservers.Add(new ConformingMeshPlotter(outputDirectory, model, true));

				//// Plot bulk and boundary integration points of each element
				//model.ModelObservers.Add(new IntegrationPointsPlotter(outputDirectory, model));

				// Propagate the crack. During this the observers will plot the data they pull from model. 
				for (int t = 0; t < numGrowthSteps; ++t)
				{
					if (t == 0)
					{
						model.Initialize();
					}
					else
					{
						model.Update(null, null);
					}

					CheckOutputFiles(t);
				}
			}
			finally
			{
				if (Directory.Exists(outputDirectory))
				{
					DirectoryInfo di = new DirectoryInfo(outputDirectory);
					di.Delete(true);//true means delete subdirectories and files
				}
			}
		}

		private static void CheckOutputFiles(int t)
		{
			// Compare output
			var computedFiles = new List<string>();
			computedFiles.Add(Path.Combine(outputDirectory, "mesh.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"crack_surface_0_t{t}.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"crack_surface_normals_cells_0_t{t}.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"crack_surface_normals_vertices_0_t{t}.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"crack_extension_0_t{t}.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"crack_extension_normals_cells_0_t{t}.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"crack_extension_normals_vertices_0_t{t}.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"crack_front_systems_0_t{t}.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"level_sets_0_t{t}.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"tip_elements_0_t{t}.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, $"intersected_elements_0_t{t}.vtk"));
			//computedFiles.Add(Path.Combine(outputDirectory, $"conforming_elements_0_t{t}.vtk"));

			var expectedFiles = new List<string>();
			expectedFiles.Add(Path.Combine(expectedDirectory, "mesh.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"crack_surface_0_t{t}.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"crack_surface_normals_cells_0_t{t}.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"crack_surface_normals_vertices_0_t{t}.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"crack_extension_0_t{t}.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"crack_extension_normals_cells_0_t{t}.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"crack_extension_normals_vertices_0_t{t}.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"crack_front_systems_0_t{t}.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"level_sets_0_t{t}.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"tip_elements_0_t{t}.vtk"));
			expectedFiles.Add(Path.Combine(expectedDirectory, $"intersected_elements_0_t{t}.vtk"));
			//expectedFiles.Add(Path.Combine(expectedDirectory, $"conforming_elements_0_t{t}.vtk"));

			double tolerance = 1E-6;
			for (int i = 0; i < expectedFiles.Count; ++i)
			{
				Assert.True(IOUtilities.AreDoubleValueFilesEquivalent(expectedFiles[i], computedFiles[i], tolerance));
			}
		}

		private static XModel<IXCrackElement> CreateModel()
		{
			var model = new XModel<IXCrackElement>(3);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);
			model.FindConformingSubcells = false;

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

			//var jIntegrationRule =
			//    new IntegrationWithNonconformingSubhexahedra3D(8, GaussLegendre3D.GetQuadratureWithOrder(4, 4, 4));
			//IPropagator propagator = null;
			//var crack = new LsmCrack3D(0, lsmGeometry, model, propagator);
			//geometryModel.Cracks[crack.ID] = crack;

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
				var barycenter = new Vertex3D(vertices.Count, new double[] { 0, 0, 0});
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
			crack.CrackFront = new ImmersedCrackFront3D(crack);

			return crack;
		}

		private class MockPropagator : IPropagator
		{
			public (double growthAngle, double growthLength) Propagate(
				IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements,
				double[] crackTip, double[] extensionVector, IEnumerable<IXCrackElement> tipElements)
			{
				return (growthAngle, growthLength);
			}
		}
	}
}
