using System.Collections.Generic;
using System.IO;
using System.Linq;

using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Output;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Tests.Utilities;

using Xunit;

namespace MGroup.XFEM.Tests.MultiphaseThermal
{
	public static class UnionHollowBalls2DTests
	{
		private static readonly string outputDirectory = Path.Combine(
			Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "union_hollow_balls_2D_temp");
		private static readonly string expectedDirectory = Path.Combine(
			Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "union_hollow_balls_2D");

		private static readonly double[] minCoords = { -1.0, -1.0 };
		private static readonly double[] maxCoords = { +1.0, +1.0 };
		private const double thickness = 1.0;
		private static readonly int[] numElements = { 15, 15 };
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;

		private const int defaultPhaseID = 0;

		private const double conductMatrix = 1E0, conductInclusion = 1E5;
		private const double conductBoundaryMatrixInclusion = 1E1, conductBoundaryInclusionInclusion = 1E2;
		private const double specialHeatCoeff = 1.0;

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
				XModel<IXMultiphaseElement> model = CreateModel();
				model.FindConformingSubcells = true;
				PhaseGeometryModel geometryModel = CreatePhases(model);

				// Plot level sets
				geometryModel.GeometryObservers.Add(new PhaseLevelSetPlotter(outputDirectory, model, geometryModel));

				// Plot phases of nodes
				geometryModel.InteractionObservers.Add(new NodalPhasesPlotter(outputDirectory, model));

				// Plot element - phase boundaries interactions
				geometryModel.InteractionObservers.Add(new LsmElementIntersectionsPlotter(outputDirectory, model));

				// Plot element subcells
				model.ModelObservers.Add(new ConformingMeshPlotter(outputDirectory, model));

				// Plot phases of each element subcell
				model.ModelObservers.Add(new ElementPhasePlotter(outputDirectory, model, geometryModel, defaultPhaseID));

				// Write the size of each phase
				model.ModelObservers.Add(new PhasesSizeWriter(outputDirectory, model, geometryModel));

				// Plot bulk and boundary integration points of each element
				model.ModelObservers.Add(new IntegrationPointsPlotter(outputDirectory, model));

				// Plot enrichments
				double elementSize = (maxCoords[0] - minCoords[0]) / numElements[0];
				model.GeometryModel.Enricher.Observers.Add(new PhaseEnrichmentPlotter(outputDirectory, model, elementSize, 2));

				// Initialize model state so that everything described above can be tracked
				model.Initialize();

				// Compare output
				var computedFiles = new List<string>();
				computedFiles.Add(Path.Combine(outputDirectory, "level_set1_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "level_set2_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "level_set4_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "nodal_phases_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "intersections_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "conforming_mesh_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "element_phases_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "phase_sizes_t0.txt"));
				computedFiles.Add(Path.Combine(outputDirectory, "gauss_points_bulk_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "gauss_points_boundary_t0.vtk"));
				computedFiles.Add(Path.Combine(outputDirectory, "enriched_nodes_heaviside_t0.vtk"));

				var expectedFiles = new List<string>();
				expectedFiles.Add(Path.Combine(expectedDirectory, "level_set1_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "level_set2_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "level_set4_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "nodal_phases_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "intersections_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "conforming_mesh_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "element_phases_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "phase_sizes_t0.txt"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "gauss_points_bulk_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "gauss_points_boundary_t0.vtk"));
				expectedFiles.Add(Path.Combine(expectedDirectory, "enriched_nodes_heaviside_t0.vtk"));

				double tolerance = 1E-6;
				for (int i = 0; i < expectedFiles.Count; ++i)
				{
					Assert.True(IOUtilities.AreDoubleValueFilesEquivalent(expectedFiles[i], computedFiles[i], tolerance));
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

		private static XModel<IXMultiphaseElement> CreateModel()
		{
			// Materials
			var matrixMaterial = new ThermalMaterial(conductMatrix, specialHeatCoeff);
			var inclusionMaterial = new ThermalMaterial(conductInclusion, specialHeatCoeff);
			var materialField = new MatrixInclusionsThermalMaterialField(matrixMaterial, inclusionMaterial,
				conductBoundaryMatrixInclusion, conductBoundaryInclusionInclusion, defaultPhaseID);

			return Models.CreateQuad4Model(minCoords, maxCoords, thickness, numElements, true,
				bulkIntegrationOrder, boundaryIntegrationOrder, materialField, true);
		}

		private static PhaseGeometryModel CreatePhases(XModel<IXMultiphaseElement> model)
		{
			var geometricModel = new PhaseGeometryModel(model);
			model.GeometryModel = geometricModel;
			geometricModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateThermalStep(geometricModel);
			geometricModel.MergeOverlappingPhases = true;
			var defaultPhase = new DefaultPhase();
			geometricModel.Phases[defaultPhase.ID] = defaultPhase;

			var ballsInternal = new Circle2D[2];
			ballsInternal[0] = new Circle2D(-0.3, 0, 0.15);
			ballsInternal[1] = new Circle2D(+0.3, 0, 0.1);
			var ballsExternal = new Circle2D[2];
			ballsExternal[0] = new Circle2D(-0.3, 0, 0.5);
			ballsExternal[1] = new Circle2D(+0.3, 0, 0.4);

			for (int p = 0; p < ballsInternal.Length; ++p)
			{
				var externalPhase = new HollowLsmPhase(2 * p + 1, geometricModel, 0);
				var externalCurve = new SimpleLsm2D(externalPhase.ID, model.Nodes.Values.OrderBy(n => n.ID).ToList(), 
					ballsExternal[p]);
				geometricModel.Phases[externalPhase.ID] = externalPhase;

				var externalBoundary = new ClosedPhaseBoundary(externalPhase.ID, externalCurve, defaultPhase, externalPhase);
				defaultPhase.ExternalBoundaries.Add(externalBoundary);
				defaultPhase.Neighbors.Add(externalPhase);
				externalPhase.ExternalBoundaries.Add(externalBoundary);
				externalPhase.Neighbors.Add(defaultPhase);
				geometricModel.PhaseBoundaries[externalBoundary.ID] = externalBoundary;

				var internalLsm = new SimpleLsm2D(2 * p + 2, model.Nodes.Values.OrderBy(n => n.ID).ToList(), ballsInternal[p]);
				var internalPhase = new LsmPhase(2 * p + 2, geometricModel, -1);
				geometricModel.Phases[internalPhase.ID] = internalPhase;

				var internalBoundary = new ClosedPhaseBoundary(internalPhase.ID, internalLsm, externalPhase, internalPhase);
				externalPhase.InternalBoundaries.Add(internalBoundary);
				externalPhase.InternalPhases.Add(internalPhase);
				externalPhase.Neighbors.Add(internalPhase);
				internalPhase.ExternalBoundaries.Add(internalBoundary);
				internalPhase.Neighbors.Add(externalPhase);
			}
			return geometricModel;
		}
	}
}
