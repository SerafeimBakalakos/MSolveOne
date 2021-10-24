using System;
using System.Collections.Generic;
using System.Text;
using Xunit;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Output;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Tests.Utilities;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution;
using MGroup.XFEM.Materials.Duplicates;
using System.IO;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Tests.Output
{
	public static class ConformingMeshTests
	{
		private static readonly string outputDirectory = @"C:\Users\Serafeim\Desktop\HEAT\2020\conforming_meshes";
		//private static readonly string outputDirectory = Path.Combine(
		//	Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "conforming_meshes_temp");
		private static readonly string expectedDirectory = Path.Combine(
			Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "conforming_meshes");
		private const string continuousMeshFileName = "continuous_mesh.vtk";

		private static readonly double[] minCoords = { -1.0, -1.0 };
		private static readonly double[] maxCoords = { +1.0, +1.0 };
		private const double thickness = 1.0;
		private static readonly int[] numElements = { 15, 15 };
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;
		private static readonly int[] numBalls = { 1, 1 };
		private const double ballRadius = 0.4;

		private const int defaultPhaseID = 0;

		private const double conductMatrix = 1E0, conductInclusion = 1E5;
		private const double conductBoundaryMatrixInclusion = 1E1, conductBoundaryInclusionInclusion = 1E2;
		private const double specialHeatCoeff = 1.0;


		[Theory]
		[InlineData(true)]
		public static void TestModel(bool cartesianMesh)
		{
			try
			{
				if (!Directory.Exists(outputDirectory))
				{
					Directory.CreateDirectory(outputDirectory);
				}

				// Create model and LSM
				XModel<IXMultiphaseElement> model = CreateModel(cartesianMesh);
				model.FindConformingSubcells = true;
				PhaseGeometryModel geometryModel = CreatePhases(model, cartesianMesh);

				// Plot level sets
				geometryModel.GeometryObservers.Add(new PhaseLevelSetPlotter(outputDirectory, model, geometryModel));

				// Plot phases of each element subcell
				model.ModelObservers.Add(new ElementPhasePlotter(outputDirectory, model, geometryModel, defaultPhaseID));

				// Initialize model state so that everything described above can be tracked
				model.Initialize();

				PlotConformingContinuousMesh(model, geometryModel);

				//// Compare output
				//var computedFiles = new List<string>();
				//computedFiles.Add(Path.Combine(outputDirectory, "level_set1_t0.vtk"));
				//computedFiles.Add(Path.Combine(outputDirectory, "element_phases_t0.vtk"));

				//var expectedFiles = new List<string>();
				//expectedFiles.Add(Path.Combine(expectedDirectory, "level_set1_t0.vtk"));
				//expectedFiles.Add(Path.Combine(expectedDirectory, "element_phases_t0.vtk"));

				//double tolerance = 1E-6;
				//for (int i = 0; i < expectedFiles.Count; ++i)
				//{
				//	Assert.True(IOUtilities.AreDoubleValueFilesEquivalent(expectedFiles[i], computedFiles[i], tolerance));
				//}
			}
			finally
			{
				//if (Directory.Exists(outputDirectory))
				//{
				//	DirectoryInfo di = new DirectoryInfo(outputDirectory);
				//	di.Delete(true);//true means delete subdirectories and files
				//}
			}
		}

		private static XModel<IXMultiphaseElement> CreateModel(bool cartesianMesh)
		{
			// Materials
			var matrixMaterial = new ThermalMaterial(conductMatrix, specialHeatCoeff);
			var inclusionMaterial = new ThermalMaterial(conductInclusion, specialHeatCoeff);
			var materialField = new MatrixInclusionsThermalMaterialField(matrixMaterial, inclusionMaterial,
				conductBoundaryMatrixInclusion, conductBoundaryInclusionInclusion, defaultPhaseID);

			return Models.CreateQuad4Model(minCoords, maxCoords, thickness, numElements, cartesianMesh,
				bulkIntegrationOrder, boundaryIntegrationOrder, materialField, true);
		}

		private static PhaseGeometryModel CreatePhases(XModel<IXMultiphaseElement> model, bool cartesianMesh)
		{
			List<ICurve2D> balls = Utilities.Phases.CreateBallsStructured2D(minCoords, maxCoords, numBalls, ballRadius, 1.0);
			PhaseGeometryModel geometryModel = Utilities.Phases.CreateLsmPhases2D(model, balls, cartesianMesh);
			geometryModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateThermalStep(geometryModel);
			return geometryModel;
		}

		private static void PlotConformingContinuousMesh(IXModel model, PhaseGeometryModel geometryModel)
		{
			IPhaseBoundary boundary = geometryModel.Phases[1].ExternalBoundaries[0];
			var mesh = new ConformingContinuousMesh(model, boundary.Geometry);
			string path = Path.Combine(outputDirectory, continuousMeshFileName);
			using (var writer = new VtkFileWriter(path))
			{
				writer.WriteMesh(mesh);
			}
		}
	}
}
