using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.XFEM.Output;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Tests.Utilities;
using Xunit;

//TODO: Is the intagration order enough for ridge enrichment?
namespace MGroup.XFEM.Tests.MultiphaseStructural
{
	public class HalfAndHalf2D
	{
		private static readonly string outputDirectory = @"C:\Users\Serafeim\Desktop\HEAT\elasticity\half_half_2D";

		private const int dim = 2;
		private static readonly double[] minCoords = { -1.0, -1.0 };
		private static readonly double[] maxCoords = { +1.0, +1.0 };
		private const double thickness = 1.0;
		private static readonly int[] numElements = { 35, 35 };
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;

		private const int defaultPhaseID = 0;

		private const double matrixE = 1, v = 0.3;
		//private const double inclusionE = 2;
		//private const bool cohesiveInterfaces = false;
		//private const double cohesiveness = 1E8;

		[Fact]
		public static void TestModel()
		{
			double inclusionE = 2;
			bool cohesiveInterfaces = false;
			double cohesiveness = 1E8;

			if (!Directory.Exists(outputDirectory))
			{
				Directory.CreateDirectory(outputDirectory);
			}

			// Create model and LSM
			XModel<IXMultiphaseElement> model = CreateModel(inclusionE, cohesiveInterfaces, cohesiveness);
			model.FindConformingSubcells = true;
			PhaseGeometryModel geometryModel = CreatePhases(model, cohesiveInterfaces);

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
		}

		[Fact]
		public static void TestSolution()
		{
			double inclusionE = 2;
			bool cohesiveInterfaces = false;
			double cohesiveness = 1E8;

			if (!Directory.Exists(outputDirectory))
			{
				Directory.CreateDirectory(outputDirectory);
			}

			// Create model and LSM
			XModel<IXMultiphaseElement> model = CreateModel(inclusionE, cohesiveInterfaces, cohesiveness);
			model.FindConformingSubcells = true;
			PhaseGeometryModel geometryModel = CreatePhases(model, cohesiveInterfaces);

			// Run analysis
			model.Initialize();
			(IAlgebraicModel algebraicModel, ISolver solver) = SolverChoice.Skyline.Create(model);
			IGlobalVector solution = Utilities.Analysis.RunStructuralStaticAnalysis(model, algebraicModel, solver);

			// Plot displacements, strains, stresses
			var outputFiles = new List<string>();
			outputFiles.Add(Path.Combine(outputDirectory, "displacement_nodes_t0.vtk"));
			outputFiles.Add(Path.Combine(outputDirectory, "displacement_gauss_points_t0.vtk"));
			outputFiles.Add(Path.Combine(outputDirectory, "strains_gauss_points_t0.vtk"));
			outputFiles.Add(Path.Combine(outputDirectory, "stresses_gauss_points_t0.vtk"));
			outputFiles.Add(Path.Combine(outputDirectory, "displacement_strain_stress_field_t0.vtk"));
			Plotting.PlotDisplacements(model, algebraicModel, solution, outputFiles[0], outputFiles[1]);
			Plotting.PlotStrainsStressesAtGaussPoints(model, algebraicModel, solution, outputFiles[2], outputFiles[3]);
			Plotting.PlotDisplacementStrainStressFields(model, algebraicModel, solution, outputFiles[4]);
		}

		//[Theory] //TODO: Add more test cases
		//[InlineData(matrixE, false, 0)]
		//[InlineData(matrixE, true, 1E8)]
		//public static void TestHomogenization(double inclusionE, bool cohesiveInterfaces, double cohesiveness)
		//{
		//    if (!Directory.Exists(outputDirectory))
		//    {
		//        Directory.CreateDirectory(outputDirectory);
		//    }

		//    // Create model and LSM
		//    XModel<IXMultiphaseElement> model = CreateModel(inclusionE, cohesiveInterfaces, cohesiveness);
		//    model.FindConformingSubcells = true;
		//    PhaseGeometryModel geometryModel = CreatePhases(model, cohesiveInterfaces);

		//    // Run analysis
		//    model.Initialize();
		//    IMatrix elasticity = Analysis.RunHomogenizationAnalysisStructural2D(model, minCoords, maxCoords, thickness);

		//    // Check
		//    var matrixMaterial = new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = matrixE, PoissonRatio = v };
		//    IMatrixView elasticityExpected = matrixMaterial.ConstitutiveMatrix;
		//    double tol = 1E-3;
		//    Assert.True(elasticityExpected.Equals(elasticity, tol));

		//    // Print results
		//    //string pathResults = outputDirectory + "\\equivalent_elasticity.txt";
		//    //Plotting.PrintElasticityTensor(pathResults, elasticity);
		//    //Plotting.PrintElasticityTensor(pathResults, matrixE, v, 2);
		//}

		private static XModel<IXMultiphaseElement> CreateModel(double inclusionE, bool cohesiveInterfaces, double cohesiveness)
		{
			// Materials
			var materialMatrix = new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = matrixE, PoissonRatio = v };
			var materialInclusion = new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = inclusionE, PoissonRatio = v };
			var interfaceMaterial = new CohesiveInterfaceMaterial(Matrix.CreateFromArray(new double[,]
			{
				{ cohesiveness, 0 },
				{ 0, cohesiveness }
			}));
			var materialField = new MatrixInclusionsStructuralMaterialField(
				materialMatrix, materialInclusion, interfaceMaterial, 0);

			// Setup model
			XModel<IXMultiphaseElement> model = Models.CreateQuad4Model(minCoords, maxCoords, thickness, numElements,
				bulkIntegrationOrder, boundaryIntegrationOrder, materialField, cohesiveInterfaces);
			Models.ApplyBCsCantileverTension(model);

			return model;
		}

		private static PhaseGeometryModel CreatePhases(XModel<IXMultiphaseElement> model, bool cohesiveInterfaces)
		{
			List<ICurve2D> discontinuities = Utilities.Phases.CreateHalfSpace2D(minCoords, maxCoords, true);
			PhaseGeometryModel geometryModel = Utilities.Phases.CreateLsmPhases2D(model, discontinuities);
			if (cohesiveInterfaces)
			{
				geometryModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateStructuralStep(geometryModel, dim);
			}
			else
			{
				geometryModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateStructuralRidge(geometryModel, dim);
			}
			return geometryModel;
		}
	}
}
