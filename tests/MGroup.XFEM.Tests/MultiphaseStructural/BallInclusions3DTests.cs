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

//BUG: When the inclusions are close enough to have nodes enriched with more than 1 enrichments, then at some Gauss points and
//     nodes there are very high and very low strains. For example if inclusionE=2*matrixE and numBalls = {2,1,1}. Investigate!
//TODO: Is the integration order enough for ridge enrichment?
namespace MGroup.XFEM.Tests.MultiphaseStructural
{
	public class BallInclusions3DTests
	{
		private static readonly string outputDirectory = @"C:\Users\Serafeim\Desktop\HEAT\elasticity\ball_inclusions_3D";

		private const int dim = 3;
		private static readonly double[] minCoords = { -1.0, -1.0, -1.0 };
		private static readonly double[] maxCoords = { +1.0, +1.0, +1.0 };
		private static readonly int[] numElements = { 19, 19, 19 };
		private const int bulkIntegrationOrder = 2, boundaryIntegrationOrder = 2;
		private static readonly int[] numBalls = { 1, 1, 1 };
		private const double ballRadius = 0.3;

		private const int defaultPhaseID = 0;

		private const double matrixE = 1, v = 0.3;

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
			model.GeometryModel.Enricher.Observers.Add(new PhaseEnrichmentPlotter(outputDirectory, model, elementSize, 3));

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
			var computedFiles = new List<string>();
			computedFiles.Add(Path.Combine(outputDirectory, "displacement_nodes_t0.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, "displacement_gauss_points_t0.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, "strains_gauss_points_t0.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, "stresses_gauss_points_t0.vtk"));
			computedFiles.Add(Path.Combine(outputDirectory, "displacement_strain_stress_field_t0.vtk"));
			//Utilities.Plotting.PlotDisplacements(model, solution, computedFiles[0], computedFiles[1]);
			Plotting.PlotStrainsStressesAtGaussPoints(model, algebraicModel, solution, computedFiles[2], computedFiles[3]);
			Plotting.PlotDisplacementStrainStressFields(model, algebraicModel, solution, computedFiles[4]);
		}

		//[Theory]
		//[InlineData(matrixE, false, 0)]
		////[InlineData(matrixE, true, 1E8)]
		//public static void TestHomogenization(double inclusionE, bool cohesiveInterfaces, double cohesiveness)
		//{
		//	if (!Directory.Exists(outputDirectory))
		//	{
		//		Directory.CreateDirectory(outputDirectory);
		//	}

		//	// Create model and LSM
		//	XModel<IXMultiphaseElement> model = CreateModel(inclusionE, cohesiveInterfaces, cohesiveness);
		//	model.FindConformingSubcells = true;
		//	PhaseGeometryModel geometryModel = CreatePhases(model, cohesiveInterfaces);

		//	// Run analysis
		//	model.Initialize();
		//	IMatrix elasticity = Analysis.RunHomogenizationAnalysisStructural3D(model, minCoords, maxCoords);

		//	// Check
		//	var matrixMaterial = new ElasticMaterial3D() { YoungModulus = matrixE, PoissonRatio = v };
		//	IMatrixView elasticityExpected = matrixMaterial.ConstitutiveMatrix;
		//	double tol = 1E-3;
		//	Assert.True(elasticityExpected.Equals(elasticity, tol));

		//	// Print results
		//	string pathResults = outputDirectory + "\\equivalent_elasticity.txt";
		//	Plotting.PrintElasticityTensor(pathResults, elasticity);
		//	Plotting.PrintElasticityTensor(pathResults, matrixE, v, 3);
		//}

		private static XModel<IXMultiphaseElement> CreateModel(double inclusionE, bool cohesiveInterfaces, double cohesiveness)
		{
			// Materials
			var materialMatrix = new ElasticMaterial3D() { YoungModulus = matrixE, PoissonRatio = v };
			var materialInclusion = new ElasticMaterial3D() { YoungModulus = inclusionE, PoissonRatio = v };
			var interfaceMaterial = new CohesiveInterfaceMaterial(Matrix.CreateFromArray(new double[,]
			{
				{ cohesiveness, 0, 0 },
				{ 0, cohesiveness, 0 },
				{ 0, 0, cohesiveness }
			}));
			var materialField = new MatrixInclusionsStructuralMaterialField(
				materialMatrix, materialInclusion, interfaceMaterial, 0);

			// Setup model
			XModel<IXMultiphaseElement> model = Models.CreateHexa8Model(minCoords, maxCoords, numElements,
				bulkIntegrationOrder, boundaryIntegrationOrder, materialField, false);
			Models.ApplyBCsCantileverTension(model);

			return model;
		}

		private static PhaseGeometryModel CreatePhases(XModel<IXMultiphaseElement> model, bool cohesiveInterfaces)
		{
			List<ISurface3D> balls = Utilities.Phases.CreateBallsStructured3D(minCoords, maxCoords, numBalls, ballRadius, 1);
			PhaseGeometryModel geometryModel = Utilities.Phases.CreateLsmPhases3D(model, balls);
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
