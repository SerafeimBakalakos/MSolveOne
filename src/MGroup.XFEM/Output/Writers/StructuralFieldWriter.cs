using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Fields;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class StructuralFieldWriter : IResultsWriter
	{
		private readonly IXModel model;
		private readonly string outputDirectory;
		private readonly bool plotFields;
		private readonly bool plotTensorsAtGaussPoints;
		private readonly bool plotSmoothFields;
		private int iteration;

		public StructuralFieldWriter(IXModel model, string outputDirectory, bool plotFields = true, 
			bool plotTensorsAtGaussPoints = true, bool plotSmoothFields = true)
		{
			this.iteration = 0;
			this.model = model;
			this.outputDirectory = outputDirectory;
			this.plotFields = plotFields;
			this.plotTensorsAtGaussPoints = plotTensorsAtGaussPoints;
			this.plotSmoothFields = plotSmoothFields;
		}

		public void WriteResults(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			if (plotFields)
			{
				PlotFields(algebraicModel, solution);
			}
			if (plotTensorsAtGaussPoints)
			{
				PlotTensorsAtGaussPoints(algebraicModel, solution);
			}
			if (plotSmoothFields)
			{
				PlotSmoothFields(algebraicModel, solution);
			}

			++iteration;
		}

		private void PlotFields(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			var conformingMesh = new ConformingOutputMesh(model);
			var displacementField = new DisplacementField(model, algebraicModel, conformingMesh);
			Dictionary<int, double[]> displacements = displacementField.CalcValuesAtVertices(solution);
			var strainStressField = new StrainStressField_v2(model, algebraicModel, conformingMesh);
			(Dictionary<int, double[]> strains, Dictionary<int, double[]> stresses)
				= strainStressField.CalcTensorsAtVertices(solution);
			string path = Path.Combine(outputDirectory, $"displacements_strains_stresses_t{iteration}.vtk");
			using (var writer = new VtkFileWriter(path))
			{
				writer.WriteMesh(conformingMesh);
				writer.WriteVectorField("displacements", model.Dimension, conformingMesh, v => displacements[v.ID]);
				if (model.Dimension == 2)
				{
					writer.WriteTensor2DField("strains", conformingMesh, v => strains[v.ID]);
					writer.WriteTensor2DField("stresses", conformingMesh, v => stresses[v.ID]);
				}
				else
				{
					writer.WriteTensor3DField("strains", conformingMesh, v => strains[v.ID]);
					writer.WriteTensor3DField("stresses", conformingMesh, v => stresses[v.ID]);
				}
			}
		}

		private void PlotSmoothFields(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			var outputMesh = new ContinuousOutputMesh(model.Nodes.Values, model.EnumerateElements());
			var strainStressField = new SmoothStrainStressField(model, algebraicModel, outputMesh);
			(Dictionary<int, double[]> strains, Dictionary<int, double[]> stresses)
				= strainStressField.CalcTensorsAtVertices(solution);
			string path = Path.Combine(outputDirectory, $"smooth_strains_stresses_t{iteration}.vtk");
			using (var writer = new VtkFileWriter(path))
			{
				writer.WriteMesh(outputMesh);
				if (model.Dimension == 2)
				{
					writer.WriteTensor2DField("strains", outputMesh, v => strains[v.ID]);
					writer.WriteTensor2DField("stresses", outputMesh, v => stresses[v.ID]);
				}
				else
				{
					writer.WriteTensor3DField("strains", outputMesh, v => strains[v.ID]);
					writer.WriteTensor3DField("stresses", outputMesh, v => stresses[v.ID]);
				}
			}
		}

		private void PlotTensorsAtGaussPoints(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			var strainStressFieldAtGPs = new StrainsStressesAtGaussPointsField_v2(model, algebraicModel);
			(Dictionary<double[], double[]> strainsAtGPs, Dictionary<double[], double[]> stressesAtGPs)
				= strainStressFieldAtGPs.CalcTensorsAtPoints(solution);
			if (model.Dimension == 2)
			{
				using (var writer = new VtkPointWriter(Path.Combine(outputDirectory, $"strains_at_gauss_points_t{iteration}.vtk")))
				{
					writer.WriteTensor2DField("strain", strainsAtGPs);
				}

				// Stresses at Gauss Points
				using (var writer = new VtkPointWriter(Path.Combine(outputDirectory, $"stresses_at_gauss_points_t{iteration}.vtk")))
				{
					writer.WriteTensor2DField("stress", stressesAtGPs);
				}
			}
			else
			{
				using (var writer = new VtkPointWriter(Path.Combine(outputDirectory, $"strains_at_gauss_points_t{iteration}.vtk")))
				{
					writer.WriteTensor3DField("strain", strainsAtGPs);
				}

				// Stresses at Gauss Points
				using (var writer = new VtkPointWriter(Path.Combine(outputDirectory, $"stresses_at_gauss_points_t{iteration}.vtk")))
				{
					writer.WriteTensor3DField("stress", stressesAtGPs);
				}
			}
		}
	}
}
