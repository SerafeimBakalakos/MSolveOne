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
	public class StrainStressFieldWriter : IResultsWriter
	{
		private readonly IXModel model;
		private readonly string outputDirectory;
		private int iteration;

		public StrainStressFieldWriter(IXModel model, string outputDirectory)
		{
			this.iteration = 0;
			this.model = model;
			this.outputDirectory = outputDirectory;
		}

		public void WriteResults(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			PlotTensorFields(algebraicModel, solution);
			PlotTensorsAtGaussPoints(algebraicModel, solution);

			++iteration;
		}

		private void PlotTensorFields(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			var conformingMesh = new ConformingOutputMesh(model);
			var strainStressField = new StrainStressField(model, algebraicModel, conformingMesh);
			(Dictionary<int, double[]> strains, Dictionary<int, double[]> stresses)
				= strainStressField.CalcTensorsAtVertices(solution);
			string path = Path.Combine(outputDirectory, $"strains_stresses_t{iteration}.vtk");
			using (var writer = new VtkFileWriter(path))
			{
				writer.WriteMesh(conformingMesh);
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

		private void PlotTensorsAtGaussPoints(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			var strainStressFieldAtGPs = new StrainsStressesAtGaussPointsField(model, algebraicModel);
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
