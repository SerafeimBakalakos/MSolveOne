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
	public class DisplacementFieldWriter : IResultsWriter
	{
		private readonly IXModel model;
		private readonly string outputDirectory;
		private int iteration;

		public DisplacementFieldWriter(IXModel model, string outputDirectory)
		{
			this.iteration = 0;
			this.model = model;
			this.outputDirectory = outputDirectory;
		}

		public void WriteResults(IAlgebraicModel algebraicModel, IGlobalVector solution)
		{
			var conformingMesh = new ConformingOutputMesh(model);
			var displacementField = new DisplacementField(model, algebraicModel, conformingMesh);
			Dictionary<int, double[]> nodalDisplacements = displacementField.CalcValuesAtVertices(solution);
			string path = Path.Combine(outputDirectory, $"displacements_t{iteration}.vtk");
			using (var writer = new VtkFileWriter(path))
			{
				writer.WriteMesh(conformingMesh);
				writer.WriteVectorField("displacements", model.Dimension, conformingMesh, v => nodalDisplacements[v.ID]);
			}

			++iteration;
		}
	}
}
