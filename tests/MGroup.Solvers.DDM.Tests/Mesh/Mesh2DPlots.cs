using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.MSolve.Meshes.Output.VTK;
using MGroup.MSolve.Meshes.Structured;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.Mesh
{
	public class Mesh2DPlots
	{
		[Fact]
		public static void PlotMesh()
		{
			string outputDirectory = @"C:\Users\Serafeim\Desktop\DDM\Meshes\2D";

			double[] minCoords = { 0, 0 };
			double[] maxCoords = { 12, 12 };
			int[] numElementsFine = { 12, 12 };
			int[] numElementsCoarse = { 3, 3 };
			var fineMesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElementsFine).BuildMesh();
			var coarseMesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElementsCoarse).BuildMesh();

			var meshWriter = new VtkMeshWriter();
			meshWriter.WriteMesh(Path.Combine(outputDirectory, "fine_mesh.vtk"), fineMesh, 2);
			meshWriter.WriteMesh(Path.Combine(outputDirectory, "coarse_mesh.vtk"), coarseMesh, 2);

			List<VtkPoint> corners = CreateCorners(minCoords, maxCoords, numElementsCoarse);
			using (var pointWriter = new VtkPointWriter(Path.Combine(outputDirectory, "corner_nodes.vtk")))
			{
				pointWriter.WritePoints(corners);
			}
		}

		private static List<VtkPoint> CreateCorners(double[] minCoords, double[] maxCoords, int[] numElementsCoarse)
		{
			var corners = new List<VtkPoint>();
			for (int i = 0; i < numElementsCoarse[0] + 1; ++i)
			{
				for (int j = 0; j < numElementsCoarse[1] + 1; ++j)
				{
					if ((i == 0 && j == 0) || (i == 0 && j == numElementsCoarse[1]) 
						|| (i == numElementsCoarse[0] && j == 0) || (i == numElementsCoarse[0] && j == numElementsCoarse[1]))
					{
						continue;
					}

					double dx = (maxCoords[0] - minCoords[0]) / numElementsCoarse[0];
					double dy = (maxCoords[1] - minCoords[1]) / numElementsCoarse[1];

					double x = minCoords[0] + i * dx;
					double y = minCoords[1] + j * dy;
					int id = corners.Count;
					corners.Add(new VtkPoint(id, x, y));
				}
			}
			return corners;
		}
	}
}
