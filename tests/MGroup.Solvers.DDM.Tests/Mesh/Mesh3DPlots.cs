using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.MSolve.Meshes.Output.VTK;
using MGroup.MSolve.Meshes.Structured;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.Mesh
{
	public class Mesh3DPlots
	{
		[Fact]
		public static void PlotMesh()
		{
			string outputDirectory = @"C:\Users\Serafeim\Desktop\DDM\Meshes\3D";

			double[] minCoords = { 0, 0, 0 };
			double[] maxCoords = { 4, 4, 2 };
			int[] numElementsFine = { 4, 4, 2 };
			int[] numElementsCoarse = { 2, 2, 1 };
			var fineMesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElementsFine).BuildMesh();
			var coarseMesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElementsCoarse).BuildMesh();

			var meshWriter = new VtkMeshWriter();
			meshWriter.WriteMesh(Path.Combine(outputDirectory, "fine_mesh.vtk"), fineMesh, 3);
			meshWriter.WriteMesh(Path.Combine(outputDirectory, "coarse_mesh.vtk"), coarseMesh, 3);

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
					for (int k = 0; k < numElementsCoarse[2] + 1; ++k)
					{
						double dx = (maxCoords[0] - minCoords[0]) / numElementsCoarse[0];
						double dy = (maxCoords[1] - minCoords[1]) / numElementsCoarse[1];
						double dz = (maxCoords[2] - minCoords[2]) / numElementsCoarse[2];

						double x = minCoords[0] + i * dx;
						double y = minCoords[1] + j * dy;
						double z = minCoords[2] + k * dz;
						int id = corners.Count;
						corners.Add(new VtkPoint(id, x, y, z));
					}
				}
			}
			return corners;
		}
	}
}
