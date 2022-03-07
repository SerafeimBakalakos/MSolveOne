using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.MSolve.Meshes.Output.VTK;
using MGroup.MSolve.Meshes.Structured;
using Xunit;

namespace MGroup.MSolve.Meshes.Tests.Structured
{
	public static class UniformSimplicialSymmetricMesh3DTests
	{
		[Fact]
		public static void PlotMesh()
		{
			string path = @"C:\Users\Serafeim\Desktop\DDM\PFETIDP\meshes\symmetric_tetra_mesh3D.vtk";
			int[] numNodes = { 7, 5, 3 };
			double[] minCoords = { 0, 0, 0 };
			double[] maxCoords = { numNodes[0] - 1, numNodes[1] - 1, numNodes[2] - 1 };
			var mesh = new UniformSimplicialSymmetricMesh3D.Builder(minCoords, maxCoords, numNodes).SetMajorMinorAxis(2, 0).BuildMesh();
			//var mesh = new UniformSimplicialSymmetricMesh3D.Builder(minCoords, maxCoords, numNodes).BuildMesh();

			var outputMesh = new VtkMeshDiscontinuous_v2(mesh);
			outputMesh.OffsetVerticesTowardsCentroids(0.3);

			using (var writer = new VtkFileWriter(path, 3))
			{
				writer.WriteMesh(outputMesh);
			}
		}

		//[Fact]
		public static void PlotSimplexize()
		{
			string dir = @"C:\Users\Serafeim\Desktop\DDM\PFETIDP\meshes\hexa_to_tet";
			int[] numNodes = { 3, 3, 3 };
			double[] minCoords = { 0, 0, 0 };
			double[] maxCoords = { numNodes[0] - 1, numNodes[1] - 1, numNodes[2] - 1 };
			var mesh = new UniformSimplicialSymmetricMesh3D.Builder(minCoords, maxCoords, numNodes)
				.SetMajorMinorAxis(2, 0).BuildMesh();
			//var mesh = new UniformSimplicialSymmetricMesh3D.Builder(minCoords, maxCoords, numNodes).BuildMesh();
			
			int[] numElementsHexa8 = { numNodes[0] - 2, numNodes[1] - 2, numNodes[2] - 2 };
			var hexa8Mesh = new UniformCartesianMesh3D.Builder(minCoords, maxCoords, numElementsHexa8)
				.SetMajorMinorAxis(2, 0).BuildMesh();
			var hexa8Writer = new VtkMeshWriter();
			hexa8Writer.WriteMesh(Path.Combine(dir, "hexa8.vtk"), hexa8Mesh, 3);

			for (int e = 0; e < mesh.NumElementsTotal; ++e)
			{
				string path = Path.Combine(dir, $"subtet_{e}.vtk");
				var outputMesh = new VtkMeshDiscontinuous_v2(mesh);
				var cells = (List<VtkCell>)(outputMesh.VtkCells);
				VtkCell cell = cells[e];
				cells.Clear();
				cells.Add(cell);
				using (var writer = new VtkFileWriter(path, 3))
				{
					writer.WriteMesh(outputMesh);
				}
			}
		}
	}
}
