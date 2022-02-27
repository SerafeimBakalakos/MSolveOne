using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.MSolve.Meshes.Output.VTK;
using MGroup.MSolve.Meshes.Structured;
using Xunit;

namespace MGroup.MSolve.Meshes.Tests.Structured
{
	public static class UniformSimplicialSymmetricMesh2DTests
	{
		[Fact]
		public static void PlotMesh()
		{
			var writer = new VtkMeshWriter();

			string path = @"C:\Users\Serafeim\Desktop\DDM\PFETIDP\meshes\symmetric_triangle_mesh2D.vtk";
			double[] minCoords = { 0, 0 };
			double[] maxCoords = { 6, 4 };
			int[] numNodes = { 7, 5 };
			var mesh = new UniformSimplicialSymmetricMesh2D.Builder(minCoords, maxCoords, numNodes).SetMajorAxis(0).BuildMesh();
			//var mesh = new UniformSimplicialSymmetricMesh2D.Builder(minCoords, maxCoords, numNodes).BuildMesh();
			writer.WriteMesh(path, mesh, 2);
		}
	}
}
