using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.FriesHybridCrack
{
	public class CrackFront2DObserver : ICrackObserver
	{
		private readonly CrackCurve2D crack;
		private readonly string outputDirectory;
		private int iteration;

		public CrackFront2DObserver(CrackCurve2D crack, string outputDirectory)
		{
			this.crack = crack;
			this.outputDirectory = outputDirectory;
			iteration = 0;
		}

		public void Update()
		{
			// Orthogonal systems at vertices
			string pathSystems = $"{outputDirectory}\\crack_front_systems_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(pathSystems))
			{
				List<VtkPoint> points = crack.CrackFront.Vertices.Select(v => new VtkPoint(v.ID, v.CoordsGlobal)).ToList();
				List<double[]> tangentVectors = crack.CrackFront.CoordinateSystems.Select(sys => sys.Extension).ToList();
				List<double[]> normalVectors = crack.CrackFront.CoordinateSystems.Select(sys => sys.Normal).ToList();

				writer.WritePoints(points, true);
				writer.WriteVectorField("tangent_vectors", tangentVectors);
				writer.WriteVectorField("normal_vectors", normalVectors);
			}

			++iteration;
		}
	}
}
