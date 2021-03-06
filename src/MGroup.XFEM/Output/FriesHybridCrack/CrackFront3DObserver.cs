using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.FriesHybridCrack
{
	public class CrackFront3DObserver : ICrackObserver
	{
		private readonly CrackSurface3D crack;
		private readonly string outputDirectory;
		private int iteration;

		public CrackFront3DObserver(CrackSurface3D crack, string outputDirectory)
		{
			this.crack = crack;
			this.outputDirectory = outputDirectory;
			iteration = 0;
		}

		public void Update()
		{
			// Mesh
			var outputMesh = new OutputCrackMesh(crack.CrackFront);
			string pathMesh = $"{outputDirectory}\\crack_front_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkFileWriter(pathMesh))
			{
				writer.WriteMesh(outputMesh);
			}

			// Orthogonal systems at all tips
			string pathSystems = $"{outputDirectory}\\crack_front_systems_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(pathSystems))
			{
				List<double[]> tangentVectors = crack.CrackFront.CoordinateSystems.Select(sys => sys.Tangent).ToList();
				List<double[]> normalVectors = crack.CrackFront.CoordinateSystems.Select(sys => sys.Normal).ToList();
				List<double[]> extensionVectors = crack.CrackFront.CoordinateSystems.Select(sys => sys.Extension).ToList();

				writer.WritePoints(outputMesh.OutVertices, true);
				writer.WriteVectorField("tangent_vectors", tangentVectors);
				writer.WriteVectorField("normal_vectors", normalVectors);
				writer.WriteVectorField("extension_vectors", extensionVectors);
			}

			// Active tips
			string pathActiveTips = $"{outputDirectory}\\active_tips_{crack.ID}_t{iteration}.vtk";
			using (var writer = new VtkPointWriter(pathActiveTips))
			{
				List<VtkPoint> activeTips = FindActiveTips();
				writer.WritePoints(activeTips, false);
			}

			++iteration;
		}

		private List<VtkPoint> FindActiveTips()
		{
			var activeTips = new List<VtkPoint>();
			for (int i = 0; i < crack.CrackFront.ActiveTips.Count; ++i)
			{
				Vertex3D tip = crack.CrackFront.Vertices[crack.CrackFront.ActiveTips[i]];
				activeTips.Add(new VtkPoint(i, tip.CoordsGlobal));
			}
			return activeTips;
		}
	}
}
