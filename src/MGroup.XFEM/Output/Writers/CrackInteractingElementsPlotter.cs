using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class CrackInteractingElementsPlotter : ICrackObserver
	{
		private static readonly double[] dummyPoint = new double[] { 0.0, 0.0, 0.0 }; //TODO: find a more elegant solution.

		private readonly ICrack crack;
		private readonly string outputDirectory;
		private int iteration;

		public CrackInteractingElementsPlotter(ICrack crack, string outputDirectory)
		{
			this.crack = crack;
			this.outputDirectory = outputDirectory;
			iteration = 0;
		}

		public void Update()
		{
			//PlotInteractingElementsCentroids();
			PlotInteractingElements();
			
			++iteration;
		}

		private void PlotInteractingElements()
		{
			PlotSubmesh($"{outputDirectory}\\tip_elements_{crack.ID}_t{iteration}.vtk",
				"is_in_tip_element", crack.TipElements);
			PlotSubmesh($"{outputDirectory}\\intersected_elements_{crack.ID}_t{iteration}.vtk",
				"is_in_intersected_element", crack.IntersectedElements);
			PlotSubmesh($"{outputDirectory}\\conforming_elements_{crack.ID}_t{iteration}.vtk",
				"is_in_conforming_element", crack.ConformingElements);
		}

		private void PlotInteractingElementsCentroids()
		{
			PlotElementCentroids($"{outputDirectory}\\tip_elements_centroids_{crack.ID}_t{iteration}.vtk",
				"tip_element_centroids", crack.TipElements);
			PlotElementCentroids($"{outputDirectory}\\intersected_element_centroids_{crack.ID}_t{iteration}.vtk",
				"intersected_element_centroids", crack.TipElements);
			PlotElementCentroids($"{outputDirectory}\\conforming_elements_centroids_{crack.ID}_t{iteration}.vtk",
				"conforming_element_centroids", crack.TipElements);
		}

		private void PlotElementCentroids(string path, string fieldName, IEnumerable<IXFiniteElement> elements)
		{
			using (var writer = new VtkPointWriter(path))
			{
				var tipElementCentroids = new Dictionary<double[], double>();
				foreach (IXFiniteElement element in elements)
				{
					double[] centroid = element.FindCentroidCartesian();
					tipElementCentroids[centroid] = 0;
				}
				if (tipElementCentroids.Count == 0) //a dummy node just to get the Paraview reader working.
				{
					tipElementCentroids[dummyPoint] = 0;
				}
				writer.WriteScalarField(fieldName, tipElementCentroids);
			}
		}

		private void PlotSubmesh(string path, string fieldName, IEnumerable<IXFiniteElement> elements)
		{
			if (elements.Count() == 0)
			{
				return;
			}

			using (var writer = new VtkFileWriter(path))
			{
				var submesh = new DiscontinuousOutputMesh(elements);
				writer.WriteMesh(submesh);
				writer.WriteScalarField(fieldName, submesh, v => 1.0);
			}
		}
	}
}
