using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
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
            using (var writer = new VtkPointWriter($"{outputDirectory}\\tip_elements_{iteration}.vtk"))
            {
                var tipElementCentroids = new Dictionary<double[], double>();
                foreach (IXFiniteElement element in crack.TipElements)
                {
                    double[] centroid = element.FindCentroidCartesian();
                    tipElementCentroids[centroid] = 0;
                }
                if (tipElementCentroids.Count == 0) //a dummy node just to get the Paraview reader working.
                {
                    tipElementCentroids[dummyPoint] = 0;
                }
                writer.WriteScalarField("tip_element_centroids", tipElementCentroids);
            }

            using (var writer = new VtkPointWriter($"{outputDirectory}\\intersected_elements_{iteration}.vtk"))
            {
                var intersectedElementCentroids = new Dictionary<double[], double>();
                foreach (IXFiniteElement element in crack.IntersectedElements)
                {
                    double[] centroid = element.FindCentroidCartesian();
                    intersectedElementCentroids[centroid] = 0.0;
                }
                if (intersectedElementCentroids.Count == 0) //a dummy node just to get the Paraview reader working.
                {
                    intersectedElementCentroids[dummyPoint] = 0;
                }
                writer.WriteScalarField("intersected_element_centroids", intersectedElementCentroids);
            }

            using (var writer = new VtkPointWriter($"{outputDirectory}\\conforming_elements_{iteration}.vtk"))
            {
                var conformingElementCentroids = new Dictionary<double[], double>();
                foreach (IXFiniteElement element in crack.ConformingElements)
                {
                    double[] centroid = element.FindCentroidCartesian();
                    conformingElementCentroids[centroid] = 0.0;
                }
                if (conformingElementCentroids.Count == 0) //a dummy node just to get the Paraview reader working.
                {
                    conformingElementCentroids[dummyPoint] = 0;
                }
                writer.WriteScalarField("conforming_element_centroids", conformingElementCentroids);
            }
            ++iteration;
        }
    }
}
