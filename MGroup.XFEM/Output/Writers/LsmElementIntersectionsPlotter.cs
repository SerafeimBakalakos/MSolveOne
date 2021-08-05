using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Writers;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
    public class LsmElementIntersectionsPlotter : IPhaseMeshInteractionObserver
    {
        private readonly string outputDirectory;
        private readonly IXModel model;
        private readonly bool shuffleIDs;
        private int iteration;

        public LsmElementIntersectionsPlotter(string outputDirectory, IXModel model, bool shuffleIDs = false)
        {
            this.outputDirectory = outputDirectory;
            this.model = model;
            this.shuffleIDs = shuffleIDs;
            iteration = 0;
        }

        public void Update()
        {
            var allIntersections = new List<IElementDiscontinuityInteraction>();
            foreach (IXFiniteElement element in model.EnumerateElements())
            {
                foreach (IElementDiscontinuityInteraction interaction in element.InteractingDiscontinuities.Values)
                {
                    if ((interaction.RelativePosition == RelativePositionCurveElement.Intersecting)
                        || (interaction.RelativePosition == RelativePositionCurveElement.Conforming))
                    {
                        allIntersections.Add(interaction);
                    }
                }
            }
            var intersectionMesh = new LsmIntersectionSegmentsMesh(allIntersections);
            List<double> elementIDs = new List<double>(intersectionMesh.ParentElementIDsOfVertices);
            List<double> geometryIDs = new List<double>(intersectionMesh.ParentGeometryIDsOfVertices);

            string path = Path.Combine(outputDirectory, $"intersections_t{iteration}.vtk");
            if (shuffleIDs)
            {
                double[] uniqueIDs = geometryIDs.Distinct().ToArray();
                double[] shuffledIDs = new double[uniqueIDs.Length];
                Array.Copy(uniqueIDs, shuffledIDs, uniqueIDs.Length);
                Shuffle(shuffledIDs);

                var shuffleMap = new Dictionary<double, double>();
                for (int i = 0; i < uniqueIDs.Length; i++)
                {
                    shuffleMap[uniqueIDs[i]] = shuffledIDs[i];
                }

                for (int i = 0; i < geometryIDs.Count; i++)
                {
                    geometryIDs[i] = shuffleMap[geometryIDs[i]];
                }

                using (var writer = new VtkFileWriter(path))
                {
                    writer.WriteMesh(intersectionMesh);
                    writer.WriteScalarField("elementID", intersectionMesh, elementIDs);
                    writer.WriteScalarField("lsm_geometryID", intersectionMesh, geometryIDs);
                }
            }
            else
            {
                using (var writer = new VtkFileWriter(path))
                {
                    writer.WriteMesh(intersectionMesh);
                    writer.WriteScalarField("elementID", intersectionMesh, elementIDs);
                    writer.WriteScalarField("lsm_geometryID", intersectionMesh, geometryIDs);
                }
            }

            ++iteration;
        }

        public void PlotIntersections(string path, IEnumerable<IElementDiscontinuityInteraction> intersections)
        {
            var intersectionMesh = new LsmIntersectionSegmentsMesh(intersections);

            List<double> elementIDs = new List<double>(intersectionMesh.ParentElementIDsOfVertices);
            List<double> geometryIDs = new List<double>(intersectionMesh.ParentGeometryIDsOfVertices);
            using (var writer = new VtkFileWriter(path))
            {
                writer.WriteMesh(intersectionMesh);
                writer.WriteScalarField("elementID", intersectionMesh, elementIDs);
                writer.WriteScalarField("lsm_geometryID", intersectionMesh, geometryIDs);
            }

            if (shuffleIDs)
            {
                double[] uniqueIDs = geometryIDs.Distinct().ToArray();
                double[] shuffledIDs = new double[uniqueIDs.Length];
                Array.Copy(uniqueIDs, shuffledIDs, uniqueIDs.Length);
                Shuffle(shuffledIDs);

                var shuffleMap = new Dictionary<double, double>();
                for (int i = 0; i < uniqueIDs.Length; i++)
                {
                    shuffleMap[uniqueIDs[i]] = shuffledIDs[i];
                }

                for (int i = 0; i < geometryIDs.Count; i++)
                {
                    geometryIDs[i] = shuffleMap[geometryIDs[i]];
                }

                using (var writer = new VtkFileWriter(path))
                {
                    writer.WriteMesh(intersectionMesh);
                    writer.WriteScalarField("elementID", intersectionMesh, elementIDs);
                    writer.WriteScalarField("lsm_geometryID", intersectionMesh, geometryIDs);
                }
            }
        }

        private static void Shuffle<T>(IList<T> list)
        {
            var rng = new Random();
            int n = list.Count;
            while (n > 1)
            {
                n--;
                int k = rng.Next(n + 1);
                T value = list[k];
                list[k] = list[n];
                list[n] = value;
            }
        }

    }
}
