using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.Environments;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Mesh;
using MGroup.Solvers.DDM.Output;
using MGroup.Solvers.DDM.Partitioning;
using MGroup.Solvers.DDM.Tests.ExampleModels;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.Partitioning
{
    public static class UniformMeshPartitioner3DTests
    {
        //[Fact]
        public static void PlotPartitioning()
        {
            IModel model = Brick3DExample.CreateSingleSubdomainModel();
            model.ConnectDataStructures();

            var mesh = new UniformMesh3D.Builder(Brick3DExample.MinCoords, Brick3DExample.MaxCoords, Brick3DExample.NumElements)
                .BuildMesh();
            var partitioner = new UniformMeshPartitioner3D(mesh, Brick3DExample.NumSubdomains, Brick3DExample.NumClusters);
            partitioner.Partition(model);

            string outputDirectory = @"C:\Users\Serafeim\Desktop\PFETIDP\partitioning\brick3D";
            var writer = new PartitioningWriter(outputDirectory, 3);
            writer.PlotPartitioning(model, partitioner);
        }

        [Fact]
        public static void TestMeshPartitioning()
        {
            var mesh = new UniformMesh3D.Builder(Brick3DExample.MinCoords, Brick3DExample.MaxCoords, Brick3DExample.NumElements)
                .SetMajorMinorAxis(0, 2).BuildMesh();
            var partitioner = new UniformMeshPartitioner3D(mesh, Brick3DExample.NumSubdomains, Brick3DExample.NumClusters);
            partitioner.Partition(null);

            Dictionary<int, int> expectedPartitioning = Brick3DExample.GetSubdomainsOfElements();
            foreach (int elementID in expectedPartitioning.Keys)
            {
                Assert.Equal(expectedPartitioning[elementID], partitioner.GetSubdomainOfElement(elementID));
            }
        }

        [Fact]
        public static void TestSubdomainClustering()
        {
            var mesh = new UniformMesh3D.Builder(Brick3DExample.MinCoords, Brick3DExample.MaxCoords, Brick3DExample.NumElements)
                .SetMajorMinorAxis(0, 2).BuildMesh();
            var partitioner = new UniformMeshPartitioner3D(mesh, Brick3DExample.NumSubdomains, Brick3DExample.NumClusters);
            partitioner.Partition(null);

            Dictionary<int, int> expectedClustering = Brick3DExample.GetSubdomainClusters();
            foreach (int subdomainID in expectedClustering.Keys)
            {
                Assert.Equal(expectedClustering[subdomainID], partitioner.GetClusterOfSubdomain(subdomainID));
            }
        }

        [Fact]
        public static void TestSubdomainNeighbors()
        {
            var mesh = new UniformMesh3D.Builder(Brick3DExample.MinCoords, Brick3DExample.MaxCoords, Brick3DExample.NumElements)
                .SetMajorMinorAxis(0, 2).BuildMesh();
            var partitioner = new UniformMeshPartitioner3D(mesh, Brick3DExample.NumSubdomains, Brick3DExample.NumClusters);
            partitioner.Partition(null);

            Dictionary<int, int[]> expectedSubdomainNeighbors = Brick3DExample.GetSubdomainNeighbors();
            foreach (int subdomainID in expectedSubdomainNeighbors.Keys)
            {
                int[] expectedNeighbors = expectedSubdomainNeighbors[subdomainID].OrderBy(n => n).ToArray();
                int[] computedNeighbors = partitioner.GetNeighboringSubdomains(subdomainID).OrderBy(n => n).ToArray();
                Assert.True(Utilities.AreEqual(expectedNeighbors, computedNeighbors));
            }
        }
    }
}
