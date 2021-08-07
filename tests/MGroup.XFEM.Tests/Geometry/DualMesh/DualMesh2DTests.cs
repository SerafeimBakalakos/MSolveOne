using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Geometry.Mesh;
using Xunit;

namespace MGroup.XFEM.Tests.Geometry.DualMesh
{
    public static class DualMesh2DTests
    {
        [Fact]
        public static void TestFindFineNodesEdgesOfCoarseElement()
        {
            (DualCartesianMesh2D dualMesh, IDualMesh mockMesh) = PrepareMeshes(3);


            for (int coarseElem = 0; coarseElem < dualMesh.CoarseMesh.NumElementsTotal; ++coarseElem)
            {
                DualCartesianMesh2D.Submesh submeshExpected = ((MockMesh1To3)mockMesh).FindFineNodesEdgesOfCoarseElement(coarseElem);
                DualCartesianMesh2D.Submesh submeshComputed = dualMesh.FindFineNodesEdgesOfCoarseElement(coarseElem);

                // Check nodes
                Assert.Equal(submeshExpected.FineNodeIDs.Count, submeshComputed.FineNodeIDs.Count);
                for (int i = 0; i < submeshExpected.FineNodeIDs.Count; ++i)
                {
                    Assert.Equal(submeshExpected.FineNodeIDs[i], submeshComputed.FineNodeIDs[i]);
                }

                // Check edges
                Assert.Equal(submeshExpected.FineEdgesToNodes.Count, submeshComputed.FineEdgesToNodes.Count);
                for (int i = 0; i < submeshExpected.FineEdgesToNodes.Count; ++i)
                {
                    Assert.Equal(submeshExpected.FineEdgesToNodes[i].Item1, submeshComputed.FineEdgesToNodes[i].Item1);
                    Assert.Equal(submeshExpected.FineEdgesToNodes[i].Item2, submeshComputed.FineEdgesToNodes[i].Item2);
                }

                //// Check elements
                //Assert.Equal(submeshExpected.FineElementToEdges.Count, submeshComputed.FineElementToEdges.Count);
                //for (int i = 0; i < submeshExpected.FineElementToEdges.Count; ++i)
                //{
                //    int[] expectedArray = submeshExpected.FineElementToEdges[i];
                //    int[] computedArray = submeshComputed.FineElementToEdges[i];
                //    Assert.Equal(expectedArray.Length, computedArray.Length);
                //    for (int j = 0; j < expectedArray.Length; ++j)
                //    {
                //        Assert.Equal(expectedArray[j], computedArray[j]);
                //    }
                //}
            }
        }

        [Theory]
        [InlineData(1)]
        [InlineData(3)]
        public static void TestMapNodeFineToCoarse(int multiplicity)
        {
            (DualCartesianMesh2D dualMesh, IDualMesh mockMesh) = PrepareMeshes(multiplicity);

            for (int fineNode = 0; fineNode < dualMesh.FineMesh.NumNodesTotal; ++fineNode)
            {
                int coarseNodeExpected = mockMesh.MapNodeFineToCoarse(fineNode);
                int coarseNodeComputed = dualMesh.MapNodeFineToCoarse(fineNode);
                Assert.Equal(coarseNodeExpected, coarseNodeComputed);
            }
        }

        [Theory]
        [InlineData(1)]
        [InlineData(3)]
        public static void TestMapNodeCoarseToFine(int multiplicity)
        {
            (DualCartesianMesh2D dualMesh, IDualMesh mockMesh) = PrepareMeshes(multiplicity);

            for (int coarseNode = 0; coarseNode < dualMesh.CoarseMesh.NumNodesTotal; ++coarseNode)
            {
                int fineNodeExpected = mockMesh.MapNodeIDCoarseToFine(coarseNode);
                int fineNodeComputed = dualMesh.MapNodeIDCoarseToFine(coarseNode);
                Assert.Equal(fineNodeExpected, fineNodeComputed);
            }
        }

        [Theory]
        [InlineData(1)]
        [InlineData(3)]
        public static void TestMapElementFineToCoarse(int multiplicity)
        {
            (DualCartesianMesh2D dualMesh, IDualMesh mockMesh) = PrepareMeshes(multiplicity);

            for (int fineElem = 0; fineElem < dualMesh.FineMesh.NumElementsTotal; ++fineElem)
            {
                int coarseElemExpected = mockMesh.MapElementFineToCoarse(fineElem);
                int coarseElemComputed = dualMesh.MapElementFineToCoarse(fineElem);
                Assert.Equal(coarseElemExpected, coarseElemComputed);
            }
        }

        [Theory]
        [InlineData(1)]
        [InlineData(3)]
        public static void TestMapElementCoarseToFine(int multiplicity)
        {
            (DualCartesianMesh2D dualMesh, IDualMesh mockMesh) = PrepareMeshes(multiplicity);

            for (int coarseElem = 0; coarseElem < dualMesh.CoarseMesh.NumElementsTotal; ++coarseElem)
            {
                int[] fineElemExpected = mockMesh.MapElementCoarseToFine(coarseElem);
                int[] fineElemComputed = dualMesh.MapElementCoarseToFine(coarseElem);
                Assert.Equal(fineElemExpected.Length, fineElemComputed.Length);
                for (int i = 0; i < fineElemExpected.Length; ++i)
                {
                    Assert.Equal(fineElemExpected[i], fineElemComputed[i]);
                }
            }
        }

        private static (DualCartesianMesh2D dualMesh, IDualMesh mockMesh) PrepareMeshes(int multiplicity)
        {
            var minCoordinates = new double[] { 0, 0 };
            var maxCoordinates = new double[] { 2, 3 };
            var numElementsCoarse = new int[] { 2, 3 };

            IDualMesh mockMesh;
            DualCartesianMesh2D dualMesh;
            if (multiplicity == 1)
            {
                var numElementsFine = new int[] { 2, 3 };
                mockMesh = new MockMesh1To1();
                dualMesh = new DualCartesianMesh2D.Builder(minCoordinates, maxCoordinates, numElementsCoarse, numElementsFine)
                    .BuildMesh();
            }
            else if (multiplicity == 3)
            {
                var numElementsFine = new int[] { 6, 9 };
                mockMesh = new MockMesh1To3();
                dualMesh = new DualCartesianMesh2D.Builder(minCoordinates, maxCoordinates, numElementsCoarse, numElementsFine)
                    .BuildMesh();
            }
            else
            {
                throw new NotImplementedException();
            }

            return (dualMesh, mockMesh);
        }

        private class MockMesh1To1 : IDualMesh
        {
            public IStructuredMesh CoarseMesh => throw new NotImplementedException();

            public IStructuredMesh FineMesh => throw new NotImplementedException();

            public int Dimension => 2;

            public DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords)
            {
                throw new NotImplementedException();
            }

            public int[] MapElementCoarseToFine(int coarseElementID)
            {
                return new int[] { coarseElementID };
            }

            public int MapElementFineToCoarse(int fineElementID)
            {
                return fineElementID;
            }

            public int MapNodeIDCoarseToFine(int coarseNodeID)
            {
                return coarseNodeID;
            }

            public int MapNodeFineToCoarse(int fineNodeID)
            {
                return fineNodeID;
            }

            public double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural)
            {
                throw new NotImplementedException();
            }
        }

        private class MockMesh1To3 : IDualMesh
        {
            public IStructuredMesh CoarseMesh => throw new NotImplementedException();

            public IStructuredMesh FineMesh => throw new NotImplementedException();

            public int Dimension => 2;

            public DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords)
            {
                throw new NotImplementedException();
            }

            public DualCartesianMesh2D.Submesh FindFineNodesEdgesOfCoarseElement(int coarseElementID)
            {
                // Elements to edges
                var elements = new List<int[]>();
                elements.Add(new int[] { 0, 13, 3, 12 });
                elements.Add(new int[] { 0, 14, 3, 13 });
                elements.Add(new int[] { 0, 15, 3, 14 });
                elements.Add(new int[] { 3, 17, 6, 16 });
                elements.Add(new int[] { 3, 18, 6, 17 });
                elements.Add(new int[] { 3, 19, 6, 18 });
                elements.Add(new int[] { 6, 21, 9, 20 });
                elements.Add(new int[] { 6, 22, 9, 21 });
                elements.Add(new int[] { 6, 23, 9, 22 });

                if (coarseElementID == 0)
                {
                    var fineNodes = new List<int>(new int[]
                    {
                        0, 1, 2, 3, 7, 8, 9, 10, 14, 15, 16, 17, 21, 22, 23, 24
                    });

                    var edges = new List<(int, int)>();

                    //Horizontal
                    edges.Add((0, 1)); edges.Add((1, 2)); edges.Add((2, 3));
                    edges.Add((7, 8)); edges.Add((8, 9)); edges.Add((9, 10));
                    edges.Add((14, 15)); edges.Add((15, 16)); edges.Add((16, 17));
                    edges.Add((21, 22)); edges.Add((22, 23)); edges.Add((23, 24));

                    // Vertical
                    edges.Add((0, 7)); edges.Add((1, 8)); edges.Add((2, 9)); edges.Add((3, 10));
                    edges.Add((7, 14)); edges.Add((8, 15)); edges.Add((9, 16)); edges.Add((10, 17));
                    edges.Add((14, 21)); edges.Add((15, 22)); edges.Add((16, 23)); edges.Add((17, 24));

                    return new DualCartesianMesh2D.Submesh(fineNodes, edges, elements);
                }
                else if (coarseElementID == 1)
                {
                    var fineNodes = new List<int>(new int[]
                    {
                        3, 4, 5, 6, 10, 11, 12, 13, 17, 18, 19, 20, 24, 25, 26, 27
                    });

                    var edges = new List<(int, int)>();

                    //Horizontal
                    edges.Add((3, 4)); edges.Add((4, 5)); edges.Add((5, 6));
                    edges.Add((10, 11)); edges.Add((11, 12)); edges.Add((12, 13));
                    edges.Add((17, 18)); edges.Add((18, 19)); edges.Add((19, 20));
                    edges.Add((24, 25)); edges.Add((25, 26)); edges.Add((26, 27));

                    // Vertical
                    edges.Add((3, 10)); edges.Add((4, 11)); edges.Add((5, 12)); edges.Add((6, 13));
                    edges.Add((10, 17)); edges.Add((11, 18)); edges.Add((12, 19)); edges.Add((13, 20));
                    edges.Add((17, 24)); edges.Add((18, 25)); edges.Add((19, 26)); edges.Add((20, 27));

                    return new DualCartesianMesh2D.Submesh(fineNodes, edges, elements);
                }
                else if (coarseElementID == 2)
                {
                    var fineNodes = new List<int>(new int[]
                    {
                        21, 22, 23, 24, 28, 29, 30, 31, 35, 36, 37, 38, 42, 43, 44, 45
                    });

                    var edges = new List<(int, int)>();

                    //Horizontal
                    edges.Add((21, 22)); edges.Add((22, 23)); edges.Add((23, 24));
                    edges.Add((28, 29)); edges.Add((29, 30)); edges.Add((30, 31));
                    edges.Add((35, 36)); edges.Add((36, 37)); edges.Add((37, 38));
                    edges.Add((42, 43)); edges.Add((43, 44)); edges.Add((44, 45));

                    // Vertical
                    edges.Add((21, 28)); edges.Add((22, 29)); edges.Add((23, 30)); edges.Add((24, 31));
                    edges.Add((28, 35)); edges.Add((29, 36)); edges.Add((30, 37)); edges.Add((31, 38));
                    edges.Add((35, 42)); edges.Add((36, 43)); edges.Add((37, 44)); edges.Add((38, 45));

                    return new DualCartesianMesh2D.Submesh(fineNodes, edges, elements);
                }
                else if (coarseElementID == 3)
                {
                    var fineNodes = new List<int>(new int[]
                    {
                        24, 25, 26, 27, 31, 32, 33, 34, 38, 39, 40, 41, 45, 46, 47, 48
                    });

                    var edges = new List<(int, int)>();

                    //Horizontal
                    edges.Add((24, 25)); edges.Add((25, 26)); edges.Add((26, 27));
                    edges.Add((31, 32)); edges.Add((32, 33)); edges.Add((33, 34));
                    edges.Add((38, 39)); edges.Add((39, 40)); edges.Add((40, 41));
                    edges.Add((45, 46)); edges.Add((46, 47)); edges.Add((47, 48));

                    // Vertical
                    edges.Add((24, 31)); edges.Add((25, 32)); edges.Add((26, 33)); edges.Add((27, 34));
                    edges.Add((31, 38)); edges.Add((32, 39)); edges.Add((33, 40)); edges.Add((34, 41));
                    edges.Add((38, 45)); edges.Add((39, 46)); edges.Add((40, 47)); edges.Add((41, 48));

                    return new DualCartesianMesh2D.Submesh(fineNodes, edges, elements);
                }
                else if (coarseElementID == 4)
                {
                    var fineNodes = new List<int>(new int[]
                    {
                        42, 43, 44, 45, 49, 50, 51, 52, 56, 57, 58, 59, 63, 64, 65, 66
                    });

                    var edges = new List<(int, int)>();

                    //Horizontal
                    edges.Add((42, 43)); edges.Add((43, 44)); edges.Add((44, 45));
                    edges.Add((49, 50)); edges.Add((50, 51)); edges.Add((51, 52));
                    edges.Add((56, 57)); edges.Add((57, 58)); edges.Add((58, 59));
                    edges.Add((63, 64)); edges.Add((64, 65)); edges.Add((65, 66));

                    // Vertical
                    edges.Add((42, 49)); edges.Add((43, 50)); edges.Add((44, 51)); edges.Add((45, 52));
                    edges.Add((49, 56)); edges.Add((50, 57)); edges.Add((51, 58)); edges.Add((52, 59));
                    edges.Add((56, 63)); edges.Add((57, 64)); edges.Add((58, 65)); edges.Add((59, 66));

                    return new DualCartesianMesh2D.Submesh(fineNodes, edges, elements);
                }
                else if (coarseElementID == 5)
                {
                    var fineNodes = new List<int>(new int[]
                    {
                        45, 46, 47, 48, 52, 53, 54, 55, 59, 60, 61, 62, 66, 67, 68, 69
                    });

                    var edges = new List<(int, int)>();

                    //Horizontal
                    edges.Add((45, 46)); edges.Add((46, 47)); edges.Add((47, 48));
                    edges.Add((52, 53)); edges.Add((53, 54)); edges.Add((54, 55));
                    edges.Add((59, 60)); edges.Add((60, 61)); edges.Add((61, 62));
                    edges.Add((66, 67)); edges.Add((67, 68)); edges.Add((68, 69));

                    // Vertical
                    edges.Add((45, 52)); edges.Add((46, 53)); edges.Add((47, 54)); edges.Add((48, 55));
                    edges.Add((52, 59)); edges.Add((53, 60)); edges.Add((54, 61)); edges.Add((55, 62));
                    edges.Add((59, 66)); edges.Add((60, 67)); edges.Add((61, 68)); edges.Add((62, 69));

                    return new DualCartesianMesh2D.Submesh(fineNodes, edges, elements);
                }
                else throw new IndexOutOfRangeException();
            }

            public int[] MapElementCoarseToFine(int coarseElementID)
            {
                var coarseToFineElements = new int[6][];
                coarseToFineElements[0] = new int[] { 0, 1, 2, 6, 7, 8, 12, 13, 14 };
                coarseToFineElements[1] = new int[] { 3, 4, 5, 9, 10, 11, 15, 16, 17 };
                coarseToFineElements[2] = new int[] { 18, 19, 20, 24, 25, 26, 30, 31, 32 };
                coarseToFineElements[3] = new int[] { 21, 22, 23, 27, 28, 29, 33, 34, 35 };
                coarseToFineElements[4] = new int[] { 36, 37, 38, 42, 43, 44, 48, 49, 50 };
                coarseToFineElements[5] = new int[] { 39, 40, 41, 45, 46, 47, 51, 52, 53 };
                return coarseToFineElements[coarseElementID];
            }

            public int MapElementFineToCoarse(int fineElementID)
            {
                var fineToCoarseElements = new int[54];

                fineToCoarseElements[0] = 0;
                fineToCoarseElements[1] = 0;
                fineToCoarseElements[2] = 0;
                fineToCoarseElements[6] = 0;
                fineToCoarseElements[7] = 0;
                fineToCoarseElements[8] = 0;
                fineToCoarseElements[12] = 0;
                fineToCoarseElements[32] = 0;
                fineToCoarseElements[14] = 0;

                fineToCoarseElements[3] = 1;
                fineToCoarseElements[4] = 1;
                fineToCoarseElements[5] = 1;
                fineToCoarseElements[9] = 1;
                fineToCoarseElements[10] = 1;
                fineToCoarseElements[11] = 1;
                fineToCoarseElements[15] = 1;
                fineToCoarseElements[16] = 1;
                fineToCoarseElements[17] = 1;

                fineToCoarseElements[18] = 2;
                fineToCoarseElements[19] = 2;
                fineToCoarseElements[20] = 2;
                fineToCoarseElements[24] = 2;
                fineToCoarseElements[25] = 2;
                fineToCoarseElements[26] = 2;
                fineToCoarseElements[30] = 2;
                fineToCoarseElements[31] = 2;
                fineToCoarseElements[32] = 2;

                fineToCoarseElements[21] = 3;
                fineToCoarseElements[22] = 3;
                fineToCoarseElements[23] = 3;
                fineToCoarseElements[27] = 3;
                fineToCoarseElements[28] = 3;
                fineToCoarseElements[29] = 3;
                fineToCoarseElements[33] = 3;
                fineToCoarseElements[34] = 3;
                fineToCoarseElements[35] = 3;

                fineToCoarseElements[36] = 4;
                fineToCoarseElements[37] = 4;
                fineToCoarseElements[38] = 4;
                fineToCoarseElements[42] = 4;
                fineToCoarseElements[43] = 4;
                fineToCoarseElements[44] = 4;
                fineToCoarseElements[48] = 4;
                fineToCoarseElements[49] = 4;
                fineToCoarseElements[50] = 4;

                fineToCoarseElements[39] = 5;
                fineToCoarseElements[40] = 5;
                fineToCoarseElements[41] = 5;
                fineToCoarseElements[45] = 5;
                fineToCoarseElements[46] = 5;
                fineToCoarseElements[47] = 5;
                fineToCoarseElements[51] = 5;
                fineToCoarseElements[52] = 5;
                fineToCoarseElements[53] = 5;

                return fineToCoarseElements[fineElementID];
            }

            public int MapNodeIDCoarseToFine(int coarseNodeID)
            {
                var coarseToFineNodes = new int[12];
                coarseToFineNodes[0] = 0;
                coarseToFineNodes[1] = 3;
                coarseToFineNodes[2] = 6;
                coarseToFineNodes[3] = 21;
                coarseToFineNodes[4] = 24;
                coarseToFineNodes[5] = 27;
                coarseToFineNodes[6] = 42;
                coarseToFineNodes[7] = 45;
                coarseToFineNodes[8] = 48;
                coarseToFineNodes[9] = 63;
                coarseToFineNodes[10] = 66;
                coarseToFineNodes[11] = 69;

                return coarseToFineNodes[coarseNodeID];
            }

            public int MapNodeFineToCoarse(int fineNodeID)
            {
                var fineToCoarseNodes = new int[70];
                for (int i = 0; i < fineToCoarseNodes.Length; ++i) fineToCoarseNodes[i] = -1;

                fineToCoarseNodes[0] = 0;
                fineToCoarseNodes[3] = 1;
                fineToCoarseNodes[6] = 2;
                fineToCoarseNodes[21] = 3;
                fineToCoarseNodes[24] = 4;
                fineToCoarseNodes[27] = 5;
                fineToCoarseNodes[42] = 6;
                fineToCoarseNodes[45] = 7;
                fineToCoarseNodes[48] = 8;
                fineToCoarseNodes[63] = 9;
                fineToCoarseNodes[66] = 10;
                fineToCoarseNodes[69] = 11;

                return fineToCoarseNodes[fineNodeID];

            }

            public double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural)
            {
                throw new NotImplementedException();
            }
        }
    }
}
