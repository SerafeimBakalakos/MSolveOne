using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.Primitives;
using Xunit;

namespace MGroup.XFEM.Tests.Geometry
{
    public static class Line2DIntersectionTests
    {
        public static void TestQuadDisjoint()
        {

        }

        public static void TestQuadTangent()
        {

        }

        public static void TestQuadIntersecting0Nodes()
        {

        }

        public static void TestQuadIntersecting1Node()
        {

        }

        public static void TestQuadIntersecting2Nodes()
        {

        }



        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public static void TestTriangleDisjoint(bool vectorized)
        {
            // 3             |         
            //       /\      |
            //      /  \     |
            //     /    \    |
            //    /      \   |
            // 1 /________\  |
            //               |
            //  1    2   3  4

            double[][] triangle = CreateTriangle();
            double[] p1 = new double[] { 4, 0 };
            double[] p2 = new double[] { 4, 1 };
            ICurve2D line;
            if (vectorized) line = new Line2DVectorized(p1, p2);
            else line = new Line2D(p1, p2);
            IElementDiscontinuityInteraction intersection = line.IntersectPolygon(triangle);

            Assert.True(intersection.RelativePosition == RelativePositionCurveElement.Disjoint);
            Assert.True(intersection is NullElementDiscontinuityInteraction);
        }

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public static void TestTriangleTangent(bool vectorized)
        {
            // 3  ___________          
            //       /\     
            //      /  \    
            //     /    \   
            //    /      \  
            // 1 /________\ 
            //   
            //  1    2    3  

            double[][] triangle = CreateTriangle();
            double[] p1 = new double[] { 0, 3 };
            double[] p2 = new double[] { 4, 3 };
            ICurve2D line;
            if (vectorized) line = new Line2DVectorized(p1, p2);
            else line = new Line2D(p1, p2);
            IElementDiscontinuityInteraction intersection = line.IntersectPolygon(triangle);

            Assert.True(intersection.RelativePosition == RelativePositionCurveElement.Disjoint); //TODO: This should be tangent
            Assert.True(intersection is NullElementDiscontinuityInteraction);

            //Assert.True(pos == RelativePositionCurveDisc.Tangent);
            //Assert.Equal(1, intersection.Length);

            //double[] intersection = line.LocalToGlobal(intersection[0]);
            //Assert.Equal(2, intersection[0], 5);
            //Assert.Equal(3, intersection[1], 5);

        }

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public static void TestTriangleIntersecting0Nodes(bool vectorized)
        {
            // 3          
            //       /\     
            //      /  \    
            // 2 --/----\---   
            //    /      \  
            // 1 /________\ 
            //  1    2     3 

            double[][] triangle = CreateTriangle();
            double[] p1 = new double[] { 0, 2 };
            double[] p2 = new double[] { 4, 2 };
            ICurve2D line;
            if (vectorized) line = new Line2DVectorized(p1, p2);
            else line = new Line2D(p1, p2);
            IElementDiscontinuityInteraction intersection = line.IntersectPolygon(triangle);
            IIntersectionMesh intersectionMesh = intersection.ApproximateGlobalCartesian();

            Assert.True(intersection.RelativePosition == RelativePositionCurveElement.Intersecting);
            Assert.True(intersection is LineSegmentIntersection2D);

            Assert.Equal(1.5, intersectionMesh.Vertices[0][0], 5);
            Assert.Equal(2, intersectionMesh.Vertices[0][1], 5);

            Assert.Equal(2.5, intersectionMesh.Vertices[1][0], 5);
            Assert.Equal(2, intersectionMesh.Vertices[1][1], 5);
        }

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public static void TestTriangleIntersecting1Node(bool vectorized)
        {
            // 3      |     
            //       /|\     
            //      / | \    
            //     /  |  \   
            //    /   |   \  
            // 1 /____|____\ 
            //        |     
            //  1     2     3 

            double[][] triangle = CreateTriangle();
            double[] p1 = new double[] { 2, 0 };
            double[] p2 = new double[] { 2, 4 };
            ICurve2D line;
            if (vectorized) line = new Line2DVectorized(p1, p2);
            else line = new Line2D(p1, p2);
            IElementDiscontinuityInteraction intersection = line.IntersectPolygon(triangle);
            IIntersectionMesh intersectionMesh = intersection.ApproximateGlobalCartesian();

            Assert.True(intersection.RelativePosition == RelativePositionCurveElement.Intersecting);
            Assert.True(intersection is LineSegmentIntersection2D);

            Assert.Equal(2, intersectionMesh.Vertices[0][0], 5);
            Assert.Equal(1, intersectionMesh.Vertices[0][1], 5);

            Assert.Equal(2, intersectionMesh.Vertices[1][0], 5);
            Assert.Equal(3, intersectionMesh.Vertices[1][1], 5);
        }

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public static void TestTriangleConforming(bool vectorized)
        {
            //      \
            // 3     \         
            //       /\     
            //      /  \    
            //     /    \   
            //    /      \  
            // 1 /________\ 
            //             \
            //              \
            //  1    2    3  

            double[][] triangle = CreateTriangle();
            //double[] p1 = new double[] { 2, 3 };
            //double[] p2 = new double[] { 3, 1 };
            double[] p1 = new double[] { 0, 7 };
            double[] p2 = new double[] { 3.5, 0 };
            ICurve2D line;
            if (vectorized) line = new Line2DVectorized(p1, p2);
            else line = new Line2D(p1, p2);
            IElementDiscontinuityInteraction intersection = line.IntersectPolygon(triangle);
            IIntersectionMesh intersectionMesh = intersection.ApproximateGlobalCartesian();

            Assert.True(intersection.RelativePosition == RelativePositionCurveElement.Conforming);
            Assert.True(intersection is LineSegmentIntersection2D);

            Assert.Equal(2, intersectionMesh.Vertices[0][0], 5);
            Assert.Equal(3, intersectionMesh.Vertices[0][1], 5);

            Assert.Equal(3, intersectionMesh.Vertices[1][0], 5);
            Assert.Equal(1, intersectionMesh.Vertices[1][1], 5);
        }

        private static double[][] CreateQuad()
        {
            var nodes = new double[4][];
            nodes[0] = new double[] { 0, 0 };
            nodes[1] = new double[] { 4, 0 };
            nodes[2] = new double[] { 4, 1 };
            nodes[3] = new double[] { 0, 1 };
            return nodes;
        }

        private static double[][] CreateTriangle()
        {
            var nodes = new double[3][];
            nodes[0] = new double[] { 1, 1 };
            nodes[1] = new double[] { 3, 1 };
            nodes[2] = new double[] { 2, 3 };
            return nodes;
        }
    }
}
