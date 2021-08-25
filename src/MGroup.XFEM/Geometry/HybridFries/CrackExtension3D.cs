using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Geometry.HybridFries
{
    /// <summary>
    /// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
    /// section 3.2.5
    /// </summary>
    public class CrackExtension3D
    {
        public CrackExtension3D(CrackSurface3D crack, double maxDomainDimension)
        {
            // Vertices
            ICrackFront3D crackFront = crack.CrackFront;
            ExtensionVertices = new List<Vertex3D>(crackFront.Vertices.Count);

            Dictionary<int, Vertex3D> frontToExtensionVertices = CreateExtensionVertices(crack, maxDomainDimension);
            foreach (Vertex3D vertex in crackFront.Vertices)
            {
                Vertex3D extension = frontToExtensionVertices[vertex.ID];
                extension.PseudoNormal = vertex.PseudoNormal;
                ExtensionVertices.Add(extension);
            }

            // Edges
            ExtensionEdges = CreateEdges(crackFront, frontToExtensionVertices);

            // Cells
            Cells = CreateCells(crackFront, frontToExtensionVertices);
        }

        public List<TriangleCell3D> Cells { get; }

        public List<Edge3D> ExtensionEdges { get; }

        public List<Vertex3D> ExtensionVertices { get; }

        private static List<TriangleCell3D> CreateCells(ICrackFront3D crackFront, 
            Dictionary<int, Vertex3D> frontToExtensionVertices)
        {
            var cells = new List<TriangleCell3D>(2 * crackFront.Edges.Count);
            foreach (Edge3D edge in crackFront.Edges)
            {
                Vertex3D start = edge.Start;
                Vertex3D end = edge.End;
                cells.Add(new TriangleCell3D(end, start, frontToExtensionVertices[start.ID], true));
                cells.Add(new TriangleCell3D(end, frontToExtensionVertices[start.ID], frontToExtensionVertices[end.ID], true));
            }

            return cells;
        }

        private static List<Edge3D> CreateEdges(ICrackFront3D crackFront, Dictionary<int, Vertex3D> frontToExtensionVertices)
        {
            // Front edges
            var edges = new List<Edge3D>(3 * crackFront.Edges.Count);

            // Edges parallel to front
            foreach (Edge3D edge in crackFront.Edges)
            {
                Vertex3D start = frontToExtensionVertices[edge.Start.ID];
                Vertex3D end = frontToExtensionVertices[edge.End.ID];
                var extensionEdge = new Edge3D(start, end, true);
                extensionEdge.PseudoNormal = edge.PseudoNormal;
                edges.Add(extensionEdge);
            }

            // Edges from front to extension points
            foreach (Vertex3D frontVertex in crackFront.Vertices)
            {
                Vertex3D extensionVertex = frontToExtensionVertices[frontVertex.ID];
                var extensionEdge = new Edge3D(frontVertex, extensionVertex, true);
                extensionEdge.PseudoNormal = frontVertex.PseudoNormal;
                edges.Add(extensionEdge); 
            }

            // Edges to split each extension quad into 2 triangles
            foreach (Edge3D edge in crackFront.Edges)
            {
                Vertex3D start = edge.End;
                Vertex3D end = frontToExtensionVertices[edge.Start.ID];
                var extensionEdge = new Edge3D(start, end, true);
                extensionEdge.PseudoNormal = edge.PseudoNormal;
                edges.Add(extensionEdge);
            }

            return edges;
        }

        private static Dictionary<int, Vertex3D> CreateExtensionVertices(CrackSurface3D crack, double maxDomainDimension)
        {
            ICrackFront3D crackFront = crack.CrackFront;
            var frontToExtensionVertices = new Dictionary<int, Vertex3D>();
            double extensionLength = 10 * maxDomainDimension;
            int numVertices = crack.Vertices.Count;
            for (int v = 0; v < crackFront.Vertices.Count; ++v)
            {
                var vertex = Vector.CreateFromArray(crackFront.Vertices[v].CoordsGlobal);
                var extensionVector = Vector.CreateFromArray(crackFront.CoordinateSystems[v].Extension);
                var newVertex = vertex.Axpy(extensionVector, extensionLength / extensionVector.Norm2());
                frontToExtensionVertices[crackFront.Vertices[v].ID] = new Vertex3D(numVertices, newVertex.RawData, true);
                ++numVertices;
            }
            return frontToExtensionVertices;
        }
    }
}
