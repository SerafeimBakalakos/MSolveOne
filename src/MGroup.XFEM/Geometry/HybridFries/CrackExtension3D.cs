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
			CreateExtensionVertices(crack, maxDomainDimension);
			CreateExtensionCellsAndEdges(crack);
		}

		public List<TriangleCell3D> Cells { get; private set; }

		public List<Edge3D> ExtensionEdges { get; private set; }

		public List<Vertex3D> ExtensionVertices { get; private set; }

		private void CreateExtensionCellsAndEdges(CrackSurface3D crack)
		{
			ICrackFront3D crackFront = crack.CrackFront;
			Cells = new List<TriangleCell3D>(2 * crackFront.Edges.Count);
			ExtensionEdges = new List<Edge3D>(3 * crackFront.Edges.Count);

			for (int i = 0; i < crackFront.Vertices.Count; ++i)
			{
				// Use 4 vertices to create edges and cells. 
				// The order of extension vertices is identical to the order of front vertices.
				int next = (i + 1) % crackFront.Vertices.Count;
				Vertex3D vertexA = crackFront.Vertices[next];
				Vertex3D vertexB = crackFront.Vertices[i];
				Vertex3D vertexC = ExtensionVertices[i];
				Vertex3D vertexD = ExtensionVertices[next];
				Edge3D edgeBA = crackFront.Edges[i];

				// Edge "parallel" to front edge
				ExtensionEdges.Add(new Edge3D(vertexC, vertexD, true) { PseudoNormal = edgeBA.PseudoNormal });

				// Edge from current front vertex to corresponding extension vertex
				ExtensionEdges.Add(new Edge3D(vertexB, vertexC, true) { PseudoNormal = vertexB.PseudoNormal });

				// Split the quadrilateral into 2 triangles
				var triangleABC = new TriangleCell3D(vertexA, vertexB, vertexC, true);
				var triangleACD = new TriangleCell3D(vertexA, vertexC, vertexD, true);
				var triangleBDA = new TriangleCell3D(vertexB, vertexD, vertexA, true);
				var triangleBCD = new TriangleCell3D(vertexB, vertexC, vertexD, true);
				double areaDiff0 = Math.Abs(triangleABC.Area - triangleACD.Area) / (triangleABC.Area + triangleACD.Area);
				double areaDiff1 = Math.Abs(triangleBDA.Area - triangleBCD.Area) / (triangleBDA.Area + triangleBCD.Area);
				if (areaDiff0 <= areaDiff1)
				{
					// The triangles ABC & ACD have more similar areas, and thus less degenerate angles than BDA & BCD.
					Cells.Add(triangleABC);
					Cells.Add(triangleACD);
					ExtensionEdges.Add(new Edge3D(vertexA, vertexC) { PseudoNormal = edgeBA.PseudoNormal });
				}
				else
				{
					// The triangles BDA & BCD have more similar areas, and thus less degenerate angles than ABC & ACD.
					Cells.Add(triangleBDA);
					Cells.Add(triangleBCD);
					ExtensionEdges.Add(new Edge3D(vertexB, vertexD) { PseudoNormal = edgeBA.PseudoNormal });
				}
			}
		}

		private void CreateExtensionVertices(CrackSurface3D crack, double maxDomainDimension)
		{
			ICrackFront3D crackFront = crack.CrackFront;
			ExtensionVertices = new List<Vertex3D>(crackFront.Vertices.Count);
			double extensionLength = 10 * maxDomainDimension;
			int numVertices = crack.Vertices.Count;
			for (int v = 0; v < crackFront.Vertices.Count; ++v)
			{
				Vertex3D originalVertex = crackFront.Vertices[v];
				var originalCoords = Vector.CreateFromArray(originalVertex.CoordsGlobal);
				var extensionVector = Vector.CreateFromArray(crackFront.CoordinateSystems[v].Extension);
				var extensionsCoords = originalCoords.Axpy(extensionVector, extensionLength / extensionVector.Norm2());
				var extensionVertex = new Vertex3D(numVertices, extensionsCoords.RawData, true);
				extensionVertex.PseudoNormal = originalVertex.PseudoNormal;
				ExtensionVertices.Add(extensionVertex);
				++numVertices;
			}
		}
	}
}
