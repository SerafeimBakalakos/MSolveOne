using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Geometry.HybridFries
{
	/// <summary>
	/// See "Crack propagation with the XFEM and a hybrid explicit-implicit crack description, Fries & Baydoun, 2012", 
	/// section 3.1.5
	/// </summary>
	public class CrackExtension2D
	{
		public CrackExtension2D(CrackCurve2D crack, double maxDomainDimension)
		{
			ICrackFront2D crackFront = crack.CrackFront;
			int numVertices = crack.Vertices.Count;
			double extensionLength = 10 * maxDomainDimension;
			ExtensionVertices = new List<Vertex2D>(crackFront.Vertices.Count);
			Cells = new List<LineCell2D>(crackFront.Vertices.Count);

			for (int v = 0; v < crackFront.Vertices.Count; ++v)
			{
				Vertex2D tip = crackFront.Vertices[v];
				CrackFrontSystem2D frontSystem = crackFront.CoordinateSystems[v];

				// Create extension vertex
				var vertex = Vector.CreateFromArray(tip.CoordsGlobal);
				var extensionVector = Vector.CreateFromArray(frontSystem.Tangent);
				var newVertex = vertex.Axpy(extensionVector, extensionLength);
				var extension = new Vertex2D(numVertices++, newVertex.RawData, true);
				extension.PseudoNormal = tip.PseudoNormal;
				ExtensionVertices.Add(extension);

				// Create extension cell
				if (crackFront.CoordinateSystems[v].IsCounterClockwise) //TODO: the coordinate system could create both the vertex and the cell
				{
					Cells.Add(new LineCell2D(tip, extension, true));
				}
				else
				{
					Cells.Add(new LineCell2D(extension, tip, true));
				}
			}
		}

		public List<LineCell2D> Cells { get; }

		public List<Vertex2D> ExtensionVertices { get; }
	}
}
