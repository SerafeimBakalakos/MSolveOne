using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.IsoXFEM
{
	public class VtkFileWriter
	{
		public const string vtkReaderVersion = "4.1";
		private readonly StreamWriter writer;
		private bool writeFieldsNext;
		//private int numVertices = -1;

		public VtkFileWriter(string filePath)
		{
			this.writer = new StreamWriter(filePath);
			writer.Write("# vtk DataFile Version ");
			writer.WriteLine(vtkReaderVersion);
			//writer.WriteLine(filePath);
			writer.WriteLine("Header:");
			writer.Write("ASCII\n\n");
			writeFieldsNext = false;
		}

		public void Dispose()
		{
			if (writer != null) writer.Dispose();
		}

		//TODO: Perhaps the mesh should be injected into the contructor
		public void WriteMesh(Dictionary<int, XNode>  nodes, List<IsoXfemElement2D> elements)
		{
			if (writeFieldsNext) throw new InvalidOperationException("A mesh has already been written.");

			// Vertices 
			//this.numVertices = mesh.NumOutVertices;
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {nodes.Count} double");
			//if (nodes[0].Z != null)
			//{
			//	for (int i = 0; i < nodeCoords.NumRows; ++i)
			//	{
			//		writer.WriteLine($"{nodeCoords[i, 0]} {nodeCoords[i, 1]} {nodeCoords[i, 2]}");
			//	}
			//}
			//else
			//{
				for (int i = 0; i < nodes.Count; ++i)
				{
					writer.WriteLine($"{nodes[i].X} {nodes[i].Y} 0.0");
				}
			//}

			// Cell connectivity
			int cellDataCount = 0;
			for (int i = 0; i < elements.Count; ++i)
			{
				cellDataCount += 1 + elements[i].nodesOfElement.Count;
			}
			writer.WriteLine($"\nCELLS {elements.Count} {cellDataCount}");
			for (int i = 0; i < elements.Count; ++i)
			{
				writer.Write(elements[i].nodesOfElement.Count);
				for (int j = 0; j < elements[i].nodesOfElement.Count; ++j)
				{
					writer.Write(' ');
					writer.Write(elements[i].nodesOfElement[j].ID);
				}
				writer.WriteLine();
			}

			// Cell types
			writer.WriteLine("\nCELL_TYPES " + elements.Count);
			int quad4Code = 9;
			for (int i = 0; i < elements.Count; ++i)
			{
				writer.WriteLine(quad4Code);
			}
		}

		public void WriteScalarField(string fieldName, IList<double> scalars)
		{
			WriteFieldsHeader(scalars.Count);
			writer.WriteLine($"SCALARS {fieldName} double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double value in scalars)
			{
				writer.WriteLine(value);
			};
			writer.WriteLine();
		}
		

		public void WriteVectorField(string fieldName, IList<double[]> vectors)
		{
			WriteFieldsHeader(vectors.Count);
			writer.WriteLine($"VECTORS {fieldName} double");
			foreach (double[] vector in vectors)
			{
				if (vector.Length == 3)
				{
					writer.WriteLine($"{vector[0]} {vector[1]} {vector[2]}");

				}
				else
				{
					writer.WriteLine($"{vector[0]} {vector[1]} 0.0");
				}
			}
			writer.WriteLine();
		}

		/// <summary>
		/// If the user only wants the mesh, this should not be called. Therefore only call it if one or more field output is 
		/// written.
		/// </summary>
		private void WriteFieldsHeader(int numVertices)
		{
			if (!writeFieldsNext) // Fields header
			{
				writer.Write("\n\n");
				writer.WriteLine("POINT_DATA " + numVertices);
				writeFieldsNext = true;
			}
		}
	}
}
