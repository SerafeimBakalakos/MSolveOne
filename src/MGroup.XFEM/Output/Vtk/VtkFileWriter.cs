using System;
using System.Collections.Generic;
using System.IO;
using MGroup.XFEM.Output.Mesh;

namespace MGroup.XFEM.Output.Vtk
{
	public class VtkFileWriter : IDisposable
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
		public void WriteMesh(IOutputMesh mesh)
		{
			if (writeFieldsNext) throw new InvalidOperationException("A mesh has already been written.");

			// Vertices 
			//this.numVertices = mesh.NumOutVertices;
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {mesh.NumOutVertices} double");
			foreach (VtkPoint point in mesh.OutVertices.Values)
			{
				double[] coords = point.Get3DCoordinates();
				writer.WriteLine($"{coords[0]} {coords[1]} {coords[2]}");
			}

			// Cell connectivity
			int cellDataCount = 0;
			foreach (VtkCell cell in mesh.OutCells) cellDataCount += 1 + cell.Vertices.Count;
			writer.WriteLine($"\nCELLS {mesh.NumOutCells} {cellDataCount}");
			foreach (VtkCell cell in mesh.OutCells)
			{
				writer.Write(cell.Vertices.Count);
				foreach (VtkPoint point in cell.Vertices)
				{
					writer.Write(' ');
					writer.Write(point.ID);
				}
				writer.WriteLine();
			}

			// Cell types
			writer.WriteLine("\nCELL_TYPES " + mesh.NumOutCells);
			foreach (VtkCell cell in mesh.OutCells) writer.WriteLine(cell.Code);
		}

		public void WriteScalarField(string fieldName, IOutputMesh mesh, Func<VtkPoint, double> getScalarValue)
		{
			WriteFieldsHeader(mesh.NumOutVertices);
			writer.WriteLine($"SCALARS {fieldName} double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (VtkPoint vertex in mesh.OutVertices.Values) writer.WriteLine(getScalarValue(vertex));
			writer.WriteLine();
		}

		public void WriteScalarField(string fieldName, IOutputMesh mesh, IEnumerable<double> scalarsAtVertices)
		{
			WriteFieldsHeader(mesh.NumOutVertices);
			writer.WriteLine($"SCALARS {fieldName} double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double val in scalarsAtVertices) writer.WriteLine(val);
			writer.WriteLine();
		}

		public void WriteTensor2DField(string fieldName, IReadOnlyList<double[]> tensorsAtVertices)
		{
			WriteFieldsHeader(tensorsAtVertices.Count);

			// Component 11
			writer.WriteLine($"SCALARS {fieldName}_11 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[0]);
			writer.WriteLine();

			// Component 22
			writer.WriteLine($"SCALARS {fieldName}_22 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[1]);
			writer.WriteLine();

			// Component 12
			writer.WriteLine($"SCALARS {fieldName}_12 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[2]);
			writer.WriteLine();
		}

		public void WriteTensor2DField(string fieldName, IOutputMesh mesh, Func<VtkPoint, double[]> getTensorValues)
		{
			WriteFieldsHeader(mesh.NumOutVertices);

			// Component 11
			writer.WriteLine($"SCALARS {fieldName}_11 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (VtkPoint vertex in mesh.OutVertices.Values) writer.WriteLine(getTensorValues(vertex)[0]);
			writer.WriteLine();

			// Component 22
			writer.WriteLine($"SCALARS {fieldName}_22 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (VtkPoint vertex in mesh.OutVertices.Values) writer.WriteLine(getTensorValues(vertex)[1]);
			writer.WriteLine();

			// Component 12
			writer.WriteLine($"SCALARS {fieldName}_12 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (VtkPoint vertex in mesh.OutVertices.Values) writer.WriteLine(getTensorValues(vertex)[2]);
			writer.WriteLine();
		}

		public void WriteTensor2DField(string fieldName, IOutputMesh mesh, IEnumerable<double[]> tensorsAtVertices)
		{
			WriteFieldsHeader(mesh.NumOutVertices);

			// Component 11
			writer.WriteLine($"SCALARS {fieldName}_11 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[0]);
			writer.WriteLine();

			// Component 22
			writer.WriteLine($"SCALARS {fieldName}_22 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[1]);
			writer.WriteLine();

			// Component 12
			writer.WriteLine($"SCALARS {fieldName}_12 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[2]);
			writer.WriteLine();
		}

		public void WriteTensor3DField(string fieldName, IReadOnlyList<double[]> tensorsAtVertices)
		{
			WriteFieldsHeader(tensorsAtVertices.Count);

			// Component 11
			writer.WriteLine($"SCALARS {fieldName}_11 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[0]);
			writer.WriteLine();

			// Component 22
			writer.WriteLine($"SCALARS {fieldName}_22 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[1]);
			writer.WriteLine();

			// Component 33
			writer.WriteLine($"SCALARS {fieldName}_33 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[2]);
			writer.WriteLine();

			// Component 12
			writer.WriteLine($"SCALARS {fieldName}_12 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[3]);
			writer.WriteLine();

			// Component 23
			writer.WriteLine($"SCALARS {fieldName}_23 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[3]);
			writer.WriteLine();

			// Component 13
			writer.WriteLine($"SCALARS {fieldName}_13 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in tensorsAtVertices) writer.WriteLine(tensor[5]);
			writer.WriteLine();
		}

		public void WriteVector2DField(string fieldName, IOutputMesh mesh, IEnumerable<double[]> vectorsAtVertices)
		{
			WriteFieldsHeader(mesh.NumOutVertices);
			writer.WriteLine($"VECTORS {fieldName} double");
			foreach (double[] vector in vectorsAtVertices) writer.WriteLine($"{vector[0]} {vector[1]} 0.0");
			writer.WriteLine();
		}

		public void WriteVector2DField(string fieldName, IOutputMesh mesh, Func<VtkPoint, double[]> getVectorValue)
		{
			WriteFieldsHeader(mesh.NumOutVertices);
			writer.WriteLine($"VECTORS {fieldName} double");
			foreach (VtkPoint vertex in mesh.OutVertices.Values)
			{
				double[] vector = getVectorValue(vertex);
				writer.WriteLine($"{vector[0]} {vector[1]} 0.0");
			}
			writer.WriteLine();
		}

		public void WriteVector2DField(string fieldName, IReadOnlyList<double[]> vectorsAtVertices)
		{
			WriteFieldsHeader(vectorsAtVertices.Count);
			writer.WriteLine($"VECTORS {fieldName} double");
			for (int i = 0; i < vectorsAtVertices.Count; ++i)
			{
				writer.WriteLine($"{vectorsAtVertices[i][0]} {vectorsAtVertices[i][1]} 0.0");
			}
			writer.WriteLine();
		}

		public void WriteVector3DField(string fieldName, IReadOnlyList<double[]> vectorsAtVertices)
		{
			WriteFieldsHeader(vectorsAtVertices.Count);
			writer.WriteLine($"VECTORS {fieldName} double");
			for (int i = 0; i < vectorsAtVertices.Count; ++i)
			{
				writer.WriteLine($"{vectorsAtVertices[i][0]} {vectorsAtVertices[i][1]} {vectorsAtVertices[i][2]}");
			}
			writer.WriteLine();
		}

		public void WriteVector3DField(string fieldName, IOutputMesh mesh, Func<VtkPoint, double[]> getVectorValue)
		{
			WriteFieldsHeader(mesh.NumOutVertices);
			writer.WriteLine($"VECTORS {fieldName} double");
			foreach (VtkPoint vertex in mesh.OutVertices.Values)
			{
				double[] vector = getVectorValue(vertex);
				writer.WriteLine($"{vector[0]} {vector[1]} {vector[2]}");
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
