using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Extensions;

namespace MGroup.XFEM.Output.Vtk
{
	public class VtkPointWriter : IDisposable
	{
		public const string vtkReaderVersion = "4.1";
		private readonly StreamWriter writer;

		public VtkPointWriter(string filePath)
		{
			this.writer = new StreamWriter(filePath);
			writer.Write("# vtk DataFile Version ");
			writer.WriteLine(vtkReaderVersion);
			writer.WriteLine("Header:");
			writer.Write("ASCII\n\n");
		}

		public void Dispose()
		{
			if (writer != null) writer.Dispose();
		}

		public void WritePoints(IEnumerable<VtkPoint> points, bool willPointDataFollow = false)
		{
			int numPoints = points.Count();
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {numPoints} double");
			foreach (VtkPoint point in points)
			{
				double[] coords = point.Get3DCoordinates();
				writer.WriteLine($"{coords[0]} {coords[1]} {coords[2]}");
			}

			if (willPointDataFollow)
			{
				writer.Write("\n\n");
				writer.WriteLine($"POINT_DATA {numPoints}");
			}
		}

		public void WritePoints(IEnumerable<double[]> points, bool willPointDataFollow = false)
		{
			int numPoints = points.Count();
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {numPoints} double");
			foreach (double[] coords in points)
			{
				if (coords.Length == 2)
				{
					writer.WriteLine($"{coords[0]} {coords[1]} 0");

				}
				else
				{
					writer.WriteLine($"{coords[0]} {coords[1]} {coords[2]}");
				}
			}

			if (willPointDataFollow)
			{
				writer.Write("\n\n");
				writer.WriteLine($"POINT_DATA {numPoints}");
			}
		}

		public void WriteScalarField(string fieldName, IReadOnlyList<double[]> points, IReadOnlyList<double> pointValues)
		{
			// Points
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {points.Count} double");
			foreach (double[] point in points)
			{
				if (point.Length == 2) writer.WriteLine($"{point[0]} {point[1]} 0.0");
				else if (point.Length == 3) writer.WriteLine($"{point[0]} {point[1]} {point[2]}");
				else throw new NotImplementedException();
			}

			// Values
			writer.Write("\n\n");
			writer.WriteLine($"POINT_DATA {pointValues.Count}");
			writer.WriteLine($"SCALARS {fieldName} double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double value in pointValues)
			{
				writer.WriteLine(value);
			}
			writer.WriteLine();
		}

		public void WriteScalarField(string fieldName, IReadOnlyDictionary<double[], double> pointValues)
		{
			// Points
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {pointValues.Count} double");
			foreach (double[] point in pointValues.Keys)
			{
				if (point.Length == 2) writer.WriteLine($"{point[0]} {point[1]} 0.0");
				else if (point.Length == 3) writer.WriteLine($"{point[0]} {point[1]} {point[2]}");
				else throw new NotImplementedException();
			}

			// Values
			writer.Write("\n\n");
			writer.WriteLine($"POINT_DATA {pointValues.Count}");
			writer.WriteLine($"SCALARS {fieldName} double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double value in pointValues.Values)
			{
				writer.WriteLine(value);
			}
			writer.WriteLine();
		}

		//TODO: Avoid the duplication.
		public void WriteScalarField(string fieldName, IReadOnlyDictionary<INode, double> nodalValues)
		{
			// Points
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {nodalValues.Count} double");
			foreach (var point in nodalValues.Keys)
			{
				double[] coords = point.Coordinates();
				if (coords.Length == 2) writer.WriteLine($"{coords[0]} {coords[1]} 0.0");
				else if (coords.Length == 3) writer.WriteLine($"{coords[0]} {coords[1]} {coords[2]}");
				else throw new NotImplementedException();
			}

			// Values
			writer.Write("\n\n");
			writer.WriteLine($"POINT_DATA {nodalValues.Count}");
			writer.WriteLine($"SCALARS {fieldName} double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (var value in nodalValues.Values)
			{
				writer.WriteLine(value);
			}
			writer.WriteLine();
		}

		public void WriteScalarField(string fieldName, IReadOnlyList<double> pointValues)
		{
			writer.WriteLine($"SCALARS {fieldName} double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double value in pointValues)
			{
				writer.WriteLine(value);
			}
			writer.WriteLine();
		}

		public void WriteTensor2DField(string fieldName, IReadOnlyDictionary<double[], double[]> pointTensors)
		{
			// Points
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {pointTensors.Count} double");
			foreach (double[] point in pointTensors.Keys)
			{
				if (point.Length == 2) writer.WriteLine($"{point[0]} {point[1]} 0.0");
				else if (point.Length == 3) writer.WriteLine($"{point[0]} {point[1]} {point[2]}");
				else throw new NotImplementedException();
			}

			// Values
			writer.Write("\n\n");
			writer.WriteLine($"POINT_DATA {pointTensors.Count}");
			// Component 11
			writer.WriteLine($"SCALARS {fieldName}_11 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in pointTensors.Values) writer.WriteLine(tensor[0]);
			writer.WriteLine();

			// Component 22
			writer.WriteLine($"SCALARS {fieldName}_22 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in pointTensors.Values) writer.WriteLine(tensor[1]);
			writer.WriteLine();

			// Component 12
			writer.WriteLine($"SCALARS {fieldName}_12 double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double[] tensor in pointTensors.Values) writer.WriteLine(tensor[2]);
			writer.WriteLine();
		}

		public void WriteVectorField(string fieldName, IReadOnlyDictionary<double[], double[]> pointVectors)
		{
			// Points
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {pointVectors.Count} double");
			foreach (double[] point in pointVectors.Keys)
			{
				if (point.Length == 2) writer.WriteLine($"{point[0]} {point[1]} 0.0");
				else if (point.Length == 3) writer.WriteLine($"{point[0]} {point[1]} {point[2]}");
				else throw new NotImplementedException();
			}

			// Values
			writer.Write("\n\n");
			writer.WriteLine($"POINT_DATA {pointVectors.Count}");
			writer.WriteLine($"VECTORS {fieldName} double");
			foreach (double[] vector in pointVectors.Values)
			{
				if (vector.Length == 2) writer.WriteLine($"{vector[0]} {vector[1]} 0.0");
				else if (vector.Length == 3) writer.WriteLine($"{vector[0]} {vector[1]} {vector[2]}");
				else throw new NotImplementedException();
			}
			writer.WriteLine();
		}

		public void WriteVectorField(string fieldName, IReadOnlyList<double[]> pointVectors)
		{
			// Values
			writer.WriteLine($"VECTORS {fieldName} double");
			foreach (double[] vector in pointVectors)
			{
				if (vector.Length == 2) writer.WriteLine($"{vector[0]} {vector[1]} 0.0");
				else if (vector.Length == 3) writer.WriteLine($"{vector[0]} {vector[1]} {vector[2]}");
				else throw new NotImplementedException();
			}
			//writer.WriteLine();
			writer.Write("\n\n");
		}
	}
}
