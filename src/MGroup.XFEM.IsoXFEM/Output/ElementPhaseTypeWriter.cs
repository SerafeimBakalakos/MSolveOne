using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.LinearAlgebra.Matrices;

namespace MGroup.XFEM.IsoXFEM.Output
{
	public class ElementPhaseTypeWriter
	{
		public const string vtkReaderVersion = "4.1";
		private readonly StreamWriter writer;
		//private int numVertices = -1;

		public ElementPhaseTypeWriter(string filePath)
		{
			this.writer = new StreamWriter(filePath);
			writer.Write("# vtk DataFile Version ");
			writer.WriteLine(vtkReaderVersion);
			//writer.WriteLine(filePath);
			writer.WriteLine("Header:");
			writer.Write("ASCII\n\n");
		}

		public void Dispose()
		{
			if (writer != null)
			{
				writer.Dispose();
			}
		}

		public void PlotElementPhaseTypes(List<int> elementPhaseTypes, Matrix nodeCoordinates, int[,] connection)
		{
			(List<double[]> nodeCoords, List<int[]> elementConnectivity, List<double> nodeValues) = 
				CalcPlotData(elementPhaseTypes, nodeCoordinates, connection);

			// Vertices 
			int numNodes = nodeCoords.Count;
			writer.WriteLine("DATASET UNSTRUCTURED_GRID");
			writer.WriteLine($"POINTS {numNodes} double");
			for (int n = 0; n < numNodes; ++n)
			{
				writer.WriteLine($"{nodeCoords[n][0]} {nodeCoords[n][1]} 0.0");
			}

			// Cell connectivity
			int cellDataCount = 0;
			for (int e = 0; e < elementConnectivity.Count; ++e)
			{
				cellDataCount += 1 + elementConnectivity[0].Length;
			}
			writer.WriteLine($"\nCELLS {elementConnectivity.Count} {cellDataCount}");
			for (int e = 0; e < elementConnectivity.Count; ++e)
			{
				writer.Write(elementConnectivity[e].Length);
				for (int n = 0; n < elementConnectivity[e].Length; ++n)
				{
					writer.Write(' ');
					writer.Write(elementConnectivity[e][n]);
				}
				writer.WriteLine();
			}

			// Cell types
			writer.WriteLine("\nCELL_TYPES " + elementConnectivity.Count);
			int quad4Code = 9;
			for (int i = 0; i < elementConnectivity.Count; ++i)
			{
				writer.WriteLine(quad4Code);
			}

			// Nodal values
			writer.Write("\n\n");
			writer.WriteLine("POINT_DATA " + numNodes);
			writer.WriteLine($"SCALARS phase_type double 1");
			writer.WriteLine("LOOKUP_TABLE default");
			foreach (double value in nodeValues)
			{
				writer.WriteLine(value);
			};
			writer.WriteLine();
		}


		private (List<double[]> nodeCoords, List<int[]> elementConnectivity, List<double> nodeValues) CalcPlotData(
			List<int> elementPhaseTypes, Matrix nodeCoordinates, int[,] elementConnectivity)
		{
			var nodesExtended = new List<double[]>();
			var connectivityExtended = new List<int[]>();
			var nodeValues = new List<double>();
			for (int e = 0; e < elementConnectivity.GetLength(0); ++e)
			{
				var connectivity = new int[elementConnectivity.GetLength(1)];
				for (int n = 0; n < elementConnectivity.GetLength(1); ++n)
				{
					int nodeIdx = elementConnectivity[e, n];
					double[] coords = { nodeCoordinates[nodeIdx, 0], nodeCoordinates[nodeIdx, 1] };
					nodesExtended.Add(coords);
					connectivity[n] = nodesExtended.Count - 1;
					nodeValues.Add(elementPhaseTypes[e]);
				}
				connectivityExtended.Add(connectivity);
			}

			return (nodesExtended, connectivityExtended, nodeValues);
		}


		#region code alternatives
		public void Foo()
		{
			double res;
			bool success = TryCalcSqrt(-3, out res);
		}

		public double CalcSqrt(double input)
		{
			if (input >= 0)
			{
				return Math.Sqrt(input);
			}
			else
			{
				//throw new Exception("Blah blah went wrong");
				return double.NaN;
			}
		}

		public (bool success, double result) TryCalcSqrt(double input)
		{
			if (input >= 0)
			{
				return (true, Math.Sqrt(input));
			}
			else
			{
				//throw new Exception("Blah blah went wrong");
				return (false, double.NaN);
			}

			
		}

		public bool TryCalcSqrt(double input, out double result)
		{
			if (input >= 0)
			{
				result = Math.Sqrt(input);
				return true;
			}
			else
			{
				result = double.NaN;
				return false;
			}
		}
		#endregion
	}
}
