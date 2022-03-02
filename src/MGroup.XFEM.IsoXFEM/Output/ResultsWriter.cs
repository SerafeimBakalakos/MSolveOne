using MGroup.XFEM.IsoXFEM.Output;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Output;
using MGroup.LinearAlgebra.Output.Formatting;
using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.IsoXFEM.IsoXfemElements;
using System.Linq;
using System.IO;
using MGroup.XFEM.Geometry;

namespace MGroup.XFEM.IsoXFEM.Output
{
	class ResultsWriter
	{
		public static void ResultsWriterToTxt(Matrix results)
		{
			//var writer = new FullMatrixWriter();
			//writer.NumericFormat = new ExponentialFormat() { NumDecimalDigits = 17 };
			string path = $"{Paths.OutputForTxtResults}\\2DTriangulator_100x50.txt";
			using (var writer = new StreamWriter(path))
			{
				for (int i = 0; i < results.NumRows; i++)
				{
					writer.WriteLine($"iter {results[i,0]}:  ");
					writer.WriteLine($"\t\t SED={ results[i, 1]} ");
					writer.WriteLine($"\t\t VolumeFraction={ results[i, 2]} ");
				}
			}
            //writer.WriteToFile(results, path);
		}

		public static void VolumeForEachElementWriter(int iter, int dimension, Dictionary<int, IIsoXfemElement> Elements)
		{
			string path = $"{Paths.OutputElementsSize}\\ElementsSize_iter_{iter}_Dimension_{dimension}.txt";

			using (var writer = new StreamWriter(path))
			{
				if (dimension == 3)
				{
					foreach (var element in Elements.Values)
					{
						string index = $"{element.IdOnAxis[0]},{element.IdOnAxis[1]},{element.IdOnAxis[2]}";
						writer.WriteLine($"element {index}:  ");
						writer.WriteLine($"\t\t size={ element.SizeOfElement} ");
					}
				}
				else
				{
					foreach (var element in Elements.Values)
					{
						string index = $"{element.IdOnAxis[0]},{element.IdOnAxis[1]}";
						writer.WriteLine($"element {index}:  ");
						writer.WriteLine($"\t\t size={ element.SizeOfElement} ");
					}
				}
			}
		}

		public static void GaussPointsWriter(int iter, int dimension, Dictionary<int, IIsoXfemElement> Elements)
		{
			string path = $"{Paths.OutputElementsSize}\\GaussPointsElementsSize_iter_{iter}.txt";

			using (var writer = new StreamWriter(path))
				if (dimension == 2)
				{
					for (int i = 0; i < Elements.Count; i++)
					{
						string index = $"{Elements[i].IdOnAxis[0]},{Elements[i].IdOnAxis[1]}";
						writer.WriteLine($"element {index}:  ");
						writer.WriteLine($"\t\t size={ Elements[i].SizeOfElement} ");
						if (Elements[i].ConformingSubcells != null)
						{
							foreach (var gausspoint in Elements[i].BulkIntegrationPoints)
							{
								string gp = $"{gausspoint.Coordinates[0]},{gausspoint.Coordinates[1]}";
								writer.WriteLine($"\t\t ξ,ζ {gp}");
								writer.WriteLine($"\t\t weight= {gausspoint.Weight}");
							}
						}
						else
						{
							foreach (var gausspoint in Elements[i].IntegrationStandard.IntegrationPoints)
							{
								string gp = $"{gausspoint.Coordinates[0]},{gausspoint.Coordinates[1]},{gausspoint.Coordinates[2]}";
								writer.WriteLine($"\t\t ξ,ζ,η {gp}");
								writer.WriteLine($"\t\t weight= {gausspoint.Weight}");
							}
						}
					}
				}
				else
				{
					for (int i = 0; i < Elements.Count; i++)
					{
						string index = $"{Elements[i].IdOnAxis[0]},{Elements[i].IdOnAxis[1]},{Elements[i].IdOnAxis[2]}";
						writer.WriteLine($"element {index}:  ");
						writer.WriteLine($"\t\t size={ Elements[i].SizeOfElement} ");
						if (Elements[i].ConformingSubcells != null)
						{
							foreach (var gausspoint in Elements[i].BulkIntegrationPoints)
							{
								string gp = $"{gausspoint.Coordinates[0]},{gausspoint.Coordinates[1]},{gausspoint.Coordinates[2]}";
								writer.WriteLine($"\t\t ξ,ζ,η {gp}");
								writer.WriteLine($"\t\t weight= {gausspoint.Weight}");
							}
						}
						else
						{
							foreach (var gausspoint in Elements[i].IntegrationStandard.IntegrationPoints)
							{
								string gp = $"{gausspoint.Coordinates[0]},{gausspoint.Coordinates[1]},{gausspoint.Coordinates[2]}";
								writer.WriteLine($"\t\t ξ,ζ,η {gp}");
								writer.WriteLine($"\t\t weight= {gausspoint.Weight}");
							}
						}
					}

				}

		}

		public static void InteractionPoints3DWriter(int iter, int dimension, Dictionary<int, IIsoXfemElement> Elements)
		{
			string path = $"{Paths.OutputElementsSize}\\ElementsInteraction_iter_{iter}.txt";

			using (var writer = new StreamWriter(path))
			{
				if (dimension == 3)
				{
					foreach (var element in Elements.Values)
					{
						string index = $"{element.IdOnAxis[0]},{element.IdOnAxis[1]},{element.IdOnAxis[2]}";
						writer.WriteLine($"element {index}:  ");
						if (element.InteractingDiscontinuities != null)
						{
							var pointcompare = new Point3DComparer();
							var sortedVertices = new SortedSet<double[]>(pointcompare);
							foreach (var intersections in element.InteractingDiscontinuities.Values)
							{
								var intersectionVertices = intersections.GetVerticesForTriangulation();
								foreach (var intersection in intersectionVertices)
								{
									sortedVertices.Add(intersection);									
								}
								foreach (var pointvertice in sortedVertices)
								{
									string vertices = $"{pointvertice[0]},{pointvertice[1]},{pointvertice[2]}";
									writer.WriteLine($"\t\t ξ,ζ,η {vertices}");
								}
							}	
						}
					}
				}
			}
		}
		public static void NodalLevelSetsWriter(int iter, int dimension, Dictionary<int, XNode> Nodes, Vector nodalLevelSet)
		{
			string path = $"{Paths.OutputElementsSize}\\NodalLevelSets_iter_{iter}.txt";

			using (var writer = new StreamWriter(path))
			{
				if (dimension == 3)
				{
					foreach (var node in Nodes.Values)
					{
						string index = $"{node.Coordinates[0]},{node.Coordinates[1]},{node.Coordinates[2]}";
						writer.WriteLine($"node {index}:  ");
						writer.WriteLine($"\t\t levelsetValue={ nodalLevelSet[node.ID]} ");
					}
				}
				else
				{
					foreach (var node in Nodes.Values)
					{
						string index = $"{node.Coordinates[0]},{node.Coordinates[1]}";
						writer.WriteLine($"node {index}:  ");
						writer.WriteLine($"\t\t levelsetValue={ nodalLevelSet[node.ID]} ");
					}
				}
			}
		}
	}
}
