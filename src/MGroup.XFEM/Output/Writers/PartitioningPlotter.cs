using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Meshes.Output.VTK;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Output.Writers
{
	public class PartitioningPlotter : IModelObserver
	{
		private readonly int dimension;
		private readonly IXModel model;
		private readonly string outputDirectory;
		private int iteration;

		public PartitioningPlotter(string outputDirectory, IXModel model, int dimension)
		{
			this.outputDirectory = outputDirectory;
			this.model = model;
			this.dimension = dimension;
			iteration = 0;
		}

		public void Update()
		{
			PlotPartitionedElements();

			++iteration;
		}

		private void PlotPartitionedElements()
		{
			string path = Path.Combine(outputDirectory, $"partitioning_{iteration}.vtk");

			List<INode> nodes = model.EnumerateNodes().ToList();
			var allElements = new HashSet<IElement>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				allElements.UnionWith(subdomain.EnumerateElements());
			}
			var outputMesh = new VtkMeshDiscontinuous(nodes, allElements.ToList(), 2);
			var subdomainIDs = new double[outputMesh.VtkPoints.Count];
			for (int e = 0; e < outputMesh.OriginalElements.Count; ++e)
			{
				int subdomainID = outputMesh.OriginalElements[e].SubdomainID;
				VtkCell vtkCell = outputMesh.VtkCells[e];
				foreach (VtkPoint vtkPoint in vtkCell.Vertices)
				{
					subdomainIDs[vtkPoint.ID] = subdomainID;
				}
			}

			using (var writer = new VtkFileWriter(path, dimension))
			{
				writer.WriteMesh(outputMesh);
				writer.WriteScalarField("subdomainIDs", subdomainIDs);
			}
		}
	}
}
