using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Meshes.Output.VTK;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Mesh;

namespace MGroup.XFEM.Output.Writers
{
	public class PartitioningPlotter : IModelObserver
	{
		private readonly int dimension;
		private readonly bool plotRectangularSubdomainEdges;
		private readonly IXModel model;
		private readonly string outputDirectory;
		private int iteration;

		public PartitioningPlotter(string outputDirectory, IXModel model, int dimension, bool plotRectangularSubdomainEdges = false)
		{
			this.outputDirectory = outputDirectory;
			this.model = model;
			this.dimension = dimension;
			this.plotRectangularSubdomainEdges = plotRectangularSubdomainEdges;
			iteration = 0;
		}

		public void Update()
		{
			PlotPartitionedElements();
			if (plotRectangularSubdomainEdges)
			{
				PlotRectangularSubdomainEdges();
			}

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
			var outputMesh = new VtkMeshDiscontinuous(nodes, allElements.ToList(), dimension);
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

		private void PlotRectangularSubdomainEdges()
		{
			var subdomainMesh = new CustomMesh();
			int numPoints = 0;
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				if (dimension == 2)
				{
					//TODO: The order should not depend on how CornerNodeUtilities.FindCornersOfRectangle2D works
					INode[] corners = CornerNodeUtilities.FindCornersOfRectangle2D(subdomain);
					var p0 = new Vtk.VtkPoint(numPoints++, new double[] { corners[0].X, corners[0].Y });
					var p1 = new Vtk.VtkPoint(numPoints++, new double[] { corners[1].X, corners[1].Y });
					var p2 = new Vtk.VtkPoint(numPoints++, new double[] { corners[3].X, corners[3].Y });
					var p3 = new Vtk.VtkPoint(numPoints++, new double[] { corners[2].X, corners[2].Y });
					subdomainMesh.Vertices.Add(p0.ID, p0);
					subdomainMesh.Vertices.Add(p1.ID, p1);
					subdomainMesh.Vertices.Add(p2.ID, p2);
					subdomainMesh.Vertices.Add(p3.ID, p3);
					subdomainMesh.Cells.Add(new Vtk.VtkCell(MSolve.Discretization.Mesh.CellType.Quad4, new Vtk.VtkPoint[] { p0, p1, p2, p3 }));
				}
				else
				{
					throw new NotImplementedException();
				}
			}

			string path = Path.Combine(outputDirectory, $"subdomain_mesh_{iteration}.vtk");
			using (var writer = new Vtk.VtkFileWriter(path))
			{
				writer.WriteMesh(subdomainMesh);
			}
		}
	}
}
