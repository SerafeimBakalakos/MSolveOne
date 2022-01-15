using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.Environments;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class BoundaryNodesPlotter : IModelObserver
	{
		private readonly IComputeEnvironment environment;
		private readonly XModel<IXCrackElement> model;
		private readonly string outputDirectory;

		private int iteration = 0;

		public BoundaryNodesPlotter(IComputeEnvironment environment, XModel<IXCrackElement> model, string outputDirectory)
		{
			this.environment = environment;
			this.model = model;
			this.outputDirectory = outputDirectory;
		}

		public void Update()
		{
			environment.DoGlobalOperation(() =>
			{
				PlotBoundaryNodes();
			});

			++iteration;
		}

		private void PlotBoundaryNodes()
		{
			string path = Path.Combine(outputDirectory, $"boundary_nodes_{iteration}.vtk");
			using (var writer = new VtkPointWriter(path))
			{
				IEnumerable<XNode> boundaryNodes = model.Nodes.Values.Where(node => node.Subdomains.Count > 1);
				IEnumerable<VtkPoint> points = boundaryNodes.Select(node => new VtkPoint(node.ID, node.Coordinates));
				writer.WritePoints(points);
			}
		}
	}
}
