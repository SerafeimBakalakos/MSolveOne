using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.Environments;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Solvers.PFetiDP
{
	public class CornerNodesPlotter : IModelObserver
	{
		private readonly IComputeEnvironment environment;
		private readonly XModel<IXCrackElement> model;
		private readonly CrackFetiDPCornerDofsPlusLogging cornerDofsSelection;
		private readonly string outputDirectory;

		private int iteration = 0;

		public CornerNodesPlotter(IComputeEnvironment environment, XModel<IXCrackElement> model, 
			CrackFetiDPCornerDofsPlusLogging cornerDofsSelection, string outputDirectory)
		{
			this.environment = environment;
			this.model = model;
			this.cornerDofsSelection = cornerDofsSelection;
			this.outputDirectory = outputDirectory;

			//Dictionary<int, int[]> subdomainCorners = environment.CalcNodeDataAndTransferToGlobalMemory(s =>
			//{
			//	ISubdomain subdomain = model.GetSubdomain(s);
			//	int[] corners = getStandardCornerNodesOfSubdomain(subdomain).Select(node => node.ID).ToArray();
			//	return corners;
			//});
			//environment.DoGlobalOperation(() =>
			//{
			//	foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			//	{
			//		stdCornerNodes.UnionWith(subdomainCorners[subdomain.ID]);
			//	}
			//});
		}

		public void Update()
		{
			environment.DoGlobalOperation(() =>
			{
				if (iteration == 0)
				{
					PlotStandardCornerNodes();
				}

				PlotEnrichedCornerNodes();
			});

			++iteration;
		}

		private void PlotEnrichedCornerNodes()
		{
			string path = Path.Combine(outputDirectory, $"corner_nodes_enr_{iteration}.vtk");
			using (var writer = new VtkPointWriter(path))
			{
				IEnumerable<XNode> corners = model.Nodes.Values.Where(node => cornerDofsSelection.HasEnrCornerDofs(node));
				IEnumerable<VtkPoint> points = corners.Select(node => new VtkPoint(node.ID, node.Coordinates));
				writer.WritePoints(points);
			}
		}

		private void PlotStandardCornerNodes()
		{
			string path = Path.Combine(outputDirectory, "corner_nodes_std.vtk");
			using (var writer = new VtkPointWriter(path))
			{
				IEnumerable<XNode> corners = model.Nodes.Values.Where(node => cornerDofsSelection.HasStdCornerDofs(node));
				IEnumerable<VtkPoint> points = corners.Select(node => new VtkPoint(node.ID, node.Coordinates));
				writer.WritePoints(points);
			}
		}
	}
}
