using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Enrichment.Observers;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class PhaseEnrichmentPlotter : IEnrichmentObserver
	{
		private readonly int dimension;
		private readonly double elementSize;
		private readonly string outputDirectory;
		private readonly XModel<IXMultiphaseElement> model;

		private int iteration = 0;

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies => Array.Empty<IEnrichmentObserver>();

		public PhaseEnrichmentPlotter(string outputDirectory, XModel<IXMultiphaseElement> model, 
			double elementSize, int dimension)
		{
			this.outputDirectory = outputDirectory;
			this.model = model;
			this.elementSize = elementSize;
			this.dimension = dimension;
		}

		public void EndCurrentAnalysisIteration()
		{
			PlotEnrichedNodesCategory(enr => enr.EnrichmentFunctions[0] is PhaseStepEnrichment,
				Path.Combine(outputDirectory, $"enriched_nodes_heaviside_t{iteration}.vtk"), "enriched_nodes_heaviside");

			PlotEnrichedNodesCategory(enr => enr.EnrichmentFunctions[0] is RidgeEnrichment,
				Path.Combine(outputDirectory, $"enriched_nodes_ridge_t{iteration}.vtk"), "enriched_nodes_ridge");

			PlotEnrichedNodesCategory(enr => enr.EnrichmentFunctions[0] is JunctionEnrichment,
				Path.Combine(outputDirectory, $"enriched_nodes_junction_t{iteration}.vtk"), "enriched_nodes_junction");

			++iteration;
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
		}

		public void StartNewAnalysisIteration()
		{
		}

		private void PlotEnrichedNodesCategory(Func<EnrichmentItem, bool> predicate, string path, string categoryName)
		{
			var nodesToPlot = new Dictionary<double[], double>();
			foreach (XNode node in model.Nodes.Values)
			{
				if (node.Enrichments.Count == 0) continue;
				EnrichmentItem[] enrichments = node.Enrichments.Where(predicate).ToArray();
				if (enrichments.Length == 1)
				{
					var point = node.Coordinates;
					nodesToPlot[point] = enrichments[0].ID;
				}
				else
				{
					double[][] nodeInstances = DuplicateNodeForBetterViewing(node, enrichments.Length);
					for (int e = 0; e < enrichments.Length; ++e)
					{
						double[] point = nodeInstances[e];
						nodesToPlot[point] = enrichments[e].ID;
					}
				}
			}
			if (nodesToPlot.Count > 0)
			{
				using (var writer = new VtkPointWriter(path))
				{
					writer.WriteScalarField(categoryName, nodesToPlot);
				}
			}
		}

		private double[][] DuplicateNodeForBetterViewing(XNode node, int numInstances)
		{
			//TODO: Add more. If numInstances > 4 (or 8) for 2D (or 3D), then scatter points in a cloud around the node with radius = offset

			double[][] possibilites; // The further ones apart go to top
			if (dimension == 2)
			{
				possibilites = new double[4][]; // The further ones apart go to top
				double offset = 0.05 * elementSize;
				possibilites[0] = new double[] { node.X - offset, node.Y - offset };
				possibilites[1] = new double[] { node.X + offset, node.Y + offset };
				possibilites[2] = new double[] { node.X + offset, node.Y - offset };
				possibilites[3] = new double[] { node.X - offset, node.Y + offset };
			}
			else
			{
				possibilites = new double[8][];
				double offset = 0.05 * elementSize;
				possibilites[0] = new double[] { node.X - offset, node.Y - offset, node.Z - offset };
				possibilites[1] = new double[] { node.X + offset, node.Y + offset, node.Z - offset };
				possibilites[2] = new double[] { node.X + offset, node.Y - offset, node.Z - offset };
				possibilites[3] = new double[] { node.X - offset, node.Y + offset, node.Z - offset };
				possibilites[4] = new double[] { node.X - offset, node.Y - offset, node.Z + offset };
				possibilites[5] = new double[] { node.X + offset, node.Y + offset, node.Z + offset };
				possibilites[6] = new double[] { node.X + offset, node.Y - offset, node.Z + offset };
				possibilites[7] = new double[] { node.X - offset, node.Y + offset, node.Z + offset };
			}

			var instances = new double[numInstances][];
			for (int i = 0; i < numInstances; ++i) instances[i] = possibilites[i];
			return instances;
		}
	}
}
