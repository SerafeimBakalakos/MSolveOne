using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Observers;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class CrackEnrichmentPlotterBasic : IEnrichmentObserver
	{
		private static readonly double[] dummyPoint = new double[] { 0.0, 0.0, 0.0 }; //TODO: find a more elegant solution.

		private readonly AllCrackStepNodesObserver bodyNodesObserver;
		private readonly NewCrackTipNodesObserver newTipNodesObserver;
		private readonly string outputDirectory;

		private int iteration;

		public CrackEnrichmentPlotterBasic(string outputDirectory, 
			NewCrackTipNodesObserver newTipNodesObserver, AllCrackStepNodesObserver bodyNodesObserver)
		{
			this.outputDirectory = outputDirectory.TrimEnd('\\');
			this.newTipNodesObserver = newTipNodesObserver;
			this.bodyNodesObserver = bodyNodesObserver;
			this.iteration = 0;
		}

		public void EndCurrentAnalysisIteration()
		{
			// Write the new tip enriched nodes and the signs of their crack body level sets.
			using (var writer = new VtkPointWriter($"{outputDirectory}\\tip_nodes_new_{iteration}.vtk"))
			{
				var tipNodesNew = new Dictionary<double[], double>();
				foreach (XNode node in newTipNodesObserver.NewCrackTipNodes)
				{
					tipNodesNew[node.Coordinates] = 0.0;
				}
				writer.WriteScalarField("Tip_nodes_new", tipNodesNew);
			}

			// Write all Heaviside enriched nodes and the signs of their crack body level sets.
			using (var writer = new VtkPointWriter($"{outputDirectory}\\heaviside_nodes_all_{iteration}.vtk"))
			{
				var heavisideNodesAll = new Dictionary<double[], double>();
				foreach (XNode node in bodyNodesObserver.AllCrackStepNodes)
				{
					heavisideNodesAll[node.Coordinates] = 0.0;
				}
				if (heavisideNodesAll.Count == 0) //a dummy node just to get the Paraview reader working.
				{
					heavisideNodesAll[dummyPoint] = 0;
				}
				writer.WriteScalarField("Heaviside_nodes_all", heavisideNodesAll);
			}

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

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies
		{
			get
			{
				return new IEnrichmentObserver[]
				{
					newTipNodesObserver, bodyNodesObserver
				};
			}
		}
	}
}
