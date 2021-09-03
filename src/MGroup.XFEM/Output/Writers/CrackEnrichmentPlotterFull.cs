using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Observers;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

//TODO: Let the user decide which of these node categories to plot (and the corresponding output filenames).
namespace MGroup.XFEM.Output.Writers
{
	public class CrackEnrichmentPlotterFull : IEnrichmentObserver
	{
		private static readonly double[] dummyPoint = new double[] { 0.0, 0.0, 0.0 }; //TODO: find a more elegant solution.

		private readonly AllCrackStepNodesObserver bodyNodesObserver;
		private readonly ExteriorLsmCrack2D crack;
		private readonly NodesNearModifiedNodesObserver nearModifiedNodesObserver;
		private readonly NewCrackStepNodesObserver newBodyNodesObserver;
		private readonly NewCrackTipNodesObserver newTipNodesObserver;
		private readonly PreviousCrackTipNodesObserver previousTipNodesObserver;
		private readonly RejectedCrackStepNodesObserver rejectedBodyNodesObserver;
		private readonly string outputDirectory;

		private int iteration;

		public CrackEnrichmentPlotterFull(ExteriorLsmCrack2D crack, string outputDirectory,
			NewCrackTipNodesObserver newTipNodesObserver, PreviousCrackTipNodesObserver previousTipNodesObserver,
			AllCrackStepNodesObserver bodyNodesObserver, NewCrackStepNodesObserver newBodyNodesObserver,
			RejectedCrackStepNodesObserver rejectedBodyNodesObserver, NodesNearModifiedNodesObserver nearModifiedNodesObserver)
		{
			this.crack = crack;
			this.outputDirectory = outputDirectory.TrimEnd('\\');
			this.newTipNodesObserver = newTipNodesObserver;
			this.previousTipNodesObserver = previousTipNodesObserver;
			this.bodyNodesObserver = bodyNodesObserver;
			this.newBodyNodesObserver = newBodyNodesObserver;
			this.rejectedBodyNodesObserver = rejectedBodyNodesObserver;
			this.nearModifiedNodesObserver = nearModifiedNodesObserver;
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
					tipNodesNew[node.Coordinates] = crack.LsmGeometry.LevelSetsTip[node.ID];
				}
				writer.WriteScalarField("Tip_nodes_new", tipNodesNew);
			}

			// Write the old tip enriched nodes and the signs of their crack body level sets.
			using (var writer = new VtkPointWriter($"{outputDirectory}\\tip_nodes_old_{iteration}.vtk"))
			{
				var tipNodesOld = new Dictionary<double[], double>();
				if (iteration > 0)
				{
					foreach (XNode node in previousTipNodesObserver.PreviousCrackTipNodes)
					{
						tipNodesOld[node.Coordinates] = crack.LsmGeometry.LevelSetsTip[node.ID];
					}
				}
				else // else a dummy node just to get the Paraview reader working. 
				{
					tipNodesOld[dummyPoint] = 0;
				}
				writer.WriteScalarField("Tip_nodes_old", tipNodesOld);
			}

			// Write all Heaviside enriched nodes and the signs of their crack body level sets.
			using (var writer = new VtkPointWriter($"{outputDirectory}\\heaviside_nodes_all_{iteration}.vtk"))
			{
				var heavisideNodesAll = new Dictionary<double[], double>();
				foreach (XNode node in bodyNodesObserver.AllCrackStepNodes)
				{
					heavisideNodesAll[node.Coordinates] = crack.LsmGeometry.LevelSets[node.ID];
				}
				writer.WriteScalarField("Heaviside_nodes_all", heavisideNodesAll);
			}

			// Write only the new Heaviside enriched nodes and the signs of their crack body level sets.
			using (var writer = new VtkPointWriter($"{outputDirectory}\\heaviside_nodes_new_{iteration}.vtk"))
			{
				var heavisideNodesNew = new Dictionary<double[], double>();
				foreach (XNode node in newBodyNodesObserver.NewCrackStepNodes)
				{
					heavisideNodesNew[node.Coordinates] = crack.LsmGeometry.LevelSets[node.ID];
				}
				writer.WriteScalarField("Heaviside_nodes_new", heavisideNodesNew);
			}

			// Write the nodes that belong to elements intersected by the crack, but are not enriched with Heaviside 
			using (var writer = new VtkPointWriter($"{outputDirectory}\\heaviside_rejected_nodes_{iteration}.vtk"))
			{
				var rejectedNodes = new Dictionary<double[], double>();
				foreach (XNode node in rejectedBodyNodesObserver.RejectedHeavisideNodes)
				{
					rejectedNodes[node.Coordinates] = crack.LsmGeometry.LevelSets[node.ID];
				}
				if (rejectedNodes.Count == 0) //a dummy node just to get the Paraview reader working.
				{
					rejectedNodes[dummyPoint] = 0;
				}
				writer.WriteScalarField("Heaviside_rejected_nodes", rejectedNodes);
			}

			// Write unmodified Heaviside nodes of elements with at least one modified node
			using (var writer = new VtkPointWriter($"{outputDirectory}\\near_modified_heaviside_nodes_{iteration}.vtk"))
			{
				var nearModifiedHeavisideNodes = new Dictionary<double[], double>();
				foreach (XNode node in nearModifiedNodesObserver.NearModifiedNodes)
				{
					if (node.Enrichments.Contains(crack.CrackBodyEnrichment))
					{
						nearModifiedHeavisideNodes[node.Coordinates] = crack.LsmGeometry.LevelSets[node.ID];
					}
				}
				if (nearModifiedHeavisideNodes.Count == 0) // a dummy node just to get the Paraview reader working.
				{
					nearModifiedHeavisideNodes[dummyPoint] = 0;
				}
				writer.WriteScalarField("near_modified_heaviside_nodes", nearModifiedHeavisideNodes);
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
					newTipNodesObserver, previousTipNodesObserver, bodyNodesObserver, newBodyNodesObserver,
					rejectedBodyNodesObserver, nearModifiedNodesObserver
				};
			}
		}
	}
}
