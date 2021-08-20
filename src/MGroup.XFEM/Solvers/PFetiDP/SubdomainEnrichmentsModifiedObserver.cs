using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Observers;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Solvers.PFetiDP
{
	public class SubdomainEnrichmentsModifiedObserver : IModifiedSubdomains, IEnrichmentObserver
	{
		private readonly HashSet<int> modifiedSubdomains = new HashSet<int>();

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies => Array.Empty<IEnrichmentObserver>();

		public void EndCurrentAnalysisIteration()
		{
		}

		public bool IsConnectivityModified(int subdomainID)
		{
			return modifiedSubdomains.Contains(subdomainID);
		}

		public bool IsMatrixModified(int subdomainID)
		{
			return modifiedSubdomains.Contains(subdomainID);
		}

		public bool IsRhsModified(int subdomainID) => IsConnectivityModified(subdomainID);


		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			modifiedSubdomains.UnionWith(node.Subdomains);
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			modifiedSubdomains.UnionWith(node.Subdomains);
		}

		public void StartNewAnalysisIteration()
		{
			modifiedSubdomains.Clear();
		}
	}
}
