using System;
using System.Collections.Generic;
using System.Text;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Solvers.PFetiDP
{
	public class SubdomainEnrichmentsModifiedObserver : ISubdomainModification, IEnrichmentObserver_v2
	{
		private readonly HashSet<int> modifiedSubdomains = new HashSet<int>();

		public void IncrementAnalysisIteration()
		{
			modifiedSubdomains.Clear();
		}

		public bool IsConnectivityModified(int subdomainID)
		{
			return modifiedSubdomains.Contains(subdomainID);
		}

		public bool IsStiffnessModified(int subdomainID)
		{
			return modifiedSubdomains.Contains(subdomainID);
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			modifiedSubdomains.UnionWith(node.Subdomains);
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			modifiedSubdomains.UnionWith(node.Subdomains);
		}
	}
}
