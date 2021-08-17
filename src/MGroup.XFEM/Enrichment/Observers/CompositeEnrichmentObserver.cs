using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Enrichment.Observers
{
	/// <summary>
	/// Acts simultaneously as multiple observers which may depend on each other to be called in specific order.
	/// </summary>
	public class CompositeEnrichmentObserver : IEnrichmentObserver_v2
	{
		/// <summary>
		/// Observers are stored in levels depending on how many other observers they depend on.
		/// </summary>
		private readonly List<List<IEnrichmentObserver_v2>> observers = new List<List<IEnrichmentObserver_v2>>();

		public CompositeEnrichmentObserver()
		{
			observers = new List<List<IEnrichmentObserver_v2>>(10);
			for (int i = 0; i < 5; ++i)
			{
				observers.Add(new List<IEnrichmentObserver_v2>());
			}
		}

		public void AddObserver(IEnrichmentObserver_v2 observer)
		{
			int level = observer.NumObserverDependencies;
			ResizeToStore(level);
			observers[level].Add(observer);
		}

		public void AddObservers(params IEnrichmentObserver_v2[] observers)
		{
			foreach (IEnrichmentObserver_v2 observer in observers)
			{
				AddObserver(observer);
			}
		}

		public void EndCurrentAnalysisIteration()
		{
			foreach (List<IEnrichmentObserver_v2> nestedObservers in observers)
			{
				foreach (IEnrichmentObserver_v2 observer in nestedObservers)
				{
					observer.EndCurrentAnalysisIteration();
				}
			}
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			foreach (List<IEnrichmentObserver_v2> nestedObservers in observers)
			{
				foreach (IEnrichmentObserver_v2 observer in nestedObservers)
				{
					observer.LogEnrichmentAddition(node, enrichment);
				}
			}
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			foreach (List<IEnrichmentObserver_v2> nestedObservers in observers)
			{
				foreach (IEnrichmentObserver_v2 observer in nestedObservers)
				{
					observer.LogEnrichmentRemoval(node, enrichment);
				}
			}
		}

		public void StartNewAnalysisIteration()
		{
			foreach (List<IEnrichmentObserver_v2> nestedObservers in observers)
			{
				foreach (IEnrichmentObserver_v2 observer in nestedObservers)
				{
					observer.StartNewAnalysisIteration();
				}
			}
		}

		private void ResizeToStore(int index)
		{
			int numLevelsTarget = index + 1;
			int numLevelsCurrent = observers.Count;
			if (numLevelsCurrent >= numLevelsTarget)
			{
				return;
			}

			if (numLevelsTarget > observers.Capacity)
			{
				observers.Capacity = numLevelsTarget;
				for (int i = numLevelsCurrent; i < numLevelsTarget; ++i)
				{
					observers.Add(new List<IEnrichmentObserver_v2>());
				}
			}
		}
	}
}
