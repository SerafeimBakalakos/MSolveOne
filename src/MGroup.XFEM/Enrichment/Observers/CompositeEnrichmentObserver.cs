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
	public class CompositeEnrichmentObserver : IEnrichmentObserver
	{
		/// <summary>
		/// Observers are stored in levels depending on how many other observers they depend on.
		/// </summary>
		private readonly List<List<IEnrichmentObserver>> observers = new List<List<IEnrichmentObserver>>();

		public CompositeEnrichmentObserver()
		{
			observers = new List<List<IEnrichmentObserver>>(2);
			for (int i = 0; i < 5; ++i)
			{
				observers.Add(new List<IEnrichmentObserver>());
			}
		}

		public IReadOnlyCollection<IEnrichmentObserver> ObserverDependencies => Array.Empty<IEnrichmentObserver>();

		public void AddObserver(IEnrichmentObserver observer)
		{
			int level = FindDependencyLevel(observer, 0);
			ResizeToStore(level);
			observers[level].Add(observer);
		}

		public void AddObservers(params IEnrichmentObserver[] observers)
		{
			foreach (IEnrichmentObserver observer in observers)
			{
				AddObserver(observer);
			}
		}

		public void EndCurrentAnalysisIteration()
		{
			foreach (List<IEnrichmentObserver> nestedObservers in observers)
			{
				foreach (IEnrichmentObserver observer in nestedObservers)
				{
					observer.EndCurrentAnalysisIteration();
				}
			}
		}

		public void LogEnrichmentAddition(XNode node, EnrichmentItem enrichment)
		{
			foreach (List<IEnrichmentObserver> nestedObservers in observers)
			{
				foreach (IEnrichmentObserver observer in nestedObservers)
				{
					observer.LogEnrichmentAddition(node, enrichment);
				}
			}
		}

		public void LogEnrichmentRemoval(XNode node, EnrichmentItem enrichment)
		{
			foreach (List<IEnrichmentObserver> nestedObservers in observers)
			{
				foreach (IEnrichmentObserver observer in nestedObservers)
				{
					observer.LogEnrichmentRemoval(node, enrichment);
				}
			}
		}

		public void StartNewAnalysisIteration()
		{
			foreach (List<IEnrichmentObserver> nestedObservers in observers)
			{
				foreach (IEnrichmentObserver observer in nestedObservers)
				{
					observer.StartNewAnalysisIteration();
				}
			}
		}

		/// <summary>
		/// 
		/// </summary>
		/// <param name="observer"></param>
		/// <param name="currentLevel">0 for top level clients. Progressively increasing while called by itself.</param>
		private int FindDependencyLevel(IEnrichmentObserver observer, int currentLevel)
		{
			// Recursive method
			if (observer.ObserverDependencies.Count == 0)
			{
				return currentLevel;
			}
			else
			{
				int maxLevelOfDependencies = 0;
				foreach (IEnrichmentObserver dependency in observer.ObserverDependencies)
				{
					int levelOfDependency = FindDependencyLevel(dependency, currentLevel + 1);
					if (levelOfDependency > maxLevelOfDependencies)
					{
						maxLevelOfDependencies = levelOfDependency;
					}
				}
				return maxLevelOfDependencies;
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
					observers.Add(new List<IEnrichmentObserver>());
				}
			}
		}
	}
}
